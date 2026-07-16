#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive.hpp>
#include <std_msgs/msg/bool.hpp>
#include <pthread.h>
#include <cmath>
#include <sstream>
#include <memory>
#include <fsai_api/msg/vcu2_ai.hpp>

extern "C" {
#include "fsai_api/fs-ai_api.h"
}

// Constants
const float DEGREE_CONVERSION = 180.0 / M_PI;
const float WHEEL_RADIUS = 0.2575; // Example wheel radius in meters

class AckermannCanNode : public rclcpp::Node
{
public:
    AckermannCanNode() : Node("ackermann_can")
    {
        // Initialize global variables
        drive_enabled_ = false;
        chequered_flag_ = false;
        destroy_node_ = false;
        braking_ = false;
        timing_us_ = 10000;

        // Create publishers
        vcu2ai_pub_ = this->create_publisher<fsai_api::msg::VCU2AI>("vcu2ai", 10);

        // Create subscribers
        ackermann_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDrive>(
            "/ackermann_cmd", 10,
            std::bind(&AckermannCanNode::ackermannCmdCallback, this, std::placeholders::_1));

        emergency_brake_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/emergency_brake", 10,
            std::bind(&AckermannCanNode::emergencyBrakeCallback, this, std::placeholders::_1));

        brake_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/brake", 10,
            std::bind(&AckermannCanNode::brakeCallback, this, std::placeholders::_1));

        chequered_flag_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/chequered_flag", 10,
            std::bind(&AckermannCanNode::chequeredFlagCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "AckermannCanNode initialized");
    }

    ~AckermannCanNode()
    {
        if (loop_thread_running_) {
            loop_thread_running_ = false;
            if (loop_thread_.joinable()) {
                loop_thread_.join();
            }
        }
    }

    bool initializeCanInterface(const std::string& can_interface)
    {
        if (fs_ai_api_init((char *)can_interface.c_str(), 1, 0)) {
            RCLCPP_ERROR(this->get_logger(), "fs_ai_api_init() failed");
            return false;
        }

        // Start the loop thread
        loop_thread_running_ = true;
        loop_thread_ = std::thread(&AckermannCanNode::loopThread, this);

        return true;
    }

private:
    // Publishers and subscribers
    rclcpp::Publisher<fsai_api::msg::VCU2AI>::SharedPtr vcu2ai_pub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr ackermann_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_brake_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr brake_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr chequered_flag_sub_;

    // Global variables
    volatile fs_ai_api_ai2vcu ai2vcu_data_{};
    std::thread loop_thread_;
    std::atomic<bool> loop_thread_running_{false};
    int timing_us_;
    std::atomic<bool> drive_enabled_;
    std::atomic<bool> chequered_flag_;
    std::atomic<bool> destroy_node_;
    std::atomic<bool> braking_;

    void ackermannCmdCallback(const ackermann_msgs::msg::AckermannDrive::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Message received");
     
        if (drive_enabled_) {
            RCLCPP_INFO(this->get_logger(), "Propagating commands");

            // Convert speed from m/s to RPM (taking into account wheel radius)
            float car_speed_mps = msg->speed;
            float wheel_rpm = (car_speed_mps / WHEEL_RADIUS) * 60.0 / (2 * M_PI);

            RCLCPP_INFO(this->get_logger(), "wheel rpm %f", wheel_rpm);

            // Temporary: set finish on negative speed
            if (wheel_rpm < 0) {
                wheel_rpm = 0;
                chequered_flag_ = true;
            }

            // Convert steering angle from radians to degrees
            float steering_angle_deg = msg->steering_angle * DEGREE_CONVERSION;

            // Update the ai2vcu_data struct
            ai2vcu_data_.AI2VCU_STEER_ANGLE_REQUEST_deg = steering_angle_deg;
            if (!braking_){
                ai2vcu_data_.AI2VCU_AXLE_SPEED_REQUEST_rpm = wheel_rpm;
                ai2vcu_data_.AI2VCU_AXLE_TORQUE_REQUEST_Nm = 195;
            }
        }
        else{
                ai2vcu_data_.AI2VCU_STEER_ANGLE_REQUEST_deg = 0;
                ai2vcu_data_.AI2VCU_AXLE_SPEED_REQUEST_rpm = 0;
                ai2vcu_data_.AI2VCU_AXLE_TORQUE_REQUEST_Nm = 0;
                ai2vcu_data_.AI2VCU_BRAKE_PRESS_REQUEST_pct = 0;
        }
    }

    void brakeCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (!chequered_flag_) { // chequered flag brakes anyway
            braking_ = msg->data;
        }
    }

    void emergencyBrakeCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data == true) {
            ai2vcu_data_.AI2VCU_ESTOP_REQUEST = ESTOP_YES;
            RCLCPP_WARN(this->get_logger(), "Emergency brake triggered!");
        } else {
            ai2vcu_data_.AI2VCU_ESTOP_REQUEST = ESTOP_NO;
            RCLCPP_INFO(this->get_logger(), "Emergency brake released.");
        }
    }

    void chequeredFlagCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data == true) {
            chequered_flag_ = true;
        }
    }

    void loopThread()
    {
        fs_ai_api_vcu2ai vcu2ai_data;

        while (rclcpp::ok() && loop_thread_running_) {
            // Get data from VCU
            fs_ai_api_vcu2ai_get_data(&vcu2ai_data);

            // Handshake logic
            if (vcu2ai_data.VCU2AI_HANDSHAKE_RECEIVE_BIT == HANDSHAKE_RECEIVE_BIT_OFF) {
                ai2vcu_data_.AI2VCU_HANDSHAKE_SEND_BIT = HANDSHAKE_SEND_BIT_OFF;
            } else if (vcu2ai_data.VCU2AI_HANDSHAKE_RECEIVE_BIT == HANDSHAKE_RECEIVE_BIT_ON) {
                ai2vcu_data_.AI2VCU_HANDSHAKE_SEND_BIT = HANDSHAKE_SEND_BIT_ON;
            } else {
                RCLCPP_ERROR(this->get_logger(), "HANDSHAKE_BIT error");
            }

            // Check for mission mode selection
            if (vcu2ai_data.VCU2AI_AMI_STATE != AMI_NOT_SELECTED && vcu2ai_data.VCU2AI_AS_STATE == AS_OFF) {
                ai2vcu_data_.AI2VCU_MISSION_STATUS = MISSION_SELECTED;
                RCLCPP_INFO(this->get_logger(), "Mission Selected");
            }

            // Check if AS_STATE is ready (2)
            if (vcu2ai_data.VCU2AI_AS_STATE == AS_READY) {
                ai2vcu_data_.AI2VCU_DIRECTION_REQUEST = DIRECTION_NEUTRAL;
                ai2vcu_data_.AI2VCU_MISSION_STATUS = MISSION_SELECTED;
                ai2vcu_data_.AI2VCU_AXLE_TORQUE_REQUEST_Nm = 0;
                ai2vcu_data_.AI2VCU_STEER_ANGLE_REQUEST_deg = 0;
                RCLCPP_INFO(this->get_logger(), "AS Ready");
            }

            // Goes AS_Driving
            if (vcu2ai_data.VCU2AI_AS_STATE == AS_DRIVING) {
                ai2vcu_data_.AI2VCU_DIRECTION_REQUEST = DIRECTION_FORWARD;
                ai2vcu_data_.AI2VCU_MISSION_STATUS = MISSION_RUNNING;
                drive_enabled_ = true;
                RCLCPP_INFO(this->get_logger(), "AS Driving");
            }

            if (chequered_flag_) {
                if (vcu2ai_data.VCU2AI_FL_WHEEL_SPEED_rpm > 5 
                    || vcu2ai_data.VCU2AI_FR_WHEEL_SPEED_rpm > 5
                    || vcu2ai_data.VCU2AI_RL_WHEEL_SPEED_rpm > 5
                    || vcu2ai_data.VCU2AI_RR_WHEEL_SPEED_rpm > 5)
                    {
                        RCLCPP_INFO(this->get_logger(), "Mission finished, braking...");
                        braking_ = true;
                    }
                else{
                    RCLCPP_INFO(this->get_logger(), "Finished");
                    ai2vcu_data_.AI2VCU_MISSION_STATUS = MISSION_FINISHED;
                    braking_ = false;
                }
            }

            if (braking_){
                ai2vcu_data_.AI2VCU_AXLE_SPEED_REQUEST_rpm = 0;
                ai2vcu_data_.AI2VCU_AXLE_TORQUE_REQUEST_Nm = 0;
                ai2vcu_data_.AI2VCU_BRAKE_PRESS_REQUEST_pct = 50;
            }
            else{
                ai2vcu_data_.AI2VCU_BRAKE_PRESS_REQUEST_pct = 0;
            }

            // reset state in case we don't power cycle the PC before starting a new mission
            if (vcu2ai_data.VCU2AI_AS_STATE == AS_FINISHED || vcu2ai_data.VCU2AI_AS_STATE == AS_EMERGENCY_BRAKE){
                chequered_flag_ = false;
                drive_enabled_ = false;
                braking_ = false;
                ai2vcu_data_.AI2VCU_MISSION_STATUS = MISSION_NOT_SELECTED;
                ai2vcu_data_.AI2VCU_DIRECTION_REQUEST = DIRECTION_NEUTRAL;
                ai2vcu_data_.AI2VCU_ESTOP_REQUEST = ESTOP_NO;
                ai2vcu_data_.AI2VCU_STEER_ANGLE_REQUEST_deg = 0;
                ai2vcu_data_.AI2VCU_AXLE_SPEED_REQUEST_rpm = 0;
                ai2vcu_data_.AI2VCU_AXLE_TORQUE_REQUEST_Nm = 0;
                ai2vcu_data_.AI2VCU_BRAKE_PRESS_REQUEST_pct = 0;
            }

            // Send data to VCU
            fs_ai_api_ai2vcu_set_data(&ai2vcu_data_);

            // Publish VCU data
            auto msg = fsai_api::msg::VCU2AI();
            msg.handshake_receive_bit = vcu2ai_data.VCU2AI_HANDSHAKE_RECEIVE_BIT;
            msg.res_go_signal = vcu2ai_data.VCU2AI_RES_GO_SIGNAL;
            msg.as_state = vcu2ai_data.VCU2AI_AS_STATE;
            msg.ami_state = vcu2ai_data.VCU2AI_AMI_STATE;
            msg.steer_angle_deg = vcu2ai_data.VCU2AI_STEER_ANGLE_deg;
            msg.brake_press_f_pct = vcu2ai_data.VCU2AI_BRAKE_PRESS_F_pct;
            msg.brake_press_r_pct = vcu2ai_data.VCU2AI_BRAKE_PRESS_R_pct;
            msg.fl_wheel_speed_rpm = vcu2ai_data.VCU2AI_FL_WHEEL_SPEED_rpm;
            msg.fr_wheel_speed_rpm = vcu2ai_data.VCU2AI_FR_WHEEL_SPEED_rpm;
            msg.rl_wheel_speed_rpm = vcu2ai_data.VCU2AI_RL_WHEEL_SPEED_rpm;
            msg.rr_wheel_speed_rpm = vcu2ai_data.VCU2AI_RR_WHEEL_SPEED_rpm;
            msg.fl_pulse_count = vcu2ai_data.VCU2AI_FL_PULSE_COUNT;
            msg.fr_pulse_count = vcu2ai_data.VCU2AI_FR_PULSE_COUNT;
            msg.rl_pulse_count = vcu2ai_data.VCU2AI_RL_PULSE_COUNT;
            msg.rr_pulse_count = vcu2ai_data.VCU2AI_RR_PULSE_COUNT;

            vcu2ai_pub_->publish(msg);

            // Loop timing
            usleep(timing_us_);
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    if (argc < 2) {
        RCLCPP_ERROR(rclcpp::get_logger("ackermann_can"), "Usage: ros2 run fsai_api ackermann_can <can_interface>");
        return 1;
    }

    auto node = std::make_shared<AckermannCanNode>();

    if (!node->initializeCanInterface(argv[1])) {
        return 1;
    }

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}