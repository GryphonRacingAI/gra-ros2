// This node subscribes to 'ackermann_cmd' (ackermann_msgs/msg/AckermannDrive) and publishes:
// - 'speed_cmd' (std_msgs/msg/Float64): vehicle speed command (angular speed for wheel joint controller)
// - 'steer_angle_cmd' (std_msgs/msg/Float64): bicycle-model steering angle command
//
// It also subscribes to 'joint_states' and reads the front left / front right steering joint angles
// to compute the (approximate) bicycle-model steering angle

#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>

class AckermannToSpeedSteerNode : public rclcpp::Node {
public:
    AckermannToSpeedSteerNode() : Node("ackermann_to_speed_steer") {
        // Topic names as parameters
        speed_cmd_topic_ = this->declare_parameter<std::string>("speed_cmd_topic", "speed_cmd");
        steer_cmd_topic_ = this->declare_parameter<std::string>("steer_cmd_topic", "steer_angle_cmd");
        steer_angle_topic_ = this->declare_parameter<std::string>("steer_angle_topic", "steer_angle");
        ackermann_cmd_topic_ = this->declare_parameter<std::string>("ackermann_cmd_topic", "ackermann_cmd");
        joint_states_topic_ = this->declare_parameter<std::string>("joint_states_topic", "joint_states");

        // Publishers for commands
        speed_cmd_publisher_ = this->create_publisher<std_msgs::msg::Float64>(speed_cmd_topic_, 10);
        steer_cmd_publisher_ = this->create_publisher<std_msgs::msg::Float64>(steer_cmd_topic_, 10);
        // Publisher for current steering angle
        steer_angle_publisher_ = this->create_publisher<std_msgs::msg::Float64>(steer_angle_topic_, 10);

        ackermann_subscription_ = this->create_subscription<ackermann_msgs::msg::AckermannDrive>(
            ackermann_cmd_topic_, 10,
            std::bind(&AckermannToSpeedSteerNode::ackermannCallback, this, std::placeholders::_1));

        joint_state_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            joint_states_topic_, 10,
            std::bind(&AckermannToSpeedSteerNode::jointStateCallback, this, std::placeholders::_1));
    }

private:
    std::string speed_cmd_topic_;
    std::string steer_cmd_topic_;
    std::string steer_angle_topic_;
    std::string ackermann_cmd_topic_;
    std::string joint_states_topic_;

    std_msgs::msg::Float64 speed_cmd_msg_;
    std_msgs::msg::Float64 steer_cmd_msg_;
    std_msgs::msg::Float64 steer_angle_msg_;
    
    double left_angle_{0.0};
    double right_angle_{0.0};
    bool found_left_{false};
    bool found_right_{false};
    double tanL_{0.0};
    double tanR_{0.0};
    double delta_b_{0.0};
    const double eps_{1e-6};
    const double wheel_radius_{0.2575};  // Wheel radius in meters

    void ackermannCallback(const ackermann_msgs::msg::AckermannDrive::SharedPtr msg) {
        // Publish speed command
        speed_cmd_msg_.data = msg->speed / wheel_radius_;
        speed_cmd_publisher_->publish(speed_cmd_msg_);
        
        // Publish steering angle command
        steer_cmd_msg_.data = msg->steering_angle;
        steer_cmd_publisher_->publish(steer_cmd_msg_);
    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        found_left_ = found_right_ = false;

        // Find angles by name in joint states
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == "front_left_wheel_steering_joint") {
                left_angle_ = msg->position[i];
                found_left_ = true;
            }
            if (msg->name[i] == "front_right_wheel_steering_joint") {
                right_angle_ = msg->position[i];
                found_right_ = true;
            }
        }

        // Only proceed if both angles were found
        if (!found_left_ || !found_right_) {
            RCLCPP_WARN(this->get_logger(), "Missing steering joint(s).");
            return;
        }

        // Compute bicycle-model steering angle approximation (<25deg)
        delta_b_ = 0.5 * (left_angle_ + right_angle_);

        // Output zero if computed angle is negligible
        steer_angle_msg_.data = (std::fabs(delta_b_) < eps_) ? 0.0 : delta_b_;
        steer_angle_publisher_->publish(steer_angle_msg_);
    }

    // Publishers and subscriptions
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_cmd_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steer_cmd_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steer_angle_publisher_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr ackermann_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AckermannToSpeedSteerNode>());
    rclcpp::shutdown();
    return 0;
}