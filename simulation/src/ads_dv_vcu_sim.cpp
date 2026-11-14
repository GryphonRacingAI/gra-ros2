// ads_dv_vcu_sim: stands in for the ADS-DV vehicle control unit (VCU) + drivetrain in Gazebo.
//
// It takes the same inputs as fsai_api/ackermann_can on the real car (/ackermann_cmd, /brake,
// /emergency_brake, /chequered_flag) and turns them into what the real car does physically:
//   - steering: requested angle, clamped and rate limited (actuator), sent to AckermannSteering
//   - drive: 4WD, one motor per axle (3.5:1). Each axle runs a speed loop on the requested wheel
//     speed, with |torque| limited by the request (195 Nm) and the VCU's speed-dependent cap
//     (fs-ai_api.c). Equal torque to both wheels of an axle (open differential).
//   - brakes: /brake and the chequered flag -> 50 % pressure (as ackermann_can does), emergency
//     brake or lost commands -> 100 %. Braking overrides torque, as in fs-ai_api.c.
//   - resistance: rolling resistance and aerodynamic drag.
// Wheel torques go to Gazebo's ApplyJointForce plugins, so tyre grip, mass and inertia are
// Gazebo's (car: ~300 kg, tyres mu 1.4). Feedback is published on /vcu2ai like the real car.
//
// Numbers from the team's code: wheel radius, 3.5 ratio, 4000 motor rpm, 195 Nm request,
// torque caps, 50 % brake. Estimates (tune against the real car): speed loop gains, brake torque,
// steering rate, rolling resistance, CdA, regenerative braking.

#include <algorithm>
#include <array>
#include <cmath>
#include <string>

#include <ackermann_msgs/msg/ackermann_drive.hpp>
#include <fsai_api/msg/vcu2_ai.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>

using std::placeholders::_1;

class AdsDvVcuSim : public rclcpp::Node {
public:
  AdsDvVcuSim() : Node("ads_dv_vcu_sim") {
    wheel_radius_ = declare_parameter("wheel_radius", 0.2575);
    motor_ratio_ = declare_parameter("motor_ratio", 3.5);
    motor_max_rpm_ = declare_parameter("motor_max_rpm", 4000.0);
    axle_torque_request_ = declare_parameter("axle_torque_request_nm", 195.0);
    regen_ = declare_parameter("regen", true);  // motor may brake to hold the speed request
    kp_ = declare_parameter("speed_kp", 60.0);   // axle Nm per rad/s of wheel speed error
    ki_ = declare_parameter("speed_ki", 120.0);
    brake_torque_max_ = declare_parameter("brake_torque_max_nm", 230.0);  // per wheel at 100 %
    brake_pct_ = declare_parameter("brake_pct", 50.0);                    // as ackermann_can
    steer_max_ = declare_parameter("steer_max", 0.424250477138);          // = sdf joint limit
    steer_rate_ = declare_parameter("steer_rate", 1.0);                   // rad/s at the wheels
    mass_ = declare_parameter("mass", 300.0);
    crr_ = declare_parameter("rolling_resistance", 0.015);
    cda_ = declare_parameter("drag_area", 1.0);  // Cd * frontal area, m^2
    cmd_timeout_ = declare_parameter("cmd_timeout", 1.0);
    const double rate = declare_parameter("rate", 100.0);  // Hz of sim time
    ami_state_ = static_cast<uint8_t>(declare_parameter("ami_state", 0));

    steer_pub_ = create_publisher<std_msgs::msg::Float64>("/steer_angle_cmd", 10);
    steer_angle_pub_ = create_publisher<std_msgs::msg::Float64>("/steer_angle", 10);
    vcu_pub_ = create_publisher<fsai_api::msg::VCU2AI>("/vcu2ai", 10);
    for (size_t i = 0; i < kWheels.size(); ++i) {
      force_pub_[i] = create_publisher<std_msgs::msg::Float64>(
          "/model/ads_dv/joint/" + kWheels[i] + "/cmd_force", 10);
    }

    create_sub<ackermann_msgs::msg::AckermannDrive>("/ackermann_cmd", [this](const auto & m) {
      cmd_ = m;
      have_cmd_ = true;
      last_cmd_ = now();
    });
    create_sub<std_msgs::msg::Bool>("/brake", [this](const auto & m) {
      if (!chequered_) braking_ = m.data;  // chequered flag brakes anyway (ackermann_can)
    });
    create_sub<std_msgs::msg::Bool>("/chequered_flag", [this](const auto & m) {
      if (m.data && !chequered_) {
        chequered_ = true;
        RCLCPP_WARN(get_logger(), "chequered flag: braking to a stop");
      }
    });
    create_sub<std_msgs::msg::Bool>("/emergency_brake", [this](const auto & m) {
      if (m.data && !estop_) RCLCPP_ERROR(get_logger(), "EMERGENCY BRAKE requested");
      estop_ = estop_ || m.data;  // the EBS latches
    });
    js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::SensorDataQoS(), std::bind(&AdsDvVcuSim::joint_cb, this, _1));

    // Sim-time timer: on a wall timer the message load (4 torques + steering through the bridge)
    // grew as the sim slowed down, which slowed it down further.
    timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(1.0 / rate), [this] { step(); });
    RCLCPP_INFO(get_logger(), "ADS-DV VCU emulator: 4WD torque drive, %.0f Nm/axle, steer rate %.2f rad/s",
                axle_torque_request_, steer_rate_);
  }

private:
  const std::array<std::string, 4> kWheels{"front_left_wheel_joint", "front_right_wheel_joint",
                                           "rear_left_wheel_joint", "rear_right_wheel_joint"};

  template <typename T, typename F>
  void create_sub(const std::string & topic, F f) {
    subs_.push_back(create_subscription<T>(topic, 10, [f](const typename T::SharedPtr m) { f(*m); }));
  }

  void joint_cb(const sensor_msgs::msg::JointState::SharedPtr msg) {
    double left = 0, right = 0;
    for (size_t i = 0; i < msg->name.size(); ++i) {
      for (size_t w = 0; w < kWheels.size(); ++w) {
        if (msg->name[i] == kWheels[w] && i < msg->velocity.size()) omega_[w] = msg->velocity[i];
      }
      if (msg->name[i] == "front_left_wheel_steering_joint" && i < msg->position.size()) left = msg->position[i];
      if (msg->name[i] == "front_right_wheel_steering_joint" && i < msg->position.size()) right = msg->position[i];
    }
    steer_actual_ = 0.5 * (left + right);
    have_js_ = true;
  }

  // VCU torque cap per axle from the fastest wheel speed (fs-ai_api.c, constant-power limit)
  static double torque_cap(double fastest_rpm) {
    if (fastest_rpm > 700) return 50.0;
    if (fastest_rpm > 600) return 85.0;
    if (fastest_rpm > 500) return 100.0;
    if (fastest_rpm > 400) return 120.0;
    if (fastest_rpm > 300) return 150.0;
    return 1e9;
  }

  void step() {
    if (!have_js_) return;
    const rclcpp::Time t = now();
    const double dt = last_step_.nanoseconds() > 0 ? std::clamp((t - last_step_).seconds(), 0.0, 0.05) : 0.0;
    last_step_ = t;

    // lost commands while driving -> EBS, as the real VCU does when the AI stops talking
    if (have_cmd_ && !estop_ && (t - last_cmd_).seconds() > cmd_timeout_ && !finished_) {
      estop_ = true;
      RCLCPP_ERROR(get_logger(), "no /ackermann_cmd for %.1f s: emergency brake", cmd_timeout_);
    }

    double speed_req = have_cmd_ ? cmd_.speed : 0.0;
    if (speed_req < 0.0 && !chequered_) {  // ackermann_can: "Temporary: set finish on negative speed"
      chequered_ = true;
      RCLCPP_WARN(get_logger(), "negative speed request: the real car treats this as the chequered flag");
    }

    std::array<double, 4> rpm{};
    double fastest = 0.0, mean_omega = 0.0;
    for (size_t w = 0; w < 4; ++w) {
      rpm[w] = omega_[w] * 60.0 / (2.0 * M_PI);
      fastest = std::max(fastest, std::fabs(rpm[w]));
      mean_omega += omega_[w] / 4.0;
    }
    if (chequered_ && !finished_ && fastest < 5.0) {
      finished_ = true;
      RCLCPP_INFO(get_logger(), "mission finished: car stopped");
    }

    double brake_pct = 0.0;
    if (estop_) brake_pct = 100.0;
    else if (braking_ || chequered_) brake_pct = brake_pct_;
    const bool drive = have_cmd_ && brake_pct == 0.0;

    // ---- drive: per-axle speed loop, |torque| <= min(request, VCU cap)
    const double max_wheel_omega = motor_max_rpm_ / motor_ratio_ * 2.0 * M_PI / 60.0;
    const double omega_req = std::clamp(std::max(speed_req, 0.0) / wheel_radius_, 0.0, max_wheel_omega);
    const double cap = std::min(axle_torque_request_, torque_cap(fastest));
    std::array<double, 4> torque{};
    for (int axle = 0; axle < 2; ++axle) {
      double tq = 0.0;
      if (drive) {
        const double err = omega_req - 0.5 * (omega_[2 * axle] + omega_[2 * axle + 1]);
        integ_[axle] = std::clamp(integ_[axle] + ki_ * err * dt, regen_ ? -cap : 0.0, cap);
        tq = std::clamp(kp_ * err + integ_[axle], regen_ ? -cap : 0.0, cap);
      } else {
        integ_[axle] = 0.0;
      }
      torque[2 * axle] = torque[2 * axle + 1] = 0.5 * tq;
    }

    // ---- brakes (smoothed sign so a stopped wheel doesn't chatter) and resistance
    const double v = mean_omega * wheel_radius_;
    const double f_res = crr_ * mass_ * 9.81 * std::tanh(v / 0.2) + 0.5 * 1.225 * cda_ * v * std::fabs(v);
    for (size_t w = 0; w < 4; ++w) {
      const double brake = brake_pct / 100.0 * brake_torque_max_ * std::clamp(omega_[w] / 0.5, -1.0, 1.0);
      std_msgs::msg::Float64 f;
      f.data = torque[w] - brake - f_res * wheel_radius_ / 4.0;
      force_pub_[w]->publish(f);
    }

    // ---- steering actuator: clamp + rate limit
    const double steer_target = (have_cmd_ && !estop_) ? std::clamp(static_cast<double>(cmd_.steering_angle), -steer_max_, steer_max_)
                                                       : steer_cmd_;
    steer_cmd_ += std::clamp(steer_target - steer_cmd_, -steer_rate_ * dt, steer_rate_ * dt);
    std_msgs::msg::Float64 s;
    s.data = steer_cmd_;
    steer_pub_->publish(s);
    s.data = steer_actual_;
    steer_angle_pub_->publish(s);

    if (++tick_ % 2 == 0) publish_vcu2ai(rpm, brake_pct);  // 50 Hz
  }

  void publish_vcu2ai(const std::array<double, 4> & rpm, double brake_pct) {
    fsai_api::msg::VCU2AI m;
    m.handshake_receive_bit = true;
    m.res_go_signal = true;
    m.as_state = estop_ ? fsai_api::msg::VCU2AI::AS_EMERGENCY_BRAKE
               : finished_ ? fsai_api::msg::VCU2AI::AS_FINISHED
               : fsai_api::msg::VCU2AI::AS_DRIVING;
    m.ami_state = ami_state_;
    m.steer_angle_deg = static_cast<float>(steer_actual_ * 180.0 / M_PI);
    m.brake_press_f_pct = m.brake_press_r_pct = static_cast<float>(brake_pct);
    m.fl_wheel_speed_rpm = static_cast<float>(rpm[0]);
    m.fr_wheel_speed_rpm = static_cast<float>(rpm[1]);
    m.rl_wheel_speed_rpm = static_cast<float>(rpm[2]);
    m.rr_wheel_speed_rpm = static_cast<float>(rpm[3]);
    vcu_pub_->publish(m);
  }

  double wheel_radius_, motor_ratio_, motor_max_rpm_, axle_torque_request_, kp_, ki_, brake_torque_max_,
      brake_pct_, steer_max_, steer_rate_, mass_, crr_, cda_, cmd_timeout_;
  bool regen_;
  uint8_t ami_state_;

  ackermann_msgs::msg::AckermannDrive cmd_;
  bool have_cmd_{false}, have_js_{false}, braking_{false}, chequered_{false}, estop_{false}, finished_{false};
  rclcpp::Time last_cmd_{0, 0, RCL_ROS_TIME}, last_step_{0, 0, RCL_ROS_TIME};
  std::array<double, 4> omega_{};
  std::array<double, 2> integ_{};
  double steer_cmd_{0.0}, steer_actual_{0.0};
  uint64_t tick_{0};

  std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, 4> force_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steer_pub_, steer_angle_pub_;
  rclcpp::Publisher<fsai_api::msg::VCU2AI>::SharedPtr vcu_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AdsDvVcuSim>());
  rclcpp::shutdown();
  return 0;
}
