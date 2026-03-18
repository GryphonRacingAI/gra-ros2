#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <rclcpp/time.hpp>
#include <cmath>

class HeadlessMonitorNode : public rclcpp::Node {
public:
  HeadlessMonitorNode() : Node("headless_monitor") {
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10,
      std::bind(&HeadlessMonitorNode::odom_callback, this, std::placeholders::_1));

    clock_sub_ = this->create_subscription<rosgraph_msgs::msg::Clock>(
      "/clock", 10,
      std::bind(&HeadlessMonitorNode::clock_callback, this, std::placeholders::_1));
  }

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;

  nav_msgs::msg::Odometry latest_odom_;      // continuously updated
  nav_msgs::msg::Odometry last_checked_odom_; // snapshot taken every 5 s
  rclcpp::Time last_check_time_{0, 0, RCL_ROS_TIME};
  bool initialized_ = false;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    latest_odom_ = *msg;
  }

  void clock_callback(const rosgraph_msgs::msg::Clock::SharedPtr msg) {
    rclcpp::Time current_time(msg->clock);

    if (!initialized_) {
      // Wait until we have received at least one odometry message
      if (rclcpp::Time(latest_odom_.header.stamp).seconds() > 0.0) {
        last_checked_odom_ = latest_odom_;
        last_check_time_ = current_time;
        initialized_ = true;
      }
      return;
    }

    // Check if 5 seconds of simulation time have passed
    if ((current_time - last_check_time_).seconds() >= 5.0) {
      if (isCarMoving()) {
        RCLCPP_INFO(this->get_logger(), "Car is moving");
      } else {
        RCLCPP_INFO(this->get_logger(), "Car is not moving, stop the simulation.");
      }

      // Snapshot the current pose for the next interval
      last_checked_odom_ = latest_odom_;
      last_check_time_ = current_time;
    }
  }

  bool isCarMoving() {
    const auto& p1 = last_checked_odom_.pose.pose.position;
    const auto& p2 = latest_odom_.pose.pose.position;

    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    double dz = p1.z - p2.z;

    // Cleaner distance calculation (std::hypot is available in C++17+)
    double distance = std::hypot(dx, dy, dz);
    return distance > 0.1;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HeadlessMonitorNode>());
  rclcpp::shutdown();
  return 0;
}