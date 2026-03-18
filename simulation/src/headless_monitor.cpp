#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>

class HeadlessMonitorNode : public rclcpp::Node {
public:
  HeadlessMonitorNode() : Node("headless_monitor") {
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10,                                           
      std::bind(&HeadlessMonitorNode::odom_callback, this, std::placeholders::_1));
    
  }

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  nav_msgs::msg::Odometry last_odom_msg_;
  nav_msgs::msg::Odometry curr_odom_msg_;
  rclcpp::TimerBase::SharedPtr timer_;

  /**
   * @brief Within last 5 seconds, check if the Pose has changed significantly
   * @return true if the car is moving, false otherwise
   */
  bool isCarMoving() {
    double x_diff = last_odom_msg_.pose.pose.position.x - curr_odom_msg_.pose.pose.position.x;
    double y_diff = last_odom_msg_.pose.pose.position.y - curr_odom_msg_.pose.pose.position.y;
    double z_diff = last_odom_msg_.pose.pose.position.z - curr_odom_msg_.pose.pose.position.z;
    
    double distance = sqrt(x_diff * x_diff + y_diff * y_diff + z_diff * z_diff);
    
    return distance > 0.1; 
  }
  
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (curr_odom_msg_.header.stamp.sec % 5 == 0) {
      last_odom_msg_ = curr_odom_msg_;
      curr_odom_msg_ = *msg;

      if (isCarMoving()) {
        RCLCPP_INFO(this->get_logger(), "Car is moving");
      } else {
        RCLCPP_INFO(this->get_logger(), "Car is not moving, stop the simulation.");
      }
      
      rclcpp::sleep_for(std::chrono::seconds(2));
      return;
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HeadlessMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
