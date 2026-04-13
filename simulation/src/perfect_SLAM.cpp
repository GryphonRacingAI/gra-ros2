#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/header.hpp>
#include <common_msgs/msg/logical_camera_image.hpp>
#include <common_msgs/msg/cone.hpp>
#include <common_msgs/msg/cone_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <unordered_set>
#include <vector>

class PerfectSLAMNode : public rclcpp::Node {
public:
  PerfectSLAMNode() : Node("perfect_SLAM"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
    pub_ = this->create_publisher<common_msgs::msg::ConeArray>("/perfect_cone_map", 10);
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/perfect_odom", 10);
    odom_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),  // 20 Hz publishing rate
      std::bind(&PerfectSLAMNode::publish_perfect_odometry, this)
    );
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/perfect_cone_map_markers", 1);
    sub_ = this->create_subscription<common_msgs::msg::LogicalCameraImage>(
      "/logical_camera", 10, std::bind(&PerfectSLAMNode::callback, this, std::placeholders::_1));
  }

private:
  struct ConeSeen {
    std::string id;
    geometry_msgs::msg::Point position;
    uint8_t type;
  };

  bool is_new_cone(const std::string &id) {
    return seen_cone_ids_.find(id) == seen_cone_ids_.end();
  }

  void callback(const common_msgs::msg::LogicalCameraImage::SharedPtr msg) {
    last_header_ = msg->header;
    for (const auto &model : msg->model) {
      std::string id = model.name;
      uint8_t type = common_msgs::msg::Cone::UNKNOWN;
      bool is_cone = false;
      if (id.find("yellow") != std::string::npos) {
        type = common_msgs::msg::Cone::YELLOW;
        is_cone = true;
      } else if (id.find("blue") != std::string::npos) {
        type = common_msgs::msg::Cone::BLUE;
        is_cone = true;
      } else if (id.find("large_orange") != std::string::npos) {
        type = common_msgs::msg::Cone::LARGE_ORANGE;
        is_cone = true;
      } else if (id.find("orange") != std::string::npos) {
        type = common_msgs::msg::Cone::ORANGE;
        is_cone = true;
      }
      if (!is_cone) continue;
      if (is_new_cone(id)) {
        geometry_msgs::msg::TransformStamped tf_map_to_cone;
        try {
          tf_map_to_cone = tf_buffer_.lookupTransform("map","cones/" + id, msg->header.stamp, rclcpp::Duration::from_seconds(0.1));
        } catch (const tf2::TransformException &ex) {
          RCLCPP_WARN(this->get_logger(), "Could not transform map to %s: %s", id.c_str(), ex.what());
          continue;
        }
        geometry_msgs::msg::Point pos;
        pos.x = tf_map_to_cone.transform.translation.x;
        pos.y = tf_map_to_cone.transform.translation.y;
        pos.z = tf_map_to_cone.transform.translation.z;
        seen_cones_.push_back({id, pos, type});
        seen_cone_ids_.insert(id);
      }
    }
    publish_map();
  }

  void publish_map() {
    common_msgs::msg::ConeArray cone_array;
    cone_array.header = last_header_;
    cone_array.header.frame_id = "odom";
    cone_array.unknown_cones.clear();
    cone_array.yellow_cones.clear();
    cone_array.blue_cones.clear();
    cone_array.orange_cones.clear();
    cone_array.large_orange_cones.clear();
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker delete_all_marker;
    delete_all_marker.header = cone_array.header;
    delete_all_marker.ns = "perfect_cone_map";
    delete_all_marker.id = 0;
    delete_all_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_all_marker);
    int marker_id = 0;
    for (const auto &c : seen_cones_) {
      common_msgs::msg::Cone cone;
      cone.position = c.position;
      cone.type = c.type;
      switch (c.type) {
        case common_msgs::msg::Cone::YELLOW:
          cone_array.yellow_cones.push_back(cone); break;
        case common_msgs::msg::Cone::BLUE:
          cone_array.blue_cones.push_back(cone); break;
        case common_msgs::msg::Cone::LARGE_ORANGE:
          cone_array.large_orange_cones.push_back(cone); break;
        case common_msgs::msg::Cone::ORANGE:
          cone_array.orange_cones.push_back(cone); break;
        default:
          break;
      }
      visualization_msgs::msg::Marker marker;
      marker.header = cone_array.header;
      marker.ns = "perfect_cone_map";
      marker.id = marker_id++;
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position = c.position;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.2;
      marker.scale.y = 0.2;
      marker.scale.z = 0.5;
      marker.color.a = 0.8f;
      switch (c.type) {
        case common_msgs::msg::Cone::YELLOW:
          marker.color.r = 1.0f; marker.color.g = 1.0f; marker.color.b = 0.0f; break;
        case common_msgs::msg::Cone::BLUE:
          marker.color.r = 0.0f; marker.color.g = 0.0f; marker.color.b = 1.0f; break;
        case common_msgs::msg::Cone::LARGE_ORANGE:
          marker.color.r = 1.0f; marker.color.g = 0.3f; marker.color.b = 0.0f; break;
        case common_msgs::msg::Cone::ORANGE:
          marker.color.r = 1.0f; marker.color.g = 0.5f; marker.color.b = 0.0f; break;
        default:
          marker.color.r = 0.5f; marker.color.g = 0.5f; marker.color.b = 0.5f; break;
      }
      marker_array.markers.push_back(marker);
    }
    pub_->publish(cone_array);
    marker_pub_->publish(marker_array);
  }

  void publish_perfect_odometry() {
    geometry_msgs::msg::TransformStamped current_transform;
    rclcpp::Time current_time = this->get_clock()->now();

    try {
      current_transform = tf_buffer_.lookupTransform("map", "base_link", current_time, rclcpp::Duration::from_seconds(0.05));
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "TF lookup failed: %s", ex.what());
      return;
    }

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = current_transform.header.stamp;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    // Set pose
    odom_msg.pose.pose.position.x = current_transform.transform.translation.x;
    odom_msg.pose.pose.position.y = current_transform.transform.translation.y;
    odom_msg.pose.pose.position.z = current_transform.transform.translation.z;
    odom_msg.pose.pose.orientation = current_transform.transform.rotation;

    if (has_last_transform_) {
      double dt = (current_time - last_time_).seconds();

      // Handle invalid or too small dt
      if (dt <= 0.0 || dt < 1e-4) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "Skipping odom update due to small or invalid dt: %.8f", dt);
      } else {
        // Linear velocity
        odom_msg.twist.twist.linear.x =
            (current_transform.transform.translation.x - last_transform_.transform.translation.x) / dt;
        odom_msg.twist.twist.linear.y =
            (current_transform.transform.translation.y - last_transform_.transform.translation.y) / dt;
        odom_msg.twist.twist.linear.z =
            (current_transform.transform.translation.z - last_transform_.transform.translation.z) / dt;

        // Angular velocity (yaw diff over time)
        tf2::Quaternion q1, q2;
        tf2::fromMsg(last_transform_.transform.rotation, q1);
        tf2::fromMsg(current_transform.transform.rotation, q2);
        tf2::Quaternion q_delta = q2 * q1.inverse();
        tf2::Matrix3x3 m(q_delta);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        odom_msg.twist.twist.angular.x = roll / dt;
        odom_msg.twist.twist.angular.y = pitch / dt;
        odom_msg.twist.twist.angular.z = yaw / dt;
      }
    }

    // Save for next time
    last_transform_ = current_transform;
    last_time_ = current_time;
    has_last_transform_ = true;

    odom_pub_->publish(odom_msg);
  }

  std::vector<ConeSeen> seen_cones_;
  std::unordered_set<std::string> seen_cone_ids_;
  std_msgs::msg::Header last_header_;
  rclcpp::Publisher<common_msgs::msg::ConeArray>::SharedPtr pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr odom_timer_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<common_msgs::msg::LogicalCameraImage>::SharedPtr sub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  geometry_msgs::msg::TransformStamped last_transform_;
  rclcpp::Time last_time_;
  bool has_last_transform_ = false;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PerfectSLAMNode>());
  rclcpp::shutdown();
  return 0;
}