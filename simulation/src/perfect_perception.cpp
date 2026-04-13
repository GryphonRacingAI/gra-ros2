#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/header.hpp>
#include <common_msgs/msg/logical_camera_image.hpp>
#include <common_msgs/msg/logical_camera_image_model.hpp>
#include <common_msgs/msg/cone.hpp>
#include <common_msgs/msg/cone_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using std::placeholders::_1;

class PerfectPerceptionNode : public rclcpp::Node {
public:
  PerfectPerceptionNode() : Node("perfect_perception"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
    pub_ = this->create_publisher<common_msgs::msg::ConeArray>(
      "/perfect_cone_array", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/perfect_cone_array_markers", 1);
    sub_ = this->create_subscription<common_msgs::msg::LogicalCameraImage>(
      "/logical_camera", 10,
      std::bind(&PerfectPerceptionNode::callback, this, _1));
  }

private:
  void callback(const common_msgs::msg::LogicalCameraImage::SharedPtr msg) {
    last_msg_ = msg;
    publish_latest();
  }

  void publish_latest() {
    if (!last_msg_) return;
    const auto &msg = last_msg_;
    common_msgs::msg::ConeArray cone_array;
    cone_array.header = msg->header;
    cone_array.header.frame_id = "map";
    cone_array.unknown_cones.clear();
    cone_array.yellow_cones.clear();
    cone_array.blue_cones.clear();
    cone_array.orange_cones.clear();
    cone_array.large_orange_cones.clear();
    visualization_msgs::msg::MarkerArray marker_array;
    // Clear all previous markers before publishing new ones
    visualization_msgs::msg::Marker delete_all_marker;
    delete_all_marker.header = cone_array.header;
    delete_all_marker.ns = "perfect_cone_array";
    delete_all_marker.id = 0;
    delete_all_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_all_marker);
    int marker_id = 0;
    geometry_msgs::msg::TransformStamped tf_cam_to_map;
    try {
      tf_cam_to_map = tf_buffer_.lookupTransform(
        "map", msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(0.1));
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Could not transform %s to map: %s", msg->header.frame_id.c_str(), ex.what());
      return;
    }
    for (const auto &model : msg->model) {
      common_msgs::msg::Cone cone;
      geometry_msgs::msg::PointStamped pt_in, pt_out;
      pt_in.header = msg->header;
      pt_in.point = model.pose.position;
      tf2::doTransform(pt_in, pt_out, tf_cam_to_map);
      cone.position = pt_out.point;
      std::string name = model.name;
      uint8_t type = common_msgs::msg::Cone::UNKNOWN;
      bool is_cone = false;
      if (name.find("yellow") != std::string::npos) {
        type = common_msgs::msg::Cone::YELLOW;
        cone_array.yellow_cones.push_back(cone);
        is_cone = true;
      } else if (name.find("blue") != std::string::npos) {
        type = common_msgs::msg::Cone::BLUE;
        cone_array.blue_cones.push_back(cone);
        is_cone = true;
      } else if (name.find("large_orange") != std::string::npos) {
        type = common_msgs::msg::Cone::LARGE_ORANGE;
        cone_array.large_orange_cones.push_back(cone);
        is_cone = true;
      } else if (name.find("orange") != std::string::npos) {
        type = common_msgs::msg::Cone::ORANGE;
        cone_array.orange_cones.push_back(cone);
        is_cone = true;
      }
      if (!is_cone) continue;
      cone.type = type;
      // Visualization marker
      visualization_msgs::msg::Marker marker;
      marker.header = cone_array.header;
      marker.ns = "perfect_cone_array";
      marker.id = marker_id++;
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position = cone.position;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.2;
      marker.scale.y = 0.2;
      marker.scale.z = 0.5;
      marker.color.a = 0.8f;
      switch (type) {
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
    // Delete old markers if needed
    for (int id = marker_id; id < last_marker_count_; ++id) {
      visualization_msgs::msg::Marker del_marker;
      del_marker.header = cone_array.header;
      del_marker.ns = "perfect_cone_array";
      del_marker.id = id;
      del_marker.action = visualization_msgs::msg::Marker::DELETE;
      marker_array.markers.push_back(del_marker);
    }
    last_marker_count_ = marker_id;
    pub_->publish(cone_array);
    marker_pub_->publish(marker_array);
  }

  rclcpp::Publisher<common_msgs::msg::ConeArray>::SharedPtr pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<common_msgs::msg::LogicalCameraImage>::SharedPtr sub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  common_msgs::msg::LogicalCameraImage::SharedPtr last_msg_;
  int last_marker_count_ = 0;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PerfectPerceptionNode>());
  rclcpp::shutdown();
  return 0;
}
