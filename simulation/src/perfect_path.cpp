#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <limits>
#include <chrono>

class PerfectPathNode final : public rclcpp::Node {
public:
  PerfectPathNode()
  : Node("perfect_path"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    const auto pkg_share = ament_index_cpp::get_package_share_directory("simulation");
    const auto config_dir = pkg_share + "/config";
    const auto mppi_track_dir = pkg_share + "/tracks/mppi_track";

    track_ = this->declare_parameter<std::string>("track", "acceleration");
    std::string default_config;
    if (track_ == "mppi_track") {
      default_config = config_dir + "/perfect_path_mppi_track_pairs.txt";
    } else {
      default_config = config_dir + "/perfect_path_acceleration_pairs.txt";
    }

    config_file_ = this->declare_parameter<std::string>("config_file", default_config);
    inner_cones_csv_ = this->declare_parameter<std::string>(
      "inner_cones_csv", mppi_track_dir + "/inner_cones.csv");
    outer_cones_csv_ = this->declare_parameter<std::string>(
      "outer_cones_csv", mppi_track_dir + "/outer_cones.csv");
    lookahead_distance_ = this->declare_parameter<double>("lookahead_distance", 30.0);
    max_points_ = this->declare_parameter<int>("max_points", 200);
    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 20.0);

    marker_scale_ = this->declare_parameter<double>("marker_scale", 0.25);
    line_width_ = this->declare_parameter<double>("line_width", 0.15);

    use_csv_midline_ = (track_ == "mppi_track");
    const std::string path_topic = use_csv_midline_ ? "/path" : "/perfect_path";
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(path_topic, 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/perfect_path_markers", 1);

    if (use_csv_midline_) {
      load_midpoints_from_csvs();
    } else {
      odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&PerfectPathNode::odom_callback, this, std::placeholders::_1));
      load_midpoints_from_config();
    }

    const auto period = std::chrono::duration<double>(1.0 / std::max(1e-3, publish_rate_hz_));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&PerfectPathNode::publish, this));
  }

private:
  struct Pt2 {
    double x;
    double y;
  };

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    last_odom_ = *msg;
    has_odom_ = true;
  }

  static std::vector<Pt2> load_csv_points(const std::string &csv_path) {
    std::vector<Pt2> pts;
    std::ifstream in(csv_path);
    if (!in.is_open()) {
      return pts;
    }

    std::string line;
    while (std::getline(in, line)) {
      if (line.empty()) {
        continue;
      }
      const auto comma = line.find(',');
      if (comma == std::string::npos) {
        continue;
      }
      try {
        const double x = std::stod(line.substr(0, comma));
        const double y = std::stod(line.substr(comma + 1));
        pts.push_back(Pt2{x, y});
      } catch (const std::exception &) {
        continue;
      }
    }
    return pts;
  }

  void load_midpoints_from_csvs() {
    const auto inner = load_csv_points(inner_cones_csv_);
    const auto outer = load_csv_points(outer_cones_csv_);

    if (inner.empty() || outer.empty()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to load cone CSVs (inner=%zu outer=%zu): %s, %s",
        inner.size(), outer.size(),
        inner_cones_csv_.c_str(), outer_cones_csv_.c_str());
      return;
    }

    const std::size_t n = std::min(inner.size(), outer.size());
    std::vector<Pt2> mids;
    mids.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
      mids.push_back(Pt2{
        0.5 * (inner[i].x + outer[i].x),
        0.5 * (inner[i].y + outer[i].y)
      });
    }

    if (mids.size() < 2) {
      RCLCPP_ERROR(this->get_logger(), "CSV midline produced too few points: %zu", mids.size());
      return;
    }

    midpoints_map_ = std::move(mids);
    RCLCPP_INFO(
      this->get_logger(),
      "Loaded %zu midline points from %s and %s",
      midpoints_map_.size(),
      inner_cones_csv_.c_str(),
      outer_cones_csv_.c_str());
  }

  void load_midpoints_from_config() {
    std::ifstream in(config_file_);
    if (!in.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open config_file: %s", config_file_.c_str());
      return;
    }

    std::vector<Pt2> mids;
    std::string line;
    while (std::getline(in, line)) {
      if (line.empty()) continue;
      std::istringstream iss(line);
      double bx, by, yx, yy;
      if (!(iss >> bx >> by >> yx >> yy)) continue;
      mids.push_back(Pt2{0.5 * (bx + yx), 0.5 * (by + yy)});
    }

    if (mids.size() < 2) {
      RCLCPP_ERROR(this->get_logger(), "Config produced too few midpoints: %zu", mids.size());
      return;
    }

    midpoints_map_ = std::move(mids);
  }

  bool transform_midpoints_to_odom(std::vector<Pt2> &out) {
    if (midpoints_map_.empty()) return false;

    geometry_msgs::msg::TransformStamped tf_map_to_odom;
    try {
      tf_map_to_odom = tf_buffer_.lookupTransform(
        "odom", "map", rclcpp::Time(0), rclcpp::Duration::from_seconds(0.05));
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "TF lookup failed: %s", ex.what());
      return false;
    }

    out.clear();
    out.reserve(midpoints_map_.size());
    for (const auto &p : midpoints_map_) {
      geometry_msgs::msg::PointStamped pt_in;
      pt_in.header.frame_id = "map";
      pt_in.point.x = p.x;
      pt_in.point.y = p.y;
      pt_in.point.z = 0.0;
      geometry_msgs::msg::PointStamped pt_out;
      tf2::doTransform(pt_in, pt_out, tf_map_to_odom);
      out.push_back(Pt2{pt_out.point.x, pt_out.point.y});
    }
    return true;
  }

  int closest_index(const std::vector<Pt2> &pts, double x, double y) const {
    double best = std::numeric_limits<double>::infinity();
    int best_i = 0;
    for (int i = 0; i < static_cast<int>(pts.size()); ++i) {
      const double dx = pts[i].x - x;
      const double dy = pts[i].y - y;
      const double d2 = dx * dx + dy * dy;
      if (d2 < best) {
        best = d2;
        best_i = i;
      }
    }
    return best_i;
  }

  std::vector<Pt2> slice_lookahead(const std::vector<Pt2> &pts, double x, double y) const {
    if (pts.size() < 2) return {};
    const int start = closest_index(pts, x, y);
    std::vector<Pt2> out;
    out.reserve(std::min<int>(max_points_, static_cast<int>(pts.size() - start)));

    double acc = 0.0;
    Pt2 prev = pts[start];
    for (int i = start; i < static_cast<int>(pts.size()) && static_cast<int>(out.size()) < max_points_; ++i) {
      const Pt2 cur = pts[i];
      const double dx = cur.x - prev.x;
      const double dy = cur.y - prev.y;
      if (i != start) acc += std::sqrt(dx * dx + dy * dy);
      if (i != start && acc > lookahead_distance_) break;
      out.push_back(cur);
      prev = cur;
    }
    return out;
  }

  static nav_msgs::msg::Path make_path(
    const rclcpp::Time &stamp,
    const std::vector<Pt2> &pts)
  {
    nav_msgs::msg::Path path;
    path.header.stamp = stamp;
    path.header.frame_id = "odom";
    path.poses.reserve(pts.size());
    for (const auto &p : pts) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      ps.pose.position.x = p.x;
      ps.pose.position.y = p.y;
      ps.pose.position.z = 0.0;
      ps.pose.orientation.w = 1.0;
      path.poses.push_back(ps);
    }
    return path;
  }

  void publish_markers(const rclcpp::Time &stamp, const std::vector<Pt2> &all_pts, const std::vector<Pt2> &seg) {
    visualization_msgs::msg::MarkerArray arr;

    visualization_msgs::msg::Marker del;
    del.header.frame_id = "odom";
    del.header.stamp = stamp;
    del.ns = "perfect_path";
    del.id = 0;
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(del);

    visualization_msgs::msg::Marker pts;
    pts.header.frame_id = "odom";
    pts.header.stamp = stamp;
    pts.ns = "perfect_path";
    pts.id = 1;
    pts.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    pts.action = visualization_msgs::msg::Marker::ADD;
    pts.scale.x = marker_scale_;
    pts.scale.y = marker_scale_;
    pts.scale.z = marker_scale_;
    pts.color.a = 0.9f;
    pts.color.r = 0.2f;
    pts.color.g = 1.0f;
    pts.color.b = 0.2f;
    pts.points.reserve(all_pts.size());
    for (const auto &p : all_pts) {
      geometry_msgs::msg::Point gp;
      gp.x = p.x;
      gp.y = p.y;
      gp.z = 0.1;
      pts.points.push_back(gp);
    }
    arr.markers.push_back(pts);

    visualization_msgs::msg::Marker line;
    line.header.frame_id = "odom";
    line.header.stamp = stamp;
    line.ns = "perfect_path";
    line.id = 2;
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.scale.x = line_width_;
    line.color.a = 0.9f;
    line.color.r = 0.2f;
    line.color.g = 0.2f;
    line.color.b = 1.0f;
    line.points.reserve(seg.size());
    for (const auto &p : seg) {
      geometry_msgs::msg::Point gp;
      gp.x = p.x;
      gp.y = p.y;
      gp.z = 0.1;
      line.points.push_back(gp);
    }
    arr.markers.push_back(line);

    marker_pub_->publish(arr);
  }

  void publish() {
    if (midpoints_map_.empty()) return;

    std::vector<Pt2> mids_odom;
    if (!transform_midpoints_to_odom(mids_odom)) return;

    const auto stamp = this->get_clock()->now();

    if (use_csv_midline_) {
      if (mids_odom.size() < 2) return;
      path_pub_->publish(make_path(stamp, mids_odom));
      publish_markers(stamp, mids_odom, mids_odom);
      return;
    }

    if (!has_odom_) return;

    const auto &odom = last_odom_;
    const double cx = odom.pose.pose.position.x;
    const double cy = odom.pose.pose.position.y;

    const auto seg = slice_lookahead(mids_odom, cx, cy);
    if (seg.size() < 2) return;

    path_pub_->publish(make_path(stamp, seg));
    publish_markers(stamp, mids_odom, seg);
  }

  std::string track_;
  std::string config_file_;
  std::string inner_cones_csv_;
  std::string outer_cones_csv_;
  double lookahead_distance_{30.0};
  int max_points_{200};
  double publish_rate_hz_{20.0};
  double marker_scale_{0.25};
  double line_width_{0.15};
  bool use_csv_midline_{false};

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  nav_msgs::msg::Odometry last_odom_;
  bool has_odom_{false};

  std::vector<Pt2> midpoints_map_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PerfectPathNode>());
  rclcpp::shutdown();
  return 0;
}
