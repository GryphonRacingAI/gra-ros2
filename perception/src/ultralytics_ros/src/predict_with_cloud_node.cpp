/**
 * ultralytics_ros
 * Copyright (C) 2023-2024  Alpaca-zip
 * Copyright (C) 2025 A. Shehata, Leeds Gryphon Racing AI
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "predict_with_cloud_node/predict_with_cloud_node.h"
#include <unordered_map>

PredictWithCloudNode::PredictWithCloudNode(const rclcpp::NodeOptions & options)
    : rclcpp::Node("predict_with_cloud_node", options)
{
  // Declare parameters
  this->declare_parameter<std::string>("camera_info_topic", "camera_info");
  this->declare_parameter<std::string>("lidar_topic", "points_raw");
  this->declare_parameter<std::string>("yolo_result_topic", "yolo_result");
  this->declare_parameter<std::string>("yolo_3d_result_topic", "yolo_3d_result");
  this->declare_parameter<float>("cluster_tolerance", 0.1);
  this->declare_parameter<float>("voxel_leaf_size", 0.07);
  this->declare_parameter<int>("min_cluster_size", 10);
  this->declare_parameter<int>("max_cluster_size", 700);
  this->declare_parameter<float>("ransac_distance_threshold", 0.03);
  this->declare_parameter<float>("preprocessing_filter_threshold", 0.0);
  this->declare_parameter<bool>("gz_camera_convention", true);

  // Store parameters as private members
  this->get_parameter("camera_info_topic", camera_info_topic_);
  this->get_parameter("lidar_topic", lidar_topic_);
  this->get_parameter("yolo_result_topic", yolo_result_topic_);
  this->get_parameter("yolo_3d_result_topic", yolo_3d_result_topic_);
  this->get_parameter("cluster_tolerance", cluster_tolerance_);
  this->get_parameter("voxel_leaf_size", voxel_leaf_size_);
  this->get_parameter("min_cluster_size", min_cluster_size_);
  this->get_parameter("max_cluster_size", max_cluster_size_);
  this->get_parameter("ransac_distance_threshold", ransac_distance_threshold_);
  this->get_parameter("preprocessing_filter_threshold", preprocessing_filter_threshold_);
  this->get_parameter("gz_camera_convention", gz_camera_convention_);

  camera_info_sub_.subscribe(this, camera_info_topic_);
  lidar_sub_.subscribe(this, lidar_topic_);
  yolo_result_sub_.subscribe(this, yolo_result_topic_);

  // Set up synchronizer
  sync_ = std::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy>>(10);
  sync_->connectInput(camera_info_sub_, lidar_sub_, yolo_result_sub_);
  sync_->registerCallback(std::bind(&PredictWithCloudNode::syncCallback, this, std::placeholders::_1,
                                    std::placeholders::_2, std::placeholders::_3));

  detection3d_pub_ = this->create_publisher<vision_msgs::msg::Detection3DArray>(yolo_3d_result_topic_, 1);
  detection_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("detection_cloud", 1);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("detection_marker", 1);
  filtered_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_cloud", 1);
  cone_array_pub_ = this->create_publisher<common_msgs::msg::ConeArray>("cone_array", 1);

  // Initialize time and TF listener
  last_call_time_ = this->now();
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  RCLCPP_INFO(this->get_logger(), "PredictWithCloudNode started.");
}

void PredictWithCloudNode::syncCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info_msg,
                                        const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg,
                                        const ultralytics_ros::msg::YoloResult::ConstSharedPtr& yolo_result_msg)
{
  rclcpp::Time start_time = this->now();
  rclcpp::Time current_call_time = this->now();
  rclcpp::Duration callback_interval = current_call_time - last_call_time_;
  last_call_time_ = current_call_time;

  RCLCPP_DEBUG(this->get_logger(), "syncCallback triggered at time: %.2f, interval: %.2f seconds",
               current_call_time.seconds(), callback_interval.seconds());

  // Convert the input cloud to PCL format
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*cloud_msg, *cloud);

  // Remove points above the horizontal (definitely not cone points!)
  // convert to camera frame
  // using that assumption, remove all points above the up direction of the optical frame
  // (gz: +z, real: -y) or maybe a small positive threshold

  // Downsample the point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud = downsampleCloudMsg(
      std::make_shared<sensor_msgs::msg::PointCloud2>(*cloud_msg));
  RCLCPP_DEBUG(this->get_logger(), "Downsampled cloud size: %zu", downsampled_cloud->points.size());

  // Remove ground plane after downsampling && FOV clipping
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  removeGroundPlane(downsampled_cloud, filtered_cloud);
  RCLCPP_DEBUG(this->get_logger(), "Filtered cloud size after ground removal: %zu", filtered_cloud->points.size());

  // Publish the filtered cloud
  sensor_msgs::msg::PointCloud2 filtered_cloud_msg;
  pcl::toROSMsg(*filtered_cloud, filtered_cloud_msg);
  filtered_cloud_msg.header = cloud_msg->header;
  filtered_cloud_pub_->publish(filtered_cloud_msg);
  RCLCPP_DEBUG(this->get_logger(), "Published filtered cloud with %zu points", filtered_cloud->points.size());

  // Update camera model
  cam_model_.fromCameraInfo(camera_info_msg);
  RCLCPP_DEBUG(this->get_logger(), "Camera model updated.");

  // Transform point cloud in camera frame to optical frame
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud = cloud2TransformedCloud(filtered_cloud,
                                                                                  cloud_msg->header.frame_id,
                                                                                  cam_model_.tfFrame(),
                                                                                  cloud_msg->header.stamp);
  RCLCPP_DEBUG(this->get_logger(), "Transformed cloud size: %zu", transformed_cloud->points.size());

  // Project cloud to detection
  vision_msgs::msg::Detection3DArray detection3d_array_msg;
  sensor_msgs::msg::PointCloud2 detection_cloud_msg;
  
  projectCloud(transformed_cloud, yolo_result_msg, cloud_msg->header, detection3d_array_msg, detection_cloud_msg);

  RCLCPP_DEBUG(this->get_logger(), "Detection3DArray contains %zu detections", detection3d_array_msg.detections.size());

  // Create marker array for visualization
  visualization_msgs::msg::MarkerArray marker_array_msg = createMarkerArray(detection3d_array_msg, callback_interval.seconds());

  detection3d_pub_->publish(detection3d_array_msg);
  detection_cloud_pub_->publish(detection_cloud_msg);
  marker_pub_->publish(marker_array_msg);

  RCLCPP_DEBUG(this->get_logger(), "Published detections and markers.");
  rclcpp::Time end_time = this->now();
  RCLCPP_DEBUG(this->get_logger(), "Execution time for syncCallback: %.6f seconds", (end_time - start_time).seconds());
}

void PredictWithCloudNode::transformPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
                                               pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out,
                                               const Eigen::Affine3f& transform)
{
  int cloud_size = cloud_in->size();
  cloud_out->resize(cloud_size);
  RCLCPP_DEBUG(this->get_logger(), "Applying transform to %d points.", cloud_size);

  for (int i = 0; i < cloud_size; i++)
  {
    const auto& point = cloud_in->points[i];
    cloud_out->points[i].x =
        transform(0, 0) * point.x + transform(0, 1) * point.y + transform(0, 2) * point.z + transform(0, 3);
    cloud_out->points[i].y =
        transform(1, 0) * point.x + transform(1, 1) * point.y + transform(1, 2) * point.z + transform(1, 3);
    cloud_out->points[i].z =
        transform(2, 0) * point.x + transform(2, 1) * point.y + transform(2, 2) * point.z + transform(2, 3);
  }
}

void PredictWithCloudNode::projectCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                        const ultralytics_ros::msg::YoloResult::ConstSharedPtr& yolo_result_msg,
                                        const std_msgs::msg::Header& header,
                                        vision_msgs::msg::Detection3DArray& detection3d_array_msg,
                                        sensor_msgs::msg::PointCloud2& combine_detection_cloud_msg)
{
  // Map from string class_id to uint8 Cone type
  static const std::unordered_map<std::string, uint8_t> class_map = {
    {"UNKNOWN", 0},
    {"YELLOW", 1},
    {"BLUE", 2},
    {"ORANGE", 3},
    {"LARGE_ORANGE", 4}
  };
  pcl::PointCloud<pcl::PointXYZ>::Ptr combine_detection_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  detection3d_array_msg.header = header;
  detection3d_array_msg.header.stamp = yolo_result_msg->header.stamp;

  common_msgs::msg::ConeArray cone_array_msg;
  cone_array_msg.header = header;

  RCLCPP_DEBUG(this->get_logger(), "Processing %zu 2D detections for projection.", yolo_result_msg->detections.detections.size());

  for (size_t i = 0; i < yolo_result_msg->detections.detections.size(); i++)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr detection_cloud_raw(new pcl::PointCloud<pcl::PointXYZ>());

    if (yolo_result_msg->masks.empty())
    {
      processPointsWithBbox(cloud, yolo_result_msg->detections.detections[i], detection_cloud_raw);
    }
    else
    {
      processPointsWithMask(cloud, yolo_result_msg->masks[i], detection_cloud_raw);
    }
    
    RCLCPP_DEBUG(this->get_logger(), "Detection %zu raw cloud size: %zu", i, detection_cloud_raw->points.size());

    if (!detection_cloud_raw->points.empty())
    {
      // Transform the detection cloud to the header frame
      pcl::PointCloud<pcl::PointXYZ>::Ptr detection_cloud = cloud2TransformedCloud(detection_cloud_raw,
                                                                                   cam_model_.tfFrame(),
                                                                                   header.frame_id,
                                                                                   header.stamp);

      // Extract the closest cluster
      pcl::PointCloud<pcl::PointXYZ>::Ptr closest_detection_cloud = euclideanClusterExtraction(detection_cloud);
      if (closest_detection_cloud->points.empty()) {
        RCLCPP_WARN(this->get_logger(), "Detection %zu: No valid cluster, skipping.", i);
        continue;
      }
      RCLCPP_DEBUG(this->get_logger(), "Detection %zu cluster size: %zu", i, closest_detection_cloud->points.size());

      *combine_detection_cloud += *closest_detection_cloud;

      // Create the 3D bounding box
      createBoundingBox(detection3d_array_msg, closest_detection_cloud,
                        yolo_result_msg->detections.detections[i].results);

      // Add cone to ConeArray
      common_msgs::msg::Cone cone_msg;
      std::string class_id_str = yolo_result_msg->detections.detections[i].results[0].hypothesis.class_id;
      auto it = class_map.find(class_id_str);
      cone_msg.type = (it != class_map.end()) ? it->second : 0; // Default to UNKNOWN if not found
      cone_msg.position.x = detection3d_array_msg.detections.back().bbox.center.position.x;
      cone_msg.position.y = detection3d_array_msg.detections.back().bbox.center.position.y;
      cone_msg.position.z = detection3d_array_msg.detections.back().bbox.center.position.z;
      // Place cone in the correct array
      switch (cone_msg.type) {
        case 1: cone_array_msg.yellow_cones.push_back(cone_msg); break;
        case 2: cone_array_msg.blue_cones.push_back(cone_msg); break;
        case 3: cone_array_msg.orange_cones.push_back(cone_msg); break;
        case 4: cone_array_msg.large_orange_cones.push_back(cone_msg); break;
        case 0:
        default: cone_array_msg.unknown_cones.push_back(cone_msg); break;
      }
    }
    else
    {
      RCLCPP_DEBUG(this->get_logger(), "Detection %zu: No points passed the filtering.", i);
    }
  }

  pcl::toROSMsg(*combine_detection_cloud, combine_detection_cloud_msg);
  combine_detection_cloud_msg.header = header;
  RCLCPP_DEBUG(this->get_logger(), "Combined detection cloud size: %zu", combine_detection_cloud->points.size());

  // Publish the ConeArray
  cone_array_pub_->publish(cone_array_msg);
}

void PredictWithCloudNode::processPointsWithBbox(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                                 const vision_msgs::msg::Detection2D& detection2d_msg,
                                                 pcl::PointCloud<pcl::PointXYZ>::Ptr& detection_cloud_raw)
{
  for (const auto& point : cloud->points)
  {
    cv::Point3d optical_pt;
    if (gz_camera_convention_)
    {
      // Gazebo simulation convention
      // First, convert from sensor frame (with +X forward) to pseudo-optical frame
      // Optical frame (ROS): X right, Y down, Z forward.
      // Transformation: p_optical = (-point.y, -point.z, point.x)
      optical_pt = cv::Point3d(-point.y, -point.z, point.x);
    }
    else
    {
      // Real-world ROS camera standard convention
      optical_pt = cv::Point3d(point.x, point.y, point.z);
    }

    // Now use the existing projection function which expects optical frame points.
    cv::Point2d uv = cam_model_.project3dToPixel(optical_pt);

    // Log the projection for the first point (optional)
    if (&point == &cloud->points.front())
    {
      RCLCPP_DEBUG(this->get_logger(), "First sensor point: (%.2f, %.2f, %.2f) transformed to optical: (%.2f, %.2f, %.2f) projects to (%.2f, %.2f)",
                  point.x, point.y, point.z,
                  optical_pt.x, optical_pt.y, optical_pt.z,
                  uv.x, uv.y);
    }

    // Check that the original point is in front of the sensor (x > 0)
    // and that the projected pixel falls within the detection's bounding box.
    auto forward = point.z;
    if(gz_camera_convention_) {
      forward = point.x;
    }
    if (forward > 0 &&
        uv.x > 0 &&
        uv.x >= detection2d_msg.bbox.center.position.x - detection2d_msg.bbox.size_x / 2 &&
        uv.x <= detection2d_msg.bbox.center.position.x + detection2d_msg.bbox.size_x / 2 &&
        uv.y >= detection2d_msg.bbox.center.position.y - detection2d_msg.bbox.size_y / 2 &&
        uv.y <= detection2d_msg.bbox.center.position.y + detection2d_msg.bbox.size_y / 2)
    {
      detection_cloud_raw->points.push_back(point);
    }
  }
  RCLCPP_DEBUG(this->get_logger(), "processPointsWithBbox: %zu points passed bbox filter", detection_cloud_raw->points.size());
}


void PredictWithCloudNode::processPointsWithMask(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                                 const sensor_msgs::msg::Image& mask_image_msg,
                                                 pcl::PointCloud<pcl::PointXYZ>::Ptr& detection_cloud_raw)
{
  cv_bridge::CvImagePtr cv_ptr;

  try
  {
    cv_ptr = cv_bridge::toCvCopy(mask_image_msg, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "CV Bridge Exception: %s", e.what());
    return;
  }

  for (const auto& point : cloud->points)
  {
    cv::Point3d pt_cv(point.x, point.y, point.z);
    cv::Point2d uv = cam_model_.project3dToPixel(pt_cv);

    // Only consider valid uv coordinates that fall within the image dimensions
    auto forward = point.z;
    if(gz_camera_convention_) {
      forward = point.x;
    }
    if (forward > 0 && uv.x >= 0 && uv.x < cv_ptr->image.cols && uv.y >= 0 && uv.y < cv_ptr->image.rows)
    {
      if (cv_ptr->image.at<uchar>(cv::Point(uv.x, uv.y)) > 0)
      {
        detection_cloud_raw->points.push_back(point);
      }
    }
  }
  RCLCPP_DEBUG(this->get_logger(), "processPointsWithMask: %zu points passed mask filter", detection_cloud_raw->points.size());
}

void PredictWithCloudNode::createBoundingBox(
    vision_msgs::msg::Detection3DArray& detection3d_array_msg, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const std::vector<vision_msgs::msg::ObjectHypothesisWithPose>& detections_results_msg)
{
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cloud, centroid);
  double theta = -atan2(centroid[1], sqrt(pow(centroid[0], 2) + pow(centroid[2], 2)));
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));

  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  transformPointCloud(cloud, transformed_cloud, transform);

  pcl::PointXYZ min_pt, max_pt;
  pcl::getMinMax3D(*transformed_cloud, min_pt, max_pt);
  Eigen::Vector4f transformed_bbox_center =
      Eigen::Vector4f((min_pt.x + max_pt.x) / 2, (min_pt.y + max_pt.y) / 2, (min_pt.z + max_pt.z) / 2, 1);
  Eigen::Vector4f bbox_center = transform.inverse() * transformed_bbox_center;
  Eigen::Quaternionf q(transform.inverse().rotation());

  vision_msgs::msg::Detection3D detection3d_msg;
  detection3d_msg.bbox.center.position.x = bbox_center[0];
  detection3d_msg.bbox.center.position.y = bbox_center[1];
  detection3d_msg.bbox.center.position.z = bbox_center[2];
  detection3d_msg.bbox.center.orientation.x = q.x();
  detection3d_msg.bbox.center.orientation.y = q.y();
  detection3d_msg.bbox.center.orientation.z = q.z();
  detection3d_msg.bbox.center.orientation.w = q.w();
  detection3d_msg.bbox.size.x = max_pt.x - min_pt.x;
  detection3d_msg.bbox.size.y = max_pt.y - min_pt.y;
  detection3d_msg.bbox.size.z = max_pt.z - min_pt.z;
  detection3d_msg.results = detections_results_msg;

  detection3d_array_msg.detections.push_back(detection3d_msg);

  RCLCPP_DEBUG(this->get_logger(), "Created bounding box with center (%.2f, %.2f, %.2f) and size (%.2f, %.2f, %.2f)",
               bbox_center[0], bbox_center[1], bbox_center[2],
               detection3d_msg.bbox.size.x, detection3d_msg.bbox.size.y, detection3d_msg.bbox.size.z);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
PredictWithCloudNode::downsampleCloudMsg(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*cloud_msg, *cloud);
  cloud->is_dense = false;

  // Step 0: FOV clipping
  pcl::PointCloud<pcl::PointXYZ>::Ptr clipped_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud);
  if(gz_camera_convention_) {
    pass.setFilterFieldName ("z"); // ASSUMING WE ARE IN THE CAMERA (NOT OPTICAL) FRAME
    pass.setFilterLimits (-999.99, preprocessing_filter_threshold_);
  } else {
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (-999.99, preprocessing_filter_threshold_);
  }
  pass.filter(*clipped_cloud);
  
  // Step 1: Downsampling
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
  voxel_grid.setInputCloud(clipped_cloud);
  voxel_grid.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
  voxel_grid.filter(*downsampled_cloud);

  RCLCPP_DEBUG(this->get_logger(), "Downsampled cloud: original %zu points, filtered to %zu points",
              cloud->points.size(), downsampled_cloud->points.size());

  return downsampled_cloud;
}


void PredictWithCloudNode::removeGroundPlane(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud)
{
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());

  // Configure RANSAC
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(1000);
  seg.setDistanceThreshold(ransac_distance_threshold_);

  seg.setInputCloud(input_cloud);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.empty())
  {
    RCLCPP_WARN(this->get_logger(), "Ground plane segmentation failed.");
    output_cloud = input_cloud;
    return;
  }

  // Extract non-ground points
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(input_cloud);
  extract.setIndices(inliers);
  extract.setNegative(true); // Remove ground points
  extract.filter(*output_cloud);

  RCLCPP_DEBUG(this->get_logger(), "Removed %zu ground points, %zu remain.",
              inliers->indices.size(), output_cloud->points.size());
}


pcl::PointCloud<pcl::PointXYZ>::Ptr
PredictWithCloudNode::cloud2TransformedCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                             const std::string& source_frame, const std::string& target_frame,
                                             const rclcpp::Time& stamp)
{
  try
  {
    geometry_msgs::msg::TransformStamped tf_stamped = tf_buffer_->lookupTransform(target_frame, source_frame, stamp);
    Eigen::Affine3f eigen_transform = tf2::transformToEigen(tf_stamped.transform).cast<float>();
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    transformPointCloud(cloud, transformed_cloud, eigen_transform);

    RCLCPP_DEBUG(this->get_logger(), "Transformed cloud from '%s' to '%s' using TF.",
                 source_frame.c_str(), target_frame.c_str());
    return transformed_cloud;
  }
  catch (tf2::TransformException& e)
  {
    RCLCPP_DEBUG(this->get_logger(), "TF transform exception (using original cloud): %s", e.what());
    return cloud;
  }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
PredictWithCloudNode::euclideanClusterExtraction(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(cluster_tolerance_);
  ec.setMinClusterSize(min_cluster_size_);
  ec.setMaxClusterSize(max_cluster_size_);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);

  float min_distance = std::numeric_limits<float>::max();
  pcl::PointCloud<pcl::PointXYZ>::Ptr closest_cluster(new pcl::PointCloud<pcl::PointXYZ>());

  RCLCPP_DEBUG(this->get_logger(), "Found %zu clusters", cluster_indices.size());
  for (const auto& cluster : cluster_indices)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>());
    for (const auto& indice : cluster.indices)
    {
      cloud_cluster->push_back((*cloud)[indice]);
    }

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud_cluster, centroid);
    float distance = centroid.norm();

    if (distance < min_distance)
    {
      min_distance = distance;
      *closest_cluster = *cloud_cluster;
    }
  }
  RCLCPP_DEBUG(this->get_logger(), "Selected cluster with distance: %.2f", min_distance);
  return closest_cluster;
}

visualization_msgs::msg::MarkerArray
PredictWithCloudNode::createMarkerArray(const vision_msgs::msg::Detection3DArray& detection3d_array_msg,
                                        const double& duration)
{
  visualization_msgs::msg::MarkerArray marker_array_msg;

  for (size_t i = 0; i < detection3d_array_msg.detections.size(); i++)
  {
    if (std::isfinite(detection3d_array_msg.detections[i].bbox.size.x) &&
        std::isfinite(detection3d_array_msg.detections[i].bbox.size.y) &&
        std::isfinite(detection3d_array_msg.detections[i].bbox.size.z))
    {
      visualization_msgs::msg::Marker marker_msg;
      marker_msg.header = detection3d_array_msg.header;
      marker_msg.ns = "detection";
      marker_msg.id = i;
      marker_msg.type = visualization_msgs::msg::Marker::CUBE;
      marker_msg.action = visualization_msgs::msg::Marker::ADD;
      marker_msg.pose = detection3d_array_msg.detections[i].bbox.center;
      marker_msg.scale.x = detection3d_array_msg.detections[i].bbox.size.x;
      marker_msg.scale.y = detection3d_array_msg.detections[i].bbox.size.y;
      marker_msg.scale.z = detection3d_array_msg.detections[i].bbox.size.z;
      marker_msg.color.r = 0.0;
      marker_msg.color.g = 1.0;
      marker_msg.color.b = 0.0;
      marker_msg.color.a = 0.5;
      marker_msg.lifetime = rclcpp::Duration(std::chrono::duration<double>(duration));
      marker_array_msg.markers.push_back(marker_msg);
      RCLCPP_DEBUG(this->get_logger(), "Created marker id %zu with lifetime %.2f sec", i, duration);
    }
    else
    {
      RCLCPP_DEBUG(this->get_logger(), "Skipping marker creation for detection %zu due to non-finite dimensions", i);
    }
  }

  return marker_array_msg;
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  std::string package_share_dir = ament_index_cpp::get_package_share_directory("ultralytics_ros");
  std::string param_file = package_share_dir + "/config/predict_with_cloud_node.yaml";

  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "--params-file", param_file});

  auto node = std::make_shared<PredictWithCloudNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
