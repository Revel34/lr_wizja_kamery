// Fuses the data of N depth cameras into one point cloud in the robot frame. Each camera delivers
// either a PointCloud2 (our own realsense drivers) or a depth image + CameraInfo (a camera owned by
// another stack, e.g. the rover's front D455 shared with vision / visual odometry).

#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include "wizja_perception/depth_projection.hpp"
#include "wizja_perception/ground_segmentation.hpp"

namespace wizja_perception
{

class CloudFusionNode : public rclcpp::Node
{
public:
  explicit CloudFusionNode(const rclcpp::NodeOptions & options);

private:
  struct Camera
  {
    std::string name;
    std::string topic;                 // PointCloud2, or the depth image for depth_image cameras
    bool depth_image = false;
    std::string info_topic;            // depth_image only
    int decimation = 1;                // depth_image only
    bool has_intrinsics = false;       // depth_image only, guarded by mutex_
    PinholeIntrinsics intrinsics;      // depth_image only, guarded by mutex_
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub;
    Cloud::Ptr cloud;                  // latest processed cloud, already in target_frame
    rclcpp::Time stamp;                // header stamp of that cloud
    rclcpp::Time received;             // steady-clock time of arrival (staleness check)
    std::size_t received_total = 0;
    bool reported_stale = false;
  };

  void cloudCallback(std::size_t index, const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);
  void depthCallback(std::size_t index, const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  // Voxel-filters a camera's processed cloud and stores it as that camera's latest data.
  void storeCloud(std::size_t index, Cloud::Ptr cloud, const builtin_interfaces::msg::Time & stamp);
  bool lookupTransform(const std::string & frame, Eigen::Isometry3f & out);
  void publishFused();

  // Parameters
  std::string target_frame_;
  double range_min_;
  double range_max_;
  int point_stride_;
  double voxel_leaf_;
  double depth_scale_;
  double stale_timeout_;
  bool cache_static_tf_;
  bool self_filter_enabled_;
  Eigen::Vector3f self_min_;
  Eigen::Vector3f self_max_;

  std::vector<Camera> cameras_;
  std::mutex mutex_;  // guards cameras_[*].cloud/stamp/received and tf_cache_

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unordered_map<std::string, Eigen::Isometry3f> tf_cache_;

  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  rclcpp::Time last_published_stamp_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace wizja_perception
