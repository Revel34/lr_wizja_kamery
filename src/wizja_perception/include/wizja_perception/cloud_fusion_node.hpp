// Fuses the point clouds of N depth cameras into one cloud in the robot body frame.

#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

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
    std::string topic;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub;
    Cloud::Ptr cloud;                  // latest processed cloud, already in target_frame
    rclcpp::Time stamp;                // header stamp of that cloud
    rclcpp::Time received;             // steady-clock time of arrival (staleness check)
    std::size_t received_total = 0;
    bool reported_stale = false;
  };

  void cloudCallback(std::size_t index, const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);
  bool lookupTransform(const std::string & frame, Eigen::Isometry3f & out);
  void publishFused();

  // Parameters
  std::string target_frame_;
  double range_min_;
  double range_max_;
  int point_stride_;
  double voxel_leaf_;
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
