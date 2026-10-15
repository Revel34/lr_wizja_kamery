// Detects obstacles in the fused cloud relative to the locally estimated ground surface.

#pragma once

#include <memory>
#include <string>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <wizja_msgs/msg/obstacle_array.hpp>

#include "wizja_perception/ground_segmentation.hpp"

namespace wizja_perception
{

struct ClusterParams
{
  bool enabled = true;
  double tolerance = 0.3;   // [m] max gap between points of one obstacle
  int min_size = 10;        // [points]
  int max_size = 100000;    // [points]
};

// Euclidean clustering of the obstacle points (ROS-free part, unit-tested).
wizja_msgs::msg::ObstacleArray clusterObstacles(
  const SegmentationResult & result, const ClusterParams & params);

class ObstacleDetectorNode : public rclcpp::Node
{
public:
  explicit ObstacleDetectorNode(const rclcpp::NodeOptions & options);

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);
  void publishMarkers(const wizja_msgs::msg::ObstacleArray & obstacles);

  std::unique_ptr<GroundSegmenter> segmenter_;
  OccupancyParams occupancy_params_;
  ClusterParams cluster_params_;
  bool publish_ground_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacles_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
  rclcpp::Publisher<wizja_msgs::msg::ObstacleArray>::SharedPtr clusters_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
};

}  // namespace wizja_perception
