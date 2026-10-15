#include "wizja_perception/obstacle_detector_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp_components/register_node_macro.hpp>

namespace wizja_perception
{

wizja_msgs::msg::ObstacleArray clusterObstacles(
  const SegmentationResult & result, const ClusterParams & params)
{
  wizja_msgs::msg::ObstacleArray out;
  const Cloud::ConstPtr cloud = result.obstacles;
  if (!params.enabled || cloud->size() < static_cast<std::size_t>(std::max(1, params.min_size))) {
    return out;
  }

  auto tree = std::make_shared<pcl::search::KdTree<PointT>>();
  tree->setInputCloud(cloud);
  std::vector<pcl::PointIndices> clusters;
  pcl::EuclideanClusterExtraction<PointT> extraction;
  extraction.setClusterTolerance(params.tolerance);
  extraction.setMinClusterSize(params.min_size);
  extraction.setMaxClusterSize(params.max_size);
  extraction.setSearchMethod(tree);
  extraction.setInputCloud(cloud);
  extraction.extract(clusters);

  out.obstacles.reserve(clusters.size());
  for (const auto & indices : clusters) {
    wizja_msgs::msg::Obstacle obstacle;
    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    Eigen::Vector3f lo = Eigen::Vector3f::Constant(std::numeric_limits<float>::max());
    Eigen::Vector3f hi = Eigen::Vector3f::Constant(std::numeric_limits<float>::lowest());
    float max_height = 0.0f;
    float min_dist2 = std::numeric_limits<float>::max();
    for (const int i : indices.indices) {
      const auto & p = (*cloud)[i];
      const Eigen::Vector3f v(p.x, p.y, p.z);
      sum += v.cast<double>();
      lo = lo.cwiseMin(v);
      hi = hi.cwiseMax(v);
      max_height = std::max(max_height, result.obstacle_heights[i]);
      min_dist2 = std::min(min_dist2, p.x * p.x + p.y * p.y);
    }
    const Eigen::Vector3d c = sum / static_cast<double>(indices.indices.size());
    obstacle.centroid.x = c.x();
    obstacle.centroid.y = c.y();
    obstacle.centroid.z = c.z();
    obstacle.min.x = lo.x();
    obstacle.min.y = lo.y();
    obstacle.min.z = lo.z();
    obstacle.max.x = hi.x();
    obstacle.max.y = hi.y();
    obstacle.max.z = hi.z();
    obstacle.max_height_above_ground = max_height;
    obstacle.distance = std::sqrt(min_dist2);
    obstacle.point_count = static_cast<std::uint32_t>(indices.indices.size());
    out.obstacles.push_back(obstacle);
  }

  std::sort(
    out.obstacles.begin(), out.obstacles.end(),
    [](const auto & a, const auto & b) {return a.distance < b.distance;});
  for (std::size_t i = 0; i < out.obstacles.size(); ++i) {
    out.obstacles[i].id = static_cast<std::uint32_t>(i);
  }
  return out;
}

ObstacleDetectorNode::ObstacleDetectorNode(const rclcpp::NodeOptions & options)
: Node("obstacle_detector", options)
{
  GroundSegmentationParams g;
  g.cell_size = declare_parameter("ground.cell_size", g.cell_size);
  g.range = declare_parameter("ground.range", g.range);
  g.min_points_per_cell = declare_parameter("ground.min_points_per_cell", g.min_points_per_cell);
  g.ground_percentile = declare_parameter("ground.percentile", g.ground_percentile);
  g.expected_ground_z = declare_parameter("ground.expected_ground_z", g.expected_ground_z);
  g.max_plane_offset = declare_parameter("ground.max_plane_offset", g.max_plane_offset);
  g.max_ground_tilt_deg = declare_parameter("ground.max_tilt_deg", g.max_ground_tilt_deg);
  g.ransac_distance = declare_parameter("ground.ransac_distance", g.ransac_distance);
  g.ransac_iterations = declare_parameter("ground.ransac_iterations", g.ransac_iterations);
  g.min_plane_candidates = declare_parameter("ground.min_plane_candidates", g.min_plane_candidates);
  g.ground_trust_band = declare_parameter("ground.trust_band", g.ground_trust_band);
  g.max_ground_deviation = declare_parameter("ground.max_deviation", g.max_ground_deviation);
  g.max_slope_deg = declare_parameter("ground.max_slope_deg", g.max_slope_deg);
  g.min_obstacle_height = declare_parameter("obstacle.min_height", g.min_obstacle_height);
  g.max_obstacle_height = declare_parameter("obstacle.max_height", g.max_obstacle_height);
  g.detect_negative_obstacles = declare_parameter("negative.enabled", g.detect_negative_obstacles);
  g.min_hole_depth = declare_parameter("negative.min_depth", g.min_hole_depth);
  segmenter_ = std::make_unique<GroundSegmenter>(g);

  occupancy_params_.resolution = declare_parameter("grid.resolution", occupancy_params_.resolution);
  occupancy_params_.size = declare_parameter("grid.size", occupancy_params_.size);
  occupancy_params_.min_obstacle_points =
    declare_parameter("grid.min_obstacle_points", occupancy_params_.min_obstacle_points);

  cluster_params_.enabled = declare_parameter("clusters.enabled", cluster_params_.enabled);
  cluster_params_.tolerance = declare_parameter("clusters.tolerance", cluster_params_.tolerance);
  cluster_params_.min_size = declare_parameter("clusters.min_size", cluster_params_.min_size);
  cluster_params_.max_size = declare_parameter("clusters.max_size", cluster_params_.max_size);

  publish_ground_ = declare_parameter("publish_ground", true);
  const auto input_topic = declare_parameter<std::string>("input_topic", "/wizja/fused_points");

  // Outputs are RELIABLE depth 1: Nav2's costmap observation buffers subscribe reliably.
  const auto out_qos = rclcpp::QoS(1).reliable();
  obstacles_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/obstacles/points", out_qos);
  grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/obstacles/grid", out_qos);
  clusters_pub_ = create_publisher<wizja_msgs::msg::ObstacleArray>("/obstacles/clusters", out_qos);
  markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/obstacles/markers", out_qos);
  if (publish_ground_) {
    ground_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/wizja/ground_points", out_qos);
  }

  sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic, rclcpp::QoS(1).reliable(),
    [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {cloudCallback(msg);});

  RCLCPP_INFO(
    get_logger(), "Obstacle detector on %s: obstacle height %.2f-%.2f m above local ground, "
    "max slope %.0f deg, cell %.2f m, range %.1f m", input_topic.c_str(), g.min_obstacle_height,
    g.max_obstacle_height, g.max_slope_deg, g.cell_size, g.range);
}

void ObstacleDetectorNode::cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
{
  const auto t0 = std::chrono::steady_clock::now();

  Cloud cloud;
  pcl::fromROSMsg(*msg, cloud);
  const SegmentationResult result = segmenter_->segment(cloud);

  if (!result.plane.fitted) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Ground plane fit failed (too little ground visible?) - using the level fallback plane");
  }

  // Obstacle cloud (positive obstacles + negative-obstacle markers), drop-in for the old lidar
  // slice consumed by the Nav2 costmap.
  {
    Cloud obstacles = *result.obstacles;
    obstacles += *result.holes;
    auto out = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(obstacles, *out);
    out->header = msg->header;
    obstacles_pub_->publish(std::move(out));
  }

  if (ground_pub_) {
    auto out = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*result.ground, *out);
    out->header = msg->header;
    ground_pub_->publish(std::move(out));
  }

  {
    const Occupancy occ = buildOccupancy(result, occupancy_params_);
    auto grid = std::make_unique<nav_msgs::msg::OccupancyGrid>();
    grid->header = msg->header;
    grid->info.map_load_time = msg->header.stamp;
    grid->info.resolution = occ.resolution;
    grid->info.width = static_cast<std::uint32_t>(occ.width);
    grid->info.height = static_cast<std::uint32_t>(occ.width);
    grid->info.origin.position.x = occ.origin;
    grid->info.origin.position.y = occ.origin;
    grid->info.origin.orientation.w = 1.0;
    grid->data = occ.data;
    grid_pub_->publish(std::move(grid));
  }

  wizja_msgs::msg::ObstacleArray obstacles = clusterObstacles(result, cluster_params_);
  obstacles.header = msg->header;
  publishMarkers(obstacles);
  clusters_pub_->publish(obstacles);

  const double ms =
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  RCLCPP_DEBUG(
    get_logger(), "%zu pts -> %zu ground, %zu obstacle, %zu clusters in %.1f ms",
    cloud.size(), result.ground->size(), result.obstacles->size(), obstacles.obstacles.size(), ms);
}

void ObstacleDetectorNode::publishMarkers(const wizja_msgs::msg::ObstacleArray & obstacles)
{
  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker clear;
  clear.header = obstacles.header;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear);

  for (const auto & o : obstacles.obstacles) {
    visualization_msgs::msg::Marker box;
    box.header = obstacles.header;
    box.ns = "obstacles";
    box.id = static_cast<int>(o.id);
    box.type = visualization_msgs::msg::Marker::CUBE;
    box.action = visualization_msgs::msg::Marker::ADD;
    box.pose.position.x = 0.5 * (o.min.x + o.max.x);
    box.pose.position.y = 0.5 * (o.min.y + o.max.y);
    box.pose.position.z = 0.5 * (o.min.z + o.max.z);
    box.pose.orientation.w = 1.0;
    box.scale.x = std::max(0.05, o.max.x - o.min.x);
    box.scale.y = std::max(0.05, o.max.y - o.min.y);
    box.scale.z = std::max(0.05, o.max.z - o.min.z);
    // Red when close, yellow further away.
    const float closeness = std::clamp(1.0f - o.distance / 5.0f, 0.0f, 1.0f);
    box.color.r = 1.0f;
    box.color.g = 1.0f - closeness;
    box.color.b = 0.0f;
    box.color.a = 0.5f;
    markers.markers.push_back(box);

    visualization_msgs::msg::Marker label = box;
    label.ns = "labels";
    label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    label.pose.position.z = o.max.z + 0.2;
    label.scale.x = label.scale.y = 0.0;
    label.scale.z = 0.2;
    label.color.r = label.color.g = label.color.b = label.color.a = 1.0f;
    char text[64];
    std::snprintf(text, sizeof(text), "#%u %.1fm h%.2f", o.id, o.distance, o.max_height_above_ground);
    label.text = text;
    markers.markers.push_back(label);
  }
  markers_pub_->publish(markers);
}

}  // namespace wizja_perception

RCLCPP_COMPONENTS_REGISTER_NODE(wizja_perception::ObstacleDetectorNode)
