#include "wizja_perception/cloud_fusion_node.hpp"

#include <algorithm>
#include <cstring>
#include <functional>

#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>

namespace wizja_perception
{

namespace
{
// Byte offset of a FLOAT32 field, or -1 if the cloud has no such field.
int floatFieldOffset(const sensor_msgs::msg::PointCloud2 & msg, const std::string & name)
{
  for (const auto & field : msg.fields) {
    if (field.name == name && field.datatype == sensor_msgs::msg::PointField::FLOAT32) {
      return static_cast<int>(field.offset);
    }
  }
  return -1;
}

Eigen::Vector3f toVector3f(const std::vector<double> & v, const Eigen::Vector3f & fallback)
{
  if (v.size() != 3) {
    return fallback;
  }
  return Eigen::Vector3f(static_cast<float>(v[0]), static_cast<float>(v[1]), static_cast<float>(v[2]));
}
}  // namespace

CloudFusionNode::CloudFusionNode(const rclcpp::NodeOptions & options)
: Node("cloud_fusion", options)
{
  target_frame_ = declare_parameter<std::string>("target_frame", "base_link");
  range_min_ = declare_parameter<double>("range_min", 0.3);
  range_max_ = declare_parameter<double>("range_max", 8.0);
  point_stride_ = std::max<int>(1, declare_parameter<int>("point_stride", 1));
  voxel_leaf_ = declare_parameter<double>("voxel_leaf", 0.05);
  stale_timeout_ = declare_parameter<double>("stale_timeout", 0.5);
  cache_static_tf_ = declare_parameter<bool>("cache_static_tf", true);
  last_published_stamp_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  const double publish_rate = std::max(1.0, declare_parameter<double>("publish_rate", 15.0));
  const auto output_topic = declare_parameter<std::string>("output_topic", "/wizja/fused_points");

  self_filter_enabled_ = declare_parameter<bool>("self_filter.enabled", true);
  self_min_ = toVector3f(
    declare_parameter<std::vector<double>>("self_filter.min", {-0.6, -0.5, -0.2}),
    Eigen::Vector3f(-0.6f, -0.5f, -0.2f));
  self_max_ = toVector3f(
    declare_parameter<std::vector<double>>("self_filter.max", {0.6, 0.5, 1.0}),
    Eigen::Vector3f(0.6f, 0.5f, 1.0f));

  const auto names = declare_parameter<std::vector<std::string>>(
    "camera_names", std::vector<std::string>{"front", "left", "right"});
  if (names.empty()) {
    throw std::invalid_argument("camera_names must not be empty");
  }

  // Camera clouds: BEST_EFFORT/depth 1, like the realsense driver's default sensor QoS. Dropping an
  // occasional multi-MB cloud is fine; reliable retransmission of fragmented clouds is not.
  const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(1);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  // The node runs with intra-process comms enabled (zero-copy hand-over to the detector), which
  // rclcpp only allows for VOLATILE subscriptions. /tf_static is TRANSIENT_LOCAL, so the TF
  // subscriptions explicitly opt out of intra-process.
  auto tf_options = tf2_ros::detail::get_default_transform_listener_sub_options<>();
  auto tf_static_options = tf2_ros::detail::get_default_transform_listener_static_sub_options<>();
  tf_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
  tf_static_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
    *tf_buffer_, this, true, tf2_ros::DynamicListenerQoS(), tf2_ros::StaticListenerQoS(),
    tf_options, tf_static_options);

  cameras_.resize(names.size());
  for (std::size_t i = 0; i < names.size(); ++i) {
    Camera & cam = cameras_[i];
    cam.name = names[i];
    cam.topic = declare_parameter<std::string>(
      "cameras." + cam.name + ".topic", "/camera_" + cam.name + "/depth/color/points");
    cam.stamp = last_published_stamp_;
    cam.received = steady_clock_.now();
    cam.sub = create_subscription<sensor_msgs::msg::PointCloud2>(
      cam.topic, sensor_qos,
      [this, i](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {cloudCallback(i, msg);});
    RCLCPP_INFO(get_logger(), "Camera '%s' <- %s", cam.name.c_str(), cam.topic.c_str());
  }

  pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, rclcpp::QoS(1).reliable());
  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / publish_rate), std::bind(&CloudFusionNode::publishFused, this));

  RCLCPP_INFO(
    get_logger(), "Fusing %zu cameras into '%s' -> %s (range %.2f-%.2f m, voxel %.3f m, %.1f Hz)",
    cameras_.size(), target_frame_.c_str(), output_topic.c_str(), range_min_, range_max_, voxel_leaf_,
    publish_rate);
}

bool CloudFusionNode::lookupTransform(const std::string & frame, Eigen::Isometry3f & out)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = tf_cache_.find(frame);
    if (it != tf_cache_.end()) {
      out = it->second;
      return true;
    }
    if (!tf_buffer_) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Cloud in frame '%s' has no cached transform and the TF listener was already released "
        "(did a camera frame name change?). Restart the node or set cache_static_tf: false.",
        frame.c_str());
      return false;
    }
  }

  geometry_msgs::msg::TransformStamped tf_msg;
  try {
    tf_msg = tf_buffer_->lookupTransform(target_frame_, frame, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "No transform %s -> %s yet: %s",
      frame.c_str(), target_frame_.c_str(), ex.what());
    return false;
  }
  out = tf2::transformToEigen(tf_msg).cast<float>();

  if (!cache_static_tf_) {
    return true;
  }

  // Camera mounts are rigid, so transforms are looked up once and cached. When every camera's
  // transform is known the TF listener is dropped, which removes the /tf subscription (and its
  // executor load from high-rate odometry TF) from this process entirely.
  std::lock_guard<std::mutex> lock(mutex_);
  tf_cache_[frame] = out;
  RCLCPP_INFO(get_logger(), "Cached static transform %s -> %s", frame.c_str(), target_frame_.c_str());
  if (tf_cache_.size() >= cameras_.size() && tf_listener_) {
    tf_listener_.reset();
    tf_buffer_.reset();
    RCLCPP_INFO(get_logger(), "All camera transforms cached, TF listener released");
  }
  return true;
}

void CloudFusionNode::cloudCallback(
  std::size_t index, const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
{
  Eigen::Isometry3f tf;
  if (!lookupTransform(msg->header.frame_id, tf)) {
    return;
  }

  const int off_x = floatFieldOffset(*msg, "x");
  const int off_y = floatFieldOffset(*msg, "y");
  const int off_z = floatFieldOffset(*msg, "z");
  if (off_x < 0 || off_y < 0 || off_z < 0) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000, "Cloud on %s has no float32 x/y/z fields",
      cameras_[index].topic.c_str());
    return;
  }

  const std::size_t width = msg->width;
  const std::size_t n_points = width * msg->height;
  const float r2_min = static_cast<float>(range_min_ * range_min_);
  const float r2_max = static_cast<float>(range_max_ * range_max_);

  auto cloud = std::make_shared<Cloud>();
  cloud->reserve(n_points / point_stride_ + 1);
  const std::uint8_t * data = msg->data.data();
  for (std::size_t i = 0; i < n_points; i += point_stride_) {
    const std::uint8_t * point = data + (i / width) * msg->row_step + (i % width) * msg->point_step;
    float x, y, z;
    std::memcpy(&x, point + off_x, sizeof(float));
    std::memcpy(&y, point + off_y, sizeof(float));
    std::memcpy(&z, point + off_z, sizeof(float));
    const float r2 = x * x + y * y + z * z;  // NaN fails both comparisons and is dropped
    if (!(r2 >= r2_min && r2 <= r2_max)) {
      continue;
    }
    const Eigen::Vector3f p = tf * Eigen::Vector3f(x, y, z);
    if (self_filter_enabled_ &&
      (p.array() >= self_min_.array()).all() && (p.array() <= self_max_.array()).all())
    {
      continue;
    }
    cloud->push_back(PointT(p.x(), p.y(), p.z()));
  }

  if (voxel_leaf_ > 0.0 && !cloud->empty()) {
    auto filtered = std::make_shared<Cloud>();
    pcl::VoxelGrid<PointT> voxel;
    const float leaf = static_cast<float>(voxel_leaf_);
    voxel.setLeafSize(leaf, leaf, leaf);
    voxel.setInputCloud(cloud);
    voxel.filter(*filtered);
    cloud = filtered;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  Camera & cam = cameras_[index];
  cam.cloud = cloud;
  cam.stamp = rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type());
  cam.received = steady_clock_.now();
  ++cam.received_total;
}

void CloudFusionNode::publishFused()
{
  const rclcpp::Time now = steady_clock_.now();
  Cloud fused;
  rclcpp::Time newest(0, 0, get_clock()->get_clock_type());
  std::size_t fresh = 0;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (Camera & cam : cameras_) {
      if (cam.received_total == 0) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 10000, "No data yet from camera '%s' (%s)",
          cam.name.c_str(), cam.topic.c_str());
        continue;
      }
      const bool stale = (now - cam.received).seconds() > stale_timeout_;
      if (stale) {
        if (!cam.reported_stale) {
          RCLCPP_WARN(
            get_logger(), "Camera '%s' stale (no cloud for %.2f s), excluded from fusion",
            cam.name.c_str(), (now - cam.received).seconds());
          cam.reported_stale = true;
        }
        continue;
      }
      if (cam.reported_stale) {
        RCLCPP_INFO(get_logger(), "Camera '%s' is back", cam.name.c_str());
        cam.reported_stale = false;
      }
      if (cam.cloud) {
        fused += *cam.cloud;
        ++fresh;
        if (cam.stamp > newest) {
          newest = cam.stamp;
        }
      }
    }
  }

  // Only publish when some camera delivered a newer cloud, so the detector never reprocesses
  // an identical fused cloud when the timer runs faster than the cameras.
  if (fresh == 0 || newest <= last_published_stamp_) {
    return;
  }
  last_published_stamp_ = newest;

  auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(fused, *msg);
  msg->header.frame_id = target_frame_;
  msg->header.stamp = newest;
  pub_->publish(std::move(msg));
}

}  // namespace wizja_perception

RCLCPP_COMPONENTS_REGISTER_NODE(wizja_perception::CloudFusionNode)
