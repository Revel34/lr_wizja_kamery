// Ground-relative obstacle segmentation for a fused depth-camera point cloud.
//
// The cloud is expected in a robot body frame (base_link: X forward, Y left, Z up).
// Algorithm:
//   1. Bin the cloud into a 2D grid (cell_size). For every cell with enough points take a
//      low percentile of z as the raw ground estimate of that cell.
//   2. Fit a global ground plane (RANSAC, normal constrained to within max_ground_tilt of +Z)
//      through the per-cell ground estimates. This absorbs the rover's own pitch/roll.
//   3. Cells whose raw ground is close to (or below) the plane are trusted as ground.
//      Every other cell (raised above the plane, sparse, or unobserved) gets its ground height
//      from a slope-limited lower envelope of its neighbours:
//          g(c) = min(g_raw(c), g(n) + tan(max_slope) * dist(c, n))
//      Real slopes/hills (gentler than max_slope) are therefore kept as ground, while the
//      top of a box/rock whose base is occluded is pulled down to the surrounding terrain.
//   4. A point is an obstacle when its height above the ground of its cell lies in
//      [min_obstacle_height, max_obstacle_height]. Points above max_obstacle_height
//      (overhangs the rover passes under) are ignored.
//   5. Optionally (experimental) cells whose ground lies far below the plane are reported
//      as negative obstacles (holes / drops).

#pragma once

#include <cstdint>
#include <vector>

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace wizja_perception
{

using PointT = pcl::PointXYZ;
using Cloud = pcl::PointCloud<PointT>;

struct GroundSegmentationParams
{
  // Grid
  double cell_size = 0.25;            // [m] ground-estimation cell edge
  double range = 10.0;                // [m] half-size of the analysed square around the origin
  int min_points_per_cell = 3;        // fewer points -> cell ground is not measured, it is inferred
  double ground_percentile = 0.1;     // which z quantile of a cell is taken as its ground

  // Global plane (RANSAC)
  double expected_ground_z = 0.0;     // [m] ground height at the origin when the rover is level
  double max_plane_offset = 0.5;      // [m] reject plane if it is this far from expected_ground_z at the origin
  double max_ground_tilt_deg = 20.0;  // max angle between plane normal and +Z
  double ransac_distance = 0.08;      // [m] inlier threshold
  int ransac_iterations = 200;
  int min_plane_candidates = 10;      // fewer ground candidates -> fall back to the level plane

  // Local ground surface
  double ground_trust_band = 0.12;    // [m] cells within plane + this are trusted ground
  double max_ground_deviation = 1.0;  // [m] local ground is clamped to plane +/- this
  double max_slope_deg = 25.0;        // steepest terrain still considered drivable ground

  // Obstacle classification
  double min_obstacle_height = 0.15;  // [m] above local ground
  double max_obstacle_height = 2.0;   // [m] above local ground (higher = overhang, ignored)

  // Negative obstacles (experimental)
  bool detect_negative_obstacles = false;
  double min_hole_depth = 0.3;        // [m] cell ground below the plane by more than this -> hole
};

struct GroundPlane
{
  Eigen::Vector3f normal{0.0f, 0.0f, 1.0f};  // unit, normal.z() > 0
  float d = 0.0f;                             // normal . p + d = 0
  bool fitted = false;                        // false -> fallback level plane was used
  int inliers = 0;

  float heightAt(float x, float y) const
  {
    return -(d + normal.x() * x + normal.y() * y) / normal.z();
  }
};

enum class CellState : std::uint8_t
{
  kUnobserved = 0,  // no (or too few) points
  kGround = 1,      // measured and trusted ground
  kRaised = 2,      // measured, but ground inferred from neighbours (obstacle top / hill)
};

struct GroundGrid
{
  int size = 0;              // cells per side
  float origin = 0.0f;       // [m] x/y of the lower corner of cell (0, 0)
  float cell_size = 0.0f;
  std::vector<float> height;           // final local ground height per cell
  std::vector<float> raw_height;       // measured percentile height (NaN if unobserved)
  std::vector<CellState> state;

  bool cellOf(float x, float y, int & ix, int & iy) const;
  int index(int ix, int iy) const {return iy * size + ix;}
  float cellCenter(int i) const {return origin + (static_cast<float>(i) + 0.5f) * cell_size;}
};

struct SegmentationResult
{
  Cloud::Ptr ground{new Cloud};
  Cloud::Ptr obstacles{new Cloud};
  std::vector<float> obstacle_heights;  // height above local ground, parallel to `obstacles`
  Cloud::Ptr holes{new Cloud};          // synthetic points marking negative-obstacle cells
  GroundPlane plane;
  GroundGrid grid;
};

class GroundSegmenter
{
public:
  explicit GroundSegmenter(const GroundSegmentationParams & params);

  SegmentationResult segment(const Cloud & cloud) const;

  const GroundSegmentationParams & params() const {return params_;}

private:
  GroundPlane fitPlane(const std::vector<Eigen::Vector3f> & candidates) const;
  void propagateGround(GroundGrid & grid, const GroundPlane & plane) const;

  GroundSegmentationParams params_;
};

// 2D occupancy grid built from a segmentation result (ROS-free so it can be unit-tested).
// Values follow nav_msgs/OccupancyGrid: -1 unknown, 0 free, 100 occupied.
struct OccupancyParams
{
  double resolution = 0.1;        // [m]
  double size = 16.0;             // [m] edge length, grid centred on the origin
  int min_obstacle_points = 2;    // obstacle points needed to mark a cell occupied
};

struct Occupancy
{
  int width = 0;
  float origin = 0.0f;            // x and y of the lower corner
  float resolution = 0.0f;
  std::vector<std::int8_t> data;  // row-major, row = y
};

Occupancy buildOccupancy(const SegmentationResult & result, const OccupancyParams & params);

}  // namespace wizja_perception
