#include "wizja_perception/ground_segmentation.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

#include <Eigen/Dense>

namespace wizja_perception
{

namespace
{
constexpr float kInf = std::numeric_limits<float>::infinity();
constexpr double kDegToRad = M_PI / 180.0;
}  // namespace

bool GroundGrid::cellOf(float x, float y, int & ix, int & iy) const
{
  const float fx = (x - origin) / cell_size;
  const float fy = (y - origin) / cell_size;
  const float limit = static_cast<float>(size);
  if (!(fx >= 0.0f && fy >= 0.0f && fx < limit && fy < limit)) {  // also rejects NaN
    return false;
  }
  ix = std::min(static_cast<int>(fx), size - 1);
  iy = std::min(static_cast<int>(fy), size - 1);
  return true;
}

GroundSegmenter::GroundSegmenter(const GroundSegmentationParams & params)
: params_(params)
{
  params_.cell_size = std::max(0.01, params_.cell_size);
  params_.range = std::max(params_.cell_size, params_.range);
  params_.min_points_per_cell = std::max(1, params_.min_points_per_cell);
  params_.ground_percentile = std::clamp(params_.ground_percentile, 0.0, 1.0);
}

GroundPlane GroundSegmenter::fitPlane(const std::vector<Eigen::Vector3f> & candidates) const
{
  GroundPlane fallback;
  fallback.normal = Eigen::Vector3f::UnitZ();
  fallback.d = static_cast<float>(-params_.expected_ground_z);
  fallback.fitted = false;

  const int n = static_cast<int>(candidates.size());
  if (n < std::max(3, params_.min_plane_candidates)) {
    return fallback;
  }

  const float cos_max_tilt = static_cast<float>(std::cos(params_.max_ground_tilt_deg * kDegToRad));
  const float dist = static_cast<float>(params_.ransac_distance);

  // Fixed seed: deterministic output for identical input (easier debugging and testing).
  std::mt19937 rng(42);
  std::uniform_int_distribution<int> pick(0, n - 1);

  int best_inliers = 0;
  Eigen::Vector3f best_normal = Eigen::Vector3f::UnitZ();
  float best_d = 0.0f;

  for (int it = 0; it < params_.ransac_iterations; ++it) {
    const int a = pick(rng);
    const int b = pick(rng);
    const int c = pick(rng);
    if (a == b || b == c || a == c) {
      continue;
    }
    Eigen::Vector3f normal = (candidates[b] - candidates[a]).cross(candidates[c] - candidates[a]);
    const float norm = normal.norm();
    if (norm < 1e-6f) {
      continue;
    }
    normal /= norm;
    if (normal.z() < 0.0f) {
      normal = -normal;
    }
    if (normal.z() < cos_max_tilt) {
      continue;
    }
    const float d = -normal.dot(candidates[a]);
    int inliers = 0;
    for (const auto & p : candidates) {
      if (std::fabs(normal.dot(p) + d) < dist) {
        ++inliers;
      }
    }
    if (inliers > best_inliers) {
      best_inliers = inliers;
      best_normal = normal;
      best_d = d;
    }
  }

  if (best_inliers < 3) {
    return fallback;
  }

  // Least-squares refinement z = a*x + b*y + c over the inliers.
  Eigen::Matrix3d ata = Eigen::Matrix3d::Zero();
  Eigen::Vector3d atb = Eigen::Vector3d::Zero();
  for (const auto & p : candidates) {
    if (std::fabs(best_normal.dot(p) + best_d) < dist) {
      const Eigen::Vector3d row(p.x(), p.y(), 1.0);
      ata += row * row.transpose();
      atb += row * static_cast<double>(p.z());
    }
  }

  GroundPlane plane;
  plane.fitted = true;
  plane.inliers = best_inliers;
  plane.normal = best_normal;
  plane.d = best_d;

  if (std::fabs(ata.determinant()) > 1e-9) {
    const Eigen::Vector3d abc = ata.ldlt().solve(atb);
    Eigen::Vector3d refined(-abc.x(), -abc.y(), 1.0);
    const double norm = refined.norm();
    refined /= norm;
    if (refined.z() >= cos_max_tilt) {
      plane.normal = refined.cast<float>();
      plane.d = static_cast<float>(-abc.z() / norm);
    }
  }

  if (std::fabs(plane.heightAt(0.0f, 0.0f) - params_.expected_ground_z) > params_.max_plane_offset) {
    return fallback;
  }
  return plane;
}

void GroundSegmenter::propagateGround(GroundGrid & grid, const GroundPlane & /*plane*/) const
{
  const int size = grid.size;
  const float step = static_cast<float>(std::tan(params_.max_slope_deg * kDegToRad) * grid.cell_size);
  const float diag = step * static_cast<float>(M_SQRT2);

  // Two-pass chamfer sweeps, repeated until convergence. Trusted ground cells are sources and
  // never change; every other cell takes the slope-limited minimum over its 8 neighbours.
  auto relax = [&](int ix, int iy, int nx, int ny, float cost) -> bool {
      if (nx < 0 || ny < 0 || nx >= size || ny >= size) {
        return false;
      }
      const float candidate = grid.height[grid.index(nx, ny)] + cost;
      float & h = grid.height[grid.index(ix, iy)];
      if (candidate < h - 1e-4f) {
        h = candidate;
        return true;
      }
      return false;
    };

  for (int iteration = 0; iteration < 32; ++iteration) {
    bool changed = false;
    for (int iy = 0; iy < size; ++iy) {
      for (int ix = 0; ix < size; ++ix) {
        if (grid.state[grid.index(ix, iy)] == CellState::kGround) {
          continue;
        }
        changed |= relax(ix, iy, ix - 1, iy, step);
        changed |= relax(ix, iy, ix - 1, iy - 1, diag);
        changed |= relax(ix, iy, ix, iy - 1, step);
        changed |= relax(ix, iy, ix + 1, iy - 1, diag);
      }
    }
    for (int iy = size - 1; iy >= 0; --iy) {
      for (int ix = size - 1; ix >= 0; --ix) {
        if (grid.state[grid.index(ix, iy)] == CellState::kGround) {
          continue;
        }
        changed |= relax(ix, iy, ix + 1, iy, step);
        changed |= relax(ix, iy, ix + 1, iy + 1, diag);
        changed |= relax(ix, iy, ix, iy + 1, step);
        changed |= relax(ix, iy, ix - 1, iy + 1, diag);
      }
    }
    if (!changed) {
      break;
    }
  }
}

SegmentationResult GroundSegmenter::segment(const Cloud & cloud) const
{
  SegmentationResult result;
  GroundGrid & grid = result.grid;
  grid.cell_size = static_cast<float>(params_.cell_size);
  grid.size = static_cast<int>(std::ceil(2.0 * params_.range / params_.cell_size));
  grid.origin = -0.5f * static_cast<float>(grid.size) * grid.cell_size;
  const int n_cells = grid.size * grid.size;
  grid.height.assign(n_cells, kInf);
  grid.raw_height.assign(n_cells, std::numeric_limits<float>::quiet_NaN());
  grid.state.assign(n_cells, CellState::kUnobserved);

  // --- 1. Bin points into cells (counting sort) ---------------------------------------------
  const std::size_t n_points = cloud.size();
  std::vector<int> point_cell(n_points, -1);
  std::vector<int> offsets(n_cells + 1, 0);
  for (std::size_t i = 0; i < n_points; ++i) {
    const auto & p = cloud[i];
    if (!std::isfinite(p.z)) {
      continue;
    }
    int ix, iy;
    if (grid.cellOf(p.x, p.y, ix, iy)) {
      point_cell[i] = grid.index(ix, iy);
      ++offsets[point_cell[i] + 1];
    }
  }
  for (int c = 0; c < n_cells; ++c) {
    offsets[c + 1] += offsets[c];
  }
  std::vector<int> order(offsets[n_cells]);
  {
    std::vector<int> cursor(offsets.begin(), offsets.end() - 1);
    for (std::size_t i = 0; i < n_points; ++i) {
      if (point_cell[i] >= 0) {
        order[cursor[point_cell[i]]++] = static_cast<int>(i);
      }
    }
  }

  // --- 2. Per-cell raw ground (low z percentile) + plane candidates --------------------------
  std::vector<Eigen::Vector3f> candidates;
  candidates.reserve(n_cells);
  for (int c = 0; c < n_cells; ++c) {
    const int begin = offsets[c];
    const int count = offsets[c + 1] - begin;
    if (count < params_.min_points_per_cell) {
      continue;
    }
    auto first = order.begin() + begin;
    auto kth = first + static_cast<int>(params_.ground_percentile * (count - 1));
    std::nth_element(first, kth, first + count, [&cloud](int a, int b) {return cloud[a].z < cloud[b].z;});
    const auto & p = cloud[*kth];
    grid.raw_height[c] = p.z;
    candidates.emplace_back(p.x, p.y, p.z);
  }

  // --- 3. Global plane -----------------------------------------------------------------------
  result.plane = fitPlane(candidates);
  const GroundPlane & plane = result.plane;
  const float trust_band = static_cast<float>(params_.ground_trust_band);
  const float max_dev = static_cast<float>(params_.max_ground_deviation);

  std::vector<float> plane_height(n_cells);
  for (int iy = 0; iy < grid.size; ++iy) {
    for (int ix = 0; ix < grid.size; ++ix) {
      const int c = grid.index(ix, iy);
      plane_height[c] = plane.heightAt(grid.cellCenter(ix), grid.cellCenter(iy));
      const float raw = grid.raw_height[c];
      if (std::isnan(raw)) {
        continue;
      }
      if (raw <= plane_height[c] + trust_band) {
        grid.state[c] = CellState::kGround;
        grid.height[c] = std::max(raw, plane_height[c] - max_dev);
      } else {
        grid.state[c] = CellState::kRaised;
        grid.height[c] = raw;
      }
    }
  }

  // --- 4. Slope-limited local ground surface -------------------------------------------------
  propagateGround(grid, plane);
  for (int c = 0; c < n_cells; ++c) {
    if (grid.state[c] == CellState::kGround) {
      continue;
    }
    float & h = grid.height[c];
    if (!std::isfinite(h)) {
      h = plane_height[c];
    }
    h = std::clamp(h, plane_height[c] - max_dev, plane_height[c] + max_dev);
  }

  // --- 5. Classify points --------------------------------------------------------------------
  const float min_h = static_cast<float>(params_.min_obstacle_height);
  const float max_h = static_cast<float>(params_.max_obstacle_height);
  result.ground->reserve(n_points);
  for (std::size_t i = 0; i < n_points; ++i) {
    const int c = point_cell[i];
    if (c < 0) {
      continue;
    }
    const auto & p = cloud[i];
    const float h = p.z - grid.height[c];
    if (h < min_h) {
      result.ground->push_back(p);
    } else if (h <= max_h) {
      result.obstacles->push_back(p);
      result.obstacle_heights.push_back(h);
    }
  }

  // --- 6. Negative obstacles (experimental) --------------------------------------------------
  if (params_.detect_negative_obstacles) {
    const float depth = static_cast<float>(params_.min_hole_depth);
    const float sub = grid.cell_size / 3.0f;
    for (int iy = 0; iy < grid.size; ++iy) {
      for (int ix = 0; ix < grid.size; ++ix) {
        const int c = grid.index(ix, iy);
        if (grid.state[c] != CellState::kGround || !(grid.raw_height[c] < plane_height[c] - depth)) {
          continue;
        }
        const float x0 = grid.origin + static_cast<float>(ix) * grid.cell_size + 0.5f * sub;
        const float y0 = grid.origin + static_cast<float>(iy) * grid.cell_size + 0.5f * sub;
        for (int sy = 0; sy < 3; ++sy) {
          for (int sx = 0; sx < 3; ++sx) {
            const float x = x0 + static_cast<float>(sx) * sub;
            const float y = y0 + static_cast<float>(sy) * sub;
            result.holes->push_back(PointT(x, y, plane.heightAt(x, y)));
          }
        }
      }
    }
  }

  for (Cloud * c : {result.ground.get(), result.obstacles.get(), result.holes.get()}) {
    c->width = static_cast<std::uint32_t>(c->size());
    c->height = 1;
    c->is_dense = true;
  }
  return result;
}

Occupancy buildOccupancy(const SegmentationResult & result, const OccupancyParams & params)
{
  Occupancy grid;
  grid.resolution = static_cast<float>(std::max(0.01, params.resolution));
  grid.width = std::max(1, static_cast<int>(std::lround(params.size / grid.resolution)));
  grid.origin = -0.5f * static_cast<float>(grid.width) * grid.resolution;
  grid.data.assign(static_cast<std::size_t>(grid.width) * grid.width, -1);

  auto cell = [&grid](const PointT & p) -> int {
      const float fx = (p.x - grid.origin) / grid.resolution;
      const float fy = (p.y - grid.origin) / grid.resolution;
      if (!(fx >= 0.0f && fy >= 0.0f) || fx >= grid.width || fy >= grid.width) {
        return -1;
      }
      return static_cast<int>(fy) * grid.width + static_cast<int>(fx);
    };

  for (const auto & p : *result.ground) {
    const int c = cell(p);
    if (c >= 0) {
      grid.data[c] = 0;
    }
  }
  std::vector<int> counts(grid.data.size(), 0);
  for (const auto & p : *result.obstacles) {
    const int c = cell(p);
    if (c >= 0 && ++counts[c] >= params.min_obstacle_points) {
      grid.data[c] = 100;
    }
  }
  for (const auto & p : *result.holes) {
    const int c = cell(p);
    if (c >= 0) {
      grid.data[c] = 100;
    }
  }
  return grid;
}

}  // namespace wizja_perception
