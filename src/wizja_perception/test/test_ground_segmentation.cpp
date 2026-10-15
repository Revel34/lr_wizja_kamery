#include <gtest/gtest.h>

#include <cmath>
#include <functional>
#include <random>

#include <Eigen/Geometry>

#include "wizja_perception/ground_segmentation.hpp"
#include "wizja_perception/obstacle_detector_node.hpp"

using wizja_perception::Cloud;
using wizja_perception::GroundSegmentationParams;
using wizja_perception::GroundSegmenter;
using wizja_perception::PointT;

namespace
{

constexpr double kStep = 0.05;
const double kTan15 = std::tan(15.0 * M_PI / 180.0);

// Terrain z = f(x, y) sampled on a regular grid with small noise; `skip` cuts out regions
// (e.g. the shadow behind an obstacle).
void addTerrain(
  Cloud & cloud, const std::function<double(double, double)> & f, double x0, double x1, double y0,
  double y1, const std::function<bool(double, double)> & skip = nullptr)
{
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> noise(-0.01, 0.01);
  for (double x = x0; x <= x1; x += kStep) {
    for (double y = y0; y <= y1; y += kStep) {
      if (skip && skip(x, y)) {
        continue;
      }
      cloud.push_back(PointT(x, y, f(x, y) + noise(rng)));
    }
  }
}

// All faces of an axis-aligned box.
void addBox(Cloud & cloud, Eigen::Vector3d lo, Eigen::Vector3d hi, bool top_only = false)
{
  for (double x = lo.x(); x <= hi.x() + 1e-9; x += kStep) {
    for (double y = lo.y(); y <= hi.y() + 1e-9; y += kStep) {
      cloud.push_back(PointT(x, y, hi.z()));
    }
  }
  if (top_only) {
    return;
  }
  for (double z = lo.z(); z <= hi.z() + 1e-9; z += kStep) {
    for (double x = lo.x(); x <= hi.x() + 1e-9; x += kStep) {
      cloud.push_back(PointT(x, lo.y(), z));
      cloud.push_back(PointT(x, hi.y(), z));
    }
    for (double y = lo.y(); y <= hi.y() + 1e-9; y += kStep) {
      cloud.push_back(PointT(lo.x(), y, z));
      cloud.push_back(PointT(hi.x(), y, z));
    }
  }
}

double flat(double, double) {return 0.0;}

// Flat until x = 2, 15 deg ramp up to x = 5, plateau after.
double ramp(double x, double)
{
  if (x < 2.0) {
    return 0.0;
  }
  return (std::min(x, 5.0) - 2.0) * kTan15;
}

bool allInside(const Cloud & cloud, Eigen::Vector3d lo, Eigen::Vector3d hi, double margin = 0.06)
{
  for (const auto & p : cloud) {
    if (p.x < lo.x() - margin || p.x > hi.x() + margin || p.y < lo.y() - margin ||
      p.y > hi.y() + margin || p.z < lo.z() - margin || p.z > hi.z() + margin)
    {
      return false;
    }
  }
  return true;
}

}  // namespace

TEST(GroundSegmentation, EmptyCloudFallsBackToLevelPlane)
{
  GroundSegmenter seg{GroundSegmentationParams{}};
  const auto r = seg.segment(Cloud{});
  EXPECT_FALSE(r.plane.fitted);
  EXPECT_NEAR(r.plane.heightAt(3.0f, -2.0f), 0.0f, 1e-6);
  EXPECT_TRUE(r.obstacles->empty());
  EXPECT_TRUE(r.ground->empty());
}

TEST(GroundSegmentation, FlatGroundHasNoObstacles)
{
  Cloud cloud;
  addTerrain(cloud, flat, -8, 8, -8, 8);
  GroundSegmenter seg{GroundSegmentationParams{}};
  const auto r = seg.segment(cloud);
  EXPECT_TRUE(r.plane.fitted);
  EXPECT_GT(r.plane.normal.z(), 0.999f);
  EXPECT_EQ(r.obstacles->size(), 0u);
  EXPECT_EQ(r.ground->size(), cloud.size());
}

TEST(GroundSegmentation, BoxOnFlatGroundIsDetected)
{
  Cloud cloud;
  const Eigen::Vector3d lo(3.0, -0.25, 0.0), hi(3.5, 0.25, 0.4);
  addTerrain(cloud, flat, -8, 8, -8, 8);
  addBox(cloud, lo, hi);
  GroundSegmenter seg{GroundSegmentationParams{}};
  const auto r = seg.segment(cloud);
  EXPECT_GT(r.obstacles->size(), 100u);
  EXPECT_TRUE(allInside(*r.obstacles, lo, hi));
  ASSERT_EQ(r.obstacle_heights.size(), r.obstacles->size());

  wizja_perception::ClusterParams cp;
  const auto clusters = wizja_perception::clusterObstacles(r, cp);
  ASSERT_EQ(clusters.obstacles.size(), 1u);
  const auto & o = clusters.obstacles[0];
  EXPECT_NEAR(o.centroid.x, 3.25, 0.15);
  EXPECT_NEAR(o.centroid.y, 0.0, 0.1);
  EXPECT_NEAR(o.max_height_above_ground, 0.4, 0.05);
  EXPECT_NEAR(o.distance, 3.0, 0.1);
}

TEST(GroundSegmentation, LowObjectBelowThresholdIsGround)
{
  Cloud cloud;
  addTerrain(cloud, flat, -8, 8, -8, 8);
  addBox(cloud, {3.0, -0.25, 0.0}, {3.5, 0.25, 0.08});
  GroundSegmenter seg{GroundSegmentationParams{}};
  EXPECT_EQ(seg.segment(cloud).obstacles->size(), 0u);
}

TEST(GroundSegmentation, SlopeIsNotAnObstacle)
{
  Cloud cloud;
  addTerrain(cloud, ramp, -8, 8, -8, 8);
  GroundSegmenter seg{GroundSegmentationParams{}};
  const auto r = seg.segment(cloud);
  EXPECT_TRUE(r.plane.fitted);
  EXPECT_EQ(r.obstacles->size(), 0u);
}

TEST(GroundSegmentation, BoxOnSlopeIsDetected)
{
  Cloud cloud;
  addTerrain(cloud, ramp, -8, 8, -8, 8);
  const double base = ramp(3.5, 0);
  const Eigen::Vector3d lo(3.5, 1.0, base), hi(3.9, 1.4, base + 0.5);
  addBox(cloud, lo, hi);
  GroundSegmenter seg{GroundSegmentationParams{}};
  const auto r = seg.segment(cloud);
  EXPECT_GT(r.obstacles->size(), 50u);
  // Allow the uphill side of the box base, which is buried in the ramp.
  EXPECT_TRUE(allInside(*r.obstacles, lo, hi, 0.1));
}

TEST(GroundSegmentation, StepSteeperThanMaxSlopeIsObstacle)
{
  // A 0.5 m vertical step (wall/terrace) must be an obstacle even though the terrain behind it
  // is flat.
  Cloud cloud;
  addTerrain(cloud, [](double x, double) {return x < 4.0 ? 0.0 : 0.5;}, -8, 8, -8, 8);
  addBox(cloud, {4.0, -8.0, 0.0}, {4.0, 8.0, 0.5});  // the vertical face
  GroundSegmenter seg{GroundSegmentationParams{}};
  const auto r = seg.segment(cloud);
  EXPECT_GT(r.obstacles->size(), 100u);
}

TEST(GroundSegmentation, BoxTopWithOccludedBaseIsDetected)
{
  // Only the top of a 0.6 m box is seen, its base and the ground right behind it are occluded.
  Cloud cloud;
  const Eigen::Vector3d lo(3.0, -0.5, 0.0), hi(4.0, 0.5, 0.6);
  addTerrain(
    cloud, flat, -8, 8, -8, 8,
    [](double x, double y) {return x > 2.9 && x < 5.0 && std::fabs(y) < 0.6;});
  addBox(cloud, lo, hi, true);
  GroundSegmenter seg{GroundSegmentationParams{}};
  const auto r = seg.segment(cloud);
  EXPECT_GT(r.obstacles->size(), 300u);  // most of the 21x21 top points
  EXPECT_TRUE(allInside(*r.obstacles, lo, hi));
}

TEST(GroundSegmentation, TiltedRoverStillFindsGround)
{
  // Rover pitched 10 deg and rolled 5 deg: in base_link the whole world is tilted.
  Cloud world;
  addTerrain(world, flat, -8, 8, -8, 8);
  const Eigen::Vector3d lo(3.0, -0.25, 0.0), hi(3.5, 0.25, 0.4);
  addBox(world, lo, hi);

  const Eigen::Affine3f world_to_base(
    Eigen::AngleAxisf(10.0f * M_PI / 180.0f, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(5.0f * M_PI / 180.0f, Eigen::Vector3f::UnitX()));
  Cloud cloud;
  for (const auto & p : world) {
    const Eigen::Vector3f q = world_to_base * Eigen::Vector3f(p.x, p.y, p.z);
    cloud.push_back(PointT(q.x(), q.y(), q.z()));
  }

  GroundSegmenter seg{GroundSegmentationParams{}};
  const auto r = seg.segment(cloud);
  EXPECT_TRUE(r.plane.fitted);
  const float tilt = std::acos(r.plane.normal.z()) * 180.0f / M_PI;
  EXPECT_NEAR(tilt, 11.2f, 1.0f);
  EXPECT_GT(r.obstacles->size(), 100u);

  // Every obstacle point must come from the box.
  const Eigen::Affine3f base_to_world = world_to_base.inverse();
  Cloud back;
  for (const auto & p : *r.obstacles) {
    const Eigen::Vector3f q = base_to_world * Eigen::Vector3f(p.x, p.y, p.z);
    back.push_back(PointT(q.x(), q.y(), q.z()));
  }
  EXPECT_TRUE(allInside(back, lo, hi));
}

TEST(GroundSegmentation, DitchOnlyFlagsItsRims)
{
  // A 0.5 m deep, 1 m wide ditch. The ditch floor itself is never a positive obstacle. Ground
  // cells straddling a rim take the floor as their ground, so the rim edge (a 0.5 m drop) is
  // reported as obstacle - conservative, and the only positive obstacles allowed here.
  Cloud cloud;
  auto ditch = [](double x, double) {return (x > 3.0 && x < 4.0) ? -0.5 : 0.0;};
  addTerrain(cloud, ditch, -8, 8, -8, 8);
  GroundSegmentationParams params;
  auto r = GroundSegmenter{params}.segment(cloud);
  const float cell = static_cast<float>(params.cell_size);
  for (const auto & p : *r.obstacles) {
    const bool near_rim = std::fabs(p.x - 3.0f) <= cell || std::fabs(p.x - 4.0f) <= cell;
    ASSERT_TRUE(near_rim) << "obstacle point away from the rims at x=" << p.x;
    ASSERT_GT(p.z, -0.1f) << "ditch floor point classified as obstacle";
  }
  EXPECT_TRUE(r.holes->empty());

  params.detect_negative_obstacles = true;
  r = GroundSegmenter{params}.segment(cloud);
  ASSERT_FALSE(r.holes->empty());
  // Hole markers stay within the ditch, give or take one ground cell at its edges.
  for (const auto & p : *r.holes) {
    EXPECT_GT(p.x, 3.0f - cell);
    EXPECT_LT(p.x, 4.0f + cell);
  }
}

TEST(GroundSegmentation, OverhangAboveMaxHeightIsIgnored)
{
  Cloud cloud;
  addTerrain(cloud, flat, -8, 8, -8, 8);
  addTerrain(cloud, [](double, double) {return 2.5;}, 3, 4, -1, 1);  // e.g. a branch / ceiling
  GroundSegmenter seg{GroundSegmentationParams{}};
  EXPECT_EQ(seg.segment(cloud).obstacles->size(), 0u);
}

TEST(Occupancy, MarksObstacleFreeAndUnknown)
{
  Cloud cloud;
  addTerrain(cloud, flat, 0, 6, -2, 2);
  addBox(cloud, {3.0, -0.25, 0.0}, {3.5, 0.25, 0.4});
  GroundSegmenter seg{GroundSegmentationParams{}};
  const auto r = seg.segment(cloud);

  wizja_perception::OccupancyParams op;
  const auto occ = wizja_perception::buildOccupancy(r, op);
  ASSERT_EQ(occ.width, 160);
  auto at = [&occ](float x, float y) {
      const int ix = static_cast<int>((x - occ.origin) / occ.resolution);
      const int iy = static_cast<int>((y - occ.origin) / occ.resolution);
      return occ.data[iy * occ.width + ix];
    };
  EXPECT_EQ(at(3.25f, 0.05f), 100);
  EXPECT_EQ(at(1.55f, 0.55f), 0);
  EXPECT_EQ(at(-5.0f, -5.0f), -1);
}
