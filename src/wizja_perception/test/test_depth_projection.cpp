#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include <Eigen/Core>

#include "wizja_perception/depth_projection.hpp"

using wizja_perception::DepthEncoding;
using wizja_perception::DepthImageView;
using wizja_perception::PinholeIntrinsics;
using wizja_perception::projectDepth;

namespace
{

constexpr int kW = 8;
constexpr int kH = 6;
const PinholeIntrinsics kK{4.0, 5.0, 3.5, 2.5};

std::vector<Eigen::Vector3f> project(const DepthImageView & image, int decimation = 1)
{
  std::vector<Eigen::Vector3f> out;
  projectDepth(image, kK, decimation, [&out](float x, float y, float z) {out.emplace_back(x, y, z);});
  return out;
}

DepthImageView view16(const std::vector<std::uint16_t> & data)
{
  DepthImageView v;
  v.data = reinterpret_cast<const std::uint8_t *>(data.data());
  v.width = kW;
  v.height = kH;
  v.step = kW * sizeof(std::uint16_t);
  v.encoding = DepthEncoding::kUInt16;
  v.depth_scale = 0.001;
  return v;
}

DepthImageView view32(const std::vector<float> & data)
{
  DepthImageView v;
  v.data = reinterpret_cast<const std::uint8_t *>(data.data());
  v.width = kW;
  v.height = kH;
  v.step = kW * sizeof(float);
  v.encoding = DepthEncoding::kFloat32;
  return v;
}

}  // namespace

TEST(DepthProjection, EncodingNames)
{
  EXPECT_EQ(wizja_perception::depthEncodingFromString("16UC1"), DepthEncoding::kUInt16);
  EXPECT_EQ(wizja_perception::depthEncodingFromString("mono16"), DepthEncoding::kUInt16);
  EXPECT_EQ(wizja_perception::depthEncodingFromString("32FC1"), DepthEncoding::kFloat32);
  EXPECT_EQ(wizja_perception::depthEncodingFromString("rgb8"), DepthEncoding::kUnsupported);
}

TEST(DepthProjection, KnownPixelProjectsToExpectedPoint)
{
  std::vector<std::uint16_t> depth(kW * kH, 0);
  depth[4 * kW + 7] = 2000;  // u = 7, v = 4, 2 m
  const auto points = project(view16(depth));
  ASSERT_EQ(points.size(), 1u);
  EXPECT_NEAR(points[0].z(), 2.0f, 1e-6);
  EXPECT_NEAR(points[0].x(), (7 - kK.cx) / kK.fx * 2.0, 1e-6);  // right of centre -> +x
  EXPECT_NEAR(points[0].y(), (4 - kK.cy) / kK.fy * 2.0, 1e-6);  // below centre -> +y
}

TEST(DepthProjection, ZeroAndInvalidDepthAreSkipped)
{
  std::vector<float> depth(kW * kH, 1.5f);
  depth[0] = 0.0f;
  depth[1] = std::numeric_limits<float>::quiet_NaN();
  depth[2] = std::numeric_limits<float>::infinity();
  depth[3] = -1.0f;
  EXPECT_EQ(project(view32(depth)).size(), static_cast<std::size_t>(kW * kH - 4));
}

TEST(DepthProjection, DecimationKeepsEveryNthPixelInBothDirections)
{
  std::vector<std::uint16_t> depth(kW * kH, 1000);
  EXPECT_EQ(project(view16(depth), 1).size(), static_cast<std::size_t>(kW * kH));
  EXPECT_EQ(project(view16(depth), 2).size(), static_cast<std::size_t>((kW / 2) * (kH / 2)));
  EXPECT_EQ(project(view16(depth), 4).size(), static_cast<std::size_t>(2 * 2));
}

TEST(DepthProjection, Float32MatchesUInt16)
{
  std::vector<std::uint16_t> d16(kW * kH);
  std::vector<float> d32(kW * kH);
  for (int i = 0; i < kW * kH; ++i) {
    d16[i] = static_cast<std::uint16_t>(500 + 37 * i);
    d32[i] = d16[i] * 0.001f;
  }
  const auto a = project(view16(d16));
  const auto b = project(view32(d32));
  ASSERT_EQ(a.size(), b.size());
  for (std::size_t i = 0; i < a.size(); ++i) {
    EXPECT_TRUE(a[i].isApprox(b[i], 1e-5f)) << i;
  }
}

TEST(DepthProjection, RowStepPaddingIsRespected)
{
  // Rows padded to 20 bytes (8 px * 2 B = 16 B of data + 4 B padding).
  const std::size_t step = 20;
  std::vector<std::uint8_t> buffer(step * kH, 0xFF);
  for (int v = 0; v < kH; ++v) {
    for (int u = 0; u < kW; ++u) {
      const std::uint16_t value = (u == 2 && v == 3) ? 1234 : 0;
      std::memcpy(&buffer[v * step + u * 2], &value, 2);
    }
  }
  DepthImageView image;
  image.data = buffer.data();
  image.width = kW;
  image.height = kH;
  image.step = step;
  image.encoding = DepthEncoding::kUInt16;
  const auto points = project(image);
  ASSERT_EQ(points.size(), 1u);
  EXPECT_NEAR(points[0].z(), 1.234f, 1e-6);
}
