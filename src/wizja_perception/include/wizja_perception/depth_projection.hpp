// Back-projection of a depth image into 3D points in the camera's optical frame
// (Z forward, X right, Y down), ROS-free so it can be unit-tested.

#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>

namespace wizja_perception
{

enum class DepthEncoding
{
  kUnsupported,
  kUInt16,   // 16UC1 / mono16, depth in sensor units (RealSense: mm) -> multiply by depth_scale
  kFloat32,  // 32FC1, depth in metres
};

DepthEncoding depthEncodingFromString(const std::string & encoding);

struct PinholeIntrinsics
{
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;

  bool valid() const {return fx > 0.0 && fy > 0.0;}
};

struct DepthImageView
{
  const std::uint8_t * data = nullptr;
  int width = 0;
  int height = 0;
  std::size_t step = 0;  // bytes per row
  DepthEncoding encoding = DepthEncoding::kUnsupported;
  double depth_scale = 0.001;  // only for kUInt16
};

// Calls point(x, y, z) for every valid pixel (depth > 0 and finite), taking every `decimation`-th
// pixel in both directions. The image buffer must be little-endian (as published by realsense).
// Returns the number of points emitted.
template<typename PointFn>
std::size_t projectDepth(
  const DepthImageView & image, const PinholeIntrinsics & k, int decimation, PointFn && point)
{
  if (image.data == nullptr || !k.valid() || image.encoding == DepthEncoding::kUnsupported) {
    return 0;
  }
  const int step = decimation < 1 ? 1 : decimation;
  const double inv_fx = 1.0 / k.fx;
  const double inv_fy = 1.0 / k.fy;
  std::size_t count = 0;
  for (int v = 0; v < image.height; v += step) {
    const std::uint8_t * row = image.data + static_cast<std::size_t>(v) * image.step;
    const float ry = static_cast<float>((v - k.cy) * inv_fy);
    for (int u = 0; u < image.width; u += step) {
      float z;
      if (image.encoding == DepthEncoding::kUInt16) {
        std::uint16_t raw;
        std::memcpy(&raw, row + static_cast<std::size_t>(u) * sizeof(raw), sizeof(raw));
        if (raw == 0) {
          continue;
        }
        z = static_cast<float>(raw * image.depth_scale);
      } else {
        std::memcpy(&z, row + static_cast<std::size_t>(u) * sizeof(z), sizeof(z));
        if (!(z > 0.0f) || !(z < 1e6f)) {  // rejects 0, negatives, NaN, inf
          continue;
        }
      }
      point(static_cast<float>((u - k.cx) * inv_fx) * z, ry * z, z);
      ++count;
    }
  }
  return count;
}

}  // namespace wizja_perception
