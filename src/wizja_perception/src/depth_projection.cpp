#include "wizja_perception/depth_projection.hpp"

namespace wizja_perception
{

DepthEncoding depthEncodingFromString(const std::string & encoding)
{
  if (encoding == "16UC1" || encoding == "mono16") {
    return DepthEncoding::kUInt16;
  }
  if (encoding == "32FC1") {
    return DepthEncoding::kFloat32;
  }
  return DepthEncoding::kUnsupported;
}

}  // namespace wizja_perception
