cmake_minimum_required(VERSION 3.8)
project(wizja_perception)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(wizja_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
# PCL's config still relies on <Pkg>_ROOT handling (CMP0074).
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()
find_package(PCL REQUIRED COMPONENTS common filters search kdtree segmentation)

# --- ROS-free core (ground segmentation + occupancy) --------------------------------------
add_library(wizja_ground SHARED src/ground_segmentation.cpp)
target_include_directories(wizja_ground PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(wizja_ground ${PCL_LIBRARIES} Eigen3::Eigen)
# PCL 1.12 headers trigger many -Wpedantic warnings; keep them out of our own diagnostics.
target_include_directories(wizja_ground SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})

# --- Composable nodes -----------------------------------------------------------------------
add_library(wizja_perception_components SHARED
  src/cloud_fusion_node.cpp
  src/obstacle_detector_node.cpp)
target_link_libraries(wizja_perception_components wizja_ground)
ament_target_dependencies(wizja_perception_components
  rclcpp rclcpp_components sensor_msgs nav_msgs visualization_msgs geometry_msgs
  tf2 tf2_ros tf2_eigen pcl_conversions wizja_msgs)

rclcpp_components_register_node(wizja_perception_components
  PLUGIN "wizja_perception::CloudFusionNode"
  EXECUTABLE cloud_fusion_node)
rclcpp_components_register_node(wizja_perception_components
  PLUGIN "wizja_perception::ObstacleDetectorNode"
  EXECUTABLE obstacle_detector_node)

install(TARGETS wizja_ground wizja_perception_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_ground_segmentation test/test_ground_segmentation.cpp)
  target_link_libraries(test_ground_segmentation wizja_ground wizja_perception_components)
  ament_target_dependencies(test_ground_segmentation wizja_msgs rclcpp)
endif()

ament_package()
