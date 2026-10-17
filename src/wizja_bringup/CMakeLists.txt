cmake_minimum_required(VERSION 3.8)
project(wizja_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/fake_cameras.py scripts/coverage_map.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_camera_config test/test_camera_config.py)
  ament_add_pytest_test(test_description test/test_description.py)
  ament_add_pytest_test(test_sim_kinematics test/test_sim_kinematics.py)
endif()

ament_package()
