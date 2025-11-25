cmake_minimum_required(VERSION 3.8)
project(pose_calculator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)

add_library(pose_calculator SHARED
  src/pose_calculator.cpp
)
target_include_directories(pose_calculator PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/pose_calculator>
)
target_link_libraries(pose_calculator PUBLIC
  controller_interface::controller_interface
  ${geometry_msgs_TARGETS}
  pluginlib::pluginlib
)
pluginlib_export_plugin_description_file(controller_interface pose_calculator.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_load_pose_calculator test/test_load_pose_calculator.cpp)
  target_link_libraries(test_load_pose_calculator
    pose_calculator
    controller_manager::controller_manager
    ros2_control_test_assets::ros2_control_test_assets
  )
endif()

install(
  TARGETS
    pose_calculator
  EXPORT export_pose_calculator
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

ament_export_targets(export_pose_calculator HAS_LIBRARY_TARGET)
ament_package()
