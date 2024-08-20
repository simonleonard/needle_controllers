cmake_minimum_required(VERSION 3.8)
project(spacenav_teleop_needle_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_interface REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

add_library(spacenav_teleop_needle_controller SHARED src/spacenav_teleop_controller.cpp)
target_compile_features(spacenav_teleop_needle_controller PUBLIC cxx_std_17)
target_include_directories(spacenav_teleop_needle_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/needle_controllers>
  )

ament_target_dependencies(spacenav_teleop_needle_controller PUBLIC Eigen3 rclcpp pluginlib control_msgs controller_interface geometry_msgs rclcpp)

pluginlib_export_plugin_description_file(controller_interface spacenav_teleop_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include/spacenav_teleop_needle_controller
)
install(
  TARGETS spacenav_teleop_needle_controller
  EXPORT export_velocity_controllers
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_velocity_controllers HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)
ament_package()
