cmake_minimum_required(VERSION 3.8)
project(zed_camera_calibration)

set(node_plugins "")


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(DEPENDENCIES_COMP
    rclcpp
    rclcpp_components
    builtin_interfaces
    zed_wrapper
    zed_aruco_localization
)



# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rcutils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(zed_wrapper REQUIRED)
find_package(zed_aruco_localization REQUIRED)


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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


# rclcpp_components_register_nodes(${PROJECT_NAME}_component "stereolabs::ZedArucoCalibration")
# set(node_plugins "${node_plugins}stereolabs::ZedArucoCalibration;$<TARGET_FILE:${PROJECT_NAME}_component>\n")


# install(
#   FILES zed_aruco_plugin.xml
#   DESTINATION share/${PROJECT_NAME}
# )



# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)
install(
  PROGRAMS
    scripts/interactive_aruco_calibration.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
