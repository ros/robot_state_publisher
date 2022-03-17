cmake_minimum_required(VERSION 3.5)
project(robot_state_publisher)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  builtin_interfaces
  geometry_msgs
  kdl_parser
  orocos_kdl
  rcl_interfaces
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  tf2_ros
  urdf)

add_library(
  ${PROJECT_NAME}_node SHARED
  src/robot_state_publisher.cpp)
target_include_directories(${PROJECT_NAME}_node PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
ament_target_dependencies(${PROJECT_NAME}_node ${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_targets(export_${PROJECT_NAME}_node)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "robot_state_publisher::RobotStatePublisher"
  EXECUTABLE robot_state_publisher)

install(
  TARGETS
  ${PROJECT_NAME}_node
  EXPORT export_${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

install(DIRECTORY launch urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_find_gtest()

  add_executable(test_two_links_fixed_joint test/test_two_links_fixed_joint.cpp)
  ament_target_dependencies(test_two_links_fixed_joint
    rclcpp
    tf2_ros
  )
  target_include_directories(test_two_links_fixed_joint PRIVATE ${GTEST_INCLUDE_DIRS})
  target_link_libraries(test_two_links_fixed_joint ${GTEST_LIBRARIES})
  add_launch_test(test/two_links_fixed_joint-launch.py
    ARGS "test_exe:=$<TARGET_FILE:test_two_links_fixed_joint>")

  add_executable(test_two_links_fixed_joint_prefix test/test_two_links_fixed_joint_prefix.cpp)
  ament_target_dependencies(test_two_links_fixed_joint_prefix
    rclcpp
    tf2_ros
  )
  target_include_directories(test_two_links_fixed_joint PRIVATE ${GTEST_INCLUDE_DIRS})
  target_link_libraries(test_two_links_fixed_joint_prefix ${GTEST_LIBRARIES})
  add_launch_test(test/two_links_fixed_joint_prefix-launch.py
    ARGS "test_exe:=$<TARGET_FILE:test_two_links_fixed_joint_prefix>")

  add_executable(test_two_links_moving_joint test/test_two_links_moving_joint.cpp)
  ament_target_dependencies(test_two_links_moving_joint
    rclcpp
    sensor_msgs
    tf2_ros
  )
  target_include_directories(test_two_links_fixed_joint PRIVATE ${GTEST_INCLUDE_DIRS})
  target_link_libraries(test_two_links_moving_joint ${GTEST_LIBRARIES})
  add_launch_test(test/two_links_moving_joint-launch.py
    ARGS "test_exe:=$<TARGET_FILE:test_two_links_moving_joint>")

  add_executable(test_two_links_change_fixed_joint test/test_two_links_change_fixed_joint.cpp)
  ament_target_dependencies(test_two_links_change_fixed_joint
    rclcpp
    tf2_ros
  )
  target_include_directories(test_two_links_change_fixed_joint PRIVATE ${GTEST_INCLUDE_DIRS})
  target_link_libraries(test_two_links_change_fixed_joint ${GTEST_LIBRARIES})
  add_launch_test(test/two_links_change_fixed_joint-launch.py
    ARGS "test_exe:=$<TARGET_FILE:test_two_links_change_fixed_joint>")
endif()

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
