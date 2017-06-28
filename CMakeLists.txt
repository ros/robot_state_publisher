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
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(urdf REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(SYSTEM ${Eigen_INCLUDE_DIRS})
include_directories(
  include
  ${geometry_msgs_INCLUDE_DIRS}
  ${kdl_parser_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${urdf_INCLUDE_DIRS}
  ${urdfdom_headers_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS})

add_library(
  ${PROJECT_NAME}_solver
  src/robot_state_publisher.cpp)
link_directories(${orocos_kdl_LIBRARY_DIRS})
target_link_libraries(
  ${PROJECT_NAME}_solver
  ${geometry_msgs_LIBRARIES}
  ${kdl_parser_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  ${urdf_LIBRARIES}
  ${urdfdom_LIBRARIES})

add_library(
  joint_state_listener
  src/joint_state_listener.cpp)
target_link_libraries(
  joint_state_listener
  ${PROJECT_NAME}_solver
  ${orocos_kdl_LIBRARIES})

add_executable(
  ${PROJECT_NAME}
  src/joint_state_listener.cpp)
target_link_libraries(
  ${PROJECT_NAME}
  joint_state_listener
  ${orocos_kdl_LIBRARIES}
  ${kdl_parser_LIBRARIES}
  ${tinyxml_LIBRARIES}
  ${urdf_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  ${sensor_msgs_LIBRARIES})

install(
  TARGETS
  ${PROJECT_NAME}_solver
  joint_state_listener
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  # This forces cppcheck to consider all files in this project to be C++,
  # including the headers which end with .h, which cppcheck would normally
  # consider to be C instead.
  ament_cppcheck(LANGUAGE "c++")
  ament_cpplint()
  ament_lint_cmake()
  ament_uncrustify()
endif()

ament_export_libraries(
  ${PROJECT_NAME}_solver
  joint_state_listener
  ${PROJECT_NAME})
ament_package()
