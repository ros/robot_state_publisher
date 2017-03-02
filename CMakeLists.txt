cmake_minimum_required(VERSION 3.5)
project(robot_state_publisher)
if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra")
endif()

# necessary for now for finding tinyxml
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
#find_package(TinyXML REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(urdf REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
link_directories(${orocos_kdl_LIBRARY_DIRS})

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})
include_directories(
  include
  ${geometry_msgs_INCLUDE_DIRS}
  ${kdl_parser_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${urdf_INCLUDE_DIRS}
  ${urdfdom_headers_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_msgs_INCLUDE_DIRS})

 add_library(${PROJECT_NAME}_solver
   SHARED
   src/robot_state_publisher.cpp
   #src/treefksolverposfull_recursive.cpp
   )
 target_link_libraries(${PROJECT_NAME}_solver
   ${geometry_msgs_LIBRARIES}
   ${kdl_parser_LIBRARIES}
   ${urdf_LIBRARIES}
   ${urdfdom_LIBRARIES}
   ${sensor_msgs_LIBRARIES}
   ${tf2_ros_LIBRARIES}
   ${tf2_msgs_LIBRARIES}
   ${orocos_kdl_LIBRARIES})

  add_library(joint_state_listener src/joint_state_listener.cpp)
  target_link_libraries(joint_state_listener ${PROJECT_NAME}_solver ${orocos_kdl_LIBRARIES})

  add_executable(${PROJECT_NAME} src/joint_state_listener.cpp)
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

 install(TARGETS
   ${PROJECT_NAME}_solver
   joint_state_listener
   ${PROJECT_NAME}
   ARCHIVE DESTINATION lib
   LIBRARY DESTINATION lib
   RUNTIME DESTINATION bin
 )

ament_export_libraries(${PROJECT_NAME}_solver joint_state_listener ${PROJECT_NAME})
ament_package()
