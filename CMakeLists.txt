cmake_minimum_required(VERSION 2.8)
project(robot_state_publisher)

find_package(catkin REQUIRED)

catkin_stack()

catkin_project(${PROJECT_NAME} 
  LIBRARIES robot_state_publisher_solver
  INCLUDE_DIRS include)

include_directories(include)

find_package(Orocos-KDL REQUIRED)
include_directories(${Orocos-KDL_INCLUDE_DIRS})
link_directories(${Orocos-KDL_LIBRARY_DIRS})

find_package(ROS REQUIRED COMPONENTS roscpp rosconsole rostime tf tf_conversions kdl_parser)
include_directories(${ROS_INCLUDE_DIRS})

find_package(Eigen REQUIRED)
include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS})

add_library(robot_state_publisher_solver src/robot_state_publisher.cpp src/treefksolverposfull_recursive.cpp)
target_link_libraries(robot_state_publisher_solver ${Orocos-KDL_LIBRARIES} ${ROS_LIBRARIES})

add_executable(robot_state_publisher src/joint_state_listener.cpp )
target_link_libraries(robot_state_publisher robot_state_publisher_solver)

# CATKIN has no ROS test for now. Disabling

#rosbuild_add_executable(test_publisher test/test_publisher.cpp )
#target_link_libraries(test_publisher robot_state_publisher_solver)
#rosbuild_add_gtest_build_flags(test_publisher)
#rosbuild_declare_test(test_publisher)
#rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_publisher.launch)

#rosbuild_add_executable(test_one_link test/test_one_link.cpp )
#target_link_libraries(test_one_link robot_state_publisher_solver)
#rosbuild_add_gtest_build_flags(test_one_link)
#rosbuild_declare_test(test_one_link)
#rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_one_link.launch)

#rosbuild_add_executable(test_two_links_fixed_joint test/test_two_links_fixed_joint.cpp )
#target_link_libraries(test_two_links_fixed_joint robot_state_publisher_solver)
#rosbuild_add_gtest_build_flags(test_two_links_fixed_joint)
#rosbuild_declare_test(test_two_links_fixed_joint)
#rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_two_links_fixed_joint.launch)

#rosbuild_add_executable(test_two_links_moving_joint test/test_two_links_moving_joint.cpp )
#target_link_libraries(test_two_links_moving_joint robot_state_publisher_solver)
#rosbuild_add_gtest_build_flags(test_two_links_moving_joint)
#rosbuild_declare_test(test_two_links_moving_joint)
#rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_two_links_moving_joint.launch)

# Download needed data file
#rosbuild_download_test_data(http://pr.willowgarage.com/data/robot_state_publisher/joint_states_indexed.bag test/joint_states_indexed.bag 793e0b566ebe4698265a936b92fa2bba)

install(TARGETS robot_state_publisher_solver robot_state_publisher
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/
  DESTINATION include)

# for compatibility with old build system
install(FILES manifest.xml
        DESTINATION share/${PROJECT_NAME})
