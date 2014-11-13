cmake_minimum_required(VERSION 2.8)
project(robot_state_publisher)

find_package(orocos_kdl REQUIRED) 
find_package(catkin REQUIRED
  COMPONENTS roscpp rosconsole rostime tf tf_conversions kdl_parser cmake_modules
)
find_package(Eigen REQUIRED)

catkin_package(
  LIBRARIES ${PROJECT_NAME}_solver
  INCLUDE_DIRS include
  DEPENDS roscpp rosconsole rostime tf tf_conversions kdl_parser orocos_kdl
)

include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS})

add_library(${PROJECT_NAME}_solver
  src/robot_state_publisher.cpp src/treefksolverposfull_recursive.cpp
)
target_link_libraries(${PROJECT_NAME}_solver ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_executable(${PROJECT_NAME} src/joint_state_listener.cpp)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_solver ${orocos_kdl_LIBRARIES})

# compile the same executable using the old name as well
add_executable(state_publisher src/joint_state_listener.cpp)
target_link_libraries(state_publisher ${PROJECT_NAME}_solver ${orocos_kdl_LIBRARIES})

# CATKIN has no ROS test for now. Disabling

#rosbuild_add_executable(test_publisher test/test_publisher.cpp )
#target_link_libraries(test_publisher ${PROJECT_NAME}_solver)
#rosbuild_add_gtest_build_flags(test_publisher)
#rosbuild_declare_test(test_publisher)
#rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_publisher.launch)

#rosbuild_add_executable(test_one_link test/test_one_link.cpp )
#target_link_libraries(test_one_link ${PROJECT_NAME}_solver)
#rosbuild_add_gtest_build_flags(test_one_link)
#rosbuild_declare_test(test_one_link)
#rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_one_link.launch)

#rosbuild_add_executable(test_two_links_fixed_joint test/test_two_links_fixed_joint.cpp )
#target_link_libraries(test_two_links_fixed_joint ${PROJECT_NAME}_solver)
#rosbuild_add_gtest_build_flags(test_two_links_fixed_joint)
#rosbuild_declare_test(test_two_links_fixed_joint)
#rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_two_links_fixed_joint.launch)

#rosbuild_add_executable(test_two_links_moving_joint test/test_two_links_moving_joint.cpp )
#target_link_libraries(test_two_links_moving_joint ${PROJECT_NAME}_solver)
#rosbuild_add_gtest_build_flags(test_two_links_moving_joint)
#rosbuild_declare_test(test_two_links_moving_joint)
#rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_two_links_moving_joint.launch)

# Download needed data file
#rosbuild_download_test_data(http://pr.willowgarage.com/data/robot_state_publisher/joint_states_indexed.bag test/joint_states_indexed.bag 793e0b566ebe4698265a936b92fa2bba)

install(TARGETS ${PROJECT_NAME}_solver ${PROJECT_NAME} state_publisher
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
