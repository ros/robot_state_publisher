cmake_minimum_required(VERSION 2.8)
project(robot_state_publisher)

find_package(Eigen REQUIRED)

find_package(catkin REQUIRED COMPONENTS 
  roscpp rosconsole rostime tf tf_conversions kdl_parser rostest)

catkin_package(
  LIBRARIES ${PROJECT_NAME}_solver
  INCLUDE_DIRS include
  DEPENDS roscpp rosconsole rostime tf tf_conversions kdl_parser)

include_directories(include)

find_package(orocos_kdl REQUIRED)
include_directories(${orocos_kdl_INCLUDE_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS})

include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_solver 
  src/robot_state_publisher.cpp src/treefksolverposfull_recursive.cpp
)
target_link_libraries(${PROJECT_NAME}_solver ${orocos_kdl_LIBRARIES} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME} src/joint_state_listener.cpp)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_solver log4cxx)

# compile the same executable using the old name as well
add_executable(state_publisher src/joint_state_listener.cpp)
target_link_libraries(state_publisher ${PROJECT_NAME}_solver log4cxx)

# Tests

catkin_add_gtest(test_publisher test/test_publisher.cpp)
target_link_libraries(test_publisher ${catkin_LIBRARIES} ${PROJECT_NAME}_solver)
add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_publisher.launch)

catkin_add_gtest(test_one_link test/test_one_link.cpp )
target_link_libraries(test_one_link ${catkin_LIBRARIES} ${PROJECT_NAME}_solver)
add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_one_link.launch)

catkin_add_gtest(test_two_links_fixed_joint test/test_two_links_fixed_joint.cpp )
target_link_libraries(test_two_links_fixed_joint ${catkin_LIBRARIES} ${PROJECT_NAME}_solver)
add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_two_links_fixed_joint.launch)

catkin_add_gtest(test_two_links_moving_joint test/test_two_links_moving_joint.cpp )
target_link_libraries(test_two_links_moving_joint ${catkin_LIBRARIES} ${PROJECT_NAME}_solver)
add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_two_links_moving_joint.launch)

# Download needed data file
catkin_download_test_data(
  joint_states_indexed_bag
  http://wiki.ros.org/robot_state_publisher/data?action=AttachFile&do=get&target=joint_states_indexed.bag
  DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
  FILENAME joint_states_indexed.bag 
  MD5 793e0b566ebe4698265a936b92fa2bba)

install(TARGETS ${PROJECT_NAME}_solver ${PROJECT_NAME} state_publisher
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
