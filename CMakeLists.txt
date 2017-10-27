cmake_minimum_required(VERSION 2.8)
project(robot_state_publisher)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")

find_package(orocos_kdl REQUIRED)
find_package(catkin REQUIRED
  COMPONENTS roscpp rosconsole rostime tf tf2_ros tf2_kdl kdl_parser
)
find_package(Eigen3 REQUIRED)

find_package(urdfdom_headers REQUIRED)

catkin_package(
  LIBRARIES ${PROJECT_NAME}_solver
  INCLUDE_DIRS include
  DEPENDS roscpp rosconsole rostime tf2_ros tf2_kdl kdl_parser orocos_kdl urdfdom_headers
)

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS} ${urdfdom_headers_INCLUDE_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS})

add_library(${PROJECT_NAME}_solver
  src/robot_state_publisher.cpp src/treefksolverposfull_recursive.cpp
)
target_link_libraries(${PROJECT_NAME}_solver ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_library(joint_state_listener src/joint_state_listener.cpp)
target_link_libraries(joint_state_listener ${PROJECT_NAME}_solver ${orocos_kdl_LIBRARIES})

add_executable(${PROJECT_NAME} src/joint_state_listener.cpp)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_solver ${orocos_kdl_LIBRARIES})

# compile the same executable using the old name as well
add_executable(state_publisher src/joint_state_listener.cpp)
target_link_libraries(state_publisher ${PROJECT_NAME}_solver ${orocos_kdl_LIBRARIES})

# Tests

if (CATKIN_ENABLE_TESTING)

  find_package(rostest REQUIRED)

  add_rostest_gtest(test_one_link ${CMAKE_CURRENT_SOURCE_DIR}/test/test_one_link.launch test/test_one_link.cpp)
  target_link_libraries(test_one_link ${catkin_LIBRARIES} ${PROJECT_NAME}_solver)

  add_rostest_gtest(test_two_links_fixed_joint ${CMAKE_CURRENT_SOURCE_DIR}/test/test_two_links_fixed_joint.launch test/test_two_links_fixed_joint.cpp)
  target_link_libraries(test_two_links_fixed_joint ${catkin_LIBRARIES} ${PROJECT_NAME}_solver)

  add_rostest_gtest(test_two_links_moving_joint ${CMAKE_CURRENT_SOURCE_DIR}/test/test_two_links_moving_joint.launch test/test_two_links_moving_joint.cpp)
  target_link_libraries(test_two_links_moving_joint ${catkin_LIBRARIES} ${PROJECT_NAME}_solver)

   # Download needed data file
  catkin_download_test_data(
    joint_states_indexed_bag
    http://wiki.ros.org/robot_state_publisher/data?action=AttachFile&do=get&target=joint_states_indexed.bag
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    FILENAME joint_states_indexed.bag
    MD5 793e0b566ebe4698265a936b92fa2bba)

  add_rostest_gtest(test_joint_states_bag ${CMAKE_CURRENT_SOURCE_DIR}/test/test_joint_states_bag.launch test/test_joint_states_bag.cpp)
  target_link_libraries(test_joint_states_bag ${catkin_LIBRARIES} ${PROJECT_NAME}_solver)

  add_rostest_gtest(test_subclass ${CMAKE_CURRENT_SOURCE_DIR}/test/test_subclass.launch test/test_subclass.cpp)
  target_link_libraries(test_subclass ${catkin_LIBRARIES} ${PROJECT_NAME}_solver joint_state_listener)

  install(FILES test/one_link.urdf test/pr2.urdf test/two_links_fixed_joint.urdf test/two_links_moving_joint.urdf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test)

endif()

install(TARGETS ${PROJECT_NAME}_solver joint_state_listener ${PROJECT_NAME} state_publisher
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
