/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Open Source Robotics Foundation, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <utility>

#include <gtest/gtest.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include <kdl_parser/kdl_parser.hpp>
#include "robot_state_publisher/joint_state_listener.h"

TEST(TestRobotStatePubInit, default_constructor)
{
  robot_state_publisher::JointStateListener robot_state_pub;
  // Test delayed initialization
  // Get the robot description and parse the kdl tree
  urdf::Model model;
  model.initParam("robot_description");
  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(model, tree)){
    ROS_ERROR("Failed to extract kdl tree from xml robot description");
    FAIL();
  }
  ASSERT_EQ(model.links_.size(), 2);
  ASSERT_EQ(model.joints_.size(), 1);

  MimicMap mimic;
  robot_state_publisher::JointStateListener::constructMimicMap(mimic, model);
  robot_state_pub.init(tree, mimic, model);
  ASSERT_EQ(model.links_.size(), 2);
  ASSERT_EQ(model.joints_.size(), 1);

  tf2_ros::Buffer buffer;
  tf2_ros::TransformListener listener(buffer);

  // Test that the model has the two links specified
  ros::NodeHandle n;
  ros::Publisher js_pub = n.advertise<sensor_msgs::JointState>("joint_states", 10);
  sensor_msgs::JointState js_msg;
  js_msg.name.push_back("joint1");
  js_msg.position.push_back(M_PI);

  js_msg.header.stamp = ros::Time::now();
  js_pub.publish(js_msg);

  //for (unsigned int i=0; i<100 && !buffer.canTransform("link1", "link2", ros::Time()); i++){
  while (!buffer.canTransform("link1", "link2", ros::Time())){
    ros::Duration(0.1).sleep();
    ros::spinOnce();
  }

  ASSERT_TRUE(buffer.canTransform("link1", "link2", ros::Time()));
  ASSERT_FALSE(buffer.canTransform("base_link", "wim_link", ros::Time()));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_init");
  testing::InitGoogleTest(&argc, argv);

  int res = RUN_ALL_TESTS();

  return res;
}
