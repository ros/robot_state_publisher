/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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
*********************************************************************/

/* Author: Wim Meeussen */

#include <string>

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <boost/thread/thread.hpp>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

#include "robot_state_publisher/joint_state_listener.h"

using namespace ros;
using namespace tf2_ros;
using namespace robot_state_publisher;

#define EPS 0.01

class TestPublisher : public testing::Test
{
public:
  JointStateListener* publisher;

protected:
  /// constructor
  TestPublisher()
  {}

  /// Destructor
  ~TestPublisher()
  {}
};

TEST_F(TestPublisher, test)
{
  {
    ros::NodeHandle n_tilde;
    std::string robot_description;
    ASSERT_TRUE(n_tilde.getParam("robot_description", robot_description));
  }

  ROS_INFO("Creating tf listener");
  Buffer buffer;
  TransformListener listener(buffer);

  ROS_INFO("Publishing joint state to robot state publisher");
  ros::NodeHandle n;
  ros::Publisher js_pub = n.advertise<sensor_msgs::JointState>("joint_states", 10);
  sensor_msgs::JointState js_msg;
  js_msg.name.push_back("joint1");
  js_msg.position.push_back(M_PI);
  ros::Duration(1).sleep();
  for (unsigned int i = 0; i < 100; i++) {
    js_msg.header.stamp = ros::Time::now();
    js_pub.publish(js_msg);
    ros::Duration(0.1).sleep();
  }

  for (unsigned int i = 0; i < 100 && !buffer.canTransform("link1", "link2", Time()); i++) {
    ros::Duration(0.1).sleep();
    ros::spinOnce();
  }
  EXPECT_FALSE(buffer.canTransform("base_link", "wim_link", Time()));
  ASSERT_TRUE(buffer.canTransform("link1", "link2", Time()));

  geometry_msgs::TransformStamped t = buffer.lookupTransform("link1", "link2", Time());
  EXPECT_NEAR(t.transform.translation.x, 5.0, EPS);
  EXPECT_NEAR(t.transform.translation.y, 0.0, EPS);
  EXPECT_NEAR(t.transform.translation.z, 0.0, EPS);

  SUCCEED();
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_two_links_moving_joint");
  ros::NodeHandle node;

  int res = RUN_ALL_TESTS();

  return res;
}
