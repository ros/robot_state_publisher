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
#include <tf/transform_listener.h>
#include <boost/thread/thread.hpp>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include "robot_state_publisher/joint_state_listener.h"


using namespace ros;
using namespace tf;


int g_argc;
char** g_argv;

TEST(FloatingJoint, test)
{
  ROS_INFO("Creating tf listener");
  const double TF_START_DELAY = 1.0;
  const double WAIT_DELAY = 0.1;
  TransformListener tf;
  ros::NodeHandle pn("~");
  bool pub_float_as_fixed = false;
  ASSERT_TRUE(pn.getParam("pub_float_as_fixed", pub_float_as_fixed));

  //sleep is required to seed tf (anything less than 1 second results in test failure
  ros::Duration(TF_START_DELAY).sleep();

  if (pub_float_as_fixed)
  {
    ROS_INFO("Testing for publishing floats as fixed");
    ASSERT_TRUE(tf.frameExists("world"));
    ASSERT_TRUE(tf.frameExists("floating"));
    ASSERT_TRUE(tf.waitForTransform("world", "floating", ros::Time::now(), ros::Duration(WAIT_DELAY)));
  }
  else
  {
    ROS_INFO("Testing for NOT publishing floats as fixed");
    ASSERT_FALSE(tf.waitForTransform("world", "floating", ros::Time::now(), ros::Duration(WAIT_DELAY)));
  }
  SUCCEED();
}




int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_robot_state_publisher");
  ros::NodeHandle node;
  boost::thread ros_thread(boost::bind(&ros::spin));

  g_argc = argc;
  g_argv = argv;
  int res = RUN_ALL_TESTS();
  ros_thread.interrupt();
  ros_thread.join();

  return res;
}
