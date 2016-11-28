/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2016, Willow Garage, Inc.
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

/* Author: Yuki Furuta */

#include <cmath>
#include <string>
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <sensor_msgs/JointState.h>
#include <robot_state_publisher/GetRobotTransforms.h>



using namespace ros;
using namespace robot_state_publisher;


#define EPS 0.01

TEST(TestService, test)
{
  {
    ros::NodeHandle n_tilde;
    std::string robot_description;
    ASSERT_TRUE(n_tilde.getParam("robot_description", robot_description));
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<GetRobotTransforms>("get_transforms");
  ASSERT_TRUE(client.waitForExistence());

  GetRobotTransforms srv;
  sensor_msgs::JointState js_msg;
  js_msg.name.push_back("joint1");
  js_msg.position.push_back(M_PI);
  srv.request.joint_states = js_msg;

  ASSERT_TRUE(client.call(srv));

  EXPECT_EQ(srv.response.transforms.size(), 1);

  EXPECT_EQ(srv.response.transforms[0].header.frame_id, "link1");
  EXPECT_EQ(srv.response.transforms[0].child_frame_id, "link2");

  EXPECT_NEAR(srv.response.transforms[0].transform.translation.x, 5.0, EPS);
  EXPECT_NEAR(srv.response.transforms[0].transform.translation.y, 0.0, EPS);
  EXPECT_NEAR(srv.response.transforms[0].transform.translation.z, 0.0, EPS);

  SUCCEED();
}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_service");
  return RUN_ALL_TESTS();
}
