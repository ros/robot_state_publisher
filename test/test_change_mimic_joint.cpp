// Copyright (c) 2023, Open Source Robotics Foundation, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"

TEST(test_publisher, test_two_joints)
{
  auto node = rclcpp::Node::make_shared("rsp_test_two_links_change_fixed_joint");

  // OK, now publish a new URDF to the parameter and ensure that the link has been updated
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(
    node,
    "robot_state_publisher");
  unsigned int i;
  for (i = 0; i < 100 && !parameters_client->wait_for_service(std::chrono::milliseconds(100));
    ++i)
  {
    ASSERT_TRUE(rclcpp::ok());
  }

  ASSERT_LT(i, 100u);

  std::string new_robot_description("<robot name='test'><link name='base_link'/></robot>");
  std::vector<rcl_interfaces::msg::SetParametersResult> set_parameters_result =
    parameters_client->set_parameters(
  {
    rclcpp::Parameter("robot_description", new_robot_description)
  }, std::chrono::milliseconds(500));
  ASSERT_EQ(set_parameters_result.size(), 1u);
  ASSERT_TRUE(set_parameters_result[0].successful);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  int res = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return res;
}
