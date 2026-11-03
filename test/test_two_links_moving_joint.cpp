// Copyright (c) 2008, Willow Garage, Inc.
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

#ifdef _MSC_VER
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#endif

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <thread>

#include "gtest/gtest.h"

#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/utilities.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

static constexpr double EPS = 0.01;

TEST(test_publisher, test_two_joints)
{
  auto node = rclcpp::Node::make_shared("rsp_test_two_links_moving_joint");

  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  tf2_ros::Buffer buffer(clock);
  tf2_ros::TransformListener tfl(buffer, *node, true);

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub =
    node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  sensor_msgs::msg::JointState js_msg;
  js_msg.name.push_back("joint1");
  js_msg.position.push_back(M_PI);
  js_msg.header.stamp = node->now();

  for (unsigned int i = 0; i < 100 && !buffer.canTransform("link1", "link2", rclcpp::Time()); ++i) {
    pub->publish(js_msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  ASSERT_TRUE(buffer.canTransform("link1", "link2", rclcpp::Time()));
  ASSERT_FALSE(buffer.canTransform("base_link", "wim_link", rclcpp::Time()));

  geometry_msgs::msg::TransformStamped t = buffer.lookupTransform("link1", "link2", rclcpp::Time());
  EXPECT_NEAR(t.transform.translation.x, 5.0, EPS);
  EXPECT_NEAR(t.transform.translation.y, 0.0, EPS);
  EXPECT_NEAR(t.transform.translation.z, 0.0, EPS);
}

TEST(test_publisher, test_nan_joint_is_skipped)
{
  auto node = rclcpp::Node::make_shared("rsp_test_nan_joint");

  // Watch /tf directly: tf2 rejects a NaN transform on insertion, so a buffer
  // query could not tell a skipped joint from a published-and-rejected one.
  const auto is_finite = [](const geometry_msgs::msg::Transform & t) {
      return std::isfinite(t.translation.x) && std::isfinite(t.translation.y) &&
             std::isfinite(t.translation.z) && std::isfinite(t.rotation.x) &&
             std::isfinite(t.rotation.y) && std::isfinite(t.rotation.z) &&
             std::isfinite(t.rotation.w);
    };
  unsigned int valid_count = 0;
  unsigned int nan_count = 0;
  auto sub = node->create_subscription<tf2_msgs::msg::TFMessage>(
    "tf", 10, [&](const tf2_msgs::msg::TFMessage & msg) {
      for (const auto & transform : msg.transforms) {
        if (transform.child_frame_id != "link2") {
          continue;
        }
        if (is_finite(transform.transform)) {
          ++valid_count;
        } else {
          ++nan_count;
        }
      }
    });
  auto pub = node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

  sensor_msgs::msg::JointState js_msg;
  js_msg.name.push_back("joint1");
  js_msg.position.push_back(M_PI);

  // A valid position first, until its transform arrives, so the NaN check
  // below cannot pass just because nothing was connected yet.
  for (unsigned int i = 0; i < 100 && valid_count == 0; ++i) {
    js_msg.header.stamp = node->now();
    pub->publish(js_msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    rclcpp::spin_some(node);
  }
  ASSERT_GT(valid_count, 0u);

  // A NaN position must not be turned into a transform.
  js_msg.position[0] = std::numeric_limits<double>::quiet_NaN();
  for (unsigned int i = 0; i < 10; ++i) {
    js_msg.header.stamp = node->now();
    pub->publish(js_msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    rclcpp::spin_some(node);
  }
  EXPECT_EQ(nan_count, 0u);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  int res = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return res;
}
