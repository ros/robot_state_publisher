// Copyright (c) 2008, Willow Garage, Inc.
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the copyright holders nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/* Author: Wim Meeussen */

#ifndef ROBOT_STATE_PUBLISHER__ROBOT_STATE_PUBLISHER_HPP_
#define ROBOT_STATE_PUBLISHER__ROBOT_STATE_PUBLISHER_HPP_

#include <builtin_interfaces/msg/time.hpp>
#include <kdl/tree.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <urdf/model.h>

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <vector>

using MimicMap = std::map<std::string, urdf::JointMimicSharedPtr>;

namespace robot_state_publisher
{

class SegmentPair final
{
public:
  explicit SegmentPair(
    const KDL::Segment & p_segment,
    const std::string & p_root,
    const std::string & p_tip)
  : segment(p_segment), root(p_root), tip(p_tip) {}

  KDL::Segment segment;
  std::string root;
  std::string tip;
};

class RobotStatePublisher : public rclcpp::Node
{
public:
  /// Constructor
  explicit RobotStatePublisher(const rclcpp::NodeOptions & options);

protected:
  /** Publish transforms to tf
   * \param joint_positions A map of joint names and joint positions.
   * \param time The time at which the joint positions were recorded
   */
  void publishTransforms(
    const std::map<std::string, double> & joint_positions,
    const builtin_interfaces::msg::Time & time);

  void publishFixedTransforms();
  void addChildren(const KDL::SegmentMap::const_iterator segment);
  void callbackJointState(const sensor_msgs::msg::JointState::SharedPtr state);
  void setupURDF(const std::string & urdf_xml);
  rcl_interfaces::msg::SetParametersResult parameterUpdate(
    const std::vector<rclcpp::Parameter> & parameters);

  std::map<std::string, SegmentPair> segments_;
  std::map<std::string, SegmentPair> segments_fixed_;
  std::unique_ptr<urdf::Model> model_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr description_pub_;
  std::chrono::milliseconds publish_interval_ms_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_callback_time_;
  std::map<std::string, builtin_interfaces::msg::Time> last_publish_time_;
  MimicMap mimic_;
  bool use_tf_static_;
  bool ignore_timestamp_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

}  // namespace robot_state_publisher

#endif  // ROBOT_STATE_PUBLISHER__ROBOT_STATE_PUBLISHER_HPP_
