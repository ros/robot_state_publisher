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

#include <map>
#include <string>
#include <vector>
#include <utility>

#include "kdl/frames_io.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "robot_state_publisher/robot_state_publisher.h"

namespace robot_state_publisher
{

inline
KDL::Frame transformToKDL(const geometry_msgs::msg::TransformStamped & t)
{
  return KDL::Frame(KDL::Rotation::Quaternion(t.transform.rotation.x, t.transform.rotation.y,
           t.transform.rotation.z, t.transform.rotation.w),
           KDL::Vector(t.transform.translation.x, t.transform.translation.y,
           t.transform.translation.z));
}

inline
geometry_msgs::msg::TransformStamped kdlToTransform(const KDL::Frame & k)
{
  geometry_msgs::msg::TransformStamped t;
  t.transform.translation.x = k.p.x();
  t.transform.translation.y = k.p.y();
  t.transform.translation.z = k.p.z();
  k.M.GetQuaternion(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z,
    t.transform.rotation.w);
  return t;
}

RobotStatePublisher::RobotStatePublisher(
  rclcpp::Node::SharedPtr node_handle,
  const KDL::Tree & tree,
  const urdf::Model & model,
  const std::string & model_xml)
: model_(model),
  tf_broadcaster_(node_handle),
  static_tf_broadcaster_(node_handle),
  description_published_(false)
{
  // walk the tree and add segments to segments_
  addChildren(tree.getRootSegment());

  // Creates a latched topic
  rmw_qos_profile_t qos = rmw_qos_profile_default;
  qos.depth = 1;
  qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;

  model_xml_.data = model_xml;
  description_pub_ = node_handle->create_publisher<std_msgs::msg::String>(
    "robot_description", qos);
}

// add children to correct maps
void RobotStatePublisher::addChildren(const KDL::SegmentMap::const_iterator segment)
{
  auto root = GetTreeElementSegment(segment->second).getName();

  auto children = GetTreeElementChildren(segment->second);
  for (unsigned int i = 0; i < children.size(); i++) {
    const KDL::Segment & child = GetTreeElementSegment(children[i]->second);
    SegmentPair s(GetTreeElementSegment(children[i]->second), root, child.getName());
    if (child.getJoint().getType() == KDL::Joint::None) {
      if (model_.getJoint(child.getJoint().getName()) &&
        model_.getJoint(child.getJoint().getName())->type == urdf::Joint::FLOATING)
      {
        fprintf(stderr,
          "Floating joint. Not adding segment from %s to %s.\n",
          root.c_str(), child.getName().c_str());
      } else {
        segments_fixed_.insert(make_pair(child.getJoint().getName(), s));
        fprintf(stderr, "Adding fixed segment from %s to %s\n", root.c_str(),
          child.getName().c_str());
      }
    } else {
      segments_.insert(make_pair(child.getJoint().getName(), s));
      fprintf(stderr, "Adding moving segment from %s to %s\n", root.c_str(),
        child.getName().c_str());
    }
    addChildren(children[i]);
  }
}


// publish moving transforms
void RobotStatePublisher::publishTransforms(
  const std::map<std::string, double> & joint_positions,
  const builtin_interfaces::msg::Time & time,
  const std::string & tf_prefix)
{
  // fprintf(stderr, "Publishing transforms for moving joints\n");
  std::vector<geometry_msgs::msg::TransformStamped> tf_transforms;

  // loop over all joints
  for (auto jnt = joint_positions.begin(); jnt != joint_positions.end(); ++jnt) {
    auto seg = segments_.find(jnt->first);
    if (seg != segments_.end()) {
      geometry_msgs::msg::TransformStamped tf_transform =
        kdlToTransform(seg->second.segment.pose(jnt->second));
      tf_transform.header.stamp = time;
      tf_transform.header.frame_id = tf_prefix + "/" + seg->second.root;
      tf_transform.child_frame_id = tf_prefix + "/" + seg->second.tip;
      tf_transforms.push_back(tf_transform);
    }
  }
  tf_broadcaster_.sendTransform(tf_transforms);

  // Publish the robot description, as necessary
  if (!description_published_) {
    description_pub_->publish(model_xml_);
    description_published_ = true;
  }
}

// publish fixed transforms
void RobotStatePublisher::publishFixedTransforms(const std::string & tf_prefix, bool use_tf_static)
{
  // fprintf(stderr, "Publishing transforms for fixed joints\n");
  std::vector<geometry_msgs::msg::TransformStamped> tf_transforms;
  geometry_msgs::msg::TransformStamped tf_transform;

  // loop over all fixed segments
  for (auto seg = segments_fixed_.begin(); seg != segments_fixed_.end(); seg++) {
    geometry_msgs::msg::TransformStamped tf_transform = kdlToTransform(seg->second.segment.pose(0));
    std::chrono::nanoseconds now = std::chrono::high_resolution_clock::now().time_since_epoch();
    if (!use_tf_static) {
      now += std::chrono::milliseconds(500);  // 0.5sec in NS
    }
    tf_transform.header.stamp.sec =
      static_cast<builtin_interfaces::msg::Time::_sec_type>(now.count() / 1000000000);
    tf_transform.header.stamp.nanosec = now.count() % 1000000000;

    tf_transform.header.frame_id = tf_prefix + "/" + seg->second.root;
    tf_transform.child_frame_id = tf_prefix + "/" + seg->second.tip;
    tf_transforms.push_back(tf_transform);
  }
  if (use_tf_static) {
    static_tf_broadcaster_.sendTransform(tf_transforms);
  } else {
    tf_broadcaster_.sendTransform(tf_transforms);
  }
}

}  // namespace robot_state_publisher
