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

/* Author: Wim Meeussen */

#include "robot_state_publisher/robot_state_publisher.hpp"

#include <chrono>
#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "kdl/tree.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "rcl_interfaces/msg/parameter_event.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/parameter_events_filter.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "urdf/model.h"

namespace robot_state_publisher
{

namespace
{

inline
geometry_msgs::msg::TransformStamped kdlToTransform(const KDL::Frame & k)
{
  geometry_msgs::msg::TransformStamped t;
  t.transform.translation.x = k.p.x();
  t.transform.translation.y = k.p.y();
  t.transform.translation.z = k.p.z();
  k.M.GetQuaternion(
    t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z,
    t.transform.rotation.w);
  return t;
}

}  // namespace

RobotStatePublisher::RobotStatePublisher(const rclcpp::NodeOptions & options)
: rclcpp::Node("robot_state_publisher", options)
{
  // get the XML
  std::string urdf_xml = this->declare_parameter("robot_description", std::string(""));
  if (urdf_xml.empty()) {
    // If the robot_description is empty, we fall back to looking at the
    // command-line arguments.  Since this is deprecated, we print a warning
    // but continue on.
    try {
      if (options.arguments().size() > 1) {
        RCLCPP_WARN(
          get_logger(),
          "No robot_description parameter, but command-line argument available."
          "  Assuming argument is name of URDF file."
          "  This backwards compatibility fallback will be removed in the future.");
        std::ifstream in(options.arguments()[1], std::ios::in | std::ios::binary);
        if (in) {
          in.seekg(0, std::ios::end);
          urdf_xml.resize(in.tellg());
          in.seekg(0, std::ios::beg);
          in.read(&urdf_xml[0], urdf_xml.size());
          in.close();

          this->set_parameter(rclcpp::Parameter("robot_description", urdf_xml));
        } else {
          throw std::system_error(
                  errno,
                  std::system_category(),
                  "Failed to open URDF file: " + std::string(options.arguments()[1]));
        }
      } else {
        throw std::runtime_error("robot_description parameter must not be empty");
      }
    } catch (const std::runtime_error & err) {
      RCLCPP_FATAL(get_logger(), "%s", err.what());
      throw;
    }
  }

  // set publish frequency
  double publish_freq = this->declare_parameter("publish_frequency", 20.0);
  if (publish_freq < 0.0 || publish_freq > 1000.0) {
    throw std::runtime_error("publish_frequency must be between 0 and 1000");
  }

  // set frame_prefix
  this->declare_parameter("frame_prefix", "");

  // ignore_timestamp_ == true, joint_state messages are accepted, no matter their timestamp
  this->declare_parameter("ignore_timestamp", false);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);

  description_pub_ = this->create_publisher<std_msgs::msg::String>(
    "robot_description",
    // Transient local is similar to latching in ROS 1.
    rclcpp::QoS(1).transient_local());

  setupURDF(urdf_xml);

  auto subscriber_options = rclcpp::SubscriptionOptions();
  subscriber_options.qos_overriding_options =
    rclcpp::QosOverridingOptions::with_default_policies();

  // subscribe to joint state
  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states",
    rclcpp::SensorDataQoS(),
    std::bind(&RobotStatePublisher::callbackJointState, this, std::placeholders::_1),
    subscriber_options);

  publishFixedTransforms();

  // Now that we have successfully declared the parameters and done all
  // necessary setup, install the callback for updating parameters.
  param_cb_ = add_on_set_parameters_callback(
    std::bind(&RobotStatePublisher::parameterUpdate, this, std::placeholders::_1));

  // Now that we have successfully declared the parameters and done all
  // necessary setup, install the callback for updating parameters.
  parameter_subscription_ = rclcpp::AsyncParametersClient::on_parameter_event(
    this->get_node_topics_interface(),
    std::bind(&RobotStatePublisher::onParameterEvent, this, std::placeholders::_1));
}

KDL::Tree RobotStatePublisher::parseURDF(const std::string & urdf_xml, urdf::Model & model)
{
  // Initialize the model
  if (!model.initString(urdf_xml)) {
    throw std::runtime_error("Unable to initialize urdf::model from robot description");
  }

  // Initialize the KDL tree
  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(model, tree)) {
    throw std::runtime_error("Failed to extract kdl tree from robot description");
  }

  return tree;
}

void RobotStatePublisher::setupURDF(const std::string & urdf_xml)
{
  urdf::Model model;
  KDL::Tree tree = parseURDF(urdf_xml, model);

  // Initialize the mimic map
  mimic_.clear();
  for (const std::pair<const std::string, urdf::JointSharedPtr> & i : model.joints_) {
    if (i.second->mimic) {
      mimic_.insert(std::make_pair(i.first, i.second->mimic));
    }
  }

  KDL::SegmentMap segments_map = tree.getSegments();
  for (const std::pair<const std::string, KDL::TreeElement> & segment : segments_map) {
    RCLCPP_INFO(get_logger(), "got segment %s", segment.first.c_str());
  }

  // walk the tree and add segments to segments_
  segments_.clear();
  segments_fixed_.clear();
  addChildren(model, tree.getRootSegment());

  auto msg = std::make_unique<std_msgs::msg::String>();
  msg->data = urdf_xml;

  // Publish the robot description
  description_pub_->publish(std::move(msg));
}

// add children to correct maps
void RobotStatePublisher::addChildren(
  const urdf::Model & model,
  const KDL::SegmentMap::const_iterator segment)
{
  const std::string & root = GetTreeElementSegment(segment->second).getName();

  std::vector<KDL::SegmentMap::const_iterator> children = GetTreeElementChildren(segment->second);
  for (unsigned int i = 0; i < children.size(); i++) {
    const KDL::Segment & child = GetTreeElementSegment(children[i]->second);
    SegmentPair s(GetTreeElementSegment(children[i]->second), root, child.getName());
    if (child.getJoint().getType() == KDL::Joint::None) {
      if (model.getJoint(child.getJoint().getName()) &&
        model.getJoint(child.getJoint().getName())->type == urdf::Joint::FLOATING)
      {
        RCLCPP_INFO(
          get_logger(), "Floating joint. Not adding segment from %s to %s.",
          root.c_str(), child.getName().c_str());
      } else {
        segments_fixed_.insert(make_pair(child.getJoint().getName(), s));
        RCLCPP_DEBUG(
          get_logger(), "Adding fixed segment from %s to %s", root.c_str(),
          child.getName().c_str());
      }
    } else {
      segments_.insert(make_pair(child.getJoint().getName(), s));
      RCLCPP_DEBUG(
        get_logger(), "Adding moving segment from %s to %s", root.c_str(),
        child.getName().c_str());
    }
    addChildren(model, children[i]);
  }
}

// publish moving transforms
void RobotStatePublisher::publishTransforms(
  const std::map<std::string, double> & joint_positions,
  const builtin_interfaces::msg::Time & time)
{
  RCLCPP_DEBUG(get_logger(), "Publishing transforms for moving joints");

  std::string frame_prefix = get_parameter("frame_prefix").get_value<std::string>();

  std::vector<geometry_msgs::msg::TransformStamped> tf_transforms;

  // loop over all joints
  for (const std::pair<const std::string, double> & jnt : joint_positions) {
    std::map<std::string, SegmentPair>::iterator seg = segments_.find(jnt.first);
    if (seg != segments_.end()) {
      geometry_msgs::msg::TransformStamped tf_transform =
        kdlToTransform(seg->second.segment.pose(jnt.second));
      tf_transform.header.stamp = time;
      tf_transform.header.frame_id = frame_prefix + seg->second.root;
      tf_transform.child_frame_id = frame_prefix + seg->second.tip;
      tf_transforms.push_back(tf_transform);
    }
  }
  tf_broadcaster_->sendTransform(tf_transforms);
}

// publish fixed transforms
void RobotStatePublisher::publishFixedTransforms()
{
  RCLCPP_DEBUG(get_logger(), "Publishing transforms for fixed joints");

  std::string frame_prefix = get_parameter("frame_prefix").get_value<std::string>();

  std::vector<geometry_msgs::msg::TransformStamped> tf_transforms;

  // loop over all fixed segments
  rclcpp::Time now = this->now();
  for (const std::pair<const std::string, SegmentPair> & seg : segments_fixed_) {
    geometry_msgs::msg::TransformStamped tf_transform = kdlToTransform(seg.second.segment.pose(0));
    tf_transform.header.stamp = now;

    tf_transform.header.frame_id = frame_prefix + seg.second.root;
    tf_transform.child_frame_id = frame_prefix + seg.second.tip;
    tf_transforms.push_back(tf_transform);
  }
  static_tf_broadcaster_->sendTransform(tf_transforms);
}

void RobotStatePublisher::callbackJointState(
  const sensor_msgs::msg::JointState::ConstSharedPtr state)
{
  if (state->name.size() != state->position.size()) {
    if (state->position.empty()) {
      RCLCPP_WARN(
        get_logger(), "Robot state publisher ignored a JointState message about joint(s) "
        "\"%s\"(,...) whose position member was empty.", state->name[0].c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "Robot state publisher ignored an invalid JointState message");
    }
    return;
  }

  // check if we moved backwards in time (e.g. when playing a bag file)
  rclcpp::Time now = this->now();
  if (last_callback_time_.nanoseconds() > now.nanoseconds()) {
    // force re-publish of joint ransforms
    RCLCPP_WARN(
      get_logger(), "Moved backwards in time, re-publishing joint transforms!");
    last_publish_time_.clear();
  }
  last_callback_time_ = now;

  // determine least recently published joint
  rclcpp::Time last_published = now;
  for (size_t i = 0; i < state->name.size(); i++) {
    rclcpp::Time t(last_publish_time_[state->name[i]]);
    last_published = (t.nanoseconds() < last_published.nanoseconds()) ? t : last_published;
  }
  // note: if a joint was seen for the first time,
  //       then last_published is zero.

  // check if we need to publish
  rclcpp::Time current_time(state->header.stamp);
  double publish_freq = this->get_parameter("publish_frequency").get_value<double>();
  std::chrono::milliseconds publish_interval_ms =
    std::chrono::milliseconds(static_cast<uint64_t>(1000.0 / publish_freq));
  rclcpp::Time max_publish_time = last_published + rclcpp::Duration(publish_interval_ms);
  if (get_parameter("ignore_timestamp").get_value<bool>() ||
    current_time.nanoseconds() >= max_publish_time.nanoseconds())
  {
    // get joint positions from state message
    std::map<std::string, double> joint_positions;
    for (size_t i = 0; i < state->name.size(); i++) {
      joint_positions.insert(std::make_pair(state->name[i], state->position[i]));
    }

    for (const std::pair<const std::string, urdf::JointMimicSharedPtr> & i : mimic_) {
      if (joint_positions.find(i.second->joint_name) != joint_positions.end()) {
        double pos = joint_positions[i.second->joint_name] * i.second->multiplier +
          i.second->offset;
        joint_positions.insert(std::make_pair(i.first, pos));
      }
    }

    publishTransforms(joint_positions, state->header.stamp);

    // store publish time in joint map
    for (size_t i = 0; i < state->name.size(); i++) {
      last_publish_time_[state->name[i]] = state->header.stamp;
    }
  }
}

rcl_interfaces::msg::SetParametersResult RobotStatePublisher::parameterUpdate(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const rclcpp::Parameter & parameter : parameters) {
    if (parameter.get_name() == "robot_description") {
      std::string new_urdf = parameter.as_string();
      // Ensure that it isn't empty
      if (new_urdf.empty()) {
        result.successful = false;
        result.reason = "Empty URDF is not allowed";
        break;
      }

      // And that we can successfully parse it
      try {
        urdf::Model dummy_model;
        parseURDF(new_urdf, dummy_model);
      } catch (const std::runtime_error & err) {
        RCLCPP_WARN(get_logger(), "%s", err.what());
        result.successful = false;
        result.reason = err.what();
        break;
      }
    } else if (parameter.get_name() == "publish_frequency") {
      double publish_freq = parameter.as_double();
      if (publish_freq < 0.0 || publish_freq > 1000.0) {
        result.successful = false;
        result.reason = "publish_frequency must be between 0.0 and 1000.0";
        break;
      }
    }
  }

  return result;
}

void RobotStatePublisher::onParameterEvent(
  std::shared_ptr<const rcl_interfaces::msg::ParameterEvent> event)
{
  // Filter out events from other nodes
  if (event->node != this->get_fully_qualified_name()) {
    return;
  }

  // Filter for 'robot_description' being changed.
  rclcpp::ParameterEventsFilter filter(event, {"robot_description"},
    {rclcpp::ParameterEventsFilter::EventType::CHANGED});
  for (auto & it : filter.get_events()) {
    if (it.second->name == "robot_description") {
      try {
        setupURDF(it.second->value.string_value);
        publishFixedTransforms();
      } catch (const std::runtime_error & err) {
        RCLCPP_WARN(get_logger(), "Failed to parse new URDF: %s", err.what());
      }
    }
  }
}

}  // namespace robot_state_publisher

RCLCPP_COMPONENTS_REGISTER_NODE(robot_state_publisher::RobotStatePublisher)
