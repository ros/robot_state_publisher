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

#include <chrono>
#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <utility>

#include "kdl/tree.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "urdf/model.h"

#include "rclcpp/rclcpp.hpp"

#include "robot_state_publisher/joint_state_listener.h"
#include "robot_state_publisher/robot_state_publisher.h"

using namespace std::chrono_literals;

namespace robot_state_publisher
{

JointStateListener::JointStateListener(
  rclcpp::Node::SharedPtr node, const KDL::Tree & tree,
  const MimicMap & m, const std::string & urdf_xml, const urdf::Model & model)
: node_(node),
  state_publisher_(node, tree, model, urdf_xml),
  mimic_(m)
{
  /*
   * legacy code
  // set publish frequency
  double publish_freq;
  n_tilde.param("publish_frequency", publish_freq, 50.0);
  // set whether to use the /tf_static latched static transform broadcaster
  n_tilde.param("use_tf_static", use_tf_static_, true);
  // ignore_timestamp_ == true, joins_states messages are accepted, no matter their timestamp
  n_tilde.param("ignore_timestamp", ignore_timestamp_, false);
  // get the tf_prefix parameter from the closest namespace
  std::string tf_prefix_key;
  n_tilde.searchParam("tf_prefix", tf_prefix_key);
  n_tilde.param(tf_prefix_key, tf_prefix_, std::string(""));
  publish_interval_ = ros::Duration(1.0/max(publish_freq, 1.0));
  */

  use_tf_static_ = true;
  ignore_timestamp_ = false;
  tf_prefix_ = "";
  // auto publish_freq = 50.0;
  // publish_interval_ = std::chrono::seconds(1.0/std::max(publish_freq, 1.0));
  publish_interval_ = 1s;

  // subscribe to joint state
  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", std::bind(
      &JointStateListener::callbackJointState, this,
      std::placeholders::_1));

  // trigger to publish fixed joints
  // if using static transform broadcaster, this will be a oneshot trigger and only run once
  timer_ = node_->create_wall_timer(1s, std::bind(&JointStateListener::callbackFixedJoint, this));
}

JointStateListener::~JointStateListener()
{}

void JointStateListener::callbackFixedJoint()
{
  state_publisher_.publishFixedTransforms(tf_prefix_, use_tf_static_);
}

void JointStateListener::callbackJointState(const sensor_msgs::msg::JointState::SharedPtr state)
{
  if (state->name.size() != state->position.size()) {
    if (state->position.empty()) {
      fprintf(stderr, "Robot state publisher ignored a JointState message about joint(s) \"%s\"\n",
        state->name[0].c_str());
    } else {
      fprintf(stderr, "Robot state publisher ignored an invalid JointState message");
    }
    return;
  }

  // check if we moved backwards in time (e.g. when playing a bag file)
  auto now = std::chrono::system_clock::now();
  if (last_callback_time_ > now) {
    // force re-publish of joint ransforms
    fprintf(stderr,
      "Moved backwards in time, re-publishing joint transforms!\n");
    last_publish_time_.clear();
  }
  last_callback_time_ = now;

  // determine least recently published joint
  auto last_published = now;
  for (unsigned int i = 0; i < state->name.size(); i++) {
    auto t = last_publish_time_[state->name[i]];
    last_published = (t < last_published) ? t : last_published;
  }
  // note: if a joint was seen for the first time,
  //       then last_published is zero.

  // check if we need to publish
  if (ignore_timestamp_ || true) {
    // get joint positions from state message
    std::map<std::string, double> joint_positions;
    for (unsigned int i = 0; i < state->name.size(); i++) {
      joint_positions.insert(std::make_pair(state->name[i], state->position[i]));
    }

    for (MimicMap::iterator i = mimic_.begin(); i != mimic_.end(); i++) {
      if (joint_positions.find(i->second->joint_name) != joint_positions.end()) {
        double pos = joint_positions[i->second->joint_name] * i->second->multiplier +
          i->second->offset;
        joint_positions.insert(std::make_pair(i->first, pos));
      }
    }

    state_publisher_.publishTransforms(
      joint_positions, state->header.stamp, tf_prefix_);

    // store publish time in joint map
    for (unsigned int i = 0; i < state->name.size(); i++) {
      // last_publish_time_[state->name[i]]
      // = std::chrono::time_point<std::chrono::system_clock>(msg_nanoseconds);
    }
  }
}
}  // namespace robot_state_publisher

bool read_urdf_xml(const std::string & filename, std::string & xml_string)
{
  std::fstream xml_file(filename.c_str(), std::fstream::in);
  if (xml_file.is_open()) {
    while (xml_file.good()) {
      std::string line;
      std::getline(xml_file, line);
      xml_string += (line + "\n");
    }
    xml_file.close();
    return true;
  } else {
    fprintf(stderr, "Could not open file [%s] for parsing.\n", filename.c_str());
    return false;
  }
}

// ----------------------------------
// ----- MAIN -----------------------
// ----------------------------------
int main(int argc, char ** argv)
{
  // Initialize ros
  rclcpp::init(argc, argv);

  // gets the location of the robot description on the parameter server
  if (argc < 2) {
    fprintf(stderr, "Robot State Publisher 2 requires a urdf file name\n");
    return -1;
  }
  fprintf(stderr, "Initialize urdf model from file: %s\n", argv[1]);
  urdf::Model model;
  if (!model.initFile(argv[1])) {
    fprintf(stderr, "Unable to initialize urdf::model from %s\n", argv[1]);
    return -1;
  }

  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(model, tree)) {
    fprintf(stderr, "Failed to extract kdl tree from xml robot description\n");
    return -1;
  }

  std::map<std::string, urdf::JointMimicSharedPtr> mimic;
  for (auto i = model.joints_.begin(); i != model.joints_.end(); i++) {
    if (i->second->mimic) {
      mimic.insert(make_pair(i->first, i->second->mimic));
    }
  }

  auto segments_map = tree.getSegments();
  for (auto segment : segments_map) {
    fprintf(stderr, "got segment %s\n", segment.first.c_str());
  }

  // Read the URDF XML data (this should always succeed)
  std::string urdf_xml;
  if (!read_urdf_xml(argv[1], urdf_xml)) {
    fprintf(stderr, "failed to read urdf xml '%s'\n", argv[1]);
    return -1;
  }

  auto node = std::make_shared<rclcpp::Node>("robot_state_publisher");
  robot_state_publisher::JointStateListener state_publisher(node, tree, mimic, urdf_xml, model);

  rclcpp::spin(node);
  return 0;
}
