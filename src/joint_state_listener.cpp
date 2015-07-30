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

#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <ros/ros.h>
#include "robot_state_publisher/robot_state_publisher.h"
#include "robot_state_publisher/joint_state_listener.h"
#include <kdl_parser/kdl_parser.hpp>
#include <map>
#include <string>
#include <algorithm>

using robot_state_publisher::JointStateListener;

JointStateListener::JointStateListener(const KDL::Tree& tree, const MimicMap& m):
  update_ongoing(false),
  state_publisher_(tree),
  mimic_(m)
{
  ros::NodeHandle n_tilde("~");
  ros::NodeHandle n;

  // set publish frequency
  double publish_freq;
  n_tilde.param("publish_frequency", publish_freq, 50.0);

  // get the tf_prefix parameter from the closest namespace
  std::string tf_prefix_key;
  n_tilde.searchParam("tf_prefix", tf_prefix_key);
  n_tilde.param(tf_prefix_key, tf_prefix_, std::string(""));
  publish_interval_ = ros::Duration(1.0 / max(publish_freq, 1.0));


  /// only offer reload_service on request
  bool with_reload;
  n_tilde.param("with_reload_service", with_reload, false);
  if (with_reload)
    {
      reload_server = n_tilde.advertiseService("reload_robot_model", &JointStateListener::reload_robot_model_cb, this);
    }


  // subscribe to joint state
  joint_state_sub_ = n.subscribe("joint_states", 1, &JointStateListener::callbackJointState, this);

  // trigger to publish fixed joints
  pub_fixed_trafos_timer_ = n_tilde.createTimer(publish_interval_, &JointStateListener::callbackFixedJoint, this);
  pub_fixed_trafos_timer_.start();
}



bool loadTreeAndMimicMap(KDL::Tree  tree, MimicMap *mimic_map, std::string *err_msg)
{
  // gets the location of the robot description on the parameter server
  urdf::Model model;
  bool found = model.initParam("robot_description");

  if (!found)
    {
      *err_msg = "Could not read urdf_model from parameter server";
      return false;
    }

  if (!kdl_parser::treeFromUrdfModel(model, tree))
    {
      *err_msg = "Failed to extract kdl tree from xml robot description";
      return false;
    }


  mimic_map->clear();
  std::map< std::string, boost::shared_ptr< urdf::Joint > >::iterator it;
  for (it = model.joints_.begin(); it != model.joints_.end(); it++)
    {
      if (it->second->mimic)
        {
          mimic_map->insert(make_pair(it->first, it->second->mimic));
        }
    }

  return true;
}

bool JointStateListener::reload_robot_model(std::string* msg)
{
  update_ongoing = true;

  pub_fixed_trafos_timer_.stop();  /// make sure that state_publisher is not currently publishing
  publish_interval_.sleep();       /// allow publishFixedTransforms to end


  KDL::Tree tree;
  mimic_.clear();

  if (!loadTreeAndMimicMap(tree, &mimic_, msg))
    {
      ROS_ERROR("%s", msg->c_str());
      return false;
    }

  /// if the update fails, both publishers (for fixed and non-fixed trafos) keep turned off so
  /// that the failure is obvious and no one is working with the old model

  state_publisher_.updateTree(tree);
  state_publisher_.createTreeInfo(msg);  /// create an info-string for the service caller
  pub_fixed_trafos_timer_.start();        /// fixed transforms are published again

  update_ongoing = false;

  return true;
}



bool JointStateListener::reload_robot_model_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  ROS_DEBUG("Reload of Robot model requested");
  res.success = reload_robot_model(&res.message);
  return true;
}

JointStateListener::~JointStateListener() {}


void JointStateListener::callbackFixedJoint(const ros::TimerEvent& e)
{
  state_publisher_.publishFixedTransforms(tf_prefix_);
}

void JointStateListener::callbackJointState(const JointStateConstPtr& state)
{
  /// continue processing of state-msgs to not collect old msgs in queue
  if (update_ongoing)
    {
      ROS_WARN_THROTTLE(1, "Skipping joing state while robot model is updated");
      return;
    }


  if (state->name.size() != state->position.size())
    {
      ROS_ERROR("Robot state publisher received an invalid joint state vector");
      return;
    }

  // check if we moved backwards in time (e.g. when playing a bag file)
  ros::Time now = ros::Time::now();
  if (last_callback_time_ > now)
    {
      // force re-publish of joint transforms
      ROS_WARN("Moved backwards in time (probably because ROS clock was reset), re-publishing joint transforms!");
      last_publish_time_.clear();
    }

  last_callback_time_ = now;

  // determine least recently published joint
  ros::Time last_published = now;
  for (unsigned int i = 0; i < state->name.size(); i++)
    {
      ros::Time t = last_publish_time_[state->name[i]];
      last_published = (t < last_published) ? t : last_published;
    }
  // note: if a joint was seen for the first time,
  //       then last_published is zero.


  // check if we need to publish
  if (state->header.stamp >= last_published + publish_interval_)
    {
      // get joint positions from state message
      map<string, double> joint_positions;
      for (unsigned int i = 0; i < state->name.size(); i++)
        joint_positions.insert(make_pair(state->name[i], state->position[i]));

      for (MimicMap::iterator i = mimic_.begin(); i != mimic_.end(); i++)
        {
          if (joint_positions.find(i->second->joint_name) != joint_positions.end())
            {
              double pos = joint_positions[i->second->joint_name] * i->second->multiplier + i->second->offset;
              joint_positions.insert(make_pair(i->first, pos));
            }
        }

      state_publisher_.publishTransforms(joint_positions, state->header.stamp, tf_prefix_);

      // store publish time in joint map
      for (unsigned int i = 0; i < state->name.size(); i++)
        {
          last_publish_time_[state->name[i]] = state->header.stamp;
        }
    }
}


// ----------------------------------
// ----- MAIN -----------------------
// ----------------------------------
int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_state_publisher_123");

  ///////////////////////////////////////// begin deprecation warning
  std::string exe_name = argv[0];
  std::size_t slash = exe_name.find_last_of("/");
  if (slash != std::string::npos)
    exe_name = exe_name.substr(slash + 1);
  if (exe_name == "state_publisher")
    ROS_WARN("The 'state_publisher' executable is deprecated. Please use 'robot_state_publisher' instead");
  ///////////////////////////////////////// end deprecation warning

  /// gets the location of the robot description on the parameter server
  KDL::Tree tree;
  MimicMap mimic;
  std::string err_msg;

  if (!loadTreeAndMimicMap(tree, &mimic, &err_msg))
    {
      ROS_ERROR("%s", err_msg.c_str());
      return -1;
    }

  JointStateListener state_publisher(tree, mimic);
  ros::spin();
  return 0;
}
