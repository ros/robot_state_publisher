// Copyright  (C)  2009 Willow Garage Inc

// Version: 1.0
// Author: Wim Meeussen <meeussen at willowgarage dot com>
// Maintainer: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// URL: http://www.orocos.org/kdl

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA


#include "robot_state_publisher/treefksolverposfull_recursive.hpp"
#include <ros/ros.h>
#include <iostream>
#include <cstdio>
#include <map>
#include <vector>
#include <string>

using std::map;
using std::string;
using std::vector;




namespace KDL
{

  TreeFkSolverPosFull_recursive::TreeFkSolverPosFull_recursive(const Tree& _tree):
    tree(_tree)
  {
  }

  TreeFkSolverPosFull_recursive::~TreeFkSolverPosFull_recursive()
  {
  }


  int TreeFkSolverPosFull_recursive::JntToCart(const map<string, double>& q_in, map < string,
                                               tf::Stamped<Frame> > & p_out, bool flatten_tree)
  {
    // clear output
    p_out.clear();

    addFrameToMap(q_in, p_out, tf::Stamped<KDL::Frame>(KDL::Frame::Identity(),
                                                       ros::Time(),
                                                       GetTreeElementSegment(tree.getRootSegment()->second).getName()),
                  tree.getRootSegment(), flatten_tree);

    return 0;
  }



  void TreeFkSolverPosFull_recursive::addFrameToMap(const map<string, double>& q_in,
                                                    map<string, tf::Stamped<Frame> >& p_out,
                                                    const tf::Stamped<KDL::Frame>& previous_frame,
                                                    const SegmentMap::const_iterator this_segment,
                                                    bool flatten_tree)
  {
    // get pose of this segment
    tf::Stamped<KDL::Frame> this_frame;
    double jnt_p = 0;
    if (GetTreeElementSegment(this_segment->second).getJoint().getType() != Joint::None)
      {
        map<string, double>::const_iterator jnt_pos = q_in.find(GetTreeElementSegment(this_segment->second).getJoint().getName());
        if (jnt_pos == q_in.end())
          {
            ROS_DEBUG("Warning: TreeFKSolverPosFull Could not find value for joint '%s'. Skipping this tree branch",
                      this_segment->first.c_str());
            return;
          }
        jnt_p = jnt_pos->second;
      }
    this_frame = tf::Stamped<KDL::Frame>(previous_frame * GetTreeElementSegment(this_segment->second).pose(jnt_p),
                                         ros::Time(), previous_frame.frame_id_);

    if (this_segment->first != tree.getRootSegment()->first)
      p_out.insert(make_pair(this_segment->first, tf::Stamped<KDL::Frame>(this_frame, ros::Time(), previous_frame.frame_id_)));

    // get poses of child segments
    for (vector<SegmentMap::const_iterator>::const_iterator child = GetTreeElementChildren(this_segment->second).begin();
         child != GetTreeElementChildren(this_segment->second).end(); child++)
      {
        if (flatten_tree)
          addFrameToMap(q_in, p_out, this_frame, *child, flatten_tree);
        else
          addFrameToMap(q_in, p_out, tf::Stamped<KDL::Frame>(KDL::Frame::Identity(), ros::Time(),
                                                             this_segment->first), *child, flatten_tree);
      }
  }

}  // namespace KDL
