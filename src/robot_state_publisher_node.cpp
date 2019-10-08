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

#include "robot_state_publisher/robot_state_publisher.hpp"

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <rclcpp/rclcpp.hpp>
#include <urdf/model.h>
#include <urdf_model/joint.h>

#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <utility>

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

  auto node = std::make_shared<robot_state_publisher::RobotStatePublisher>(tree, mimic, urdf_xml, model);

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
