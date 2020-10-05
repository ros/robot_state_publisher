#include <kdl_parser/kdl_parser.hpp>

#include "robot_state_publisher/robot_state_publisher.h"
#include "robot_state_publisher/joint_state_listener.h"

using namespace std;
using namespace ros;
using namespace KDL;
using namespace robot_state_publisher;

int main(int argc, char** argv)
{
  // Initialize ros
  ros::init(argc, argv, "robot_state_publisher");
  NodeHandle node;

  ///////////////////////////////////////// begin deprecation warning
  std::string exe_name = argv[0];
  std::size_t slash = exe_name.find_last_of("/");
  if (slash != std::string::npos) {
    exe_name = exe_name.substr(slash + 1);
  }
  if (exe_name == "state_publisher") {
    ROS_WARN("The 'state_publisher' executable is deprecated. Please use 'robot_state_publisher' instead");
  }
  ///////////////////////////////////////// end deprecation warning

  // gets the location of the robot description on the parameter server
  urdf::Model model;
  if (!model.initParam("robot_description"))
    return 1;

  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(model, tree)) {
    ROS_ERROR("Failed to extract kdl tree from xml robot description");
    return 1;
  }

  MimicMap mimic;

  for(std::map< std::string, urdf::JointSharedPtr >::iterator i = model.joints_.begin(); i != model.joints_.end(); i++) {
    if(i->second->mimic) {
      mimic.insert(make_pair(i->first, i->second->mimic));
    }
  }

  JointStateListener state_publisher(tree, mimic, model);
  ros::spin();

  return 0;
}
