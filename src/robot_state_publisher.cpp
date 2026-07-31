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
#include "rclcpp/duration.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/parameter_client.hpp"
#include "rclcpp/parameter_events_filter.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/qos_overriding_options.hpp"
#include "rclcpp/subscription_options.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/static_transform_broadcaster.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "urdf/model.hpp"

namespace robot_state_publisher
{

namespace
{

constexpr bool check_valid_pub_freq(double val)
{
  return val > 0.0 && val <= 1000.0;
}

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

using MimicMap = std::map<std::string, urdf::JointMimicSharedPtr>;

/// A class that represents a mapping between a KDL segment and its root and tip.
class SegmentPair final
{
public:
  /// Constructor
  explicit SegmentPair(
    const KDL::Segment & p_segment,
    const std::string & p_root,
    const std::string & p_tip)
  : segment(p_segment), root(p_root), tip(p_tip) {}

  KDL::Segment segment;  ///< The KDL segment
  std::string root;  ///< The name of the root element to which this link is attached
  std::string tip;  ///< The name of the element
};

class RobotStatePublisher::Impl final
{
public:
  /// Constructor; declares the parameters and creates the publishers and subscriptions.
  explicit Impl(rclcpp::Node & node);

private:
  KDL::Tree parseURDF(const std::string & urdf_xml, urdf::Model & model);

  /// Setup the URDF for use.
  /**
   * This method first parses the URDF into an internal representation.
   * Based on that representation, it then generates the list of joint segments
   * and mimic pairs that it needs during runtime.  Finally, it publishes
   * the text of the URDF to the network on the /robot_description topic.
   *
   * \param[in] urdf_xml The string representing the URDF XML.
   */
  void setupURDF(const std::string & urdf_xml);

  /// Recursive method to add all children to the internal segment list.
  /**
   *
   * \param[in] segment An iterator to the SegmentMap to add to the internal segment list.
   */
  void addChildren(
    const urdf::Model & model,
    const KDL::SegmentMap::const_iterator segment);

  /// Publish transforms to /tf2.
  /**
   * This method is called by callbackJointState() when new transforms are available and need to be published.
   *
   * \param[in] joint_positions A map of joint names to joint positions.
   * \param[in] time The time at which the joint positions were recorded.
   */
  void publishTransforms(
    const std::map<std::string, double> & joint_positions,
    const builtin_interfaces::msg::Time & time);

  /// Publish fixed transforms at startup time to /tf2_static.
  void publishFixedTransforms();

  /// The callback that is called when a new JointState message is received.
  /**
   * This method examines the incoming JointStates and applies a series of checks to
   * see if new transforms should be published.  If so, it calls publishTransforms() to do so.
   *
   * \param[in] state The JointState message that was delivered.
   */
  void callbackJointState(const sensor_msgs::msg::JointState::ConstSharedPtr state);

  /// The callback that is called to check that new parameters are valid.
  /**
   * This allows the class to reject parameter updates that are invalid.
   *
   * \param[in] parameters The vector of parameters that are going to change.
   * \return SetParametersResult with successful set to true on success, false otherwise.
   */
  rcl_interfaces::msg::SetParametersResult parameterUpdate(
    const std::vector<rclcpp::Parameter> & parameters);

  /// The callback that is called when parameters on the node are changed.
  /**
   * This allows the class to dynamically react to changes in parameters.
   *
   * \param[in] event The parameter change event that occurred.
   */
  void onParameterEvent(std::shared_ptr<const rcl_interfaces::msg::ParameterEvent> event);

  /// The node this class adds the robot state publisher functionality to
  rclcpp::Node & node_;

  /// A map of dynamic segment names to SegmentPair structures
  std::map<std::string, SegmentPair> segments_;

  /// A map of fixed segment names to SegmentPair structures
  std::map<std::string, SegmentPair> segments_fixed_;

  /// A pointer to the tf2 TransformBroadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  /// A pointer to the tf2 StaticTransformBroadcaster
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

  /// A pointer to the ROS 2 publisher for the robot_description
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr description_pub_;

  /// A pointer to the ROS 2 subscription for the robot_description,
  /// when use_robot_description_topic_ is true
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr description_sub_;

  /// Whether to use the robot_description from a topic instead of a parameter
  bool use_robot_description_topic_;

  /// A pointer to the ROS 2 subscription for the joint states
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  /// The last time a joint state message was received
  rclcpp::Time last_callback_time_;

  /// A map between a joint name and the last time its state was published
  std::map<std::string, builtin_interfaces::msg::Time> last_publish_time_;

  /// A map of the mimic joints that should be published
  MimicMap mimic_;

  /// Cached value of the publish_frequency parameter
  double publish_frequency_;

  /// Cached value of the ignore_timestamp parameter
  bool ignore_timestamp_;

  /// Cached value of the frame_prefix parameter
  std::string frame_prefix_;

  /// The parameter event callback that will be called when a parameter is changed
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

  /// The parameter event callback that will be called when a parameter is changed
  std::shared_ptr<rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent,
    std::allocator<void>>> parameter_subscription_;
};

RobotStatePublisher::Impl::Impl(rclcpp::Node & node)
: node_(node)
{
  use_robot_description_topic_ = node_.declare_parameter("use_robot_description_topic", false);

  if (use_robot_description_topic_) {
    description_sub_ = node_.create_subscription<std_msgs::msg::String>(
        "robot_description", rclcpp::QoS(1).transient_local().reliable(),
      [this](const std_msgs::msg::String::ConstSharedPtr msg) {
        try {
          this->setupURDF(msg->data);
          this->publishFixedTransforms();
        } catch (const std::exception & ex) {
          RCLCPP_ERROR(node_.get_logger(), "Failed to parse robot description from topic: %s",
            ex.what());
        }
        });

    RCLCPP_DEBUG(node_.get_logger(), "Waiting for robot_description on topic...");
  } else {
    std::string urdf_xml = node_.declare_parameter("robot_description", std::string(""));
    if (urdf_xml.empty()) {
      throw std::runtime_error("robot_description parameter must not be empty");
    }

    description_pub_ = node_.create_publisher<std_msgs::msg::String>(
      "robot_description",
      // Transient local is similar to latching in ROS 1.
      rclcpp::QoS(1).transient_local());

    setupURDF(urdf_xml);

    // Now that we have successfully declared the parameters and done all
    // necessary setup, install the callback for updating parameters.
    param_cb_ = node_.add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        return parameterUpdate(parameters);
      });

    parameter_subscription_ = rclcpp::AsyncParametersClient::on_parameter_event(
      node_.get_node_topics_interface(),
      [this](rcl_interfaces::msg::ParameterEvent::ConstSharedPtr event) {
        onParameterEvent(event);
      });
  }

  // set publish frequency
  publish_frequency_ = node_.declare_parameter("publish_frequency", 20.0);
  if (!check_valid_pub_freq(publish_frequency_)) {
    throw std::runtime_error("publish_frequency must be between 0 (exclusive) and 1000");
  }

  // set frame_prefix
  frame_prefix_ = node_.declare_parameter("frame_prefix", std::string(""));

  // ignore_timestamp_ == true, joint_state messages are accepted, no matter their timestamp
  ignore_timestamp_ = node_.declare_parameter("ignore_timestamp", false);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
  static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(node_);

  auto subscriber_options = rclcpp::SubscriptionOptions();
  subscriber_options.qos_overriding_options =
    rclcpp::QosOverridingOptions::with_default_policies();

  // subscribe to joint state
  joint_state_sub_ = node_.create_subscription<sensor_msgs::msg::JointState>(
    "joint_states",
    rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::JointState::ConstSharedPtr state) {
      callbackJointState(state);
    },
    subscriber_options);

  publishFixedTransforms();
}

KDL::Tree RobotStatePublisher::Impl::parseURDF(const std::string & urdf_xml, urdf::Model & model)
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

void RobotStatePublisher::Impl::setupURDF(const std::string & urdf_xml)
{
  urdf::Model model;
  KDL::Tree tree = parseURDF(urdf_xml, model);

  // Initialize the mimic map
  mimic_.clear();
  for (const auto & [joint_name, joint] : model.joints_) {
    if (joint->mimic) {
      // Just taking a reference to the model shared pointers ends up in a crash.
      // Explicitly make a copy of the JointMimic.
      auto jm = std::make_shared<urdf::JointMimic>();
      jm->offset = joint->mimic->offset;
      jm->multiplier = joint->mimic->multiplier;
      jm->joint_name = joint->mimic->joint_name;
      mimic_[joint_name] = jm;
    }
  }

  const KDL::SegmentMap & segments_map = tree.getSegments();
  for (const auto & [segment_name, element] : segments_map) {
    RCLCPP_DEBUG(node_.get_logger(), "Got segment %s", segment_name.c_str());
  }

  // walk the tree and add segments to segments_
  segments_.clear();
  segments_fixed_.clear();
  addChildren(model, tree.getRootSegment());

  auto msg = std::make_unique<std_msgs::msg::String>();
  msg->data = urdf_xml;

  // Publish the robot description
  if (!use_robot_description_topic_) {
    description_pub_->publish(std::move(msg));
  }

  RCLCPP_INFO(node_.get_logger(), "Robot initialized");
}

// add children to correct maps
void RobotStatePublisher::Impl::addChildren(
  const urdf::Model & model,
  const KDL::SegmentMap::const_iterator segment)
{
  const std::string & root = GetTreeElementSegment(segment->second).getName();

  std::vector<KDL::SegmentMap::const_iterator> children = GetTreeElementChildren(segment->second);
  for (const KDL::SegmentMap::const_iterator & child_it : children) {
    const KDL::Segment & child = GetTreeElementSegment(child_it->second);
    SegmentPair s(GetTreeElementSegment(child_it->second), root, child.getName());
    if (child.getJoint().getType() == KDL::Joint::None) {
      if (model.getJoint(child.getJoint().getName()) &&
        model.getJoint(child.getJoint().getName())->type == urdf::Joint::FLOATING)
      {
        RCLCPP_DEBUG(
          node_.get_logger(), "Floating joint is not supported; skipping segment from %s to %s.",
          root.c_str(), child.getName().c_str());
      } else {
        segments_fixed_.emplace(child.getJoint().getName(), s);
        RCLCPP_DEBUG(
          node_.get_logger(), "Adding fixed segment from %s to %s", root.c_str(),
          child.getName().c_str());
      }
    } else {
      segments_.emplace(child.getJoint().getName(), s);
      RCLCPP_DEBUG(
        node_.get_logger(), "Adding moving segment from %s to %s", root.c_str(),
        child.getName().c_str());
    }
    addChildren(model, child_it);
  }
}

// publish moving transforms
void RobotStatePublisher::Impl::publishTransforms(
  const std::map<std::string, double> & joint_positions,
  const builtin_interfaces::msg::Time & time)
{
  RCLCPP_DEBUG(node_.get_logger(), "Publishing transforms for moving joints");

  const std::string & frame_prefix = frame_prefix_;

  std::vector<geometry_msgs::msg::TransformStamped> tf_transforms;
  // Upper bound: at most one transform per incoming joint position.
  tf_transforms.reserve(joint_positions.size());

  // loop over all joints
  for (const auto & [joint_name, position] : joint_positions) {
    auto seg = segments_.find(joint_name);
    if (seg != segments_.end()) {
      geometry_msgs::msg::TransformStamped tf_transform =
        kdlToTransform(seg->second.segment.pose(position));
      tf_transform.header.stamp = time;
      tf_transform.header.frame_id = frame_prefix + seg->second.root;
      tf_transform.child_frame_id = frame_prefix + seg->second.tip;
      tf_transforms.push_back(tf_transform);
    }
  }
  tf_broadcaster_->sendTransform(tf_transforms);
}

// publish fixed transforms
void RobotStatePublisher::Impl::publishFixedTransforms()
{
  RCLCPP_DEBUG(node_.get_logger(), "Publishing transforms for fixed joints");

  const std::string & frame_prefix = frame_prefix_;

  std::vector<geometry_msgs::msg::TransformStamped> tf_transforms;
  // Exactly one transform per fixed segment.
  tf_transforms.reserve(segments_fixed_.size());

  // loop over all fixed segments
  rclcpp::Time now = node_.now();
  for (const auto & [joint_name, seg] : segments_fixed_) {
    geometry_msgs::msg::TransformStamped tf_transform = kdlToTransform(seg.segment.pose(0));
    tf_transform.header.stamp = now;

    tf_transform.header.frame_id = frame_prefix + seg.root;
    tf_transform.child_frame_id = frame_prefix + seg.tip;
    tf_transforms.push_back(tf_transform);
  }
  static_tf_broadcaster_->sendTransform(tf_transforms);
}

void RobotStatePublisher::Impl::callbackJointState(
  const sensor_msgs::msg::JointState::ConstSharedPtr state)
{
  if (state->name.size() != state->position.size()) {
    if (state->position.empty()) {
      const char * first_joint = state->name.empty() ? "<none>" : state->name[0].c_str();
      RCLCPP_WARN(
        node_.get_logger(), "Robot state publisher ignored a JointState message about joint(s) "
        "\"%s\"(,...) whose position member was empty.", first_joint);
    } else {
      RCLCPP_ERROR(
        node_.get_logger(), "Robot state publisher ignored an invalid JointState message");
    }
    return;
  }

  // check if we moved backwards in time (e.g. when playing a bag file)
  rclcpp::Time now = node_.now();
  if (last_callback_time_.nanoseconds() > now.nanoseconds()) {
    // force re-publish of joint transforms
    RCLCPP_WARN(
      node_.get_logger(), "Moved backwards in time, re-publishing joint transforms!");
    last_publish_time_.clear();
  }
  last_callback_time_ = now;

  // determine least recently published joint
  rclcpp::Time last_published = now;
  for (const std::string & name : state->name) {
    rclcpp::Time t(last_publish_time_[name]);
    last_published = (t.nanoseconds() < last_published.nanoseconds()) ? t : last_published;
  }
  // note: if a joint was seen for the first time,
  //       then last_published is zero.

  // check if we need to publish
  rclcpp::Time current_time(state->header.stamp);
  std::chrono::milliseconds publish_interval_ms =
    std::chrono::milliseconds(static_cast<uint64_t>(1000.0 / publish_frequency_));
  rclcpp::Time max_publish_time = last_published + rclcpp::Duration(publish_interval_ms);
  if (ignore_timestamp_ ||
    current_time.nanoseconds() >= max_publish_time.nanoseconds())
  {
    // get joint positions from state message
    std::map<std::string, double> joint_positions;
    for (size_t i = 0; i < state->name.size(); i++) {
      joint_positions.emplace(state->name[i], state->position[i]);
    }

    for (const auto & [mimic_name, mimic] : mimic_) {
      if (auto it = joint_positions.find(mimic->joint_name); it != joint_positions.end()) {
        const double pos = it->second * mimic->multiplier + mimic->offset;
        joint_positions.emplace(mimic_name, pos);
      }
    }

    publishTransforms(joint_positions, state->header.stamp);

    // store publish time in joint map
    for (const std::string & name : state->name) {
      last_publish_time_[name] = state->header.stamp;
    }
  }
}

rcl_interfaces::msg::SetParametersResult RobotStatePublisher::Impl::parameterUpdate(
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
        RCLCPP_WARN(node_.get_logger(), "%s", err.what());
        result.successful = false;
        result.reason = err.what();
        break;
      }
    } else if (parameter.get_name() == "publish_frequency") {
      double publish_freq = parameter.as_double();
      if (!check_valid_pub_freq(publish_freq)) {
        result.successful = false;
        result.reason = "publish_frequency must be between 0.0 (exclusive) and 1000.0";
        break;
      }
    }
  }

  return result;
}

void RobotStatePublisher::Impl::onParameterEvent(
  std::shared_ptr<const rcl_interfaces::msg::ParameterEvent> event)
{
  // Filter out events from other nodes
  if (event->node != node_.get_fully_qualified_name()) {
    return;
  }

  // Filter for changed parameters that affect runtime behaviour.
  rclcpp::ParameterEventsFilter filter(event,
    {"robot_description", "publish_frequency", "frame_prefix", "ignore_timestamp"},
    {rclcpp::ParameterEventsFilter::EventType::CHANGED});
  for (auto & it : filter.get_events()) {
    const std::string & name = it.second->name;
    if (name == "robot_description") {
      try {
        setupURDF(it.second->value.string_value);
        publishFixedTransforms();
      } catch (const std::runtime_error & err) {
        RCLCPP_WARN(node_.get_logger(), "Failed to parse new URDF: %s", err.what());
      }
    } else if (name == "publish_frequency") {
      publish_frequency_ = it.second->value.double_value;
    } else if (name == "frame_prefix") {
      frame_prefix_ = it.second->value.string_value;
    } else if (name == "ignore_timestamp") {
      ignore_timestamp_ = it.second->value.bool_value;
    }
  }
}

RobotStatePublisher::RobotStatePublisher(const rclcpp::NodeOptions & options)
: rclcpp::Node("robot_state_publisher", options),
  impl_(std::make_unique<Impl>(*this))
{
}

RobotStatePublisher::~RobotStatePublisher() = default;

}  // namespace robot_state_publisher

RCLCPP_COMPONENTS_REGISTER_NODE(robot_state_publisher::RobotStatePublisher)
