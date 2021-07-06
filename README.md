Robot State Publisher
=====================

This package contains the Robot State Publisher, a node and a class to publish the state of a robot to tf2.
At startup time, Robot State Publisher is supplied with a kinematic tree model (URDF) of the robot.
It then subscribes to the `joint_states` topic (of type `sensor_msgs/msg/JointState`) to get individual joint states.
These joint states are used to update the kinematic tree model, and the resulting 3D poses are then published to tf2.

Robot State Publisher deals with two different "classes" of joint types: fixed and movable.
Fixed joints (with the type "fixed") are published to the transient_local `/tf_static` topic once on startup (transient_local topics keep a history of what they published, so a later subscription can always get the latest state of the world).
Movable joints are published to the regular `/tf` topic any time the appropriate joint is updated in the `joint_states` message.

Examples showing how to pass the `robot_description` parameter using a launch file are available in the 'launch' subdirectory.

Published Topics
----------------
* `robot_description` (`std_msgs/msg/String`) - The description of the robot URDF as a string.  Republishes the value set in the `robot_description` parameter, which is useful for getting informed of dynamic changes to the URDF.  Published using the "transient local" quality of service, so subscribers should also be "transient local".
* `tf` (`tf2_msgs/msg/TFMessage`) - The transforms corresponding to the movable joints of the robot.
* `tf_static` (`tf2_msgs/msg/TFMessage`) - The transforms corresponding to the static joints of the robot.

Subscribed Topics
-----------------
* `joint_states` (`sensor_msgs/msg/JointState`) - The joint state updates to the robot poses.  The RobotStatePublisher class takes these updates, does transformations (such as mimic joints), and then publishes the results on the tf2 topics.

Parameters
----------
* `robot_description` (string) - The original description of the robot in URDF form.  This *must* be set at robot_state_publisher startup time, or the node will fail to start.  Updates to this parameter will be reflected in the `robot_description` topic.
* `publish_frequency` (double) - The maximum frequency at which non-static transforms (e.g. joint states) will be published to `/tf`.  Defaults to 20.0 Hz.
* `ignore_timestamp` (bool) - Whether to accept all joint states no matter what the timestamp (true), or to only publish joint state updates if they are newer than the last publish_frequency (false).  Defaults to false.
* `frame_prefix` (string) - An arbitrary prefix to add to the published tf2 frames.  Defaults to the empty string.
