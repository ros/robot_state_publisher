Robot State Publisher
=====================

This package contains the Robot State Publisher, a node and a class to publish the state of a robot to tf2.
Once the state gets published, it is available to all components in the system that also use tf2.
The package takes the joint angles of the robot as input and publishes the 3D poses of the robot links, using a kinematic tree model of the robot.

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
* `publish_frequency` (double) - The frequency at which fixed transforms will be republished to the network.  Defaults to 50.0.
* `use_tf_static` (bool) - Whether to publish fixed joints on the static broadcaster (`/tf_static` topic) or on the dynamic one (`/tf` topic).  Defaults to true, so it publishes on the `/tf_static` topic.
* `ignore_timestamp` (bool) - Whether to accept all joint states no matter what the timestamp (true), or to only publish joint state updates if they are newer than the last publish_frequency (false).  Defaults to false.
* `frame_prefix` (string) - An arbitrary prefix to add to the published tf2 frames.  Defaults to the empty string.
