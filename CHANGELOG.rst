^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package robot_state_publisher
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.14.1 (2020-04-02)
-------------------
* Bump CMake version to avoid CMP0048 warning (`#132 <https://github.com/ros/robot_state_publisher/issues/132>`_)
* Make sure to make sensor_msgs a catkin dependency. (`#123 <https://github.com/ros/robot_state_publisher/issues/123>`_)
* Add joint_state_listener to the catkin package LIBRARIES (`#112 <https://github.com/ros/robot_state_publisher/issues/112>`_)
* Contributors: Alejandro Hernández Cordero, Chris Lalancette, sevangelatos

1.14.0 (2019-08-27)
-------------------
* Revert "Remove dependency on tf and tf_prefix support (`#82 <https://github.com/ros/robot_state_publisher/issues/82>`_)" (`#113 <https://github.com/ros/robot_state_publisher/issues/113>`_)
* update how compiler flags are added (`#104 <https://github.com/ros/robot_state_publisher/issues/104>`_)
* update install destination in CMakeLists.txt (`#103 <https://github.com/ros/robot_state_publisher/issues/103>`_)
* Remove treefksolver completely from the repository. (`#100 <https://github.com/ros/robot_state_publisher/issues/100>`_)
* changed return code from -1 to 1 since its considered a reserved bash exit code (`#98 <https://github.com/ros/robot_state_publisher/issues/98>`_)
* Fixed problem when building static library version (`#92 <https://github.com/ros/robot_state_publisher/issues/92>`_) (`#96 <https://github.com/ros/robot_state_publisher/issues/96>`_)
* Add Ian as a maintainer for robot_state_publisher. (`#94 <https://github.com/ros/robot_state_publisher/issues/94>`_)
* added warning when joint is found in joint message but not in the urdf (`#83 <https://github.com/ros/robot_state_publisher/issues/83>`_)
* added ros_warn if JointStateMessage is older than 30 seconds (`#84 <https://github.com/ros/robot_state_publisher/issues/84>`_)
* Add tcp_no_delay to joint_states subscriber (`#80 <https://github.com/ros/robot_state_publisher/issues/80>`_) (`#85 <https://github.com/ros/robot_state_publisher/issues/85>`_)
* Remove dependency on tf and tf_prefix support (`#82 <https://github.com/ros/robot_state_publisher/issues/82>`_)
* make rostest in CMakeLists optional (`ros/rosdistro#3010 <https://github.com/ros/rosdistro/issues/3010>`_) (`#75 <https://github.com/ros/robot_state_publisher/issues/75>`_)
* Added c++11 target_compile_options (`#78 <https://github.com/ros/robot_state_publisher/issues/78>`_)
* Contributors: Chris Lalancette, James Xu, Lukas Bulwahn, Shane Loretz, betab0t, jgueldenstein

1.13.5 (2017-04-11)
-------------------
* Style cleanup throughout the tree.
* add Chris and Shane as maintainers (`#70 <https://github.com/ros/robot_state_publisher/issues/70>`_)
* Contributors: Chris Lalancette, William Woodall

1.13.4 (2017-01-05)
-------------------
* Use ``urdf::*ShredPtr`` instead of ``boost::shared_ptr`` (`#60 <https://github.com/ros/robot_state_publisher/issues/60>`_)
* Error log for empty JointState.position was downgraded to a throttled warning (`#64 <https://github.com/ros/robot_state_publisher/issues/64>`_)
* Contributors: Jochen Sprickerhof, Sébastien BARTHÉLÉMY

1.13.3 (2016-10-20)
-------------------
* Added a new parameter "ignore_timestamp" (`#65 <https://github.com/ros/robot_state_publisher/issues/65>`_)
* Fixed joints are not published over tf_static by default (`#56 <https://github.com/ros/robot_state_publisher/issues/56>`_)
* Fixed segfault on undefined robot_description (`#61 <https://github.com/ros/robot_state_publisher/issues/61>`_)
* Fixed cmake eigen3 warning (`#62 <https://github.com/ros/robot_state_publisher/issues/62>`_)
* Contributors: Davide Faconti, Ioan A Sucan, Johannes Meyer, Robert Haschke

1.13.2 (2016-06-10)
-------------------
* Add target_link_libraries for joint_state_listener library + install it (`#54 <https://github.com/ros/robot_state_publisher//issues/54>`_)
* Contributors: Kartik Mohta

1.13.1 (2016-05-20)
-------------------
* Add back future dating for robot_state_publisher (`#49 <https://github.com/ros/robot_state_publisher/issues/49>`_) (`#51 <https://github.com/ros/robot_state_publisher/issues/51>`_)
* Fix subclassing test (`#48 <https://github.com/ros/robot_state_publisher/issues/48>`_)
* Support for subclassing (`#45 <https://github.com/ros/robot_state_publisher/issues/45>`_)
  * Add joint_state_listener as a library
* Contributors: Jackie Kay

1.13.0 (2016-04-12)
-------------------
* fix bad rebase
* Contributors: Jackie Kay, Paul Bovbel

1.12.1 (2016-02-22)
-------------------
* Merge pull request `#42 <https://github.com/ros/robot_state_publisher/issues/42>`_ from ros/fix_tests_jade
  Fix tests for Jade
* Correct failing tests
* Re-enabling rostests
* Merge pull request `#39 <https://github.com/ros/robot_state_publisher/issues/39>`_ from scpeters/issue_38
* Fix API break in publishFixedTransforms
  A bool argument was added to
  RobotStatePublisher::publishFixedTransforms
  which broke API.
  I've added a default value of false, to match
  the default specified in the JointStateListener
  constructor.
* Contributors: Jackie Kay, Jonathan Bohren, Steven Peters

1.12.0 (2015-10-21)
-------------------
* Merge pull request `#37 <https://github.com/ros/robot_state_publisher/issues/37>`_ from clearpathrobotics/static-default
  Publish fixed joints over tf_static by default
* Merge pull request `#34 <https://github.com/ros/robot_state_publisher/issues/34>`_ from ros/tf2-static-jade
  Port to tf2 and enable using static broadcaster
* Merge pull request `#32 <https://github.com/ros/robot_state_publisher/issues/32>`_ from `shadow-robot/fix_issue#19 <https://github.com/shadow-robot/fix_issue/issues/19>`_
  Check URDF to distinguish fixed joints from floating joints. Floating joint are ignored by the publisher.
* Merge pull request `#26 <https://github.com/ros/robot_state_publisher/issues/26>`_ from xqms/remove-debug
  get rid of argv[0] debug output on startup
* Contributors: David Lu!!, Ioan A Sucan, Jackie Kay, Max Schwarz, Paul Bovbel, Toni Oliver

1.11.1 (2016-02-22)
-------------------
* Merge pull request `#41 <https://github.com/ros/robot_state_publisher/issues/41>`_ from ros/fix_tests_indigo
  Re-enable and clean up rostests
* Correct failing tests
* Re-enabling rostests
* Fix API break in publishFixedTransforms
  A bool argument was added to
  RobotStatePublisher::publishFixedTransforms
  which broke API.
  I've added a default value of false, to match
  the default specified in the JointStateListener
  constructor.
* Contributors: Jackie Kay, Jonathan Bohren, Steven Peters

1.11.0 (2015-10-21)
-------------------
* Merge pull request `#28 <https://github.com/ros/robot_state_publisher/issues/28>`_ from clearpathrobotics/tf2-static

1.10.4 (2014-11-30)
-------------------
* Merge pull request `#21 <https://github.com/ros/robot_state_publisher/issues/21>`_ from rcodddow/patch-1
* Fix for joint transforms not being published anymore after a clock reset (e.g. when playing a bagfile and looping)
* Contributors: Ioan A Sucan, Robert Codd-Downey, Timm Linder

1.10.3 (2014-07-24)
-------------------
* add version depend on orocos_kdl >= 1.3.0
  Conflicts:
  package.xml
* Update KDL SegmentMap interface to optionally use shared pointers
  The KDL Tree API optionally uses shared pointers on platforms where
  the STL containers don't support incomplete types.
* Contributors: Brian Jensen, William Woodall

1.10.0 (2014-03-03)
-------------------
* minor style fixes
* Add support for mimic tag.
* Contributors: Ioan Sucan, Konrad Banachowicz
