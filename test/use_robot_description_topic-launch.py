# Copyright (c) 2025 Open Source Robotics Foundation, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os
import time
import unittest

from launch import LaunchDescription
import launch_ros
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


@pytest.mark.launch_test
def generate_test_description():
    node_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='rsp_test',
        output='screen',
        parameters=[{'use_robot_description_topic': True}],
    )

    return LaunchDescription(
        [
            node_robot_state_publisher,
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestRobotDescriptionTopic(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_fixed_transform_after_urdf_publish(self):

        node = rclpy.create_node('test_robot_state_publisher')
        buffer = Buffer()
        _ = TransformListener(buffer, node)

        urdf_file = os.path.join(os.path.dirname(__file__), 'two_links_fixed_joint.urdf')
        with open(urdf_file, 'r') as infp:
            robot_desc = infp.read()

        # Publish the robot_description (transient local)
        pub = node.create_publisher(
            String,
            'robot_description',
            rclpy.qos.QoSProfile(
                depth=1,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            ),
        )
        msg = String()
        msg.data = robot_desc
        pub.publish(msg)

        # Wait up to timeout for TF
        timeout_sec = 5.0
        start_time = time.time()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            try:
                trans = buffer.lookup_transform('link1', 'link2', rclpy.time.Time())
                self.assertAlmostEqual(trans.transform.translation.x, 5.0, places=2)
                self.assertAlmostEqual(trans.transform.translation.y, 0.0, places=2)
                self.assertAlmostEqual(trans.transform.translation.z, 0.0, places=2)
                return
            except TransformException:
                if time.time() - start_time > timeout_sec:
                    self.fail('Timed out waiting for transform between link1 and link2')

        node.destroy_node()


@launch_testing.post_shutdown_test()
class TestHelloWorldShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check if the processes exited normally."""
        launch_testing.asserts.assertExitCodes(proc_info)
