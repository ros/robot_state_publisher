import os
import unittest

from launch import LaunchDescription
import launch
from launch_ros.actions import Node
import launch_testing
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_test_description(ready_fn):
    test_exe_arg = launch.actions.DeclareLaunchArgument(
        'test_exe',
        description='Path to executable test',
    )

    process_under_test = launch.actions.ExecuteProcess(
        cmd=[launch.substitutions.LaunchConfiguration('test_exe')],
        output='screen',
    )

    urdf_file = os.path.join(os.path.dirname(__file__), 'two_links_fixed_joint.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    params = {'robot_description': robot_desc}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        node_executable='robot_state_publisher_node',
        output='screen',
        parameters=[params]
    )

    return LaunchDescription([
        node_robot_state_publisher,
        test_exe_arg,
        process_under_test,
        launch_testing.util.KeepAliveProc(),
        launch.actions.OpaqueFunction(function=lambda context: ready_fn()),
        ]), locals()


class TestBufferClient(unittest.TestCase):

    def test_termination(self, process_under_test, proc_info):
        proc_info.assertWaitForShutdown(process=process_under_test, timeout=(10))


@launch_testing.post_shutdown_test()
class BufferClientTestAfterShutdown(unittest.TestCase):

    def test_exit_code(self):
        # Check that all processes in the launch (in this case, there's just one) exit
        # with code 0
        launch_testing.asserts.assertExitCodes(self.proc_info)
