import unittest

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_testing.actions import ReadyToTest, GTest
import launch_testing
from launch_testing.asserts import assertExitCodes


def generate_test_description():
    test_warehouse_ros_mongo_cpp = GTest(
        path=[
            PathJoinSubstitution(
                [LaunchConfiguration("test_binary_dir"), "test_warehouse_ros_mongo_cpp"]
            )
        ],
        output="screen",
    )
    return (
        LaunchDescription(
            [
                Node(
                    package="warehouse_ros_mongo",
                    executable="mongo_wrapper_ros.py",
                    parameters=[
                        {"warehouse_port": 33829},
                        {"warehouse_host": "localhost"},
                        {
                            "warehouse_plugin": "warehouse_ros_mongo::MongoDatabaseConnection"
                        },
                    ],
                    output="screen",
                ),
                test_warehouse_ros_mongo_cpp,
                ReadyToTest(),
            ]
        ),
        {"test_warehouse_ros_mongo_cpp": test_warehouse_ros_mongo_cpp},
    )


class TestTerminatingProcessStops(unittest.TestCase):
    def test_gtest_run_complete(self, proc_info, test_warehouse_ros_mongo_cpp):
        proc_info.assertWaitForShutdown(
            process=test_warehouse_ros_mongo_cpp, timeout=60
        )


@launch_testing.post_shutdown_test()
class TaskModelTestAfterShutdown(unittest.TestCase):
    def test_exit_code(self, proc_info, test_warehouse_ros_mongo_cpp):
        # Check that all processes in the launch exit with code 0
        launch_testing.asserts.assertExitCodes(
            proc_info, process=test_warehouse_ros_mongo_cpp
        )
