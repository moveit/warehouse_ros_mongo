# Copyright 2008 Willow Garage, Inc.
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
#    * Neither the name of the Willow Garage, Inc. nor the names of its
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
