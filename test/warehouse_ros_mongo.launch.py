from launch import LaunchDescription
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest

def generate_test_description():
    return LaunchDescription([
        Node(
            package='warehouse_ros_mongo',
            executable='mongo_wrapper_ros.py',
            parameters=[
                {'warehouse_port': 33829},
                {'warehouse_host': 'localhost'},
                {'warehouse_plugin': 'warehouse_ros_mongo::MongoDatabaseConnection'}
            ],
            output='screen'
        ),

        ReadyToTest()
    ])
