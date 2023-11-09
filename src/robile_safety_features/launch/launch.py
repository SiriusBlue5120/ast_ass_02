from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robile_safety_features',
            executable='my_node',
            name='my_node'),
        Node(
            package='robile_safety_features',
            executable='scanner',
            name='scanner'),
        # Node(
        #     package='robile_safety_features',
        #     executable='safety_monitoring_BT',
        #     name='safety_monitoring_BT'),
  ])