import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='robile_safety_features',
            executable='safety_monitoring_BT',
            name='safety_monitoring_BT'),
  ])