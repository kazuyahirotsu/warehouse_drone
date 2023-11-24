from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera@sensor_msgs/msg/Image@ignition.msgs.Image',
            '--ros-args',
            '-p', 'reliability:=best_effort',
            '-p', 'history_depth:=10'
        ],
        remappings=[('/camera', '/output/image_raw')]
    )

    return LaunchDescription([
        bridge,
    ])
