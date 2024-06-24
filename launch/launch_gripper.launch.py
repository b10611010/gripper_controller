from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    gripper_ros2_pkg = Node(
        package='dh_gripper_ros2',
        executable='dh_PGE_gripper_ros2',
        output = 'screen'
    )
    gripper_controller = Node(
        package='gripper_controller',
        executable='gripper_controller',
        output = 'screen'
    )

    return LaunchDescription([
        gripper_ros2_pkg,
        gripper_controller
    ])