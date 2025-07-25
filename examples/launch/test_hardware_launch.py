import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    pkg_path = get_package_share_directory('tinymovr_ros2')
    urdf_path = os.path.join(pkg_path, 'description', 'test_rig.urdf.xacro')
    controllers_path = os.path.join(pkg_path, 'config', 'test_controllers.yaml')

    robot_description = Command(['xacro ', urdf_path])

    # This node is required by the controller manager
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # This is the node we wrote that hosts our hardware interface plugin
    control_node = Node(
        package='controller_manager',          # Use the official package
        executable='ros2_control_node',        # Use the official executable
        parameters=[{'robot_description': robot_description}, controllers_path],
        output='screen'
    )

    # This loads the joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        control_node,
        joint_state_broadcaster_spawner
    ])