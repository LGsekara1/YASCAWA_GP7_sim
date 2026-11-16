import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():

    package_path = get_package_share_directory(
        'gp7robot_description'
    )

    urdf_path = os.path.join(
        package_path, 'urdf', 'GP7.urdf'
    )

    controllers_path = os.path.join(
        package_path, 'config', 'controllers.yaml'
    )

    with open(urdf_path, 'r') as file:
        robot_description = file.read()

    robot_description = robot_description.replace(
        'CONTROLLERS_YAML_PATH',
        controllers_path
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }],
        output='screen'
    )

    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '4', 'empty.sdf'],
        output='screen'
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'GP7Robot',
            '-x', '0',
            '-y', '0',
            '-z', '0'
        ],
        output='screen'
    )

    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )

    spawn_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        gazebo,

        TimerAction(
            period=5.0,
            actions=[spawn_robot]
        ),

        TimerAction(
            period=12.0,
            actions=[spawn_joint_state_broadcaster]
        ),

        TimerAction(
            period=15.0,
            actions=[spawn_arm_controller]
        )
    ])
