from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_setup(context, *args, **kwargs):
    model_path = LaunchConfiguration('model').perform(context)
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('humanoid_arm_description'),
        'rviz',
        'display.rviz',
    ]).perform(context)

    robot_description = {
        'robot_description': Command(['cat ', model_path])
    }

    return [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=[
                '--x', '0',
                '--y', '0',
                '--z', '0',
                '--roll', '0',
                '--pitch', '0',
                '--yaw', '0',
                '--frame-id', 'world',
                '--child-frame-id', 'base_dummy_link',
            ],
            output='screen',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_description],
            output='screen',
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(LaunchConfiguration('use_joint_state_gui')),
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen',
        ),
    ]


def generate_launch_description():
    model_arg = DeclareLaunchArgument(
        'model',
        default_value=PathJoinSubstitution([
            FindPackageShare('humanoid_arm_description'),
            'urdf',
            'test_arm.urdf',
        ]),
        description='Absolute path to the robot URDF file',
    )

    use_joint_state_gui_arg = DeclareLaunchArgument(
        'use_joint_state_gui',
        default_value='false',
        description='Launch joint_state_publisher_gui for manual joint tweaking',
    )

    return LaunchDescription([
        model_arg,
        use_joint_state_gui_arg,
        OpaqueFunction(function=_launch_setup),
    ])
