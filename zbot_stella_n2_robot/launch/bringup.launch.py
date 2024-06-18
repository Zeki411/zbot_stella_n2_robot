from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

    zbot_stella_n2_bringup_group_action = GroupAction([

        # Description
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('zbot_stella_n2_description'), 'launch', 'description.launch.py']
            ))
        ),

        # Base Bringup
        Node(
            package='zbot_stella_n2_robot_base',
            executable='base_node',
            name='base_node',
            output='screen',
            parameters=[
            ]
        ),

        # Base Teleop
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('zbot_stella_n2_control'), 'launch', 'teleop_base.launch.py']
            ))
        ),

        # Joy Teleop
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('zbot_stella_n2_control'), 'launch', 'teleop_joy.launch.py']
            ))
        ),
    ])

    

    ld = LaunchDescription()

    ld.add_action(zbot_stella_n2_bringup_group_action)
    return ld