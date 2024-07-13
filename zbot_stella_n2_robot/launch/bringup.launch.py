from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os



def generate_launch_description():
    yd_params = os.path.join(get_package_share_directory('zbot_stella_n2_robot'), 
                                  'params', 'YDX4.yaml')
    mw_ahrs_params = os.path.join(get_package_share_directory('zbot_stella_n2_robot'),
                                    'params', 'mw_ahrs.yaml')
    imu_filter_params = os.path.join(get_package_share_directory('zbot_stella_n2_robot'),
                                        'params', 'imu_filter.yaml')
    mw_md_params = os.path.join(get_package_share_directory('zbot_stella_n2_robot'),
                                    'params', 'mw_md.yaml')

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
            parameters=[mw_md_params],
            remappings=[
                ('odom', 'mw_md/odom'),
                ('cmd_vel', 'zbot_stella_n2_velocity_controller/cmd_vel_unstamped')

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

    zbot_stella_n2_sensors_group_action = GroupAction([

        #IMU
        Node(
            package='stella_ahrs',
            executable='stella_ahrs_node',
            name='stella_ahrs_node',
            output='screen',
            parameters=[mw_ahrs_params],
            remappings=[
                ('imu/data', 'mw_ahrs_imu/data'),
                ('imu/data_raw', 'mw_ahrs_imu/data_raw'),
                ('imu/mag', 'mw_ahrs_imu/mag'),
                ('imu/yaw', 'mw_ahrs_imu/yaw'),
            ]
        ),

        #IMU Filter
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_node',
            output='screen',
            parameters=[imu_filter_params],
            remappings=[
                ('imu/data_raw', 'mw_ahrs_imu/data'),
                ('imu/mag', 'mw_ahrs_imu/mag')
            ]
        ),

        #YDLidar
        Node(package='ydlidar',
            executable='ydlidar_node',
            name='ydlidar_node',
            output='screen',
            emulate_tty=True,
            parameters=[yd_params],
            namespace='/',
            remappings=[
                ('scan', 'front/scan')
            ]
        )
    ])

    robot_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('zbot_stella_n2_localization'), 'launch', 'ekf.launch.py']
            ))
    )

    

    ld = LaunchDescription()

    ld.add_action(zbot_stella_n2_bringup_group_action)
    ld.add_action(zbot_stella_n2_sensors_group_action)
    ld.add_action(robot_localization_launch)

    return ld