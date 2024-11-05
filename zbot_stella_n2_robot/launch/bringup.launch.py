from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
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
    

    declare_use_lidar = DeclareLaunchArgument(
        'use_lidar',
        default_value='false',
        description='Whether to use the lidar or not'
    )

    declare_use_rscam = DeclareLaunchArgument(
        'use_rscam',
        default_value='false',
        description='Whether to use the rscam or not'
    )

    declare_use_gps = DeclareLaunchArgument(
        'use_gps',
        default_value='false',
        description='Whether to use the gps or not'
    )

    use_lidar = LaunchConfiguration('use_lidar')
    use_rscam = LaunchConfiguration('use_rscam')
    use_gps = LaunchConfiguration('use_gps')

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
                ('cmd_vel', 'zbot_stella_n2_velocity_controller/cmd_vel_unstamped'),
                ('odom', 'mw_md/odom'),
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

        # IMU
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

        # Node(
        #     package='umx_driver',
        #     executable='um7_driver',
        #     output='both',
        #     parameters=[
        #         {'port': '/dev/ttyUSB1'},
        #         {'baud': 115200},
        #         {'frame_id': 'imu_link'},
        #     ],
        #     remappings=[
        #         ('/imu/data', '/um7_imu/data'),
        #         ('/imu/mag', '/um7_imu/mag'),
        #         ('/imu/rpy', '/um7_imu/rpy'),
        #         ('/imu/temperature', '/um7_imu/temperature')
        #     ]
        # ),

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

                # ('imu/data_raw', 'um7_imu/data'),
                # ('imu/mag', 'um7_imu/mag')
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
            ],
            condition=IfCondition(use_lidar)
        ),

        #RSCam

    ])

    rscam_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py']
            )),
            launch_arguments={
                'camera_name': 'front_rscam',
                'camera_namespace': '',
                'config_file': get_package_share_directory('zbot_stella_n2_robot') + '/params/rscam.yaml'
            }.items(),
            condition=IfCondition(use_rscam)
        )
    
    gps_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('ublox_gnss'), 'gnssrtk.launch.py']
            )),

            condition=IfCondition(use_gps)
    )

    robot_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('zbot_stella_n2_localization'), 'launch', 'ekf.launch.py']
            ))
    )

    

    ld = LaunchDescription()
    ld.add_action(declare_use_lidar)
    ld.add_action(declare_use_rscam)
    ld.add_action(declare_use_gps)

    ld.add_action(zbot_stella_n2_bringup_group_action)
    ld.add_action(zbot_stella_n2_sensors_group_action)
    ld.add_action(rscam_launch)
    ld.add_action(gps_launch)
    # ld.add_action(robot_localization_launch)

    return ld