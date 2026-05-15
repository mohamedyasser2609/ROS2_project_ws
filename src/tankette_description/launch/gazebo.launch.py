import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
import xacro


def generate_launch_description():
    # Get the package directory
    pkg_share = FindPackageShare(package='tankette_description').find('tankette_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'tankette.urdf')
    
    # Get the local_fusion package directory and EKF config
    local_fusion_dir = get_package_share_directory('local_fusion')
    ekf_config = os.path.join(
        local_fusion_dir,
        'config',
        'ekf.yaml'
    )
    
    # Get the uart_comstack package directory and SLAM config
    slam_config = os.path.join(
        get_package_share_directory('uart_comstack'),
        'config',
        'mapper_params_online_async.yaml'
    )
    
    # Process the URDF file
    robot_description = Command(['xacro ', urdf_file])
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )
    
    # Gazebo Server
    gazebo_server = ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen',
        cwd=[pkg_share]
    )
    
    # Gazebo Client
    gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )
    
    # Spawn Entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'tankette',
            '-topic', 'robot_description',
            '-x', '0',
            '-y', '0',
            '-z', '0'
        ],
        output='screen'
    )
    
    # ================= LOCAL EKF =================
    local_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='local_ekf',
        output='screen',
        parameters=[ekf_config]
    )
    
    # ================= SLAM TOOLBOX =================
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config,
            {
                'use_sim_time': False,
                'base_frame': 'base_link',
                'odom_frame': 'odom',
                'map_frame': 'map',
                'odom_topic': '/odometry/filtered',
                'publish_tf': True
            }
        ]
    )
    
    # ================= STATIC TRANSFORM PUBLISHER =================
    base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser',
        output='screen',
        arguments=[
            '0', '0', '0.1',
            '0', '0', '0',
            'base_link',
            'laser'
        ]
    )
    
    return LaunchDescription([
        robot_state_publisher,
        gazebo_server,
        gazebo_client,
        spawn_entity,
        local_ekf,
        slam_toolbox,
        base_to_laser,
    ])