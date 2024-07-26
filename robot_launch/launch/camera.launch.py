import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_name='robot_launch' #<--- CHANGE ME
    TF2_camera_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0.3', '0', '0.2', '0', '0', '0', 'base_link', 'camera_link'],
    )
    lidar_1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '1', '0', '0', '0', 'camera_link', 'lidar_1'],
    )
    lidar_2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '2', '0', '0', '0', 'camera_link', 'lidar_2'],
    )
    lidar_3 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0.5', '0', '0', '0', 'camera_link', 'lidar_3'],
    )
    


    depthimage_to_laserscan = Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan',
            output='screen',
            remappings=[('depth','/camera/camera/depth/image_rect_raw'),
                        ('depth_camera_info', '/camera/camera/depth/camera_info'),
                        ('scan', '/rgbd_scan')],
            parameters=[{
                'output_frame': 'camera_link',
                'range_min': 0.2,
                'range_max': 3.0,
                'scan_time': 0.033,
                'update_rate': 10.0,
                'scan_height': 1,
            }]
    )
    depthimage_to_laserscan_2 = Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan',
            output='screen',
            remappings=[('depth','/camera/camera/depth/image_rect_raw'),
                        ('depth_camera_info', '/camera/camera/depth/camera_info'),
                        ('scan', '/rgbd_scan_2')],
            parameters=[{
                'output_frame': 'lidar_2',
                'range_min': 0.3,
                'range_max': 20.0,
                'scan_time': 0.033,
                'update_rate': 10.0,
                'scan_height': 2,
            }]
    )
    depthimage_to_laserscan_3 = Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan',
            output='screen',
            remappings=[('depth','/camera/camera/depth/image_rect_raw'),
                        ('depth_camera_info', '/camera/camera/depth/camera_info'),
                        ('scan', '/rgbd_scan_3')],
            parameters=[{
                'output_frame': 'lidar_3',
                'range_min': 0.3,
                'range_max': 20.0,
                'scan_time': 0.033,
                'update_rate': 10.0,
                'scan_height': 3,
            }]
    )

    ld = LaunchDescription()
    ld.add_action(TF2_camera_node)
    ld.add_action(lidar_1)
    # ld.add_action(lidar_2)
    # ld.add_action(lidar_3)

    ld.add_action(depthimage_to_laserscan)
    # ld.add_action(depthimage_to_laserscan_2)
    # ld.add_action(depthimage_to_laserscan_3)
    return ld