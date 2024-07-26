import os

import launch.actions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    
    nav2_yaml = os.path.join(get_package_share_directory('robot_launch'), 'config', 'amcl_param.yaml')
    map_file = os.path.join(get_package_share_directory('robot_launch'), 'maps', 'map_v2.yaml')
    remappings = [('/tf', 'tf'),
                    ('/tf_static', 'tf_static')]
    
    use_respawn = LaunchConfiguration('use_respawn')

    use_sim_time = LaunchConfiguration('use_sim_time')

    autostart = LaunchConfiguration('autostart')

    use_respawn = LaunchConfiguration('use_respawn')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')
    
    lifecycle_nodes = ['map_server', 'amcl']


        ## for lidar
    channel_type =  LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200')
    frame_id = LaunchConfiguration('frame_id', default='laser')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Sensitivity')

    channel_type_Declare = DeclareLaunchArgument(
        'channel_type',
        default_value=channel_type,
        description='Specifying channel type of lidar')
    
    serial_port_Declare = DeclareLaunchArgument(
        'serial_port',
        default_value=serial_port,
        description='Specifying usb port to connected lidar')

    serial_baudrate_Declare = DeclareLaunchArgument(
        'serial_baudrate',
        default_value=serial_baudrate,
        description='Specifying usb port baudrate to connected lidar')
    
    frame_id_Declare = DeclareLaunchArgument(
        'frame_id',
        default_value=frame_id,
        description='Specifying frame_id of lidar')

    inverted_Declare = DeclareLaunchArgument(
        'inverted',
        default_value=inverted,
        description='Specifying whether or not to invert scan data')

    angle_compensate_Declare = DeclareLaunchArgument(
        'angle_compensate',
        default_value=angle_compensate,
        description='Specifying whether or not to enable angle_compensate of scan data')
    
    scan_mode_Declare = DeclareLaunchArgument(
        'scan_mode',
        default_value=scan_mode,
        description='Specifying scan mode of lidar')

    Lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{'channel_type':channel_type,
                        'serial_port': serial_port, 
                        'serial_baudrate': serial_baudrate, 
                        'frame_id': frame_id,
                        'inverted': inverted, 
                        'angle_compensate': angle_compensate}],
        output='screen')
    TF2_lidar_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0.3', '0', '0', '0', '3.14', '-3.14', 'base_link', 'laser'],
    )

    odom_node = Node(
        package='robot_odom',
        executable='odom_pub_node',
        # output='screen',
    )

    joy_node = Node(
        package='joy_package',
        executable='convert_node',
        # output='screen',
    )
    micro_ros = Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            namespace='micro_ros_agent',
            parameters=[],  # Add parameters if needed
            arguments=[
                'serial', '--dev', '/dev/ttyACM0', '-b', '115200'
            ],
            output = 'screen'

        )

    return LaunchDescription([
        declare_use_respawn_cmd,
        declare_autostart_cmd,
        declare_use_sim_time_cmd,

        channel_type_Declare,
        serial_port_Declare,
        serial_baudrate_Declare,
        frame_id_Declare,
        inverted_Declare,
        angle_compensate_Declare,
        scan_mode_Declare,
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            respawn=use_respawn,
            respawn_delay=2.0,
            output='screen',
            parameters=[nav2_yaml, {'yaml_filename':map_file} ],
            remappings=remappings),
            
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            respawn=use_respawn,
            respawn_delay=2.0,
            output='screen',
            parameters=[nav2_yaml],
            remappings=remappings
            ),


        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}])

        ])


## ros2 launch nav2_bringup localization_launch.py map:=./use_map.yaml use_sim_time:=false