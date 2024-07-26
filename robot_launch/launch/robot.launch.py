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

    TF2_camera_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0.5', '0', '0', '0', 'base_link', 'camera_link'],
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
    
    odom_node = Node(
        package='robot_odom',
        executable='odom_pub_node',
        # output='screen',
    )

    joy_convert = Node(
        package='joy_package',
        executable='convert_node',
        # output='screen',
    )
    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params],
            remappings=[('/cmd_vel_out','/cmd_vel_out')]
        )
    joy_node = Node(
            package='joy',
            executable='joy_node',
         )
    camera_launch = Node(
            package='realsense2_camera',
            executable='rs_launch.py',
            name='realsense2_camera',
            parameters=[{'enable_pointcloud': True}],
            arguments=['device_type:=d415']
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
                'range_min': 0.3,
                'range_max': 20.0,
                'scan_time': 0.033,
                'update_rate': 10.0,
                'scan_height': 1,
            }]
    )

    ld = LaunchDescription()
    
    ld.add_action(channel_type_Declare)
    ld.add_action(serial_port_Declare)
    ld.add_action(serial_baudrate_Declare)
    ld.add_action(frame_id_Declare)
    ld.add_action(inverted_Declare)
    ld.add_action(angle_compensate_Declare)
    ld.add_action(scan_mode_Declare)

    ld.add_action(twist_mux)
    ld.add_action(Lidar_node)
    ld.add_action(TF2_lidar_node)
    ld.add_action(odom_node)
    # ld.add_action(joy_node)
    # ld.add_action(camera_launch)
    # ld.add_action(TF2_camera_node)
    ld.add_action(joy_convert)
    ld.add_action(micro_ros)

    return ld