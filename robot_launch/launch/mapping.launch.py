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
    ## for lidar
    # channel_type =  LaunchConfiguration('channel_type', default='serial')
    # serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    # serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200')
    # frame_id = LaunchConfiguration('frame_id', default='laser')
    # inverted = LaunchConfiguration('inverted', default='false')
    # angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    # scan_mode = LaunchConfiguration('scan_mode', default='Sensitivity')

    # channel_type_Declare = DeclareLaunchArgument(
    #     'channel_type',
    #     default_value=channel_type,
    #     description='Specifying channel type of lidar')
    
    # serial_port_Declare = DeclareLaunchArgument(
    #     'serial_port',
    #     default_value=serial_port,
    #     description='Specifying usb port to connected lidar')

    # serial_baudrate_Declare = DeclareLaunchArgument(
    #     'serial_baudrate',
    #     default_value=serial_baudrate,
    #     description='Specifying usb port baudrate to connected lidar')
    
    # frame_id_Declare = DeclareLaunchArgument(
    #     'frame_id',
    #     default_value=frame_id,
    #     description='Specifying frame_id of lidar')

    # inverted_Declare = DeclareLaunchArgument(
    #     'inverted',
    #     default_value=inverted,
    #     description='Specifying whether or not to invert scan data')

    # angle_compensate_Declare = DeclareLaunchArgument(
    #     'angle_compensate',
    #     default_value=angle_compensate,
    #     description='Specifying whether or not to enable angle_compensate of scan data')
    
    # scan_mode_Declare = DeclareLaunchArgument(
    #     'scan_mode',
    #     default_value=scan_mode,
    #     description='Specifying scan mode of lidar')

    # Lidar_node = Node(
    #     package='sllidar_ros2',
    #     executable='sllidar_node',
    #     name='sllidar_node',
    #     parameters=[{'channel_type':channel_type,
    #                     'serial_port': serial_port, 
    #                     'serial_baudrate': serial_baudrate, 
    #                     'frame_id': frame_id,
    #                     'inverted': inverted, 
    #                     'angle_compensate': angle_compensate}],
    #     output='screen')
    # TF2_lidar_node = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_transform_publisher',
    #     arguments=['0.3', '0', '0', '0', '3.14', '-3.14', 'base_footprint', 'laser'],
    # )

    # micro_ros = Node(
    #         package='micro_ros_agent',
    #         executable='micro_ros_agent',
    #         name='micro_ros_agent',
    #         namespace='micro_ros_agent',
    #         parameters=[],  # Add parameters if needed
    #         arguments=[
    #             'serial', '--dev', '/dev/ttyACM0', '-b', '115200'
    #         ],
    #         output = 'screen'

    #     )







    slam_params_file = LaunchConfiguration('slam_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    slam_params_file_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(
            get_package_share_directory("robot_launch"), 'config', 'online_async.yaml'
        ),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node',
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation/Gazebo clock'
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': use_sim_time}],
    )
    
    # odom_node = Node(
    #     package='robot_odom',
    #     executable='odom_pub_node',
    #     # output='screen',
    # )

    # joy_node = Node(
    #     package='joy_package',
    #     executable='convert_node',
    #     # output='screen',
    # )


    ld = LaunchDescription()

    ld.add_action(use_sim_time_arg)
    ld.add_action(slam_params_file_arg)

    ld.add_action(slam_node)

    return ld