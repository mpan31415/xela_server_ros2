from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare configurable launch arguments

    # finger sensors
    fingers_config_file = LaunchConfiguration('fingers_config_file')
    fingers_port = LaunchConfiguration('fingers_port')
    fingers_ip = LaunchConfiguration('fingers_ip')

    # palm sensors
    palm_config_file = LaunchConfiguration('palm_config_file')
    palm_port = LaunchConfiguration('palm_port')
    palm_ip = LaunchConfiguration('palm_ip')

    return LaunchDescription([

        # Finger sensor service
        DeclareLaunchArgument('fingers_config_file', default_value='/etc/xela/xServ_fingers.ini'),
        DeclareLaunchArgument('fingers_port', default_value='5000'),
        DeclareLaunchArgument('fingers_ip', default_value='0.0.0.0'),

        # Palm sensor service
        DeclareLaunchArgument('palm_config_file', default_value='/etc/xela/xServ_palm.ini'),
        DeclareLaunchArgument('palm_port', default_value='6000'),
        DeclareLaunchArgument('palm_ip', default_value='0.0.0.0'),

        # Run the finger sensor server
        ExecuteProcess(
            cmd=['sudo', '-n', 'xela_server', '-f', fingers_config_file, '--port', fingers_port],
            output='screen'
        ),
        # Run the palm sensor server
        ExecuteProcess(
            cmd=['sudo', '-n', 'xela_server', '-f', palm_config_file, '--port', palm_port],
            output='screen'
        ),
        
        # Launch the finger sensor service node
        Node(
            package='xela_server_ros2',
            executable='xela_service_fingers',
            name='xela_service_fingers',
            arguments=['--ip', fingers_ip, '--port', fingers_port],
            output='screen',
        ),
        # Launch the palm sensor service node
        Node(
            package='xela_server_ros2',
            executable='xela_service_palm',
            name='xela_service_palm',
            arguments=['--ip', palm_ip, '--port', palm_port],
            output='screen',
        ),
    ])
