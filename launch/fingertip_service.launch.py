from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare configurable launch arguments

    # fingertip sensors
    fingertips_config_file = LaunchConfiguration('fingertips_config_file')
    fingertips_port = LaunchConfiguration('fingertips_port')
    fingertips_ip = LaunchConfiguration('fingertips_ip')

    return LaunchDescription([

        # Fingertip sensor service
        DeclareLaunchArgument('fingertips_config_file', default_value='/etc/xela/xServ_fingertips.ini'),
        DeclareLaunchArgument('fingertips_port', default_value='5000'),
        DeclareLaunchArgument('fingertips_ip', default_value='0.0.0.0'),

        # Run the finger sensor server
        ExecuteProcess(
            cmd=['sudo', '-n', 'xela_server', '-f', fingertips_config_file, '--port', fingertips_port],
            # output='screen'
        ),
        
        # Launch the finger sensor service node
        Node(
            package='xela_server_ros2',
            executable='xela_service_fingertips',
            name='xela_service_fingertips',
            arguments=['--ip', fingertips_ip, '--port', fingertips_port],
            # output='screen',
        ),
    ])
