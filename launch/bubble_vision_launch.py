#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera_device', default_value='0', description='Camera device ID'),
        DeclareLaunchArgument('http_port', default_value='8000', description='HTTP server port'),
        DeclareLaunchArgument('ws_port', default_value='9000', description='WebSocket server port'),
        
        Node(
            package='camera_vision',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[{
                'device_id': LaunchConfiguration('camera_device'),
                'fps': 30,
                'width': 640,
                'height': 480
            }]
        ),
        
        Node(
            package='camera_vision',
            executable='vision_processor',
            name='vision_processor',
            output='screen',
            parameters=[{
                'bubble_min_radius': 5,
                'bubble_max_radius': 50,
                'color_change_threshold': 30.0,
                'circularity_threshold': 0.65,
                'solidity_threshold': 0.75,
                'brightness_diff_threshold': 15.0
            }]
        ),
        
        Node(
            package='vision_web',
            executable='web_server',
            name='web_server',
            output='screen',
            parameters=[{
                'http_port': LaunchConfiguration('http_port'),
                'ws_port': LaunchConfiguration('ws_port')
            }]
        ),
    ])
