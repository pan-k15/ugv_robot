from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model',       default_value='yolov8n.pt',
                              description='Ultralytics model file or name'),
        DeclareLaunchArgument('confidence',  default_value='0.5',
                              description='Detection confidence threshold'),
        DeclareLaunchArgument('device',      default_value='cpu',
                              description='Inference device: cpu / cuda:0 / mps'),
        DeclareLaunchArgument('image_topic', default_value='/front_camera/image_raw',
                              description='Input image topic'),

        Node(
            package='robot_vision',
            executable='yolo_node',
            name='yolo_node',
            output='screen',
            parameters=[{
                'model':       LaunchConfiguration('model'),
                'confidence':  LaunchConfiguration('confidence'),
                'device':      LaunchConfiguration('device'),
                'image_topic': LaunchConfiguration('image_topic'),
            }],
        ),
    ])
