from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command
import os

def generate_launch_description():
    ros2_ws_path = os.path.expanduser('~/ros2_ws')
    urdf_path = os.path.join(ros2_ws_path, 'src/yolo_landmark/urdf/robot.sdf')
    rviz_config_path = os.path.join(ros2_ws_path, 'src/yolo_landmark/config/robot.rviz')
    world_path = os.path.join(ros2_ws_path, 'src/yolo_landmark/worlds/world.world')

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_path],
            output='screen'
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'source_list': ['joint_states']}]
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', urdf_path]),
                'publish_frequency': 30.0,
                'use_sim_time': True 
            }]
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic', '/robot_description',
                '-name', 'yolo_robot',
                '-x', '0', '-y', '0', '-z', '0.1'
            ],
            output='screen'
        ),
        
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                '/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
            ],
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'qos_overrides./camera/image_raw.reliability': 'reliable',
                'qos_overrides./camera_info.reliability': 'reliable'
            }]
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
            ],
            output='screen'
        ),

        Node(
            package='yolo_landmark',
            executable='yolo_node',
            name='yolo_node',
            remappings=[
                ('/image_raw', '/camera/image_raw')
            ],
            parameters=[{'use_sim_time': True}]
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])