from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare(package='yolo_landmark')
    
    sdf_path = PathJoinSubstitution([pkg_share, 'sdf', 'robot.sdf'])
    rviz_config_path = PathJoinSubstitution([pkg_share, 'config', 'robot.rviz'])
    world_path = PathJoinSubstitution([pkg_share, 'worlds', 'world.world'])
    slam_params_file = PathJoinSubstitution([pkg_share, 'nav2_params', 'slam_toolbox_params.yaml'])
    nav2_params_file = PathJoinSubstitution([pkg_share, 'nav2_params', 'nav2_params.yaml'])

    def robot_state_publisher(context):
        robot_description_content = Command([
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution([
                FindPackageShare('yolo_landmark'),
                'urdf',
                'test_diff_drive.xacro.urdf'
            ]),
        ])

        robot_description = {'robot_description': robot_description_content}
        node_robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        )
        return [node_robot_state_publisher]
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', sdf_path,
            '-name', 'yolo_robot',
            '-x', '0', '-y', '0', '-z', '0.1'
        ],
        output='screen',
        parameters=[use_sim_time]
    )

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('yolo_landmark'),
            'config',
            'diff_drive_controller.yaml',
        ]
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    diff_drive_base_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_base_controller',
            '--param-file',
            robot_controllers,
            ],
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
            launch_arguments=[('gz_args', [' -r -v 1 ', world_path])]
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[diff_drive_base_controller_spawner],
            )
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            output='screen'
        ),

        gz_spawn_entity,

        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),

        DeclareLaunchArgument(
            'description_format',
            default_value='urdf',
            description='Robot description format to use, urdf or sdf'),

        OpaqueFunction(function=robot_state_publisher),
        
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                '/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
            ],
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'qos_overrides./camera/image_raw.reliability': 'reliable',
                'qos_overrides./camera_info.reliability': 'reliable'
            }]
        ),

        Node(
            package='yolo_landmark',
            executable='yolo_node',
            name='yolo_node',
            remappings=[
                ('/image_raw', '/camera/image_raw')
            ],
            parameters=[use_sim_time]
        ),
        
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,   
                'qos_overrides./scan.reliability': 'reliable',
                'qos_overrides./scan.depth': 10
            }]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_tf_publisher',
            arguments=['0', '0', '0.9', '0', '0', '0', 'base_link', 'yolo_robot/lidar_link/lidar'],
            parameters=[{'use_sim_time': use_sim_time}]
        ),
    
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('slam_toolbox'),
                    'launch',
                    'online_async_launch.py'
                ])
            ]),
            launch_arguments={
                'slam_params_file': PathJoinSubstitution([
                    pkg_share, 'nav2_params', 'mapper_params_online_async.yaml'
                ]),
                'use_sim_time': 'true'}.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen',
            parameters=[use_sim_time],
        )
    ])