import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('yolo_landmark')

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')]),
        launch_arguments={
            'slam_params_file': os.path.join(pkg_share, 'nav2_params', 'slam_toolbox_params.yaml'),
            'use_sim_time': 'true'}.items(),
    )

    return LaunchDescription([
            slam_launch
        ])