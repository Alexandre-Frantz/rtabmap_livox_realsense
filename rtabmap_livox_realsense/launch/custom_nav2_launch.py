from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import SetRemap
import os

def generate_launch_description():
    
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch_file = os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')

    # Launch-time args
    params_file = LaunchConfiguration('params_file')
    cmd_vel_out = LaunchConfiguration('cmd_vel_out')
    use_sime_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        
        DeclareLaunchArgument(
            'params_file',
            default_value='/home/xr-dev/ros2_ws/config_files/nav2_params_repo.yaml'
        ),
        DeclareLaunchArgument(
            'cmd_vel_out',
            default_value='/leo04/cmd_vel'
        ),

        DeclareLaunchArgument(
            'use_sim_time',                     
            default_value='true',               # true for Gazebo; set to 'false' on real robot
            description='Use simulation (Gazebo) clock if true'
        ),

        # Global remaps: everything launched after this will inherit these
        SetRemap(src='cmd_vel', dst=cmd_vel_out),
        SetRemap(src='/cmd_vel', dst=cmd_vel_out),

        # Bring up Nav2 with params
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_file),
            launch_arguments={
                'params_file': params_file,
                'use_sim_time' : use_sime_time,
                              }.items(),
        ),
    ])

