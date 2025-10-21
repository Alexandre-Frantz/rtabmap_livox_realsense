
# Example:
#   $ ros2 launch velodyne_driver velodyne_driver_node-VLP16-launch.py
#   $ ros2 launch velodyne_pointcloud velodyne_transform_node-VLP16-launch.py
#
#   SLAM:
#   $ ros2 launch rtabmap_examples vlp16.launch.py


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import SetParameter

def generate_launch_description():
    
    # robot_namespace="leo04"
    robot_namespace = LaunchConfiguration('robot_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    deskewing = LaunchConfiguration('deskewing')

    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock if true'),
  
        DeclareLaunchArgument(
            'robot_namespace', 
            default_value='leo04',
            description='The robot namespace'),

        DeclareLaunchArgument(
            'deskewing', default_value='false',
            description='Enable lidar deskewing'),
        
        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),
        
        # # Nodes to launch
        # Node(
        #     package='rtabmap_sync', executable='rgb_sync', output='screen',
        #     parameters=[{
        #       'approx_sync':False
        #     }],
        #     remappings=[
        #     #   ('/depth/image', f'{robot_namespace}/realsense_d455/camera_depth'), # /realsense_camera_leo04/realsense_camera_leo04/aligned_depth_to_color/image_raw'),
        #       ('/rgb/camera_info' , f'{robot_namespace}/realsense_d455/camera_info'), #/realsense_camera_leo04/realsense_camera_leo04/color/camera_info'),
        #       ('/rgb/image', f'{robot_namespace}/realsense_d455/camera_raw'), #/realsense_camera_leo04/realsense_camera_leo04/color/image_raw')
        #     ]),

        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen', namespace=robot_namespace,
            parameters=[{
              # 'frame_id':f'{robot_namespace}/base_footprint',
              'frame_id' : [robot_namespace, '/base_footprint'],
              'map_frame_id' : [robot_namespace, '/map'],
              'subscribe_scan_cloud':True,
              'subscribe_depth':False,
              'subscribe_rgb': False,
              'approx_sync':True,
              'wait_for_transform':0.2,
              # RTAB-Map's internal parameters are strings:
              'RGBD/ProximityMaxGraphDepth': '0',
              'RGBD/ProximityPathMaxNeighbors': '1',
              'RGBD/AngularUpdate': '0.05',
              'RGBD/LinearUpdate': '0.05',
              'Mem/NotLinkedNodesKept': 'false',
              'Mem/STMSize': '30',
              'Mem/LaserScanNormalK': '20',
              'Reg/Strategy': '1',
              'Icp/VoxelSize': '0.1',
              'Icp/RangeMin': '0.0', # Ignore laser scan points on the robot itself
              'Icp/PointToPlaneK': '20',
              'Icp/PointToPlaneRadius': '0',
              'Icp/PointToPlane': 'true',
              'Icp/Iterations': '10',
              'Icp/Epsilon': '0.001',
              'Icp/MaxTranslation': '3',
              'Icp/MaxCorrespondenceDistance': '1',
              'Icp/Strategy': '1',
              'Icp/OutlierRatio': '0.7',
              'Icp/CorrespondenceRatio': '0.2',
              # Occupancy Grid / Map parameters
              'RGBD/CreateOccupancyGrid': 'true',
              'Grid/CellSize': '0.05',
              'Grid/Sensor': '0',
              'Grid/MaxObstacleHeight': '2.0',
              'Grid/MinGrounHeight':'-0.4',
              'Grid/MaxGroundHeight':'0.07',
              'Grid/MaxGroundAngle':'45',
              # 'Grid/RangeMax': '6.0',
              # 'Grid/ClusterRadious':'0.2',
              # 'Grid/MinClusterSize':'5'
              }],
            remappings=[
              # ('scan_cloud', f'{robot_namespace}/points_color'),
              ('scan_cloud', 'points_color'),

            ],
            arguments=[
              '-d' # This will delete the previous database (~/.ros/rtabmap.db)
            ]), 

        Node(
            package='rtabmap_odom', executable='icp_odometry', output='screen', namespace=robot_namespace,
            parameters=[{
              # 'frame_id':f'{robot_namespace}/base_footprint',
              'frame_id' : [robot_namespace, '/base_footprint'],
              'odom_frame_id' : [robot_namespace, '/odom'],
              'wait_for_transform':0.2,
              'expected_update_rate':15.0,
              'deskewing':deskewing,
              # RTAB-Map's internal parameters are strings:
              'Icp/PointToPlane': 'true',
              'Icp/Iterations': '10',
              'Icp/VoxelSize': '0.1',
              'Icp/Epsilon': '0.001',
              'Icp/PointToPlaneK': '20',
              'Icp/PointToPlaneRadius': '0',
              'Icp/MaxTranslation': '2',
              'Icp/MaxCorrespondenceDistance': '1',
              'Icp/RangeMin': '0.0', # Ignore laser scan points on the robot itself
              'Icp/Strategy': '1',
              'Icp/OutlierRatio': '0.7',
              'Icp/CorrespondenceRatio': '0.01',
              'Odom/ScanKeyFrameThr': '0.4',
              'OdomF2M/ScanSubtractRadius': '0.1',
              'OdomF2M/ScanMaxSize': '15000',
              'OdomF2M/BundleAdjustment': 'false',
            }],
            remappings=[
              # ('scan_cloud', f'{robot_namespace}/livox/lidar'),
              ('scan_cloud', 'livox/lidar'),
            ]),

        Node(
            package='rtabmap_util', executable='point_cloud_assembler', output='screen', namespace=robot_namespace,
            parameters=[{
              'max_clouds':10,
              'fixed_frame_id':'',
              'use_sim_time':use_sim_time,
            }],
            remappings=[
              ('cloud', 'odom_filtered_input_scan')
            ]),

        # Node(
        #     package='rtabmap_viz', executable='rtabmap_viz', output='screen',
        #     parameters=[{
        #       'frame_id': f'{robot_namespace}/base_footprint',
        #       'odom_frame_id':'odom',
        #       'subscribe_odom_info':True,
        #       'subscribe_scan_cloud':True,
        #       'approx_sync':False
        #     }],
        #     remappings=[
        #        ('scan_cloud', 'odom_filtered_input_scan')
        #     ]),
    ])