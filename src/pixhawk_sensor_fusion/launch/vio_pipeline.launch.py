import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('pixhawk_sensor_fusion')
    rtabmap_launch_dir = get_package_share_directory('rtabmap_launch')
    realsense_launch_dir = get_package_share_directory('realsense2_camera')

    # Launch Configurations
    cam_x = LaunchConfiguration('cam_x')
    cam_y = LaunchConfiguration('cam_y')
    cam_z = LaunchConfiguration('cam_z')
    cam_roll = LaunchConfiguration('cam_roll')
    cam_pitch = LaunchConfiguration('cam_pitch')
    cam_yaw = LaunchConfiguration('cam_yaw')
    use_ekf = LaunchConfiguration('use_ekf')
    rtabmap_viz = LaunchConfiguration('rtabmap_viz')
    rviz = LaunchConfiguration('rviz')
    launch_camera = LaunchConfiguration('launch_camera')

    # 1. RealSense Camera Launch (with initial_reset and no heavy pointcloud)
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(realsense_launch_dir, 'launch', 'rs_launch.py')),
        condition=IfCondition(launch_camera),
        launch_arguments={
            'align_depth.enable': 'true',
            'pointcloud.enable': 'false',
            'initial_reset': 'false',
            'depth_module.depth_profile': '640x480x30',
            'rgb_camera.color_profile': '640x480x30',
        }.items()
    )

    # 2. Static TF: base_link -> camera_link
    static_tf_base_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_tf',
        arguments=[
            '--x', cam_x, '--y', cam_y, '--z', cam_z,
            '--roll', cam_roll, '--pitch', cam_pitch, '--yaw', cam_yaw,
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_link'
        ]
    )

    # 3. RTAB-Map Visual Odometry & SLAM
    publish_tf_odom = PythonExpression(["'false' if '", use_ekf, "' == 'true' else 'true'"])

    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(rtabmap_launch_dir, 'launch', 'rtabmap.launch.py')),
        launch_arguments={
            'rgb_topic': '/camera/camera/color/image_raw',
            'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
            'camera_info_topic': '/camera/camera/color/camera_info',
            'frame_id': 'base_link',
            'approx_sync': 'true',
            'qos': '1',
            'publish_tf_odom': publish_tf_odom,
            'rtabmap_viz': rtabmap_viz,
            'rviz': rviz,
            'odom_args': '--Vis/MinInliers 12 --Vis/MaxFeatures 1500 --Odom/ResetCountdown 1',
            'args': '-d',
        }.items()
    )

    # 4. VIO to PX4 Bridge Node
    vio_px4_bridge_node = Node(
        package='pixhawk_sensor_fusion',
        executable='vio_px4_bridge',
        name='vio_px4_bridge',
        output='screen',
        parameters=[{
            'odom_topic': '/rtabmap/odom',
            'px4_topic': '/fmu/in/vehicle_visual_odometry',
            'max_allowed_variance': 90.0,
            'default_position_variance': 0.01,
            'default_orientation_variance': 0.01,
        }]
    )

    # 5. Optional EKF Node (robot_localization)
    ekf_config_path = os.path.join(pkg_share, 'config', 'ekf.yaml')
    ekf_node = Node(
        condition=IfCondition(use_ekf),
        package='robot_localization',
        executable='ekf_node',
        name='filter_node_full',
        output='screen',
        parameters=[ekf_config_path]
    )

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument('launch_camera', default_value='true', description='Whether to launch RealSense camera node'),
        DeclareLaunchArgument('cam_x', default_value='0.10', description='Camera offset X from Pixhawk (m)'),
        DeclareLaunchArgument('cam_y', default_value='0.0', description='Camera offset Y from Pixhawk (m)'),
        DeclareLaunchArgument('cam_z', default_value='0.0', description='Camera offset Z from Pixhawk (m)'),
        DeclareLaunchArgument('cam_roll', default_value='0.0', description='Camera roll angle (rad)'),
        DeclareLaunchArgument('cam_pitch', default_value='0.0', description='Camera pitch angle (rad)'),
        DeclareLaunchArgument('cam_yaw', default_value='0.0', description='Camera yaw angle (rad)'),
        DeclareLaunchArgument('use_ekf', default_value='false', description='Whether to run robot_localization EKF'),
        DeclareLaunchArgument('rtabmap_viz', default_value='true', description='Open RTAB-Map visualizer GUI'),
        DeclareLaunchArgument('rviz', default_value='false', description='Open RViz for map visualization'),

        # Nodes & Launches
        realsense_launch,
        static_tf_base_camera,
        rtabmap_launch,
        vio_px4_bridge_node,
        ekf_node,
    ])
