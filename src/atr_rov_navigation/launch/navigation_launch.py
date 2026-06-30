import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    nav_pkg = get_package_share_directory('atr_rov_navigation')
    ekf_config = os.path.join(nav_pkg, 'config', 'ekf_config.yaml')

    # 1. Static transform - ned_odom parent of odom (ENU to NED)
    enu_to_ned = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '3.14159', '--pitch', '0', '--yaw', '1.5708',
            '--frame-id', 'ned_odom', '--child-frame-id', 'odom'
        ],
        name='enu_to_ned_tf',
        parameters=[{'use_sim_time': True}])
    
    # 2. IMU republisher - fix frame ID and populate covariance
    imu_rep = Node(
        package='atr_rov_navigation',
        executable='imu_republisher',
        output='screen',
        parameters=[{'use_sim_time': True}])
    
    # 3. Depth driver - pressure to depth in metres
    depth_driver = Node(
        package='atr_rov_navigation',
        executable='depth_driver',
        output='screen',
        parameters=[{'use_sim_time': True}])
    
    # 4. DVL driver - velocity publiusher (simulation proxy)
    dvl_driver = Node(
        package='atr_rov_navigation',
        executable='dvl_driver',
        output='screen',
        parameters=[{'use_sim_time': True}])
    
    # 5. Sonar republisher - fix frame ID
    sonar = Node(
        package='atr_rov_navigation',
        executable='sonar_republisher',
        output='screen',
        parameters=[{'use_sim_time': True}])
    
    # 6. EKF node
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': True}])
    
    return LaunchDescription([
        enu_to_ned,
        imu_rep,
        depth_driver,
        dvl_driver,
        sonar,
        ekf
    ])