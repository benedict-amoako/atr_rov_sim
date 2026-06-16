import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    gz_pkg   = get_package_share_directory('atr_rov_gazebo')
    desc_pkg = get_package_share_directory('atr_rov_description')
    mesh_path = os.path.join(desc_pkg, 'meshes')

    world = os.path.join(gz_pkg, 'worlds', 'underwater.sdf')
    urdf  = os.path.join(desc_pkg, 'urdf', 'atr_rov.urdf.xacro')
    bridge_cfg = os.path.join(gz_pkg, 'config', 'ros_gz_bridge.yaml')

    robot_description = ParameterValue(
        Command(['xacro ', urdf, ' mesh_path:=', mesh_path]),
        value_type=str)

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ros_gz_sim'), '/launch/gz_sim.launch.py']),
        launch_arguments={'gz_args': f'-r {world}'}.items())

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }])

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'atr_mp_rov',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '-2.0',
        ],
        output='screen')
    
    bridge = Node(
        package='ros_gz_bridge',
        executable='bridge_node',
        parameters=[{
            'config_file': bridge_cfg,
        }],
        output='screen')
    
    spawn_delayed = TimerAction(period=5.0, actions=[spawn])

    return LaunchDescription([gz_sim, rsp, spawn_delayed, bridge])