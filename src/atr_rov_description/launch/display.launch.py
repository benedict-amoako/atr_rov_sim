import subprocess
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    urdf = PathJoinSubstitution(
        [FindPackageShare('atr_rov_description'), 'urdf', 'atr_rov.urdf.xacro'])
    
    rviz_config = PathJoinSubstitution(
        [FindPackageShare('atr_rov_description'), 'config', 'atr_rov.rviz'])

    robot_description = ParameterValue(
        Command(['xacro ', urdf, ' fixed_to_world:=true']),
        value_type=str)

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'publish_all_joints': True}])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen')

    return LaunchDescription([robot_state_pub, rviz])