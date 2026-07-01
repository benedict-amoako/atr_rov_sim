import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ctrl_pkg = get_package_share_directory('atr_rov_control')
    pid_cfg = os.path.join(ctrl_pkg, 'config', 'pid_gains.yaml')
    wls_cfg = os.path.join(ctrl_pkg, 'config', 'allocator_config.yaml')

    setpoint_manager = Node(
        package='atr_rov_control',
        executable='setpoint_manager',
        output='screen',
        parameters=[pid_cfg, {'use_sim_time': True}])
    
    cascaded_pid = Node(
        package='atr_rov_control',
        executable='cascaded_pid',
        name='cascaded_pid',  
        output='screen',
        parameters=[pid_cfg, {'use_sim_time': True}])
    
    wls_allocator = Node(
        package='atr_rov_control',
        executable='wls_allocator',
        output='screen',
        parameters=[wls_cfg, {'use_sim_time': True}])
    
    return LaunchDescription([
        setpoint_manager,
        cascaded_pid,
        wls_allocator
    ])