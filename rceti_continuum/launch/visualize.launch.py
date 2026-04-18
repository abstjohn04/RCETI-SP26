import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    This launch file handles the visual bringup of the RCETI Continuum robot.
    It orchestrates the startup of the core kinematics engine, the robot state publisher,
    and RViz2. It utilizes timed delays to ensure the dynamic URDF is fully generated 
    by the core_node before the visualization tools attempt to read it.
    """
    rceti_continuum_path = get_package_share_directory('rceti_continuum')

    continuum_node = Node(package='rceti_continuum', executable='core_node', name='core_node', output='screen', parameters=[{ "number_of_sections": 5 }])

    urdf_file_path = os.path.join(rceti_continuum_path, 'urdf', 'generated_robot.urdf')

    rsp_node = TimerAction(
        period=2.0,
        actions=[Node(package='robot_state_publisher', executable='robot_state_publisher', name='robot_state_publisher', output='screen', arguments=[urdf_file_path])]
    )

    rviz_node = TimerAction(
        period=3.0,
        actions=[Node(package='rviz2', executable='rviz2', name='rviz', arguments=['-d', os.path.join(rceti_continuum_path, 'urdf', 'continuum.rviz')], output='screen')]
    )

    return LaunchDescription([continuum_node, rsp_node, rviz_node])
