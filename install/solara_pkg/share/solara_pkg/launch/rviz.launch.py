import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # Get the package directory
    pkg_name = 'solara_pkg'
    pkg_share = get_package_share_directory(pkg_name)
    
    # Path to the xacro file
    xacro_file = os.path.join(pkg_share, 'description', 'robot.urdf.xacro')
    
    # Process the xacro file to get robot description
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}
    
    
    # Robot State Publisher - publishes robot transforms
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )
    
    # Joint State Publisher GUI - lets you move joints manually
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )
    
    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )
    
    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])


# heyyyy girl!! template i found on the standard robot simulation for ros2 (turtlebot 3 repo)

##################################

# import os

# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch_ros.actions import Node


# def generate_launch_description():
#     rviz_config_dir = os.path.join(
#         get_package_share_directory('turtlebot3_description'),
#         'rviz',
#         'model.rviz')

#     return LaunchDescription([
#         Node(
#             package='rviz2',
#             executable='rviz2',
#             name='rviz2',
#             arguments=['-d', rviz_config_dir],
#             output='screen'),
#     ])

################################






