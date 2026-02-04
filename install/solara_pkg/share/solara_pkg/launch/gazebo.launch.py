import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro

def generate_launch_description():

    # Get the package directory
    pkg_name = 'solara_pkg'
    pkg_share = get_package_share_directory(pkg_name)
    
    # Path to xacro file
    xacro_file = os.path.join(pkg_share, 'description', 'robot.urdf.xacro')
    bridge_config = os.path.join(pkg_share, 'config', 'gz_bridge.yaml')
    # world_file = os.path.join(pkg_share, 'worlds', 'garden.sdf')


    
    # Process xacro to get robot description
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

  # Launch Gazebo Harmonic
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={'gz_args': '-r -s empty.sdf'}.items()
        # SABRINA REMOVE -s before commit
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_solara',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'solara_rover',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5'  # Spawn slightly above ground
        ]
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        arguments=[
            '--ros-args',
            '-p', f'config_file:={bridge_config}'
        ],
        output='screen'
    )
    
    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        spawn_entity,
        bridge
    ])


