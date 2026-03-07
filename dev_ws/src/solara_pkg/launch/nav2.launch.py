import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node, SetParameter
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
from launch.actions import TimerAction


def generate_launch_description():
    solara_pkg = get_package_share_directory('solara_pkg')
    params_file = os.path.join(solara_pkg, 'config', 'nav2_params.yaml')
    map_file = os.path.join(solara_pkg, 'maps', 'rgarden_map.yaml')

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    # Localization lifecycle nodes (order matters)
    localization_nodes = ['map_server', 'amcl']

    # Navigation lifecycle nodes (order matters)
    navigation_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'velocity_smoother',
        'collision_monitor',
        'bt_navigator',
        'waypoint_follower',
    ]

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        SetParameter(name='use_sim_time', value=True),
        TimerAction(
            period=10.0,  # wait 10 real seconds for AMCL to publish transforms
            actions=[
                # --- Localization ---
                Node(
                    package='nav2_map_server',
                    executable='map_server',
                    name='map_server',
                    output='screen',
                    respawn=True,
                    respawn_delay=2.0,
                    parameters=[configured_params, {'yaml_filename': map_file}],
                    remappings=remappings,
                ),
                Node(
                    package='nav2_amcl',
                    executable='amcl',
                    name='amcl',
                    output='screen',
                    respawn=True,
                    respawn_delay=2.0,
                    parameters=[configured_params],
                    remappings=remappings,
                ),
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_localization',
                    output='screen',
                    parameters=[{
                        'use_sim_time': True,
                        'autostart': True,
                        'bond_timeout': 30.0,
                        'node_names': localization_nodes,
                    }],
                ),

                # --- Navigation ---
                Node(
                    package='nav2_controller',
                    executable='controller_server',
                    name='controller_server',
                    output='screen',
                    respawn=True,
                    respawn_delay=2.0,
                    parameters=[configured_params],
                    remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
                ),
                Node(
                    package='nav2_planner',
                    executable='planner_server',
                    name='planner_server',
                    output='screen',
                    respawn=True,
                    respawn_delay=2.0,
                    parameters=[configured_params],
                    remappings=remappings,
                ),
                Node(
                    package='nav2_behaviors',
                    executable='behavior_server',
                    name='behavior_server',
                    output='screen',
                    respawn=True,
                    respawn_delay=2.0,
                    parameters=[configured_params],
                    remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
                ),
                Node(
                    package='nav2_velocity_smoother',
                    executable='velocity_smoother',
                    name='velocity_smoother',
                    output='screen',
                    respawn=True,
                    respawn_delay=2.0,
                    parameters=[configured_params],
                    remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
                ),
                Node(
                    package='nav2_collision_monitor',
                    executable='collision_monitor',
                    name='collision_monitor',
                    output='screen',
                    respawn=True,
                    respawn_delay=2.0,
                    parameters=[configured_params],
                    remappings=remappings,
                ),
                Node(
                    package='nav2_bt_navigator',
                    executable='bt_navigator',
                    name='bt_navigator',
                    output='screen',
                    respawn=True,
                    respawn_delay=2.0,
                    parameters=[configured_params],
                    remappings=remappings,
                ),
                Node(
                    package='nav2_waypoint_follower',
                    executable='waypoint_follower',
                    name='waypoint_follower',
                    output='screen',
                    respawn=True,
                    respawn_delay=2.0,
                    parameters=[configured_params],
                    remappings=remappings,
                ),
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_navigation',
                    output='screen',
                    parameters=[{
                        'use_sim_time': True,
                        'autostart': True,
                        'bond_timeout': 30.0,
                        'node_names': navigation_nodes,
                    }],
                ),
             ]
            ),

    ])














# from launch.actions import TimerAction

# def generate_launch_description():
#     nav2_bringup = get_package_share_directory('nav2_bringup')
#     solara_pkg = get_package_share_directory('solara_pkg')

#     return LaunchDescription([
#     IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(nav2_bringup, 'launch', 'localization_launch.py')
#         ),
#         launch_arguments={
#             'use_sim_time': 'true',
#             'map': os.path.join(solara_pkg, 'maps', 'rgarden_map.yaml'),
#             'params_file': os.path.join(solara_pkg, 'config', 'nav2_params.yaml'),
#         }.items()
#     ),
#     IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')
#         ),
#         launch_arguments={
#             'use_sim_time': 'true',
#             'params_file': os.path.join(solara_pkg, 'config', 'nav2_params.yaml'),
#         }.items()
#     ),
#     # Node(
#     # #     package='nav2_lifecycle_manager',
#     # #     executable='lifecycle_manager',
#     # #     name='lifecycle_manager_localization',
#     # #     parameters=[{
#     # #         'use_sim_time': True,
#     # #         'autostart': True,
#     # #         'bond_timeout': 30.0,  # ADD THIS
#     # #         'node_names': ['map_server', 'amcl']
#     # #     }]
#     # # ),
# ])