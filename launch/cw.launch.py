import os
import time

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    socspioneer_p = get_package_share_directory("socspioneer")
    pf_p = get_package_share_directory("pf_localisation")
    foxglove_bridge_p = get_package_share_directory("foxglove_bridge")

    world_path = DeclareLaunchArgument(
        name="world_path",
        default_value= "/home/lukas/MOB/MOB_CW1/data/sim_data/meeting.world"
    )

    map_path = DeclareLaunchArgument(
        name="map_path",
        default_value= "/home/lukas/MOB/MOB_CW1/data/sim_data/meeting.yaml"    
    )

    lifecycle_manager = Node(
    package="nav2_lifecycle_manager",
    executable="lifecycle_manager",
    parameters=[{"node_names": ["amcl","map_server"], "autostart": True}],
    arguments= [
    '--ros-args',
    '--log-level',
    'ERROR', 
    ]
    )

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        parameters=[{"base_frame_id": "base_link",
                     "set_initial_pose":True}],
        remappings=[('scan','base_scan')],
    )

    map_server = Node(
    package="nav2_map_server",
    executable="map_server",
    parameters=[{"yaml_filename": os.path.join(socspioneer_p, "data", "meeting.yaml"),
                 "use_sim_time": True,}],
    arguments= [
    '--ros-args',
    '--log-level',
    'ERROR',
    ]
    )

    # map_server = Node(
    # package="nav2_map_server",
    # executable="map_server",
    # parameters=[{"yaml_filename": LaunchConfiguration('map_path')}],
    # arguments= [
    # '--ros-args',
    # '--log-level',
    # 'ERROR',
    # ]
    # )

    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="true",
        description="Use simulated time instead of real time.",
        choices=["true", "True", "false", "False"],
    )

    socspioneer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(socspioneer_p,'launch','stage.launch.py')),
        launch_arguments=[('map_path', LaunchConfiguration('map_path')),
                          ('world_path', LaunchConfiguration('world_path'))]
    )

    socspioneer_key = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(socspioneer_p,'launch','keyboard_teleop.launch.py'))
    )

    foxglove_bridge = IncludeLaunchDescription(os.path.join(foxglove_bridge_p,'foxglove_bridge_launch.xml'))

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        parameters=[{"use_sim_time": True,}],
        arguments=["-d", [os.path.join(socspioneer_p, "config", "map_view.rviz")],],
    )

    pf_localisation = Node(
        package="pf_localisation",
        executable="node.py",
        parameters=[{"use_sim_time": True}]
        # arguments= [
        # '--ros-args',
        # '--log-level',
        # 'ERROR',
        # ]
        )


    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg)
    ld.add_action(world_path)
    ld.add_action(map_path)
    # ld.add_action(lifecycle_manager)

#SIM_DATA
    ld.add_action(socspioneer)
    # ld.add_action(socspioneer_key)

#REAL_DATA
    
    # ld.add_action(foxglove_bridge)
    # ld.add_action(map_server)
    # ld.add_action(rviz2)
    # ld.add_action(amcl)

    time.sleep(3)
    ld.add_action(pf_localisation)

    return ld
