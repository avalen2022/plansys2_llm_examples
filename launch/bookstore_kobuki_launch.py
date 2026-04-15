"""Full integration launch: Kobuki in AWS bookstore with EasyNav + PlanSys2.

Launches:
  1. Gazebo with AWS bookstore world + Kobuki robot (lidar + camera)
  2. ROS-Gazebo bridges (clock, odom, tf, cmd_vel, scan, camera, joints)
  3. EasyNavigation system (controller, planner, localizer, maps, sensors)
  4. PlanSys2 (domain expert, problem expert, planner, executor)
  5. BT action nodes (move, pick_book, place_book)
  6. Perception simulator
"""

import os
from os.path import join

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bookstore_dir = get_package_share_directory('plan_bookstore')
    kobuki_desc_dir = get_package_share_directory('kobuki_description')
    aws_bookstore_dir = get_package_share_directory('aws_robomaker_bookstore_world')

    namespace = LaunchConfiguration('namespace')
    displaced_book = LaunchConfiguration('displaced_book')

    declare_namespace = DeclareLaunchArgument(
        'namespace', default_value='',
        description='Top-level namespace')

    declare_displaced_book = DeclareLaunchArgument(
        'displaced_book', default_value='blue_book',
        description='Which book is displaced from its shelf')

    declare_gui = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Launch Gazebo GUI')

    # -- Environment -----------------------------------------------------------
    model_path = os.path.join(aws_bookstore_dir, 'models')
    resource_path = model_path
    if 'GZ_SIM_MODEL_PATH' in os.environ:
        model_path += os.pathsep + os.environ['GZ_SIM_MODEL_PATH']
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        resource_path += os.pathsep + os.environ['GZ_SIM_RESOURCE_PATH']

    # -- 1. Gazebo -------------------------------------------------------------
    world_file = join(aws_bookstore_dir, 'worlds', 'bookstore.world')

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(get_package_share_directory('ros_gz_sim'),
                 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r -s ', world_file]}.items(),
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(get_package_share_directory('ros_gz_sim'),
                 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': [' -g ']}.items(),
        condition=IfCondition(LaunchConfiguration('gui')),
    )

    # -- 2. Kobuki -------------------------------------------------------------
    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(kobuki_desc_dir, 'launch', 'spawn.launch.py')),
        launch_arguments={
            'x': '0.0', 'y': '0.0', 'z': '0.0', 'Y': '0.0',
            'lidar_range': '15.0',
            'camera': 'true',
            'lidar': 'true',
            'use_sim_time': 'true',
        }.items(),
    )

    # -- 3. Bridges ------------------------------------------------------------
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_ros_gz',
        parameters=[{
            'config_file': join(bookstore_dir, 'config', 'bridge', 'kobuki_bridge.yaml'),
            'use_sim_time': True,
        }],
        output='screen',
    )

    # -- 4. EasyNavigation -----------------------------------------------------
    easynav_config = join(bookstore_dir, 'config', 'easynav_kobuki_bookstore.yaml')
    easynav_system = Node(
        package='easynav_system',
        executable='system_main',
        output='screen',
        parameters=[{'use_sim_time': True}],
        ros_arguments=['--params-file', easynav_config],
    )

    # -- 5. PlanSys2 -----------------------------------------------------------
    plansys2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(
            get_package_share_directory('plansys2_bringup'),
            'launch', 'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={
            'model_file': join(bookstore_dir, 'pddl', 'domain.pddl'),
            'namespace': namespace,
            'params_file': join(bookstore_dir, 'params', 'planner_param.yaml'),
        }.items(),
    )

    # -- 6. Perception ---------------------------------------------------------
    perception_sim = Node(
        package='plan_bookstore',
        executable='perception_sim_node',
        name='perception_sim',
        namespace=namespace,
        output='screen',
        parameters=[{
            'detected_object': displaced_book,
            'detected_location': 'middle_path',
            'observed_x': 0.0,
            'observed_y': -3.0,
            'use_sim_time': True,
        }],
    )

    # -- 7. BT Action Nodes ---------------------------------------------------
    move_node = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='move',
        namespace=namespace,
        output='screen',
        parameters=[
            join(bookstore_dir, 'config', 'params.yaml'),
            {
                'action_name': 'move',
                'publisher_port': 1668,
                'server_port': 1669,
                'bt_xml_file': join(bookstore_dir, 'bt_xml', 'move.xml'),
                'use_sim_time': True,
            },
        ],
    )

    pick_book_node = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='pick_book',
        namespace=namespace,
        output='screen',
        parameters=[
            join(bookstore_dir, 'config', 'params.yaml'),
            {
                'action_name': 'pick_book',
                'publisher_port': 1670,
                'server_port': 1671,
                'bt_xml_file': join(bookstore_dir, 'bt_xml', 'pick_book.xml'),
                'displaced_book': displaced_book,
                'use_sim_time': True,
            },
        ],
    )

    place_book_node = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='place_book',
        namespace=namespace,
        output='screen',
        parameters=[
            join(bookstore_dir, 'config', 'params.yaml'),
            {
                'action_name': 'place_book',
                'publisher_port': 1672,
                'server_port': 1673,
                'bt_xml_file': join(bookstore_dir, 'bt_xml', 'place_book.xml'),
                'use_sim_time': True,
            },
        ],
    )

    # -- 8. Waypoint TF Visualizer ---------------------------------------------
    waypoint_tf = ExecuteProcess(
        cmd=['python3', join(bookstore_dir, 'scripts', 'visualize_waypoints.py')],
        output='screen',
    )

    # -- Build Launch Description ----------------------------------------------
    ld = LaunchDescription()

    # GPU acceleration for Nvidia hybrid laptops
    ld.add_action(SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1'))
    ld.add_action(SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia'))
    ld.add_action(SetEnvironmentVariable('GZ_SIM_MODEL_PATH', model_path))
    ld.add_action(SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path))

    ld.add_action(declare_namespace)
    ld.add_action(declare_displaced_book)
    ld.add_action(declare_gui)

    ld.add_action(gazebo_server)
    ld.add_action(gazebo_client)
    ld.add_action(spawn_robot)
    ld.add_action(ros_gz_bridge)

    ld.add_action(easynav_system)
    ld.add_action(plansys2)
    ld.add_action(perception_sim)

    ld.add_action(move_node)
    ld.add_action(pick_book_node)
    ld.add_action(place_book_node)
    ld.add_action(waypoint_tf)

    return ld
