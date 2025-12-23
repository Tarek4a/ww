import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # 1. مسارات الملفات
    pkg_share = get_package_share_directory('robot_motion')
    nav2_params_path = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    map_yaml_path = '/home/tarek/ros_ws/my_map.yaml'  # غيّر المسار لو الخريطة في مكان تاني

    # 2. Launch Arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # 3. العقد (Nodes)

    # Map Server
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml_path},
                    {'topic': 'map'},
                    {'frame_id': 'map'},
                    {'use_sim_time': use_sim_time}]
    )

    # AMCL (Localization)
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params_path, {'use_sim_time': use_sim_time}]
    )

    # Navigation Nodes (زي ما كانت)
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params_path, {'use_sim_time': use_sim_time}]
    )

    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_path, {'use_sim_time': use_sim_time}]
    )

    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params_path, {'use_sim_time': use_sim_time}]
    )

    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params_path, {'use_sim_time': use_sim_time}]
    )

    # Lifecycle Manager (يدير كل العقد دي تلقائياً)
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                    'autostart': True,
                    'bond_timeout': 10.0,          # ← أضف ده
                    'service_timeout_ms': 5000 ,
                    'node_names': ['map_server',
                                   'amcl',
                                   'controller_server',
                                   'planner_server',
                                   'behavior_server',
                                   'bt_navigator']}]
    )

    # 4. تجميع كل حاجة
    return LaunchDescription([
        use_sim_time_arg,
        map_server_node,
        amcl_node,
        controller_server_node,
        planner_server_node,
        behavior_server_node,
        bt_navigator_node,
        lifecycle_manager_node
    ])