import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_motion')
    # تأكد أن اسم الملف mapper_params.yaml
    config_file = os.path.join(pkg_share, 'config', 'mapper_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # 1. عقدة الـ SLAM Toolbox
    slam_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node', 
        name='slam_toolbox',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    # 2. مدير دورة الحياة (خاص بالـ SLAM فقط)
    # ملاحظة: إذا كنت تريد الخريطة تبدأ فوراً، اجعل autostart: True
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True}, 
            {'node_names': ['slam_toolbox']}
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        slam_node,
        lifecycle_manager
    ])