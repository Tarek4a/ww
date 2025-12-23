import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    # 1. المسارات
    urdf_file = '/home/tarek/ros_ws/src/my_robot/urdf/my_robot.urdf'
    world_file = '/home/tarek/ros_ws/src/my_rest/worlds/res.world'
    
    with open(urdf_file, 'r') as infp:
        robot_description_urdf = infp.read()

    # 2. تشغيل Gazebo
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen'
    )

    # 3. نشر حالة الروبوت
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_urdf, 'use_sim_time': True}],
        output='screen'
    )

    joint_state_publisher_node = Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    parameters=[{'use_sim_time': True}],
    output='screen'
)

    # 4. سباون الروبوت
    robot_spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'robot', '-string', robot_description_urdf, '-x', '0.0', '-y', '0.0', '-z', '0.03'],
        output='screen'
    )

    # 5. الجسر (Bridge)
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        output='screen'
    )

    # 6. الـ Controllers Spawners (تم حذف --set-state لأنها غير مدعومة وتسبب الخطأ)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"], # ستقوم بالتفعيل تلقائياً
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_base_controller"], # ستقوم بالتفعيل تلقائياً
    )

    return LaunchDescription([
        gz_sim,
        bridge_node,
        robot_state_publisher_node,
        robot_spawn_node,
        
        # أهم خطوة: تأخير 7 ثوانٍ لضمان أن Gazebo قرأ الـ ros2_control plugin من الـ URDF
        TimerAction(
            period=7.0,
            actions=[
                joint_state_broadcaster_spawner,
                diff_drive_spawner,
                joint_state_publisher_node,
            ]
        ),
    ])