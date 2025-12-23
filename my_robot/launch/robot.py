from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    urdf_path = '/home/tarek/ros_ws/src/my_robot/urdf/my_robot.urdf'
    rviz_config_path = '/home/tarek/ros_ws/src/my_robot/rviz/urdf_config.rviz'

    with open(urdf_path, 'r') as infp:
        robot_description_content = infp.read()

    # نود وصف الروبوت (يجب أن تعمل بوقت المحاكي)
    robot_state_publisher_node = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    parameters=[{
        'robot_description': robot_description_content,
        'use_sim_time': True  # السطر ده هو اللي هيمنع الـ Jump back in time
    }]
)

    # ملاحظة: حذفنا joint_state_publisher_gui لأنها تعطل حركة العجل في الملاحة

    # تشغيل RViz بوقت المحاكي
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}] # هذا ما سيجعل الوقت تحت يظهر رقماً صغيراً
    )

    return LaunchDescription([
        robot_state_publisher_node,
        rviz2_node
    ])