from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # شغل الـ world
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '/home/tarek/ros_ws/src/my_rest/worlds/rest.world'],
            output='screen'
        ),

        # spawn الروبوت
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-file', '/home/tarek/ros_ws/src/my_robot/urdf/my_robot.urdf',
                '-entity', 'my_robot'
            ],
            output='screen'
        )
    ])
