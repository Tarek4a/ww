import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import random

def main():
    rclpy.init()
    nav = BasicNavigator()
    nav.waitUntilNav2Active() # بينتظر لحد ما الـ Launch اللي فوق يخلص

    while rclpy.ok():
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x = float(random.uniform(-5.0, 5.0))
        goal_pose.pose.position.y = float(random.uniform(-5.0, 5.0))
        goal_pose.pose.orientation.w = 1.0
        
        nav.goToPose(goal_pose)
        while not nav.isTaskComplete():
            pass 
    rclpy.shutdown()

if __name__ == '__main__':
    main()