#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDrive
from tf.transformations import euler_from_quaternion
import numpy as np
import time

class DriveNode:
    def __init__(self):
        self.goal_x = 0
        self.goal_y = 0
        self.current_x = 0
        self.current_y = 0
        self.current_heading = 0
        self.last_goal_reached_time = time.time()
        rospy.init_node('pa1_task2_node', anonymous=True)
        self.drive_pub = rospy.Publisher('/car_1/command', AckermannDrive, queue_size=10)
        self.rate = rospy.Rate(10)
        rospy.Subscriber('/car_1/odom', Odometry, self.odom_callback)
        
    def odom_callback(self, odom_msg):
        self.current_x = odom_msg.pose.pose.position.x
        self.current_y = odom_msg.pose.pose.position.y
        quaternion = (
            odom_msg.pose.pose.orientation.x,
            odom_msg.pose.pose.orientation.y,
            odom_msg.pose.pose.orientation.z,
            odom_msg.pose.pose.orientation.w
        )
        _, _, self.current_heading = euler_from_quaternion(quaternion)

    def drive_to_goal(self):
        goal_states = rospy.get_param('goal_states', [[0,0]])
        wheel_base = rospy.get_param('wheel_base', 0.324)
        dt = rospy.get_param('dt', 0.1)
        max_velocity = rospy.get_param('max_velocity', 5.0)
        min_velocity = rospy.get_param('min_velocity', 0.0)
        kinematic_update_topic = rospy.get_param('kinematic_update_topic', "/kinematic_update")
        count=0
        for goal in goal_states:  
            self.goal = goal
            self.goal_x = self.goal[0]
            self.goal_y = self.goal[1]
            count+=1
            start_time = time.time()
            while not rospy.is_shutdown():
                desired_heading = np.arctan2(self.goal_y - self.current_y, self.goal_x - self.current_x)
                steering_direction = -1
                heading_change = desired_heading - self.current_heading
                error = np.sqrt((self.goal_y - self.current_y)**2 + (self.goal_x - self.current_x)**2)
                print(f"Current Heading: {np.degrees(self.current_heading)}, Desired Heading: {np.degrees(desired_heading)}, Heading Change: {np.degrees(heading_change)}")
                print("Error:", error)
                print("pos", self.current_x, self.current_y)
                print(f"Current goal: [{self.goal_x}, {self.goal_y}]") 

                
                L = wheel_base
                Lf = 0.5
                max_steering_angle = np.radians(30)
                steering_angle = np.arctan2(2 * L * np.sin(heading_change), Lf)
                steering_angle = np.clip(steering_angle * steering_direction, -max_steering_angle, max_steering_angle)

                
                drive_msg = AckermannDrive()
                if error < 0.2:   
                    drive_msg.speed=0
                    break

                if error > 3:
                    drive_msg.speed = max_velocity
                else:
                    drive_msg.speed = min(max_velocity, error)
                drive_msg.speed *= 0.67

                drive_msg.steering_angle = steering_angle
                self.drive_pub.publish(drive_msg)
                self.rate.sleep()
            
                # if error < 0.6:
                #     self.last_goal_reached_time = time.time()
                #     break  
                # elif (time.time() - start_time) > 25:  
                #     break

        
        if count == len(goal_states) and error < 0.6:
            return False  
        return True  


if __name__ == '__main__':
    drive_node = DriveNode()
    while not rospy.is_shutdown():
        if not drive_node.drive_to_goal():  
            break  
    rospy.spin()
