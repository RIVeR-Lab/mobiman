#!/usr/bin/python3
'''
Program for testing ROS Bridge with UR5 model in Isaac Sim
author - @sarvesh
email - sarvesh101p@gmail.com
Referecnce - Isaac documentation and tutorials
'''

#imports for the program
import rospy
from sensor_msgs.msg import JointState
import numpy as np
import time
from geometry_msgs.msg import Twist


if __name__ == '__main__':
    '''
    Entry point of program that utilized ur5_check_bridge class
    '''
    rospy.init_node("jackal_isaac_test", anonymous=True)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    rate = rospy.Rate(10)
    init_once = False
    twist_msg = Twist()
    twist_msg.linear.x = 0.0
    twist_msg.linear.y = 0.0
    twist_msg.linear.z = 0.0
    twist_msg.angular.x = 0.0
    twist_msg.angular.y = 0.0
    twist_msg.angular.z = 0.0
    pub.publish(twist_msg)
    while not rospy.is_shutdown():
        if not init_once:
            twist_msg.linear.x = 2.0
            while pub.get_num_connections() == 0:
                rate.sleep()
            pub.publish(twist_msg)
            time.sleep(2)
            twist_msg.linear.x = -2.0
            while pub.get_num_connections() == 0:
                rate.sleep()
            pub.publish(twist_msg)
            time.sleep(2)
            twist_msg.angular.z = 1.0
            twist_msg.linear.x = 0.0
            while pub.get_num_connections() == 0:
                rate.sleep()
            pub.publish(twist_msg)
            time.sleep(5)
            twist_msg.angular.z = 0.0
            init_once = True
            while pub.get_num_connections() == 0:
                rate.sleep()
            pub.publish(twist_msg)
            rospy.spin()
        rate.sleep()
