#!/usr/bin/python3
'''
Program for testing ROS Bridge with kinova model in Isaac Sim
author - @sarvesh
email - sarvesh101p@gmail.com
Referecnce - Isaac documentation and tutorials
'''

#imports for the program
import rospy
from sensor_msgs.msg import JointState
import numpy as np
import time


class kinova_check_bridge():
    '''
    Class based on ROS middle-ware
    Functions
        -> shutdown
        -> main
    '''
    def __init__(self):
        '''
        Constructor that creates ros node with name `kinova_isaac_test`
        Defines default and limits of joint poses
        '''
        rospy.init_node("kinova_moveit_test", anonymous=True)
        self.pub = rospy.Publisher("/joint_command", JointState, queue_size=10)
        self.joint_state = JointState()
        self.joint_state.name = ["j2n6s300_joint_1", "j2n6s300_joint_2", "j2n6s300_joint_3", \
                                "j2n6s300_joint_4", "j2n6s300_joint_5", "j2n6s300_joint_6"]
        self.num_joints = len(self.joint_state.name)
        self.joint_state.position = np.array([0.0] * self.num_joints)
        self.default_joints = [np.pi, np.pi, np.pi, np.pi, np.pi, np.pi]
        self.max_joints = np.array(self.default_joints) + 0.5
        self.min_joints = np.array(self.default_joints) - 0.5
        self.time_start = time.time()
        self.rate = rospy.Rate(20)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo('[+] Node will start in 2 second, press ctrl+c to kill')
        rospy.sleep(2)


    def shutdown(self):
        '''
        Function - Shutdown
        Resets the kinova to default pose when ctrl+c is pressed
        '''
        self.joint_state.position = self.default_joints
        self.pub.publish(self.joint_state)
        rospy.loginfo('[+] Node killed, Reseting to default position')
    

    def main(self):
        '''
        Function - main
        Manipulates kinova's position
        '''
        while not rospy.is_shutdown():
            self.joint_state.position = np.sin(time.time() - self.time_start) * (self.max_joints - self.min_joints) * 0.5 + self.default_joints
            self.pub.publish(self.joint_state)
            self.rate.sleep()


if __name__ == '__main__':
    '''
    Entry point of program that utilized kinova_check_bridge class
    '''
    kinova = kinova_check_bridge()
    try:
        kinova.main()
    except rospy.ROSInterruptException as e:
        pass