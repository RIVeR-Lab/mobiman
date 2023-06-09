#!/usr/bin/env python3

import rospy
import rospkg
import os
import csv
import numpy as np

'''
DESCRIPTION: TODO...
'''
if __name__ == '__main__':

    print("[mobiman_drl_isaac::__main__] WELCOME BACK BRO!")
    rospy.init_node('mobiman_drl_isaac', anonymous=True, log_level=rospy.WARN)

    rospack = rospkg.RosPack()
    tentabot_path = rospack.get_path('mobiman_simulation') + "/"

    mode = "testing"
    #data_path = rospy.get_param('drl_data_path', "")

    print("[mobiman_drl_isaac::__main__] tentabot_path: " + tentabot_path)
    print("[mobiman_drl_isaac::__main__] mode: " + mode)