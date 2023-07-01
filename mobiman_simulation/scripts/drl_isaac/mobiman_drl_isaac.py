#!/usr/bin/env python3

import rospy
import rospkg
import os
import csv
import subprocess
import sys

'''
DESCRIPTION: TODO...
'''
if __name__ == '__main__':

    ## ROS NODE
    rospy.init_node('mobiman_drl_isaac', anonymous=True, log_level=rospy.WARN)

    ## USER DEFINED PARAMETERS
    isaac_sim_path = os.path.expanduser(rospy.get_param('isaac_sim_path', ""))

    ## LOCAL PARAMETERS
    rospack = rospkg.RosPack()
    tentabot_path = rospack.get_path('mobiman_simulation') + "/"
    
    ## PRINT INFO
    print("[mobiman_drl_isaac::__main__] tentabot_path: " + tentabot_path)
    print("[mobiman_drl_isaac::__main__] isaac_sim_path: " + isaac_sim_path)

    print("[mobiman_drl_isaac::__main__] START mobiman_drl_train")
    #subprocess.run([isaac_sim_path + "python.sh", isaac_sim_path + "DRL_Isaac_lib/train_d.py"], shell=False)
    subprocess.run([isaac_sim_path + "python.sh", tentabot_path + "scripts/drl_isaac/mobiman_drl_train.py"], shell=False)
    #subprocess.run([isaac_sim_path + "python.sh", tentabot_path + "scripts/drl_isaac/mobiman_env.py"], shell=False)