#!/usr/bin/python3

'''
LAST UPDATE: 2023.09.04

AUTHOR: Sarvesh Prajapati (SP)

E-MAIL: prajapati.s@northeastern.edu
DESCRIPTION: TODO...

REFERENCES:
[1] 

NUA TODO:
'''

import rospy
import rospkg
import tf
import datetime
from geometry_msgs.msg import Pose
from std_srvs.srv import Empty, EmptyRequest, SetBoolResponse
import rosservice
from gazebo_msgs.srv import DeleteModelRequest, DeleteModel, SpawnModel, SpawnModelRequest, SetModelConfiguration, SetModelConfigurationRequest
from controller_manager_msgs.srv import LoadController, SwitchController, LoadControllerRequest, SwitchControllerRequest
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from mobiman_simulation.srv import resetMobiman, resetMobimanResponse, resetMobimanRequest
import sys
from geometry_msgs.msg import Quaternion, Pose, Point
import subprocess



pkgs_ign_name = []
pkgs_ign_path = []
jackal_jacko_xml = None
urdfs_xmls = []
# pkgs_man_name = []
# pkgs_man_path = []

    

# from std_msgs.msg import Empty as EmptyMsg

def handleResetMobiman(req):
    # Set Pose and other variables
    
    
    
    # sm = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)

    # try:
    #     dm = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    #     for pkg in pkgs_ign_name:
    #         dm(pkg)
    # except Exception as e:
    #     pass
    
        
    
    
    success = True
    pose = Pose()
    pose.position.x=req.x
    pose.position.y=req.y
    pose.position.z=req.z
    pose.orientation.w = req.quat_w
    pose.orientation.x = req.quat_x
    pose.orientation.y = req.quat_y
    pose.orientation.z = req.quat_z
    joint_names = [f'j2n6s300_joint_{str(a)}' for a in range(1,7)]
    joint_positions = [req.joint_1, req.joint_2, req.joint_3, req.joint_4, req.joint_5, req.joint_6]
    robot_model = rospy.get_param('/robot_description')
    pause_physics_client = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
    unpause_physics_client = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
    controller_list = ['arm_controller', 'joint_state_controller', 'jackal_velocity_controller', 'joint_group_position_controller']
    try:
        rospy.wait_for_service('/gazebo/pause_physics')
        pause_physics_client(EmptyRequest())
    except Exception as e:
        pass
    # Delete Current Model
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        delete_model(DeleteModelRequest('mobiman'))
        rospy.wait_for_service('/gazebo/delete_model')
    except Exception as e:
        success = False
    
    # Spawn model
    try:
        rospy.wait_for_service('/gazebo/spawn_urdf_model')
        spawn_model = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        spawn_model(SpawnModelRequest(model_name='mobiman', model_xml=jackal_jacko_xml, robot_namespace='', initial_pose=pose, reference_frame='world'))
        rospy.wait_for_service('/gazebo/spawn_urdf_model')
    except Exception as e:
        success = False
    # print("[+] [RESET SCRIPT] Before Sleep")
    # rospy.sleep(1)
    # print("[+] [RESET SCRIPT] After Sleep")

    
    current_time = datetime.datetime.now().second
    while datetime.datetime.now().second <= current_time + 1:
        continue
    # while True:
    #     continue
    # pause_physics_client(EmptyRequest())
    # pause_physics_client(EmptyRequest())
    # quat = tf.transformations.quaternion_from_euler(0, 0, 0) # type: ignore
    # orient = Quaternion(quat[0], quat[1], quat[2], quat[3])
    # pose = Pose(Point(x=5, y=-2.5, z=0.5), orient)
    # for idx, pkg in enumerate(pkgs_ign_name):
    #     print(pkg, pkgs_ign_path[0])
        
    #     pose.position.x -= 1.5
    #     sm(pkg, urdfs_xmls[idx], '', pose, 'world')
    
    # Pause physics and change configuration
    
    set_configuration = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)
    for i in range(1,100):
        set_configuration(SetModelConfigurationRequest(model_name='mobiman', urdf_param_name='robot_description', joint_names=joint_names, joint_positions=joint_positions))
    
    # Load Controllers
    load_controller = rospy.ServiceProxy('/controller_manager/load_controller', LoadController)
    for controller in controller_list:
        rospy.wait_for_service('/controller_manager/load_controller')
        try:
            load_controller(LoadControllerRequest(controller))
        except Exception as e:
            pass

    # Switch Controller
    switch_controller_req = SwitchControllerRequest()
    switch_controller_req.start_asap = True
    for controller in controller_list:
        command = 'sleep 0.5 && rosservice call /gazebo/unpause_physics "{}"'
        subprocess.Popen(command, stdout=subprocess.PIPE, shell=True, stderr=subprocess.STDOUT)
        switch_controller_req.start_controllers = [controller]
        switch_controller_req.strictness = switch_controller_req.STRICT
        switch_controller = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
        res = switch_controller(switch_controller_req)
        # print(res)
    for i in range(1, 10):
        try:
            rospy.wait_for_service('/gazebo/unpause_physics')
            unpause_physics_client(EmptyRequest())
        except Exception as e:
            pass
    return resetMobimanResponse(success)

if __name__ == '__main__':
    rospy.init_node('reset_mobiman')
    rospack = rospkg.RosPack()
    path = rospack.get_path('mobiman_simulation') + "/urdf/"
    pkgs_ign_name.append("red_cube")
    pkgs_ign_path.append(path + "red_cube.urdf")
    pkgs_ign_name.append("normal_pkg")
    pkgs_ign_path.append(path + "normal_pkg.urdf")
    pkgs_ign_name.append("green_cube")
    pkgs_ign_path.append(path + "green_cube.urdf")
    pkgs_ign_name.append("long_pkg")
    pkgs_ign_path.append(path + "long_pkg.urdf")
    pkgs_ign_name.append("blue_cube")
    pkgs_ign_path.append(path + "blue_cube.urdf")
    pkgs_ign_name.append("longwide_pkg")
    pkgs_ign_path.append(path + "longwide_pkg.urdf")
    jackal_jaco_urdf = path + "jackal_jaco.urdf"
    
    
    with open(jackal_jaco_urdf, 'r') as f:
        jackal_jacko_xml = f.read()
    
    for i,j in enumerate(pkgs_ign_path):
        # print(j)
        with open(j, 'r') as f:
            urdfs_xmls.append((f.read()))

    s = rospy.Service('reset_mobiman', resetMobiman, handleResetMobiman)
    rospy.spin()