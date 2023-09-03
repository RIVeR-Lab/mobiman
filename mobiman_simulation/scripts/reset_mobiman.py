#!/usr/bin/python3
import rospy
from geometry_msgs.msg import Pose
from std_srvs.srv import Empty, EmptyRequest, SetBoolResponse
import rosservice
from gazebo_msgs.srv import DeleteModelRequest, DeleteModel, SpawnModel, SpawnModelRequest, SetModelConfiguration, SetModelConfigurationRequest
from controller_manager_msgs.srv import LoadController, SwitchController, LoadControllerRequest, SwitchControllerRequest
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from mobiman_simulation.srv import resetMobiman, resetMobimanResponse
import sys
import subprocess

# from std_msgs.msg import Empty as EmptyMsg




def handleResetMobiman(req):
    # Set Pose and other variables
    success = True
    pose = Pose()
    pose.position.z=0.2
    joint_names = [f'j2n6s300_joint_{str(a)}' for a in range(1,7)]
    joint_positions = [0.0, 2.9, 1.3, 4.2, 1.4, 0.0]
    robot_model = rospy.get_param('/robot_description')
    pause_physics_client = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
    unpause_physics_client = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
    controller_list = ['arm_controller', 'joint_state_controller', 'jackal_velocity_controller', 'joint_group_position_controller']
    # Delete Current Model
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        delete_model(DeleteModelRequest('mobiman'))
    except Exception as e:
        success = False
    # Spawn model
    try:
        spawn_model = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        spawn_model(SpawnModelRequest(model_name='mobiman', model_xml=robot_model, robot_namespace='', initial_pose=pose, reference_frame='world'))
    except Exception as e:
        success = False
    # Pause physics and change configuration
    pause_physics_client(EmptyRequest())
    set_configuration = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)
    for i in range(1,100):
        set_configuration(SetModelConfigurationRequest(model_name='mobiman', urdf_param_name='robot_description', joint_names=joint_names, joint_positions=joint_positions))
    # Load Controllers
    for controller in controller_list:
        load_controller = rospy.ServiceProxy('/controller_manager/load_controller', LoadController)
        load_controller(LoadControllerRequest(controller))
    # Switch Controller
    switch_controller_req = SwitchControllerRequest()
    switch_controller_req.start_asap = True
    for controller in controller_list:
        command = 'sleep 0.5 && rosservice call /gazebo/unpause_physics "{}"'
        subprocess.Popen(command, stdout=subprocess.PIPE, shell=True, stderr=subprocess.STDOUT)
        switch_controller_req.start_controllers = [controller]
        switch_controller = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
        res = switch_controller(switch_controller_req)
        # print(res)
    for i in range(1, 10):
        unpause_physics_client(EmptyRequest())
    return resetMobimanResponse(success)


if __name__ == '__main__':
    rospy.init_node('reset_mobiman')
    s = rospy.Service('reset_mobiman', resetMobiman, handleResetMobiman)
    rospy.spin()