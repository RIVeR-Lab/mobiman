#!/usr/bin/python3
import rospy
from geometry_msgs.msg import Pose
from std_srvs.srv import Empty, EmptyRequest, SetBoolResponse
import rosservice
from gazebo_msgs.srv import DeleteModelRequest, DeleteModel, SpawnModel, SpawnModelRequest, SetModelConfiguration, SetModelConfigurationRequest
from controller_manager_msgs.srv import LoadController, SwitchController, LoadControllerRequest, SwitchControllerRequest
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# from std_msgs.msg import Empty as EmptyMsg

if __name__ == '__main__':
    rospy.init_node('reset_mobiman')
    pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)
    joint_position = JointTrajectory()
    pose = Pose()
    pose.position.z=0.2
    joint_names = [f'j2n6s300_joint_{str(a)}' for a in range(1,7)]
    joint_position.joint_names = joint_names
    # joint_position.points[0]
    joint_position.points = [JointTrajectoryPoint(), JointTrajectoryPoint()]
    joint_positions = [0.0, 2.9, 1.3, 4.2, 1.4, 0.0]
    joint_position.points[0].positions = joint_positions
    joint_position.points[1].positions = joint_positions
    joint_position.points[1].time_from_start = rospy.Duration(0.2)
    robot_model = rospy.get_param('/robot_description')
    pause_physics_client = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
    pause_physics_client(EmptyRequest())
    delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    delete_model(DeleteModelRequest('mobiman'))
    unpause_physics_client = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
    spawn_model = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
    spawn_model(SpawnModelRequest(model_name='mobiman', model_xml=robot_model, robot_namespace='', initial_pose=pose, reference_frame='world'))
    unpause_physics_client(EmptyRequest())
    rospy.sleep(0.2)
    pause_physics_client(EmptyRequest())
    set_configuration = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)
    set_configuration(SetModelConfigurationRequest(model_name='mobiman', urdf_param_name='robot_description', joint_names=joint_names, joint_positions=joint_positions))
    # rospy.sleep(0.5)
    # unpause_physics_client = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
    unpause_physics_client(EmptyRequest())
    controller_list = ['arm_controller', 'joint_state_controller', 'jackal_velocity_controller', 'joint_group_position_controller']
    for controller in controller_list:
        load_controller = rospy.ServiceProxy('/controller_manager/load_controller', LoadController)
        load_controller(LoadControllerRequest(controller))
        switch_controller = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
        res = switch_controller(SwitchControllerRequest(start_controllers=f'[{controller}]'))
        print(res)
    rospy.sleep(1)
    switch_controller_req = SwitchControllerRequest()
    switch_controller_req.start_asap = True
    for controller in controller_list:
        switch_controller_req.start_controllers = [controller]
        switch_controller = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
        res = switch_controller(switch_controller_req)
        print(res)
    rate = rospy.Rate(10)
    while pub.get_num_connections() == 0:
        rate.sleep()
        rospy.loginfo('wait')
    pub.publish(joint_position)
    # rospy.spin()