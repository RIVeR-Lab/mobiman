#!/usr/bin/env python3

'''
LAST UPDATE: 2023.10.20

AUTHOR: Neset Unver Akmandor (NUA)

E-MAIL: akmandor.n@northeastern.edu
'''
 
import rospy
import rospkg
import roslib
import rosparam
import roslaunch 
import tf2_ros

def load_config_file(config_file_path, namespace=""):
    ## Load config files
    roslib.load_manifest("rosparam")
    paramlist=rosparam.load_file(config_file_path, default_namespace=namespace)
    for params, ns in paramlist:
        rosparam.upload_params(ns, params)

if __name__=="__main__":

    rospy.init_node("mobiman_framework_launch", anonymous=False)

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)

    rospack = rospkg.RosPack()
    mobiman_path = rospack.get_path('mobiman_simulation') + "/"
    igibson_path = rospack.get_path('igibson-ros') + "/"
    print("[mobiman_framework_launch::__main__] mobiman_path: " + str(mobiman_path))
    print("[mobiman_framework_launch::__main__] igibson_path: " + str(igibson_path))

    mobiman_launch_path = mobiman_path + "launch/"
    config_mobiman_framework = rospy.get_param('config_mobiman_framework', "")

    print("[mobiman_framework_launch:: __main__ ] config_mobiman_framework: " + str(config_mobiman_framework))

    load_config_file(mobiman_path + "config/" + config_mobiman_framework)

    '''
    ## Load config files
    roslib.load_manifest("rosparam")
    paramlist=rosparam.load_file(mobiman_path + "config/" + config_mobiman_framework, default_namespace="")
    for params, ns in paramlist:
        rosparam.upload_params(ns, params)
    '''

    #### Parameters
    ### Simulator:
    sim = rospy.get_param('sim', "")
    flag_gazebo = rospy.get_param('flag_gazebo', True)
    flag_robot = rospy.get_param('flag_robot', True)
    flag_conveyor = rospy.get_param('flag_conveyor', True)
    flag_pedsim = rospy.get_param('flag_pedsim', False)
    flag_moveit = rospy.get_param('flag_moveit', False)

    ### Rviz:
    rviz_config_path = rospy.get_param('rviz_config_path', "")
    flag_rviz = rospy.get_param('flag_rviz', True)
    
    ### Map Server:
    flag_map_server = rospy.get_param('flag_map_server', True)
    config_map_server = rospy.get_param('config_map_server', "")
    load_config_file(mobiman_path + "config/" + config_map_server + ".yaml")
    world_frame_name = rospy.get_param('world_frame_name', "")

    ### Robot:
    flag_ns = rospy.get_param('flag_ns', True)
    robot_name = rospy.get_param('robot_name', "")
    n_robot = rospy.get_param('n_robot', "")
    robot_frame_name = rospy.get_param('robot_frame_name', "")
    urdf_path = rospy.get_param('urdf_path', "")
    urdf_path_ocs2 = rospy.get_param('urdf_path_ocs2', "")
    lib_path = rospy.get_param('lib_path', "")
    collision_points_config_path = rospy.get_param('collision_points_config_path', "")

    ### Goal Server:
    flag_goal_server = rospy.get_param('flag_goal_server', True)

    ### Task:
    flag_mobiman = rospy.get_param('flag_mobiman', True)
    task_config_path = rospy.get_param('task_config_path', "")

    print("[mobiman_framework_launch:: __main__ ] General Parameters ---------- START")
    print("[mobiman_framework_launch:: __main__ ] Simulator:")
    print("[mobiman_framework_launch:: __main__ ] sim: " + str(sim))
    print("[mobiman_framework_launch:: __main__ ] flag_gazebo: " + str(flag_gazebo))
    print("[mobiman_framework_launch:: __main__ ] flag_robot: " + str(flag_robot))
    print("[mobiman_framework_launch:: __main__ ] flag_conveyor: " + str(flag_conveyor))
    print("[mobiman_framework_launch:: __main__ ] flag_pedsim: " + str(flag_pedsim))
    print("[mobiman_framework_launch:: __main__ ] flag_moveit: " + str(flag_moveit))

    print("[mobiman_framework_launch:: __main__ ] Rviz:")
    print("[mobiman_framework_launch:: __main__ ] flag_rviz: " + str(flag_rviz))

    print("[mobiman_framework_launch:: __main__ ] Map Server:")
    print("[mobiman_framework_launch:: __main__ ] flag_map_server: " + str(flag_map_server))
    print("[mobiman_framework_launch:: __main__ ] config_map_server: " + str(config_map_server))
    print("[mobiman_framework_launch:: __main__ ] world_frame_name: " + str(world_frame_name))  

    print("[mobiman_framework_launch:: __main__ ] Robot:")
    print("[mobiman_framework_launch:: __main__ ] robot_name: " + str(robot_name))
    print("[mobiman_framework_launch:: __main__ ] n_robot: " + str(n_robot))
    print("[mobiman_framework_launch:: __main__ ] robot_frame_name: " + str(robot_frame_name))
    print("[mobiman_framework_launch:: __main__ ] urdf_path: " + str(urdf_path))
    print("[mobiman_framework_launch:: __main__ ] urdf_path_ocs2: " + str(urdf_path_ocs2))
    print("[mobiman_framework_launch:: __main__ ] lib_path: " + str(lib_path))
    print("[mobiman_framework_launch:: __main__ ] collision_points_config_path: " + str(collision_points_config_path))

    print("[mobiman_framework_launch:: __main__ ] Goal Server:")
    print("[mobiman_framework_launch:: __main__ ] flag_goal_server: " + str(flag_goal_server))

    print("[mobiman_framework_launch:: __main__ ] Mobiman:")
    print("[mobiman_framework_launch:: __main__ ] flag_mobiman: " + str(flag_mobiman))
    print("[mobiman_framework_launch:: __main__ ] task_config_path: " + str(task_config_path))

    print("[mobiman_framework_launch:: __main__ ] General Parameters ---------- END")
    print("")

    ## Set Namespace
    robot_ns_vec = []
    robot_base_frame_name_vec = []
    if flag_ns:
        print("[mobiman_framework_launch:: __main__ ] robot_ns_vec:")
        for i in range(n_robot):
            ns_tmp = robot_name + "_" + str(i)
            robot_ns_vec.append(ns_tmp)
            print(ns_tmp)

            # Update frame names
            robot_base_frame_name_vec.append(ns_tmp +  "/" + str(robot_frame_name))

        print("[mobiman_framework_launch:: __main__ ] robot_base_frame_name_vec:")
        print(robot_base_frame_name_vec)
    
    else:
        print("[mobiman_framework_launch:: __main__ ] No namespace!")

    #print("[mobiman_framework_launch:: __main__ ] DEBUG_INF")
    #while 1:
    #    continue

    ## Launch Simulation
    if sim == "gazebo":
        
        # Launch Gazebo
        sim_launch_path = mobiman_launch_path + "utilities/drl.launch"
        sim_args = [sim_launch_path,
                    'sim:=' + str(sim),
                    'flag_gazebo:=' + str(flag_gazebo),
                    'flag_robot:=' + str(flag_robot),
                    'flag_conveyor:=' + str(flag_conveyor),
                    'flag_pedsim:=' + str(flag_pedsim),
                    'flag_moveit:=' + str(flag_moveit)]

        sim_launch = [ (roslaunch.rlutil.resolve_launch_arguments(sim_args)[0], sim_args[1:]) ]
        sim_obj = roslaunch.parent.ROSLaunchParent(uuid, sim_launch)
        sim_obj.start()

        print("[mobiman_framework_launch:: __main__ ] Launched simulation in Gazebo!")

    elif sim == "igibson":

        # Launch iGibson
        sim_launch_path = igibson_path + "launch/mobiman_jackal_jaco.launch"

        for i, rns in enumerate(robot_ns_vec):
            sim_args = [sim_launch_path,
                        'robot_ns:=' + str(rns),
                        'urdf_path:=' + str(urdf_path)]

            sim_launch = [ (roslaunch.rlutil.resolve_launch_arguments(sim_args)[0], sim_args[1:]) ]
            sim = roslaunch.parent.ROSLaunchParent(uuid, sim_launch)
            sim.start()

        print("[mobiman_framework_launch:: __main__ ] Launched simulation in iGibson!")
    
    else:
        print("[mobiman_framework_launch:: __main__ ] Empty or wrong simulator name! sim: " + str(sim))

    ## Launch Rviz
    if flag_rviz:

        rviz_path = mobiman_launch_path + "utilities/rviz.launch"
        rviz_args = [rviz_path,
                     'rviz_config_path:=' + str(rviz_config_path)]

        rviz_launch = [ (roslaunch.rlutil.resolve_launch_arguments(rviz_args)[0], rviz_args[1:]) ]
        rviz = roslaunch.parent.ROSLaunchParent(uuid, rviz_launch)
        rviz.start()

        print("[mobiman_framework_launch:: __main__ ] Launched Rviz!")
        rospy.sleep(1)

    ## Wait for simulation to be ready!
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    if flag_ns:
        robot_frame_name = robot_base_frame_name_vec[0]
    print("[mobiman_framework_launch:: __main__ ] Waiting the transform between " + world_frame_name + " and " + robot_frame_name + "...")
    trans = None
    while (not rospy.is_shutdown()) and (not trans):
        try:
            trans = tfBuffer.lookup_transform(world_frame_name, robot_frame_name, rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
            rospy.logwarn("[mobiman_framework_launch:: __main__ ] ERROR: " + str(ex))
            rospy.sleep(1.0)   

    ## Launch Map Server
    if flag_map_server:

        map_server_path = mobiman_launch_path + "utilities/map_server.launch"
        map_server_args = [map_server_path,
                           'config_map_server:=' + str(config_map_server)]

        map_server_launch = [ (roslaunch.rlutil.resolve_launch_arguments(map_server_args)[0], map_server_args[1:]) ]
        map_server = roslaunch.parent.ROSLaunchParent(uuid, map_server_launch)
        map_server.start()

        print("[mobiman_framework_launch:: __main__ ] Launched Map Server!")
        rospy.sleep(1) 

    ### NUA TODO: Change the name as goal utility!
    ## Launch ocs2 target 
    if flag_goal_server:

        goal_server_path = mobiman_launch_path + "utilities/ocs2_target.launch"
        goal_server_args = [goal_server_path,
                           'task_config_path:=' + str(task_config_path)]

        goal_server_launch = [ (roslaunch.rlutil.resolve_launch_arguments(goal_server_args)[0], goal_server_args[1:]) ]
        goal_server = roslaunch.parent.ROSLaunchParent(uuid, goal_server_launch)
        goal_server.start()

        print("[mobiman_framework_launch:: __main__ ] Launched Goal Server!")
        rospy.sleep(1)

    ## Launch mobiman 
    if flag_mobiman:

        mobiman_path = mobiman_launch_path + "utilities/ocs2_m4.launch"
        mobiman_args = [mobiman_path,
                        'task_config_path:=' + str(task_config_path),
                        'urdf_path:=' + str(urdf_path_ocs2),
                        'lib_path:=' + str(lib_path),
                        'collision_points_config_path:=' + str(collision_points_config_path)]

        mobiman_launch = [ (roslaunch.rlutil.resolve_launch_arguments(mobiman_args)[0], mobiman_args[1:]) ]
        mobiman = roslaunch.parent.ROSLaunchParent(uuid, mobiman_launch)
        mobiman.start()

        print("[mobiman_framework_launch:: __main__ ] Launched Mobiman!")
        rospy.sleep(1)

    '''
    ## Task-Nav Parameters
    world_name = rospy.get_param('world_name', "")
    world_frame_name = rospy.get_param('world_frame_name', "")
    robot_frame_name = rospy.get_param('robot_frame_name', "")

    print("[mobiman_framework_launch:: __main__ ] Task-Nav Parameters:")
    print("[mobiman_framework_launch:: __main__ ] world_name: " + str(world_name))
    print("[mobiman_framework_launch:: __main__ ] world_frame_name: " + str(world_frame_name))

    n_goal = rospy.get_param('n_goal', "")
    print("[mobiman_framework_launch:: __main__ ] n_goal: " + str(n_goal))
    
    goals_x = []
    goals_y = []
    goals_z = []
    goals_yaw = []

    for i in range(1, n_goal+1):

        goal_name = "goal" + str(i)

        goal_x = rospy.get_param(goal_name + '_x', 0.0)
        goal_y = rospy.get_param(goal_name + '_y', 0.0)
        goal_z = rospy.get_param(goal_name + '_z', 0.0)
        goal_yaw = rospy.get_param(goal_name + '_yaw', 0.0)

        goals_x.append(goal_x)
        goals_y.append(goal_y)
        goals_z.append(goal_z)
        goals_yaw.append(goal_yaw)

        print("[mobiman_framework_launch:: __main__ ] " + goal_name + ": " + str(goal_x))
        print("[mobiman_framework_launch:: __main__ ] " + goal_name + ": " + str(goal_y))
        print("[mobiman_framework_launch:: __main__ ] " + goal_name + ": " + str(goal_z))
        print("[mobiman_framework_launch:: __main__ ] " + goal_name + ": " + str(goal_yaw))

    robot_name = rospy.get_param('robot_name', "")
    robot_model = rospy.get_param('robot_model', "")
    robot_init_pos_x = rospy.get_param('robot_init_pos_x', 0.0)
    robot_init_pos_y = rospy.get_param('robot_init_pos_y', 0.0)
    robot_init_pos_z = rospy.get_param('robot_init_pos_z', 0.0)
    robot_init_yaw = rospy.get_param('robot_init_yaw', 0.0)
    robot_odometry_msg = rospy.get_param('robot_odometry_msg', "")
    robot_pose_control_msg = rospy.get_param('robot_pose_control_msg', "")
    robot_velo_control_msg = rospy.get_param('robot_velo_control_msg', "")

    print("[mobiman_framework_launch:: __main__ ] robot_name: " + str(robot_name))
    print("[mobiman_framework_launch:: __main__ ] robot_model: " + str(robot_model))
    print("[mobiman_framework_launch:: __main__ ] robot_init_pos_x: " + str(robot_init_pos_x))
    print("[mobiman_framework_launch:: __main__ ] robot_init_pos_y: " + str(robot_init_pos_y))
    print("[mobiman_framework_launch:: __main__ ] robot_init_pos_z: " + str(robot_init_pos_z))
    print("[mobiman_framework_launch:: __main__ ] robot_init_yaw: " + str(robot_init_yaw))
    print("[mobiman_framework_launch:: __main__ ] robot_odometry_msg: " + str(robot_odometry_msg))
    print("[mobiman_framework_launch:: __main__ ] robot_pose_control_msg: " + str(robot_pose_control_msg))
    print("[mobiman_framework_launch:: __main__ ] robot_velo_control_msg: " + str(robot_velo_control_msg))

    ## Map Utility Parameters
    config_map_utility = rospy.get_param('config_map_utility', "")

    print("[mobiman_framework_launch:: __main__ ] Map Utility Parameters:")
    print("[mobiman_framework_launch:: __main__ ] config_map_utility: " + str(config_map_utility))

    ## mobiman Server Parameters
    config_mobiman_server = rospy.get_param('config_mobiman_server', "")

    print("[mobiman_framework_launch:: __main__ ] mobiman-Server Parameters:")
    print("[mobiman_framework_launch:: __main__ ] config_mobiman_server: " + str(config_mobiman_server))

    ## Launch Simulation
    sim_paused = "false"

    if sim == "gazebo":
        
        if robot_name == "firefly":

            # Launch Mav-Garden Gazebo
            mav_garden_path = mobiman_launch_path + "utilities/mav_garden.launch"
            mav_garden_args = [mav_garden_path,
                               'world_name:=' + str(world_name),
                               'world_frame_name:=' + str(world_frame_name),
                               'robot_name:=' + str(robot_name),
                               'robot_model:=' + str(robot_model),
                               'robot_init_pos_x:=' + str(robot_init_pos_x),
                               'robot_init_pos_y:=' + str(robot_init_pos_y),
                               'robot_init_pos_z:=' + str(robot_init_pos_z),
                               'robot_odometry_msg:=' + str(robot_odometry_msg),
                               'robot_pose_control_msg:=' + str(robot_pose_control_msg)]

            mav_garden_launch = [ (roslaunch.rlutil.resolve_launch_arguments(mav_garden_args)[0], mav_garden_args[1:]) ]
            mav_garden = roslaunch.parent.ROSLaunchParent(uuid, mav_garden_launch)
            mav_garden.start()

            print("[mobiman_framework_launch:: __main__ ] Launched Mav-Garden!")

        else:

            if robot_name == "stretch":
                sim_paused = "true"

            # Launch Mobile-Garden Gazebo
            mobile_garden_path = mobiman_launch_path + "utilities/mobile_garden.launch"
            mobile_garden_args = [mobile_garden_path,
                                  'sim_paused:=' + str(sim_paused),
                                  'world_name:=' + str(world_name),
                                  'robot_name:=' + str(robot_name),
                                  'robot_model:=' + str(robot_model),
                                  'robot0_init_pos_x:=' + str(robot_init_pos_x),
                                  'robot0_init_pos_y:=' + str(robot_init_pos_y),
                                  'robot0_init_yaw:=' + str(robot_init_yaw)]

            mobile_garden_launch = [ (roslaunch.rlutil.resolve_launch_arguments(mobile_garden_args)[0], mobile_garden_args[1:]) ]
            mobile_garden = roslaunch.parent.ROSLaunchParent(uuid, mobile_garden_launch)
            mobile_garden.start()

            print("[mobiman_framework_launch:: __main__ ] Launched Mobile-Garden in Gazebo!")
        
        rospy.sleep(2)

    elif sim == "igibson":

        ns1 = "turtlebot2_0"

        # Launch Mobile-Garden iGibson
        mobile_garden_path = mobiman_launch_path + "utilities/mobile_garden_igibson.launch"
        mobile_garden_args = [mobile_garden_path,
                                'ns1:=' + str(ns1),
                                'world_frame_name:=' + str(world_frame_name)]

        mobile_garden_launch = [ (roslaunch.rlutil.resolve_launch_arguments(mobile_garden_args)[0], mobile_garden_args[1:]) ]
        mobile_garden = roslaunch.parent.ROSLaunchParent(uuid, mobile_garden_launch)
        mobile_garden.start()

        print("[mobiman_framework_launch:: __main__ ] Launched Mobile-Garden in iGibson!")
    
    else:
        print("[mobiman_framework_launch:: __main__ ] Empty or wrong simulator name!")

    ## Launch Rviz
    if rviz_flag:

        rviz_path = mobiman_launch_path + "utilities/rviz.launch"
        rviz_args = [rviz_path,
                     'robot_name:=' + str(robot_name)]

        rviz_launch = [ (roslaunch.rlutil.resolve_launch_arguments(rviz_args)[0], rviz_args[1:]) ]
        rviz = roslaunch.parent.ROSLaunchParent(uuid, rviz_launch)
        rviz.start()

        print("[mobiman_framework_launch:: __main__ ] Launched Rviz!")
        rospy.sleep(1)

    ## Wait for simulation to be ready!
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    print("Waiting the transform between " + world_frame_name + " and " + robot_frame_name + "...")
    trans = None
    while (not rospy.is_shutdown()) and (not trans):
        try:
            trans = tfBuffer.lookup_transform(world_frame_name, robot_frame_name, rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
            rospy.logwarn(str(ex))
            rospy.sleep(1.0)

    ## Launch Map Utility Server
    if map_utility_flag:

        map_utility_path = mobiman_launch_path + "utilities/map_utility_server.launch"
        map_utility_args = [map_utility_path,
                            'config_map_utility:=' + str(config_map_utility)]

        map_utility_launch = [ (roslaunch.rlutil.resolve_launch_arguments(map_utility_args)[0], map_utility_args[1:]) ]
        map_utility = roslaunch.parent.ROSLaunchParent(uuid, map_utility_launch)
        map_utility.start()

        print("[mobiman_framework_launch:: __main__ ] Launched Map Utility Server!")
        rospy.sleep(1)
    
    ## Launch mobiman Server
    if mobiman_server_flag:

        mobiman_server_path = mobiman_launch_path + "mobiman_server.launch"
        mobiman_server_args = [mobiman_server_path,
                                   'config_mobiman_server:=' + str(config_mobiman_server)]

        mobiman_server_launch = [ (roslaunch.rlutil.resolve_launch_arguments(mobiman_server_args)[0], mobiman_server_args[1:]) ]
        mobiman_server = roslaunch.parent.ROSLaunchParent(uuid, mobiman_server_launch)
        mobiman_server.start()

        print("[mobiman_framework_launch:: __main__ ] Launched mobiman Server!")
        rospy.sleep(1)

    ## Launch mobiman-DRL Training/Testing
    drl_service_flag = rospy.get_param('drl_service_flag', False)
    mode = rospy.get_param('mode', "")
    
    print("[mobiman_framework_launch:: __main__ ] drl_service_flag: " + str(drl_service_flag))
    print("[mobiman_framework_launch:: __main__ ] mode: " + str(mode))
    
    if drl_service_flag:

        mobiman_drl_path = mobiman_launch_path + "mobiman_drl/mobiman_drl_" + mode + ".launch"
        mobiman_drl_args = [mobiman_drl_path]

        mobiman_drl_launch = [ (roslaunch.rlutil.resolve_launch_arguments(mobiman_drl_args)[0]) ]
        mobiman_drl = roslaunch.parent.ROSLaunchParent(uuid, mobiman_drl_launch)
        mobiman_drl.start()

        print("[mobiman_framework_launch:: __main__ ] Launched mobiman-DRL: " + mode + "!")
    '''

    while (not rospy.is_shutdown()):
        pass