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

        for i, rns in enumerate(robot_ns_vec):
            rviz_path = mobiman_launch_path + "utilities/rviz.launch"
            rviz_args = [rviz_path,
                         'robot_ns:=' + str(rns),
                         'rviz_config_path:=' + str(rviz_config_path) + '_' + str(i)]

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

        for i, rns in enumerate(robot_ns_vec):
            map_server_path = mobiman_launch_path + "utilities/map_server.launch"
            map_server_args = [map_server_path,
                               'robot_ns:=' + str(rns),
                               'config_map_server:=' + str(config_map_server)]

            map_server_launch = [ (roslaunch.rlutil.resolve_launch_arguments(map_server_args)[0], map_server_args[1:]) ]
            map_server = roslaunch.parent.ROSLaunchParent(uuid, map_server_launch)
            map_server.start()

        print("[mobiman_framework_launch:: __main__ ] Launched Map Server!")
        rospy.sleep(1) 

    ### NUA TODO: Change the name as goal server!
    ## Launch ocs2 target 
    if flag_goal_server:

        for i, rns in enumerate(robot_ns_vec):
            goal_server_path = mobiman_launch_path + "utilities/ocs2_target.launch"
            goal_server_args = [goal_server_path,
                                'robot_ns:=' + str(rns),
                                'task_config_path:=' + str(task_config_path)]

            goal_server_launch = [ (roslaunch.rlutil.resolve_launch_arguments(goal_server_args)[0], goal_server_args[1:]) ]
            goal_server = roslaunch.parent.ROSLaunchParent(uuid, goal_server_launch)
            goal_server.start()

        print("[mobiman_framework_launch:: __main__ ] Launched Goal Server!")
        rospy.sleep(1)

    ## Launch mobiman 
    if flag_mobiman:

        for i, rns in enumerate(robot_ns_vec):
            mobiman_path = mobiman_launch_path + "utilities/ocs2_m4.launch"
            mobiman_args = [mobiman_path,
                            'robot_ns:=' + str(rns),
                            'task_config_path:=' + str(task_config_path),
                            'urdf_path:=' + str(urdf_path_ocs2),
                            'lib_path:=' + str(lib_path),
                            'collision_points_config_path:=' + str(collision_points_config_path)]

            mobiman_launch = [ (roslaunch.rlutil.resolve_launch_arguments(mobiman_args)[0], mobiman_args[1:]) ]
            mobiman = roslaunch.parent.ROSLaunchParent(uuid, mobiman_launch)
            mobiman.start()

        print("[mobiman_framework_launch:: __main__ ] Launched Mobiman!")
        rospy.sleep(1)

    while (not rospy.is_shutdown()):
        pass