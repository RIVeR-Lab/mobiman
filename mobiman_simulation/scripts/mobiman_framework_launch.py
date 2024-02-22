#!/usr/bin/env python3

'''
LAST UPDATE: 2024.02.22

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

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False) # type: ignore

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
    flag_sim = rospy.get_param('flag_sim', False)
    sim = rospy.get_param('sim', "")
    flag_gazebo = rospy.get_param('flag_gazebo', True)
    flag_robot = rospy.get_param('flag_robot', True)
    flag_conveyor = rospy.get_param('flag_conveyor', True)
    flag_pedsim = rospy.get_param('flag_pedsim', False)
    flag_moveit = rospy.get_param('flag_moveit', False)
    flag_drl = rospy.get_param('flag_drl', False)
    flag_igibson = rospy.get_param('flag_igibson', False)

    ### Rviz:
    rviz_config_path = rospy.get_param('rviz_config_path', "")
    flag_rviz = rospy.get_param('flag_rviz', True)
    
    ### Map Server:
    flag_map_server = rospy.get_param('flag_map_server', True)
    config_map_server = rospy.get_param('config_map_server', "")
    load_config_file(mobiman_path + "config/" + config_map_server + ".yaml")
    world_frame_name = rospy.get_param('world_frame_name', "")

    ### Robot:
    robot_name = rospy.get_param('robot_name', "")
    n_robot = rospy.get_param('n_robot', "")
    robot_frame_name = rospy.get_param('robot_frame_name', "")
    urdf_path = rospy.get_param('urdf_path', "")
    urdf_path_ocs2 = rospy.get_param('urdf_path_ocs2', "")
    lib_path = rospy.get_param('lib_path', "")
    collision_points_config_path = rospy.get_param('collision_points_config_path', "")

    ### Goal Server:
    flag_goal_server = rospy.get_param('flag_goal_server', True)
    config_goal_server = rospy.get_param('config_goal_server', "")
    load_config_file(mobiman_path + "config/" + config_goal_server + ".yaml")

    ### Observation Server:
    flag_observation_server = rospy.get_param('flag_observation_server', False)
    config_observation_server = rospy.get_param('config_observation_server', "")

    ### Distance Server:
    flag_distance_server = rospy.get_param('flag_distance_server', True)
    config_distance_server = rospy.get_param('config_distance_server', "")

    ### Task:
    flag_mobiman_mpc = rospy.get_param('flag_mobiman_mpc', True)
    flag_mobiman_mrt = rospy.get_param('flag_mobiman_mrt', True)
    task_config_path = rospy.get_param('task_config_path', "")
    config_mobiman_drl = rospy.get_param('config_mobiman_drl', "")
    load_config_file(mobiman_path + "config/" + config_mobiman_drl + ".yaml")

    print("[mobiman_framework_launch:: __main__ ] General Parameters ---------- START")
    print("[mobiman_framework_launch:: __main__ ] Simulator:")
    print("[mobiman_framework_launch:: __main__ ] flag_sim: " + str(flag_sim))
    print("[mobiman_framework_launch:: __main__ ] sim: " + str(sim))
    print("[mobiman_framework_launch:: __main__ ] flag_gazebo: " + str(flag_gazebo))
    print("[mobiman_framework_launch:: __main__ ] flag_robot: " + str(flag_robot))
    print("[mobiman_framework_launch:: __main__ ] flag_conveyor: " + str(flag_conveyor))
    print("[mobiman_framework_launch:: __main__ ] flag_pedsim: " + str(flag_pedsim))
    print("[mobiman_framework_launch:: __main__ ] flag_moveit: " + str(flag_moveit))
    print("[mobiman_framework_launch:: __main__ ] flag_drl: " + str(flag_drl))
    print("[mobiman_framework_launch:: __main__ ] flag_igibson: " + str(flag_igibson))

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
    print("[mobiman_framework_launch:: __main__ ] config_goal_server: " + str(config_goal_server))

    print("[mobiman_framework_launch:: __main__ ] Observation Server:")
    print("[mobiman_framework_launch:: __main__ ] flag_observation_server: " + str(flag_observation_server))
    print("[mobiman_framework_launch:: __main__ ] config_observation_server: " + str(config_observation_server))

    print("[mobiman_framework_launch:: __main__ ] Distance Server:")
    print("[mobiman_framework_launch:: __main__ ] flag_distance_server: " + str(flag_distance_server))
    print("[mobiman_framework_launch:: __main__ ] config_distance_server: " + str(config_distance_server))

    print("[mobiman_framework_launch:: __main__ ] Mobiman:")
    print("[mobiman_framework_launch:: __main__ ] flag_mobiman_mpc: " + str(flag_mobiman_mpc))
    print("[mobiman_framework_launch:: __main__ ] flag_mobiman_mrt: " + str(flag_mobiman_mrt))
    print("[mobiman_framework_launch:: __main__ ] task_config_path: " + str(task_config_path))
    print("[mobiman_framework_launch:: __main__ ] config_mobiman_drl: " + str(config_mobiman_drl))

    print("[mobiman_framework_launch:: __main__ ] General Parameters ---------- END")
    print("")

    ## Set Namespace
    robot_ns_vec = []
    robot_base_frame_name_vec = []

    print("[mobiman_framework_launch:: __main__ ] robot_ns_vec:")
    for i in range(n_robot): # type: ignore
        ns_tmp = robot_name + "_" + str(i) # type: ignore
        robot_ns_vec.append(ns_tmp)
        print(ns_tmp)

        # Update frame names
        robot_base_frame_name_vec.append(ns_tmp +  "/" + str(robot_frame_name))

    print("[mobiman_framework_launch:: __main__ ] robot_base_frame_name_vec:")
    print(robot_base_frame_name_vec)

    #print("[mobiman_framework_launch:: __main__ ] DEBUG_INF")
    #while 1:
    #    continue

    ## Launch Simulation
    if flag_sim:
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

            sim_launch = [ (roslaunch.rlutil.resolve_launch_arguments(sim_args)[0], sim_args[1:]) ] # type: ignore
            sim_obj = roslaunch.parent.ROSLaunchParent(uuid, sim_launch) # type: ignore
            sim_obj.start()

            print("[mobiman_framework_launch:: __main__ ] Launched simulation in Gazebo!")

        elif sim == "igibson":

            # Launch iGibson
            sim_launch_path = igibson_path + "launch/mobiman_jackal_jaco.launch"

            for i, rns in enumerate(robot_ns_vec):
                sim_args = [sim_launch_path,
                            'robot_ns:=' + str(rns),
                            'urdf_path:=' + str(urdf_path),
                            'flag_drl:=' + str(flag_drl),
                            'flag_igibson:=' + str(flag_igibson)]

                sim_launch = [ (roslaunch.rlutil.resolve_launch_arguments(sim_args)[0], sim_args[1:]) ] # type: ignore
                sim = roslaunch.parent.ROSLaunchParent(uuid, sim_launch) # type: ignore
                sim.start()

            ## Wait for simulation to be ready!
            tfBuffer = tf2_ros.Buffer() # type: ignore
            listener = tf2_ros.TransformListener(tfBuffer) # type: ignore
            robot_frame_name = robot_base_frame_name_vec[0]
            print("[mobiman_framework_launch:: __main__ ] Waiting the transform between " + world_frame_name + " and " + robot_frame_name + "...") # type: ignore
            trans = None
            while (not rospy.is_shutdown()) and (not trans):
                try:
                    trans = tfBuffer.lookup_transform(world_frame_name, robot_frame_name, rospy.Time(0))
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex: # type: ignore
                    #rospy.logwarn("[mobiman_framework_launch:: __main__ ] ERROR: " + str(ex))
                    rospy.sleep(1.0)   

            print("[mobiman_framework_launch:: __main__ ] Launched simulation in iGibson!")
        
        else:
            print("[mobiman_framework_launch:: __main__ ] Empty or wrong simulator name! sim: " + str(sim))

    ## Launch Rviz
    if flag_rviz:
        
        if len(robot_ns_vec) > 0:
            for i, rns in enumerate(robot_ns_vec):
                rviz_path = mobiman_launch_path + "utilities/rviz.launch"
                rviz_args = [rviz_path,
                            'robot_ns:=' + str(rns),
                            'rviz_config_path:=' + str(rviz_config_path) + '_' + str(i)]

                rviz_launch = [ (roslaunch.rlutil.resolve_launch_arguments(rviz_args)[0], rviz_args[1:]) ] # type: ignore
                rviz = roslaunch.parent.ROSLaunchParent(uuid, rviz_launch) # type: ignore
                rviz.start()
        else:
            rviz_path = mobiman_launch_path + "utilities/rviz.launch"
            rviz_args = [rviz_path,
                         'robot_ns:=/',
                         'rviz_config_path:=' + str(rviz_config_path) + '_0']  

            rviz_launch = [ (roslaunch.rlutil.resolve_launch_arguments(rviz_args)[0], rviz_args[1:]) ] # type: ignore
            rviz = roslaunch.parent.ROSLaunchParent(uuid, rviz_launch) # type: ignore
            rviz.start()      

        print("[mobiman_framework_launch:: __main__ ] Launched Rviz!")
        rospy.sleep(1)

    #print("[mobiman_framework_launch:: __main__ ] DEBUG_INF")
    #while 1:
    #    continue

    ## Launch Map Server
    if flag_map_server:

        if len(robot_ns_vec) > 0:
            for i, rns in enumerate(robot_ns_vec):
                map_server_path = mobiman_launch_path + "utilities/map_server.launch"
                map_server_args = [map_server_path,
                                'robot_ns:=' + str(rns),
                                'config_map_server:=' + str(config_map_server)]
                
                map_server_launch = [ (roslaunch.rlutil.resolve_launch_arguments(map_server_args)[0], map_server_args[1:]) ] # type: ignore
                map_server = roslaunch.parent.ROSLaunchParent(uuid, map_server_launch) # type: ignore
                map_server.start()  
        else:
            map_server_path = mobiman_launch_path + "utilities/map_server.launch"
            map_server_args = [map_server_path,
                               'robot_ns:=/',
                               'config_map_server:=' + str(config_map_server)]

            map_server_launch = [ (roslaunch.rlutil.resolve_launch_arguments(map_server_args)[0], map_server_args[1:]) ] # type: ignore
            map_server = roslaunch.parent.ROSLaunchParent(uuid, map_server_launch) # type: ignore
            map_server.start()  

        print("[mobiman_framework_launch:: __main__ ] Launched Map Server!")
        rospy.sleep(1) 

    ### NUA TODO: Change the name as goal server!
    ## Launch ocs2 target 
    if flag_goal_server:

        if len(robot_ns_vec) > 0:
            for i, rns in enumerate(robot_ns_vec):
                goal_server_path = mobiman_launch_path + "utilities/ocs2_target.launch"
                goal_server_args = [goal_server_path,
                                    'robot_ns:=' + str(rns),
                                    'config_goal_server:=' + str(config_goal_server),
                                    'task_config_path:=' + str(task_config_path)]
                
                goal_server_launch = [ (roslaunch.rlutil.resolve_launch_arguments(goal_server_args)[0], goal_server_args[1:]) ] # type: ignore
                goal_server = roslaunch.parent.ROSLaunchParent(uuid, goal_server_launch) # type: ignore
                goal_server.start()
        else:
            goal_server_path = mobiman_launch_path + "utilities/ocs2_target.launch"
            goal_server_args = [goal_server_path,
                                'robot_ns:=/',
                                'config_goal_server:=' + str(config_goal_server),
                                'task_config_path:=' + str(task_config_path)]
        
            goal_server_launch = [ (roslaunch.rlutil.resolve_launch_arguments(goal_server_args)[0], goal_server_args[1:]) ] # type: ignore
            goal_server = roslaunch.parent.ROSLaunchParent(uuid, goal_server_launch) # type: ignore
            goal_server.start()

        print("[mobiman_framework_launch:: __main__ ] Launched Goal Server!")
        rospy.sleep(1)

    ## Launch observation server 
    if flag_observation_server:

        if len(robot_ns_vec) > 0:
            for i, rns in enumerate(robot_ns_vec):
                observation_server_path = mobiman_launch_path + "utilities/observation_server.launch"
                observation_server_args = [observation_server_path,
                                        'robot_ns:=' + str(rns),
                                        'config_observation_server:=' + str(config_observation_server),
                                        'collision_points_config_path:=' + str(collision_points_config_path),
                                        'urdf_path:=' + str(urdf_path_ocs2),
                                        'lib_path:=' + str(lib_path),
                                        'task_config_path:=' + str(task_config_path)]
                
                observation_server_launch = [ (roslaunch.rlutil.resolve_launch_arguments(observation_server_args)[0], observation_server_args[1:]) ] # type: ignore
                observation_server = roslaunch.parent.ROSLaunchParent(uuid, observation_server_launch) # type: ignore
                observation_server.start()
        else:
            observation_server_path = mobiman_launch_path + "utilities/observation_server.launch"
            observation_server_args = [observation_server_path,
                                    'robot_ns:=/',
                                    'config_observation_server:=' + str(config_observation_server),
                                    'collision_points_config_path:=' + str(collision_points_config_path),
                                    'urdf_path:=' + str(urdf_path_ocs2),
                                    'lib_path:=' + str(lib_path),
                                    'task_config_path:=' + str(task_config_path)]
        
            observation_server_launch = [ (roslaunch.rlutil.resolve_launch_arguments(observation_server_args)[0], observation_server_args[1:]) ] # type: ignore
            observation_server = roslaunch.parent.ROSLaunchParent(uuid, observation_server_launch) # type: ignore
            observation_server.start()

        print("[mobiman_framework_launch:: __main__ ] Launched observation Server!")
        rospy.sleep(5)

    ## Launch distance server 
    if flag_distance_server:

        if len(robot_ns_vec) > 0:
            for i, rns in enumerate(robot_ns_vec):
                distance_server_path = mobiman_launch_path + "utilities/distance_server.launch"
                distance_server_args = [distance_server_path,
                                        'robot_ns:=' + str(rns),
                                        'config_distance_server:=' + str(config_distance_server),
                                        'collision_points_config_path:=' + str(collision_points_config_path),
                                        'urdf_path:=' + str(urdf_path_ocs2),
                                        'lib_path:=' + str(lib_path),
                                        'task_config_path:=' + str(task_config_path)]
                
                distance_server_launch = [ (roslaunch.rlutil.resolve_launch_arguments(distance_server_args)[0], distance_server_args[1:]) ] # type: ignore
                distance_server = roslaunch.parent.ROSLaunchParent(uuid, distance_server_launch) # type: ignore
                distance_server.start()
        else:
            distance_server_path = mobiman_launch_path + "utilities/distance_server.launch"
            distance_server_args = [distance_server_path,
                                    'robot_ns:=/',
                                    'config_distance_server:=' + str(config_distance_server),
                                    'collision_points_config_path:=' + str(collision_points_config_path),
                                    'urdf_path:=' + str(urdf_path_ocs2),
                                    'lib_path:=' + str(lib_path),
                                    'task_config_path:=' + str(task_config_path)]
        
            distance_server_launch = [ (roslaunch.rlutil.resolve_launch_arguments(distance_server_args)[0], distance_server_args[1:]) ] # type: ignore
            distance_server = roslaunch.parent.ROSLaunchParent(uuid, distance_server_launch) # type: ignore
            distance_server.start()

        print("[mobiman_framework_launch:: __main__ ] Launched Distance Server!")
        rospy.sleep(5)

    ## Launch mobiman 
    if flag_mobiman_mpc:

        if len(robot_ns_vec) > 0:
            for i, rns in enumerate(robot_ns_vec):
                mobiman_mpc_path = mobiman_launch_path + "utilities/ocs2_m4_mpc.launch"
                mobiman_mpc_args = [mobiman_mpc_path,
                                'robot_ns:=' + str(rns),
                                'task_config_path:=' + str(task_config_path),
                                'urdf_path:=' + str(urdf_path_ocs2),
                                'lib_path:=' + str(lib_path),
                                'collision_points_config_path:=' + str(collision_points_config_path)]
                
                mobiman_mpc_launch = [ (roslaunch.rlutil.resolve_launch_arguments(mobiman_mpc_args)[0], mobiman_mpc_args[1:]) ] # type: ignore
                mobiman_mpc = roslaunch.parent.ROSLaunchParent(uuid, mobiman_mpc_launch) # type: ignore
                mobiman_mpc.start()

                '''
                print("[mobiman_framework_launch:: __main__ ] robot_ns: " + str(rns))
                print("[mobiman_framework_launch:: __main__ ] task_config_path: " + str(task_config_path))
                print("[mobiman_framework_launch:: __main__ ] urdf_path: " + str(urdf_path_ocs2))
                print("[mobiman_framework_launch:: __main__ ] lib_path: " + str(lib_path))
                print("[mobiman_framework_launch:: __main__ ] collision_points_config_path: " + str(collision_points_config_path))
                '''
        else:
            mobiman_mpc_path = mobiman_launch_path + "utilities/ocs2_m4_mpc.launch"
            mobiman_mpc_args = [mobiman_mpc_path,
                            'robot_ns:=/',
                            'task_config_path:=' + str(task_config_path),
                            'urdf_path:=' + str(urdf_path_ocs2),
                            'lib_path:=' + str(lib_path),
                            'collision_points_config_path:=' + str(collision_points_config_path)]


            mobiman_mpc_launch = [ (roslaunch.rlutil.resolve_launch_arguments(mobiman_mpc_args)[0], mobiman_mpc_args[1:]) ] # type: ignore
            mobiman_mpc = roslaunch.parent.ROSLaunchParent(uuid, mobiman_mpc_launch) # type: ignore
            mobiman_mpc.start()

        print("[mobiman_framework_launch:: __main__ ] Launched Mobiman MPC!")
        rospy.sleep(5)

    if flag_mobiman_mrt:

        if len(robot_ns_vec) > 0:
            for i, rns in enumerate(robot_ns_vec):
                mobiman_mrt_path = mobiman_launch_path + "utilities/ocs2_m4_mrt.launch"
                mobiman_mrt_args = [mobiman_mrt_path,
                                'robot_ns:=' + str(rns),
                                'task_config_path:=' + str(task_config_path),
                                'urdf_path:=' + str(urdf_path_ocs2),
                                'lib_path:=' + str(lib_path),
                                'collision_points_config_path:=' + str(collision_points_config_path)]
                
                mobiman_mrt_launch = [ (roslaunch.rlutil.resolve_launch_arguments(mobiman_mrt_args)[0], mobiman_mrt_args[1:]) ] # type: ignore
                mobiman_mrt = roslaunch.parent.ROSLaunchParent(uuid, mobiman_mrt_launch) # type: ignore
                mobiman_mrt.start()

                '''
                print("[mobiman_framework_launch:: __main__ ] robot_ns: " + str(rns))
                print("[mobiman_framework_launch:: __main__ ] task_config_path: " + str(task_config_path))
                print("[mobiman_framework_launch:: __main__ ] urdf_path: " + str(urdf_path_ocs2))
                print("[mobiman_framework_launch:: __main__ ] lib_path: " + str(lib_path))
                print("[mobiman_framework_launch:: __main__ ] collision_points_config_path: " + str(collision_points_config_path))
                '''
        else:            
            mobiman_mrt_path = mobiman_launch_path + "utilities/ocs2_m4_mrt.launch"
            mobiman_mrt_args = [mobiman_mrt_path,
                            'robot_ns:=/',
                            'task_config_path:=' + str(task_config_path),
                            'urdf_path:=' + str(urdf_path_ocs2),
                            'lib_path:=' + str(lib_path),
                            'collision_points_config_path:=' + str(collision_points_config_path)]

            mobiman_mrt_launch = [ (roslaunch.rlutil.resolve_launch_arguments(mobiman_mrt_args)[0], mobiman_mrt_args[1:]) ] # type: ignore
            mobiman_mrt = roslaunch.parent.ROSLaunchParent(uuid, mobiman_mrt_launch) # type: ignore
            mobiman_mrt.start()

        print("[mobiman_framework_launch:: __main__ ] Launched Mobiman MRT!")
        #rospy.sleep(1)

    while (not rospy.is_shutdown()):
        pass