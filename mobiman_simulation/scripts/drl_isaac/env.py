#### LAST UPDATE: 2023.06.10
##
#### AUTHOR: 
## Neset Unver Akmandor (NUA)
##
#### E-MAIL: 
## akmandor.n@northeastern.edu
##
#### REFERENCE: 
## https://github.com/Lauqz/Easy_DRL_Isaac_Sim
##
#### DESCRIPTION: TODO...

import omni
#omni.timeline.get_timeline_interface().play()

#from omni.isaac.dynamic_control import _dynamic_control
import gym
from gym import spaces
import numpy as np
import math                 
import time

'''
DESCRIPTION: TODO...
'''
class Isaac_envs(gym.Env):
    metadata = {"render.modes": ["human"]}

    '''
    DESCRIPTION: TODO...
    '''
    def __init__(
            self,
            skip_frame=1,
            physics_dt=1.0 / 60.0,
            rendering_dt=1.0 / 60.0,
            max_episode_length=3000,
            seed=0,
            headless=True,
            env_name="random_walk",
            robot_name="jetbot",
            action_type="discrete"
        ) -> None:
        
        print("[env::Isaac_envs::__init__] START")

        from omni.isaac.kit import SimulationApp

        self.headless = headless
        self._simulation_app = SimulationApp({"headless": self.headless, "anti_aliasing": 0})
        self._skip_frame = skip_frame
        self._dt = physics_dt * self._skip_frame
        self._max_episode_length = max_episode_length
        self._steps_after_reset = int(rendering_dt / physics_dt)
        self.dc            = omni.isaac.dynamic_control._dynamic_control.acquire_dynamic_control_interface()
        
        from isaac_robots  import isaac_robot
        from isaac_envs    import isaac_envs  
        from omni.isaac.core.objects import VisualCuboid
        from omni.isaac.core.utils.extensions import enable_extension

        # Enable ROS bridge extension
        enable_extension("omni.isaac.ros_bridge")

        # Check if rosmaster node is running
        # This is to prevent this sample from waiting indefinetly if roscore is not running
        ### NUA NOTE: Can be removed in regular usage
        import rosgraph
        if not rosgraph.is_master_online():
            print("[env::Isaac_envs::__init__] ERROR: Please run roscore before executing this script!")
            self.simulation_app.close()
            exit()

        ### NUA NOTE: Note that this is not the system level rospy, but one compiled for omniverse
        import rospy
        from sensor_msgs.msg import LaserScan

        self._env_name    = env_name
        self._action_type = action_type
        self.episode_count = 0
        self.goal_count = 0
        self.step_count = 0
        self.distance_count = 0

        self.isaac_environments = isaac_envs(headless=self.headless)

        if env_name=="random_walk":
            self._my_world = self.isaac_environments.add_environment(env=env_name, name=robot_name)
            self.isaac_environments._generate_vertical_path_map(random=False)
            robot_pos, robot_ori = self.isaac_environments._robot_pose_random_walk(random=True)
            pos_target           = self.isaac_environments._target_pos_random_walk(random=True)
            
        else:
            self._my_world = self.isaac_environments.add_environment(env=env_name)
            robot_pos  = np.array([0, 0.0, 2.0])
            robot_ori  = np.array([1.0, 0.0, 0.0, 0.0])
            pos_target = np.array([60, 30, 1])


        self.robot = self._my_world.scene.add(
            isaac_robot(
                prim_path="/basic",
                name=robot_name,
                position=robot_pos,
                orientation=robot_ori,
            )
        )

        # ROS Subscribers
        rospy.Subscriber("/scan", LaserScan, self._laser_scan_callback)

        self.isaac_environments._set_camera(name=self.robot._name, prim_path=self.robot._prim_path, headless=self.headless)
        self.isaac_environments._set_lidar(name=self.robot._name, prim_path=self.robot._prim_path, headless=self.headless)

        self.goal = self._my_world.scene.add(
            VisualCuboid(
                prim_path="/new_cube_1",
                name="visual_cube",
                position=pos_target,
                size=0.01,
                scale= np.array([5, 5, 5]),
                color=np.array([1.0, 0, 0]),
            )
        )
        self.seed(seed)
        self.sd_helper = None
        self.viewport_window = None
        self.reward_range = (-float("inf"), float("inf"))
        gym.Env.__init__(self)

        self.action_space = self.robot.get_action_space(type=action_type)
        self.movements    = self.robot.get_discrete_actions()

        print("[env::Isaac_envs::__init__] self.movements:")
        print(self.movements)

        self.state_pos_space = spaces.Box(
			low=np.array([0,-np.pi], dtype=np.float32),
			high=np.array([8,np.pi], dtype=np.float32),
			dtype=np.float32
		)

        self.state_vel_space = spaces.Box(
			low=np.array([0,-np.pi/4], dtype=np.float32),
			high=np.array([10,np.pi/4], dtype=np.float32),
			dtype=np.float32
        )
        
        self.state_IR_space = spaces.Box(low=0, high=1, shape=(1,12), dtype=np.float32)

        self.depth_obs_space = spaces.Box(low=0, high=1, shape=(1, 128, 128), dtype=np.uint8)
        self.rgb_obs_space   = spaces.Box(low=0, high=255, shape=(128, 128, 3), dtype=np.uint8)

        self.observation_space = spaces.Dict(spaces={
				"IR_raleted" : self.state_IR_space, 
				"pos_raleted": self.state_pos_space,
                "vel_raleted": self.state_vel_space,
                #"rgb_raleted": self.state_h_space,
                #"depth_depth": self.cam_obs_space,
				}
		)

        print("[env::Isaac_envs::__init__] END")

        return

    '''
    DESCRIPTION: TODO...
    '''
    def get_dt(self):
        
        print("[env::Isaac_envs::get_dt] START")
        
        return self._dt

    '''
    DESCRIPTION: TODO...
    '''
    def step(self, action):

        print("[env::Isaac_envs::step] START")

        previous_jetbot_position, _ = self.robot.get_world_pose()

        #print("[env::Isaac_envs::step] previous_jetbot_position type: " + str(type(previous_jetbot_position)))
        #print("[env::Isaac_envs::step] previous_jetbot_position: " + str(previous_jetbot_position))

        ## Robot take action and run a simulation step
        for i in range(self._skip_frame):

            if self._action_type=="continuous":
                if self.robot._is_differential:
                    self.robot.differential_controller(np.array([action[0], action[1]]))
                else:
                    self.robot.holonomic_controller(np.array([action[0], action[1], action[2]]))
            
            elif self._action_type=="discrete":
                selected_action = self.movements[action]
                
                #print("[env::Isaac_envs::step] self.movements:")
                #print(selected_action)
                
                if self.robot._is_differential:
                    self.robot.differential_controller(np.array([selected_action[0], selected_action[1]]))
                else:
                    self.robot.holonomic_controller(np.array([selected_action[0], selected_action[1], selected_action[2]]))
            
            self._my_world.step(render=False)
        
        current_jetbot_position, _ = self.robot.get_world_pose()

        self.distance_count += np.linalg.norm(current_jetbot_position - previous_jetbot_position)
        
        ## Observation
        observations = self.get_observations()

        info = {}

        ## Reward function and end condition
        done = False
        if self._my_world.current_time_step_index - self._steps_after_reset >= self._max_episode_length:
            done = True
        
        goal_world_position, _ = self.goal.get_world_pose()
        current_dist_to_goal = self.robot.distance_to(goal_world_position)

        depth_points = self.isaac_environments._get_lidar_data()
        depth_points_min = np.amin(depth_points)
        
        step_conservation = 1 - (self._my_world.current_time_step_index/self._max_episode_length)# 1 + (self._my_world.current_time_step_index/self._max_episode_length)#
        landing_reward = 90
        distance_reward = (-current_dist_to_goal)/100 # previous_dist_to_goal - current_dist_to_goal -0.1

        if distance_reward > 0:
            reward = distance_reward*step_conservation
        else:
            reward = distance_reward

        #if depth_points_min <= 0.30 and depth_points_min > 0.15:
        #    #done = True
        #    reward -= (0.6-depth_points_min)

        if depth_points_min <= 0.155:
            done = True
            reward -= 10*(0.6-depth_points_min)
        
        if current_dist_to_goal <= 0.5:
            done = True
            self.goal_count += 1
            print("[env::Isaac_envs::step] ARRIBA!")
            reward += 10 + landing_reward * (1 - (self._my_world.current_time_step_index/self._max_episode_length))

        self.step_count = self._my_world.current_time_step_index

        print("[env::Isaac_envs::step] END")

        return observations, reward, done, info

    '''
    DESCRIPTION: TODO...
    '''
    def reset(self):

        print("[env::Isaac_envs::reset] START")
        
        self._my_world.reset()
        if self._env_name=="random_walk":
            # randomize goal location in circle around robot
            position = self.isaac_environments._target_pos_random_walk(random=True)
            self.goal.set_world_pose(position)
            # randomize robot pose
            robot_pos, robot_ori = self.isaac_environments._robot_pose_random_walk(random=True)
            self.robot.set_robot_pose(position=robot_pos, orientation=robot_ori)

        observations = self.get_observations()

        print("[env::Isaac_envs::reset] Hit rate: " + str(self.goal_count) + "/" + str(self.episode_count))
        print("[env::Isaac_envs::reset] Steps on episode " + str(self.episode_count) + ": " + str(self.step_count))
        print("[env::Isaac_envs::reset] Seconds on episode " + str(self.episode_count) + ": " + str( (self.step_count)/60 ))
        print("[env::Isaac_envs::reset] Distance on episode " + str(self.episode_count) + ": " + str(self.distance_count))
        self.episode_count += 1
        self.distance_count = 0
        
        print("[env::Isaac_envs::reset] END")
        
        return observations

    '''
    DESCRIPTION: TODO...
    '''
    def get_observations(self):

        print("[env::Isaac_envs::get_observations] START")

        ## Camera Data
        # rgb_data   = self.isaac_environments._get_cam_data(type="rgb")
        #depth_data = self.isaac_environments._get_cam_data(type="depth")
        self._my_world.render()
        
        #print("[env::Isaac_envs::reset] self.robot.name: " + str(self.robot.name))
        #print("[env::Isaac_envs::reset] self.robot._name: " + str(self.robot._name))

        ## Lidar Data
        if self.robot.name == "jackal_jaco":  
            lidar_data = self._get_lidar_data()
        else:
            lidar_data = self.isaac_environments._get_lidar_data()

        print("[env::Isaac_envs::get_observations] lidar_data:")
        print(lidar_data)

        #print("[env::Isaac_envs::get_observations] DEBUG INF")
        #while 1:
        #    continue

        # for transporter uncomment the next line
        # lidar_data2 = self.isaac_environments._get_lidar_data(lidar_selector=2)

        ## Distance and angular differencess
        goal_world_position, _ = self.goal.get_world_pose()
        d = self.robot.distance_to(goal_world_position)
        angle = self.robot.angular_difference_to(goal_world_position)
        target_relative_to_robot_data = np.array([d, angle])

        ## Robot base's velocities
        real_V = self.robot.get_lineal_vel_base()
        real_W = self.robot.get_angular_vel_base()

        vase_vel_data = np.array([ real_V, real_W])

        obs = {"IR_raleted" : lidar_data, "pos_raleted" : target_relative_to_robot_data, "vel_raleted" : vase_vel_data} 
        #obs = {"h_raleted" : h_state, "vel_raleted" : obs_state}

        print("[env::Isaac_envs::get_observations] END")

        return obs

    '''
    DESCRIPTION: TODO...
    '''
    def render(self, mode="human"):

        print("[env::Isaac_envs::render] START")

        return

    '''
    DESCRIPTION: TODO...
    '''
    def close(self):

        print("[env::Isaac_envs::close] START")

        self._simulation_app.close()

        print("[env::Isaac_envs::close] END")

        return

    '''
    DESCRIPTION: TODO...
    '''
    def seed(self, seed=None):

        print("[env::Isaac_envs::seed] START")

        self.np_random, seed = gym.utils.seeding.np_random(seed)
        np.random.seed(seed)

        print("[env::Isaac_envs::seed] END")

        return [seed]
    
    '''
    DESCRIPTION: TODO...
    '''
    def _laser_scan_callback(self, data):

        print("[env::Isaac_envs::_laser_scan_callback] Incoming data...")
        
        self._laser_scan = data

    '''
    DESCRIPTION: TODO...
    '''
    def _get_lidar_data(self):

        print("[env::Isaac_envs::_get_lidar_data] END")

        data_raw = self._laser_scan

        print("[env::Isaac_envs::_get_lidar_data] data_raw len: " + str(len(data_raw)))
        lidar_data = data_raw

        if len(data_raw) < 12:
            print("[env::Isaac_envs::_get_lidar_data] DEBUG INF")
            while 1:
                continue
        elif len(data_raw) > 12:
            lidar_data = data_raw[0:12]
        
        print("[env::Isaac_envs::_get_lidar_data] END")

        return lidar_data
