#!/usr/bin/env python3

'''
LAST UPDATE: 2024.02.26

AUTHOR: Neset Unver Akmandor (NUA)

E-MAIL: akmandor.n@northeastern.edu

DESCRIPTION: TODO...

NUA TODO:
'''

import drl
import rospy
import rospkg
import csv
import numpy as np

'''
DESCRIPTION: TODO...
'''
def write_data(file, data, flag_print_info=False):
    file_status = open(file, 'a')
    with file_status:
        write = csv.writer(file_status)
        write.writerows(data)
        if flag_print_info:
            print("[mobiman_drl_config::write_data] Data is written in " + str(file))

'''
DESCRIPTION: TODO...
'''
def read_data(file):
    with open(file, newline='') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        data = np.array(next(reader))
        for row in reader:
            data_row = np.array(row)
            data = np.vstack((data, data_row))
        return data

'''
DESCRIPTION: TODO...
'''
def get_training_param(initial_training_path, param_name) -> str:
    log_file = initial_training_path + 'training_log.csv'
    with open(log_file, newline='') as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            if row[0] == param_name:
                return row[1]
    return ""

'''
DESCRIPTION: TODO...
'''
class Config():

    def __init__(self, log_file="", drl_mode="training", flag_print_info=False):        
        
        self.log_file = log_file
        self.flag_print_info = flag_print_info
        if self.flag_print_info:
            print("[mobiman_drl_config::Config::__init__] START")
            #print("[mobiman_drl_config::Config::__init__] data_folder_path: " + str(data_folder_path))

        ### Training
        self.ros_pkg_name = rospy.get_param('ros_pkg_name', "")
        self.data_save_freq = rospy.get_param('data_save_freq', "")
        self.world_name = rospy.get_param('world_name', "")
        self.world_range_x_min = rospy.get_param('world_range_x_min', 0.0)
        self.world_range_x_max = rospy.get_param('world_range_x_max', 0.0)
        self.world_range_y_min = rospy.get_param('world_range_y_min', 0.0)
        self.world_range_y_max = rospy.get_param('world_range_y_max', 0.0)
        self.world_range_z_min = rospy.get_param('world_range_z_min', 0.0)
        self.world_range_z_max = rospy.get_param('world_range_z_max', 0.0)
        self.init_robot_pos_range_x_min = rospy.get_param('init_robot_pos_range_x_min', 0.0)
        self.init_robot_pos_range_x_max = rospy.get_param('init_robot_pos_range_x_max', 0.0)
        self.init_robot_pos_range_y_min = rospy.get_param('init_robot_pos_range_y_min', 0.0)
        self.init_robot_pos_range_y_max = rospy.get_param('init_robot_pos_range_y_max', 0.0)
        self.init_agent_pos_range_x_min = rospy.get_param('init_agent_pos_range_x_min', 0.0)
        self.init_agent_pos_range_x_max = rospy.get_param('init_agent_pos_range_x_max', 0.0)
        self.init_agent_pos_range_y_min = rospy.get_param('init_agent_pos_range_y_min', 0.0)
        self.init_agent_pos_range_y_max = rospy.get_param('init_agent_pos_range_y_max', 0.0)
        self.world_frame_name = rospy.get_param('world_frame_name', "")
        self.robot_frame_name = rospy.get_param('robot_frame_name', "")
        self.arm_joint_names = rospy.get_param('arm_joint_names', "")
        self.n_armstate = len(self.arm_joint_names) # type: ignore
        self.ee_frame_name = rospy.get_param('ee_frame_name', "")
        self.goal_frame_name = rospy.get_param('goal_frame_name', "")
        self.occupancy_frame_names = rospy.get_param('occupancy_frame_names', "")
        self.n_occupancy = len(self.occupancy_frame_names) # type: ignore
        
        self.base_control_msg_name = rospy.get_param('base_control_msg_name', "")
        self.arm_control_msg_name = rospy.get_param('arm_control_msg_name', "")
        self.mpc_data_msg_name = rospy.get_param('mpc_data_msg_name', "")
        
        self.rgb_image_msg_name = rospy.get_param('rgb_image_msg_name', "")
        self.depth_image_msg_name = rospy.get_param('depth_image_msg_name', "")
        self.depth_image_raw_msg_name = rospy.get_param('depth_image_raw_msg_name', "")
        self.camera_info_msg_name = rospy.get_param('camera_info_msg_name', "")
        self.lidar_msg_name = rospy.get_param('lidar_msg_name', "")
        self.odom_msg_name = rospy.get_param('odom_msg_name', "")
        self.odom_ground_truth_msg_name = rospy.get_param('odom_ground_truth_msg_name', "")

        self.imu_msg_name = rospy.get_param('imu_msg_name', "")
        self.arm_state_msg_name = rospy.get_param('arm_state_msg_name', "")
        self.modelmode_msg_name = rospy.get_param('modelmode_msg_name', "")
        self.target_msg_name = rospy.get_param('target_msg_name', "")
        self.manual_target_msg_name = rospy.get_param('manual_target_msg_name', "")
        self.occgrid_msg_name = rospy.get_param('occgrid_msg_name', "")
        self.mobiman_goal_obs_msg_name = rospy.get_param('mobiman_goal_obs_msg_name', "")
        self.mobiman_occupancy_obs_msg_name = rospy.get_param('mobiman_occupancy_obs_msg_name', "")
        self.selfcoldistance_msg_name = rospy.get_param('selfcoldistance_msg_name', "")
        self.extcoldistance_base_msg_name = rospy.get_param('extcoldistance_base_msg_name', "")
        self.extcoldistance_arm_msg_name = rospy.get_param('extcoldistance_arm_msg_name', "")
        self.pointsonrobot_msg_name = rospy.get_param('pointsonrobot_msg_name', "")
        self.goal_status_msg_name = rospy.get_param('goal_status_msg_name', "")

        #self.data_folder_path = data_folder_path

        rospack = rospkg.RosPack()
        self.ros_pkg_path = rospack.get_path(self.ros_pkg_name) + "/"
        self.training_log_name = rospy.get_param('training_log_name', "")

        self.max_episode_steps = rospy.get_param('max_episode_steps', 0)
        self.training_timesteps = rospy.get_param('training_timesteps', 0)

        ## Sensors
        self.occgrid_normalize_flag = rospy.get_param('occgrid_normalize_flag', False)
        self.occgrid_occ_min = rospy.get_param('occgrid_occ_min', False)
        self.occgrid_occ_max = rospy.get_param('occgrid_occ_max', False)

        ## Algorithm
        self.ablation_mode = rospy.get_param('ablation_mode', "")
        self.observation_space_type = rospy.get_param('observation_space_type', "")

        self.err_threshold_pos = rospy.get_param("err_threshold_pos", 0.0)
        self.err_threshold_ori_yaw = rospy.get_param("err_threshold_ori_yaw", 0.0)
        self.err_threshold_ori_quat = rospy.get_param("err_threshold_ori_quat", 0.0)

        self.obs_base_velo_lat_min = rospy.get_param("obs_base_velo_lat_min", 0.0)
        self.obs_base_velo_lat_max = rospy.get_param("obs_base_velo_lat_max", 0.0)
        self.obs_base_velo_ang_min = rospy.get_param("obs_base_velo_ang_min", 0.0)
        self.obs_base_velo_ang_max = rospy.get_param("obs_base_velo_ang_max", 0.0)
        self.obs_joint_velo_min = rospy.get_param("obs_joint_velo_min", 0.0)
        self.obs_joint_velo_max = rospy.get_param("obs_joint_velo_max", 0.0)

        self.action_time_horizon = rospy.get_param("action_time_horizon", 0.0)
        self.action_type = rospy.get_param("action_type", 0)
        self.action_discrete_trajectory_data_path = rospy.get_param('action_discrete_trajectory_data_path', [])

        self.n_discrete_action = 3       # NUA NOTE: Adding "target is goal" cases for 3 modes 
        for tp in self.action_discrete_trajectory_data_path: # type: ignore
            filepath = self.ros_pkg_path + tp + "sampling_data.csv"
            action_data = read_data(filepath)
            self.n_discrete_action += len(action_data)
            #print("[mobiman_drl_config::Config::__init__] action_data len: " + str(len(action_data)))

        #self.n_action_model = rospy.get_param("n_action_model", 0.0)
        #self.n_action_constraint = rospy.get_param("n_action_constraint", 0.0)
        #self.n_action_target = rospy.get_param("n_action_target", 0.0)

        #self.ablation_mode = rospy.get_param("ablation_mode", 0.0)
        #self.last_step_distance_threshold = rospy.get_param("last_step_distance_threshold", 0.0)
        #self.goal_distance_pos_threshold = rospy.get_param("goal_distance_pos_threshold", 0.0)
        #self.goal_distance_ori_threshold_yaw = rospy.get_param("goal_distance_ori_threshold_yaw", 0.0)
        #self.goal_distance_ori_threshold_quat = rospy.get_param("goal_distance_ori_threshold_quat", 0.0)
        self.goal_range_min_x = rospy.get_param("goal_range_min_x", 0.0)
        self.goal_range_max_x = rospy.get_param('goal_range_max_x', 0.0)
        self.goal_range_min_y = rospy.get_param('goal_range_min_y', 0.0)
        self.goal_range_max_y = rospy.get_param('goal_range_max_y', 0.0)
        self.goal_range_min_z = rospy.get_param('goal_range_min_z', 0.0)
        self.goal_range_max_z = rospy.get_param('goal_range_max_z', 0.0)
        self.self_collision_range_min = rospy.get_param('self_collision_range_min', 0.0)
        self.self_collision_range_max = rospy.get_param('self_collision_range_max', 0.0)
        self.ext_collision_range_base_min = rospy.get_param('ext_collision_range_base_min', 0.0)
        self.ext_collision_range_arm_min = rospy.get_param('ext_collision_range_arm_min', 0.0)
        self.ext_collision_range_max = rospy.get_param('ext_collision_range_max', 0.0)
        #self.rollover_pitch_threshold = rospy.get_param('rollover_pitch_threshold', 0.0)
        #self.rollover_roll_threshold = rospy.get_param('rollover_roll_threshold', 0.0)
        
        self.n_obs_stack = rospy.get_param("n_obs_stack", 0.0)
        self.n_skip_obs_stack = rospy.get_param("n_skip_obs_stack", 0.0)

        self.fc_obs_shape = (-1, )
        self.cnn_obs_shape = (1,-1)

        ## Rewards
        self.reward_terminal_goal = rospy.get_param('reward_terminal_goal', 0.0)
        self.reward_terminal_out_of_boundary = rospy.get_param('reward_terminal_out_of_boundary', 0.0)
        self.reward_terminal_collision = rospy.get_param('reward_terminal_collision', 0.0)
        self.reward_terminal_rollover = rospy.get_param('reward_terminal_rollover', 0.0)
        self.reward_terminal_max_step = rospy.get_param('reward_terminal_max_step', 0.0)
        self.reward_step_mode0 = rospy.get_param('reward_step_mode0', 0.0)
        self.reward_step_mode1 = rospy.get_param('reward_step_mode1', 0.0)
        self.reward_step_mode2 = rospy.get_param('reward_step_mode2', 0.0)
        self.reward_step_goal_scale = rospy.get_param('reward_step_goal_scale', 0.0)
        self.reward_step_goal_dist_threshold_mode1 = rospy.get_param('reward_step_goal_dist_threshold_mode1', 0.0)
        self.reward_step_goal_dist_threshold_mode2 = rospy.get_param('reward_step_goal_dist_threshold_mode2', 0.0)
        self.reward_step_target_scale = rospy.get_param('reward_step_target_scale', 0.0)
        self.reward_step_target_intermediate_point_scale = rospy.get_param('reward_step_target_intermediate_point_scale', 0.0)
        self.reward_step_target_gamma = rospy.get_param('reward_step_target_gamma', 0.0)
        self.reward_step_target_com_scale = rospy.get_param('reward_step_target_com_scale', 0.0)
        self.reward_step_target_com_threshold = rospy.get_param('reward_step_target_com_threshold', 0.0)
        self.alpha_step_mode = rospy.get_param('alpha_step_mode', 0.0)
        self.alpha_step_goal = rospy.get_param('alpha_step_goal', 0.0)
        self.alpha_step_target = rospy.get_param('alpha_step_target', 0.0)

        ### Testing
        self.initial_training_model_name = rospy.get_param('initial_training_model_name', "")
        self.testing_benchmark_name = rospy.get_param('testing_benchmark_name', "")
        self.testing_mode = rospy.get_param('testing_mode', "")
        self.n_testing_eval_episodes = rospy.get_param('n_testing_eval_episodes', 0)

        ## Write all parameters
        '''
        if drl_mode == "training":
            self.log_file = data_folder_path + self.training_log_name + ".csv" # type: ignore
        elif drl_mode == "testing":
            self.log_file = data_folder_path + "testing_log_" + self.testing_benchmark_name + ".csv" # type: ignore
        '''

        if self.flag_print_info:
            print("[mobiman_drl_config::Config::__init__] data_save_freq: " + str(self.data_save_freq))
            print("[mobiman_drl_config::Config::__init__] world_name: " + str(self.world_name))
            print("[mobiman_drl_config::Config::__init__] world_range_x_min: " + str(self.world_range_x_min))
            print("[mobiman_drl_config::Config::__init__] world_range_x_max: " + str(self.world_range_x_max))
            print("[mobiman_drl_config::Config::__init__] world_range_y_min: " + str(self.world_range_y_min))
            print("[mobiman_drl_config::Config::__init__] world_range_y_max: " + str(self.world_range_y_max))
            print("[mobiman_drl_config::Config::__init__] world_range_z_min: " + str(self.world_range_z_min))
            print("[mobiman_drl_config::Config::__init__] world_range_z_max: " + str(self.world_range_z_max))
            print("[mobiman_drl_config::Config::__init__] init_robot_pos_range_x_min: " + str(self.init_robot_pos_range_x_min))
            print("[mobiman_drl_config::Config::__init__] init_robot_pos_range_x_max: " + str(self.init_robot_pos_range_x_max))
            print("[mobiman_drl_config::Config::__init__] init_robot_pos_range_y_min: " + str(self.init_robot_pos_range_y_min))
            print("[mobiman_drl_config::Config::__init__] init_robot_pos_range_y_max: " + str(self.init_robot_pos_range_y_max))
            print("[mobiman_drl_config::Config::__init__] init_agent_pos_range_x_min: " + str(self.init_agent_pos_range_x_min))
            print("[mobiman_drl_config::Config::__init__] init_agent_pos_range_x_max: " + str(self.init_agent_pos_range_x_max))
            print("[mobiman_drl_config::Config::__init__] init_agent_pos_range_y_min: " + str(self.init_agent_pos_range_y_min))
            print("[mobiman_drl_config::Config::__init__] init_agent_pos_range_y_max: " + str(self.init_agent_pos_range_y_max))
            print("[mobiman_drl_config::Config::__init__] world_frame_name: " + str(self.world_frame_name))
            print("[mobiman_drl_config::Config::__init__] n_armstate: " + str(self.n_armstate))
            print("[mobiman_drl_config::Config::__init__] base_control_msg_name: " + str(self.base_control_msg_name))
            print("[mobiman_drl_config::Config::__init__] arm_control_msg_name: " + str(self.arm_control_msg_name))
            print("[mobiman_drl_config::Config::__init__] mpc_data_msg_name: " + str(self.mpc_data_msg_name))
            print("[mobiman_drl_config::Config::__init__] rgb_image_msg_name: " + str(self.rgb_image_msg_name))
            print("[mobiman_drl_config::Config::__init__] depth_image_msg_name: " + str(self.depth_image_msg_name))
            print("[mobiman_drl_config::Config::__init__] depth_image_raw_msg_name: " + str(self.depth_image_raw_msg_name))
            print("[mobiman_drl_config::Config::__init__] camera_info_msg_name: " + str(self.camera_info_msg_name))
            print("[mobiman_drl_config::Config::__init__] lidar_msg_name: " + str(self.lidar_msg_name))
            print("[mobiman_drl_config::Config::__init__] odom_msg_name: " + str(self.odom_msg_name))
            print("[mobiman_drl_config::Config::__init__] odom_ground_truth_msg_name: " + str(self.odom_ground_truth_msg_name))
            print("[mobiman_drl_config::Config::__init__] imu_msg_name: " + str(self.imu_msg_name))
            print("[mobiman_drl_config::Config::__init__] arm_state_msg_name: " + str(self.arm_state_msg_name))
            print("[mobiman_drl_config::Config::__init__] modelmode_msg_name: " + str(self.modelmode_msg_name))
            print("[mobiman_drl_config::Config::__init__] target_msg_name: " + str(self.target_msg_name))
            print("[mobiman_drl_config::Config::__init__] manual_target_msg_name: " + str(self.manual_target_msg_name))
            print("[mobiman_drl_config::Config::__init__] occgrid_msg_name: " + str(self.occgrid_msg_name))
            print("[mobiman_drl_config::Config::__init__] mobiman_goal_obs_msg_name: " + str(self.mobiman_goal_obs_msg_name))
            print("[mobiman_drl_config::Config::__init__] mobiman_occupancy_obs_msg_name: " + str(self.mobiman_occupancy_obs_msg_name))
            print("[mobiman_drl_config::Config::__init__] selfcoldistance_msg_name: " + str(self.selfcoldistance_msg_name))
            print("[mobiman_drl_config::Config::__init__] extcoldistance_base_msg_name: " + str(self.extcoldistance_base_msg_name))
            print("[mobiman_drl_config::Config::__init__] extcoldistance_arm_msg_name: " + str(self.extcoldistance_arm_msg_name))
            print("[mobiman_drl_config::Config::__init__] pointsonrobot_msg_name: " + str(self.pointsonrobot_msg_name))
            print("[mobiman_drl_config::Config::__init__] goal_status_msg_name: " + str(self.goal_status_msg_name))
            print("[mobiman_drl_config::Config::__init__] goal_frame_name: " + str(self.goal_frame_name))
            print("[mobiman_drl_config::Config::__init__] max_episode_steps: " + str(self.max_episode_steps))
            print("[mobiman_drl_config::Config::__init__] training_timesteps: " + str(self.training_timesteps))
            print("[mobiman_drl_config::Config::__init__] occgrid_normalize_flag: " + str(self.occgrid_normalize_flag))
            print("[mobiman_drl_config::Config::__init__] occgrid_occ_min: " + str(self.occgrid_occ_min))
            print("[mobiman_drl_config::Config::__init__] occgrid_occ_max: " + str(self.occgrid_occ_max))
            print("[mobiman_drl_config::Config::__init__] ablation_mode: " + str(self.ablation_mode))
            print("[mobiman_drl_config::Config::__init__] observation_space_type: " + str(self.observation_space_type))
            print("[mobiman_drl_config::Config::__init__] err_threshold_pos: " + str(self.err_threshold_pos))
            print("[mobiman_drl_config::Config::__init__] err_threshold_ori_yaw: " + str(self.err_threshold_ori_yaw))
            print("[mobiman_drl_config::Config::__init__] err_threshold_ori_quat: " + str(self.err_threshold_ori_quat))
            print("[mobiman_drl_config::Config::__init__] obs_base_velo_lat_min: " + str(self.obs_base_velo_lat_min))
            print("[mobiman_drl_config::Config::__init__] obs_base_velo_lat_max: " + str(self.obs_base_velo_lat_max))
            print("[mobiman_drl_config::Config::__init__] obs_base_velo_ang_min: " + str(self.obs_base_velo_ang_min))
            print("[mobiman_drl_config::Config::__init__] obs_base_velo_ang_max: " + str(self.obs_base_velo_ang_max))
            print("[mobiman_drl_config::Config::__init__] obs_joint_velo_min: " + str(self.obs_joint_velo_min))
            print("[mobiman_drl_config::Config::__init__] obs_joint_velo_max: " + str(self.obs_joint_velo_max))
            print("[mobiman_drl_config::Config::__init__] action_time_horizon: " + str(self.action_time_horizon))
            print("[mobiman_drl_config::Config::__init__] action_type: " + str(self.action_type))
            print("[mobiman_drl_config::Config::__init__] action_discrete_trajectory_data_path: " + str(self.action_discrete_trajectory_data_path))
            print("[mobiman_drl_config::Config::__init__] n_discrete_action: " + str(self.n_discrete_action))
            #print("[mobiman_drl_config::Config::__init__] n_action_model: " + str(self.n_action_model))
            #print("[mobiman_drl_config::Config::__init__] n_action_constraint: " + str(self.n_action_constraint))
            #print("[mobiman_drl_config::Config::__init__] n_action_target: " + str(self.n_action_target))
            #print("[mobiman_drl_config::Config::__init__] ablation_mode: " + str(self.ablation_mode))
            #print("[mobiman_drl_config::Config::__init__] last_step_distance_threshold: " + str(self.last_step_distance_threshold))
            #print("[mobiman_drl_config::Config::__init__] goal_distance_pos_threshold: " + str(self.goal_distance_pos_threshold))
            #print("[mobiman_drl_config::Config::__init__] goal_distance_ori_threshold_yaw: " + str(self.goal_distance_ori_threshold_yaw))
            #print("[mobiman_drl_config::Config::__init__] goal_distance_ori_threshold_quat: " + str(self.goal_distance_ori_threshold_quat))
            print("[mobiman_drl_config::Config::__init__] goal_range_min_x: " + str(self.goal_range_min_x))
            print("[mobiman_drl_config::Config::__init__] goal_range_max_x: " + str(self.goal_range_max_x))
            print("[mobiman_drl_config::Config::__init__] goal_range_min_y: " + str(self.goal_range_min_y))
            print("[mobiman_drl_config::Config::__init__] goal_range_max_y: " + str(self.goal_range_max_y))
            print("[mobiman_drl_config::Config::__init__] goal_range_min_z: " + str(self.goal_range_min_z))
            print("[mobiman_drl_config::Config::__init__] goal_range_max_z: " + str(self.goal_range_max_z))
            print("[mobiman_drl_config::Config::__init__] self_collision_range_min: " + str(self.self_collision_range_min))
            print("[mobiman_drl_config::Config::__init__] self_collision_range_max: " + str(self.self_collision_range_max))
            print("[mobiman_drl_config::Config::__init__] ext_collision_range_base_min: " + str(self.ext_collision_range_base_min))
            print("[mobiman_drl_config::Config::__init__] ext_collision_range_arm_min: " + str(self.ext_collision_range_arm_min))
            print("[mobiman_drl_config::Config::__init__] ext_collision_range_max: " + str(self.ext_collision_range_max))
            #print("[mobiman_drl_config::Config::__init__] rollover_pitch_threshold: " + str(self.rollover_pitch_threshold))
            #print("[mobiman_drl_config::Config::__init__] rollover_roll_threshold: " + str(self.rollover_roll_threshold))
            print("[mobiman_drl_config::Config::__init__] n_obs_stack: " + str(self.n_obs_stack))
            print("[mobiman_drl_config::Config::__init__] n_skip_obs_stack: " + str(self.n_skip_obs_stack))
            print("[mobiman_drl_config::Config::__init__] fc_obs_shape: " + str(self.fc_obs_shape))
            print("[mobiman_drl_config::Config::__init__] cnn_obs_shape: " + str(self.cnn_obs_shape))
            print("[mobiman_drl_config::Config::__init__] reward_terminal_goal: " + str(self.reward_terminal_goal))
            print("[mobiman_drl_config::Config::__init__] reward_terminal_out_of_boundary: " + str(self.reward_terminal_out_of_boundary))
            print("[mobiman_drl_config::Config::__init__] reward_terminal_collision: " + str(self.reward_terminal_collision))
            print("[mobiman_drl_config::Config::__init__] reward_terminal_rollover: " + str(self.reward_terminal_rollover))
            print("[mobiman_drl_config::Config::__init__] reward_terminal_max_step: " + str(self.reward_terminal_max_step))
            print("[mobiman_drl_config::Config::__init__] reward_step_mode0: " + str(self.reward_step_mode0))
            print("[mobiman_drl_config::Config::__init__] reward_step_mode1: " + str(self.reward_step_mode1))
            print("[mobiman_drl_config::Config::__init__] reward_step_mode2: " + str(self.reward_step_mode2))
            print("[mobiman_drl_config::Config::__init__] reward_step_goal_scale: " + str(self.reward_step_goal_scale))
            print("[mobiman_drl_config::Config::__init__] reward_step_goal_dist_threshold_mode1: " + str(self.reward_step_goal_dist_threshold_mode1))
            print("[mobiman_drl_config::Config::__init__] reward_step_goal_dist_threshold_mode2: " + str(self.reward_step_goal_dist_threshold_mode2))
            print("[mobiman_drl_config::Config::__init__] reward_step_target_scale: " + str(self.reward_step_target_scale))
            print("[mobiman_drl_config::Config::__init__] reward_step_target_intermediate_point_scale: " + str(self.reward_step_target_intermediate_point_scale))
            print("[mobiman_drl_config::Config::__init__] reward_step_target_gamma: " + str(self.reward_step_target_gamma))
            print("[mobiman_drl_config::Config::__init__] reward_step_target_com_scale: " + str(self.reward_step_target_com_scale))
            print("[mobiman_drl_config::Config::__init__] reward_step_target_com_threshold: " + str(self.reward_step_target_com_threshold))
            print("[mobiman_drl_config::Config::__init__] alpha_step_mode: " + str(self.alpha_step_mode))
            print("[mobiman_drl_config::Config::__init__] alpha_step_goal: " + str(self.alpha_step_goal))
            print("[mobiman_drl_config::Config::__init__] alpha_step_target: " + str(self.alpha_step_target))
            print("[mobiman_drl_config::Config::__init__] initial_training_model_name: " + str(self.initial_training_model_name))
            print("[mobiman_drl_config::Config::__init__] testing_benchmark_name: " + str(self.testing_benchmark_name))
            print("[mobiman_drl_config::Config::__init__] testing_mode: " + str(self.testing_mode))
            print("[mobiman_drl_config::Config::__init__] n_testing_eval_episodes: " + str(self.n_testing_eval_episodes))

        if self.log_file:

            log_data = []
            log_data.append(["data_save_freq", self.data_save_freq])
            log_data.append(["world_name", self.world_name])
            log_data.append(["world_range_x_min", self.world_range_x_min])
            log_data.append(["world_range_x_max", self.world_range_x_max])
            log_data.append(["world_range_y_min", self.world_range_y_min])
            log_data.append(["world_range_y_max", self.world_range_y_max])
            log_data.append(["world_range_z_min", self.world_range_z_min])
            log_data.append(["world_range_z_max", self.world_range_z_max])
            log_data.append(["init_robot_pos_range_x_min", self.init_robot_pos_range_x_min])
            log_data.append(["init_robot_pos_range_x_max", self.init_robot_pos_range_x_max])
            log_data.append(["init_robot_pos_range_y_min", self.init_robot_pos_range_y_min])
            log_data.append(["init_robot_pos_range_y_max", self.init_robot_pos_range_y_max])
            log_data.append(["init_agent_pos_range_x_min", self.init_agent_pos_range_x_min])
            log_data.append(["init_agent_pos_range_x_max", self.init_agent_pos_range_x_max])
            log_data.append(["init_agent_pos_range_y_min", self.init_agent_pos_range_y_min])
            log_data.append(["init_agent_pos_range_y_max", self.init_agent_pos_range_y_max])
            log_data.append(["world_frame_name", self.world_frame_name])
            log_data.append(["robot_frame_name", self.robot_frame_name])
            for i, jname in enumerate(self.arm_joint_names): # type: ignore
                log_data.append(["arm_joint_names_" + str(i), jname])
            log_data.append(["n_armstate", self.n_armstate])
            log_data.append(["ee_frame_name", self.ee_frame_name])
            log_data.append(["goal_frame_name", self.goal_frame_name])
            for i, oname in enumerate(self.occupancy_frame_names): # type: ignore
                log_data.append(["occupancy_frame_names" + str(i), oname])
            log_data.append(["n_occupancy", self.n_occupancy])
            log_data.append(["base_control_msg_name", self.base_control_msg_name])
            log_data.append(["arm_control_msg_name", self.arm_control_msg_name])
            log_data.append(["mpc_data_msg_name", self.mpc_data_msg_name])
            log_data.append(["rgb_image_msg_name", self.rgb_image_msg_name])
            log_data.append(["depth_image_msg_name", self.depth_image_msg_name])
            log_data.append(["depth_image_raw_msg_name", self.depth_image_raw_msg_name])
            log_data.append(["camera_info_msg_name", self.camera_info_msg_name])
            log_data.append(["lidar_msg_name", self.lidar_msg_name])
            log_data.append(["odom_msg_name", self.odom_msg_name])
            log_data.append(["odom_ground_truth_msg_name", self.odom_ground_truth_msg_name])
            log_data.append(["imu_msg_name", self.imu_msg_name])
            log_data.append(["arm_state_msg_name", self.arm_state_msg_name])
            log_data.append(["modelmode_msg_name", self.modelmode_msg_name])
            log_data.append(["target_msg_name", self.target_msg_name])
            log_data.append(["manual_target_msg_name", self.manual_target_msg_name])
            log_data.append(["occgrid_msg_name", self.occgrid_msg_name])
            log_data.append(["mobiman_goal_obs_msg_name", self.mobiman_goal_obs_msg_name])
            log_data.append(["mobiman_occupancy_obs_msg_name", self.mobiman_occupancy_obs_msg_name])
            log_data.append(["selfcoldistance_msg_name", self.selfcoldistance_msg_name])
            log_data.append(["extcoldistance_base_msg_name", self.extcoldistance_base_msg_name])
            log_data.append(["extcoldistance_arm_msg_name", self.extcoldistance_arm_msg_name])
            log_data.append(["pointsonrobot_msg_name", self.pointsonrobot_msg_name])
            log_data.append(["goal_status_msg_name", self.goal_status_msg_name])
            log_data.append(["max_episode_steps", self.max_episode_steps])
            log_data.append(["training_timesteps", self.training_timesteps])
            log_data.append(["occgrid_normalize_flag", self.occgrid_normalize_flag])
            log_data.append(["occgrid_occ_min", self.occgrid_occ_min])
            log_data.append(["occgrid_occ_max", self.occgrid_occ_max])
            log_data.append(["ablation_mode", self.ablation_mode])
            log_data.append(["observation_space_type", self.observation_space_type])
            log_data.append(["err_threshold_pos", self.err_threshold_pos])
            log_data.append(["err_threshold_ori_yaw", self.err_threshold_ori_yaw])
            log_data.append(["err_threshold_ori_quat", self.err_threshold_ori_quat])
            log_data.append(["obs_base_velo_lat_min", self.obs_base_velo_lat_min])
            log_data.append(["obs_base_velo_lat_max", self.obs_base_velo_lat_max])
            log_data.append(["obs_base_velo_ang_min", self.obs_base_velo_ang_min])
            log_data.append(["obs_base_velo_ang_max", self.obs_base_velo_ang_max])
            log_data.append(["obs_joint_velo_min", self.obs_joint_velo_min])
            log_data.append(["obs_joint_velo_max", self.obs_joint_velo_max])
            log_data.append(["action_time_horizon", self.action_time_horizon])
            log_data.append(["action_type", self.action_type])
            for i, oname in enumerate(self.action_discrete_trajectory_data_path): # type: ignore
                log_data.append(["action_discrete_trajectory_data_path" + str(i), oname])
            log_data.append(["n_discrete_action", self.n_discrete_action])
            #log_data.append(["n_action_model", self.n_action_model])
            #log_data.append(["n_action_constraint", self.n_action_constraint])
            #log_data.append(["n_action_target", self.n_action_target])
            #log_data.append(["ablation_mode", self.ablation_mode])
            #log_data.append(["last_step_distance_threshold", self.last_step_distance_threshold])
            #log_data.append(["goal_distance_pos_threshold", self.goal_distance_pos_threshold])
            #log_data.append(["goal_distance_ori_threshold_yaw", self.goal_distance_ori_threshold_yaw])
            #log_data.append(["goal_distance_ori_threshold_quat", self.goal_distance_ori_threshold_quat])
            log_data.append(["goal_range_min_x", self.goal_range_min_x])
            log_data.append(["goal_range_max_x", self.goal_range_max_x])
            log_data.append(["goal_range_min_y", self.goal_range_min_y])
            log_data.append(["goal_range_max_y", self.goal_range_max_y])
            log_data.append(["goal_range_min_z", self.goal_range_min_z])
            log_data.append(["goal_range_max_z", self.goal_range_max_z])
            log_data.append(["self_collision_range_min", self.self_collision_range_min])
            log_data.append(["self_collision_range_max", self.self_collision_range_max])
            log_data.append(["ext_collision_range_base_min", self.ext_collision_range_base_min])
            log_data.append(["ext_collision_range_arm_min", self.ext_collision_range_arm_min])
            log_data.append(["ext_collision_range_max", self.ext_collision_range_max])
            #log_data.append(["rollover_pitch_threshold", self.rollover_pitch_threshold])
            #log_data.append(["rollover_roll_threshold", self.rollover_roll_threshold])
            #log_data.append(["n_obs_stack", self.n_obs_stack])
            #log_data.append(["n_skip_obs_stack", self.n_skip_obs_stack]) 
            log_data.append(["fc_obs_shape", self.fc_obs_shape])
            log_data.append(["cnn_obs_shape", self.cnn_obs_shape])
            log_data.append(["reward_terminal_goal", self.reward_terminal_goal])
            log_data.append(["reward_terminal_out_of_boundary", self.reward_terminal_out_of_boundary])
            log_data.append(["reward_terminal_collision", self.reward_terminal_collision])
            log_data.append(["reward_terminal_rollover", self.reward_terminal_rollover])
            log_data.append(["reward_terminal_max_step", self.reward_terminal_max_step])
            log_data.append(["reward_step_mode0", self.reward_step_mode0])
            log_data.append(["reward_step_mode1", self.reward_step_mode1])
            log_data.append(["reward_step_mode2", self.reward_step_mode2])
            log_data.append(["reward_step_goal_scale", self.reward_step_goal_scale])
            log_data.append(["reward_step_goal_dist_threshold_mode1", self.reward_step_goal_dist_threshold_mode1])
            log_data.append(["reward_step_goal_dist_threshold_mode2", self.reward_step_goal_dist_threshold_mode2])
            log_data.append(["reward_step_target_scale", self.reward_step_target_scale])
            log_data.append(["reward_step_target_intermediate_point_scale", self.reward_step_target_intermediate_point_scale])
            log_data.append(["reward_step_target_gamma", self.reward_step_target_gamma])
            log_data.append(["reward_step_target_com_scale", self.reward_step_target_com_scale])
            log_data.append(["reward_step_target_com_threshold", self.reward_step_target_com_threshold])
            log_data.append(["alpha_step_mode", self.alpha_step_mode])  
            log_data.append(["alpha_step_goal", self.alpha_step_goal])
            log_data.append(["alpha_step_target", self.alpha_step_target])

            write_data(self.log_file, log_data, self.flag_print_info)

    '''
    NUA TODO: 
    '''
    def set_observation_shape(self, obs_shape):
        self.observation_shape = obs_shape
        log_data = []
        log_data.append(["observation_shape", self.observation_shape])
        #log_file = self.data_folder_path + self.training_log_name + ".csv" # type: ignore
        if self.log_file:
            write_data(self.log_file, log_data, self.flag_print_info)
        if self.flag_print_info:
            print("[mobiman_drl_config::Config::set_observation_shape] observation_shape: " + str(self.observation_shape))

    '''
    NUA TODO: 
    '''
    def set_action_shape(self, act_shape):
        self.action_shape = act_shape
        log_data = []
        log_data.append(["action_shape", self.action_shape])
        #log_file = self.data_folder_path + self.training_log_name + ".csv" # type: ignore
        if self.log_file:
            write_data(self.log_file, log_data, self.flag_print_info)
        if self.flag_print_info:
            print("[mobiman_drl_config::Config::set_action_shape] action_shape: " + str(self.action_shape))

    '''
    NUA TODO: 
    '''
    def set_laserscan_config(self, laserscan_msg):
        self.laser_frame_id = laserscan_msg.header.frame_id
        self.laser_angle_min = laserscan_msg.angle_min                   # [rad]
        self.laser_angle_max = laserscan_msg.angle_max                   # [rad]
        self.laser_angle_increment = laserscan_msg.angle_increment       # [rad]
        self.laser_range_min = laserscan_msg.range_min                   # [m]
        self.laser_range_max = laserscan_msg.range_max                   # [m]
        self.laser_time_increment = laserscan_msg.time_increment
        self.laser_scan_time = laserscan_msg.scan_time
        self.laser_n_range = len(laserscan_msg.ranges)
        self.laser_downsample_scale = 1
        '''
        if 0 < self.laser_size_downsampled < len(laserscan_msg.ranges):
            self.laser_downsample_scale = int(len(laserscan_msg.ranges) / self.laser_size_downsampled)
            self.laser_n_range = self.laser_size_downsampled
            self.laser_angle_increment = (self.laser_angle_max - self.laser_angle_min) / self.laser_size_downsampled
        '''

    '''
    NUA TODO: 
    '''
    def set_occgrid_config(self, occgrid_msg):
        self.occgrid_data_size = len(occgrid_msg.data)
        self.occgrid_width = occgrid_msg.info.width
        self.occgrid_height = occgrid_msg.info.height
        self.occgrid_resolution = round(occgrid_msg.info.resolution, 2)

        log_data = []
        log_data.append(["occgrid_data_size", self.occgrid_data_size])
        log_data.append(["occgrid_width", self.occgrid_width])
        log_data.append(["occgrid_height", self.occgrid_height])
        log_data.append(["occgrid_resolution", self.occgrid_resolution])
        #log_file = self.data_folder_path + self.training_log_name + ".csv" # type: ignore
        if self.log_file:
            write_data(self.log_file, log_data, self.flag_print_info)
        if self.flag_print_info:
            print("[mobiman_drl_config::Config::set_occgrid_config] occgrid_data_size: " + str(self.occgrid_data_size))
            print("[mobiman_drl_config::Config::set_occgrid_config] occgrid_width: " + str(self.occgrid_width))
            print("[mobiman_drl_config::Config::set_occgrid_config] occgrid_height: " + str(self.occgrid_height))
            print("[mobiman_drl_config::Config::set_occgrid_config] occgrid_resolution: " + str(self.occgrid_resolution))

    '''
    NUA TODO: 
    '''
    def set_mobiman_goal_obs_config(self, n_mobiman_goal_obs, mobiman_goal_obs_frame_id, mobiman_goal_obs_dim_dt):
        self.n_mobiman_goal_obs = n_mobiman_goal_obs
        self.mobiman_goal_obs_frame_id = mobiman_goal_obs_frame_id
        self.mobiman_goal_obs_dim_dt = mobiman_goal_obs_dim_dt
        log_data = []
        log_data.append(["n_mobiman_goal_obs", self.n_mobiman_goal_obs])
        log_data.append(["mobiman_goal_obs_frame_id", self.mobiman_goal_obs_frame_id])
        log_data.append(["mobiman_goal_obs_dim_dt", self.mobiman_goal_obs_dim_dt])
        #log_file = self.data_folder_path + self.training_log_name + ".csv" # type: ignore
        if self.log_file:
            write_data(self.log_file, log_data, self.flag_print_info)
        if self.flag_print_info:
            print("[mobiman_drl_config::Config::set_mobiman_goal_obs_config] n_mobiman_goal_obs: " + str(self.n_mobiman_goal_obs))
            print("[mobiman_drl_config::Config::set_mobiman_goal_obs_config] mobiman_goal_obs_frame_id: " + str(self.mobiman_goal_obs_frame_id))
            print("[mobiman_drl_config::Config::set_mobiman_goal_obs_config] mobiman_goal_obs_dim_dt: " + str(self.mobiman_goal_obs_dim_dt))

    '''
    NUA TODO: 
    '''
    def set_mobiman_occupancy_obs_config(self, n_mobiman_occupancy_obs, mobiman_occupancy_obs_frame_id, mobiman_occupancy_obs_dim_dt):
        self.n_mobiman_occupancy_obs = n_mobiman_occupancy_obs
        self.mobiman_occupancy_obs_frame_id = mobiman_occupancy_obs_frame_id
        self.mobiman_occupancy_obs_dim_dt = mobiman_occupancy_obs_dim_dt
        log_data = []
        log_data.append(["n_mobiman_occupancy_obs", self.n_mobiman_occupancy_obs])
        log_data.append(["mobiman_occupancy_obs_frame_id", self.mobiman_occupancy_obs_frame_id])
        log_data.append(["mobiman_occupancy_obs_dim_dt", self.mobiman_occupancy_obs_dim_dt])
        #log_file = self.data_folder_path + self.training_log_name + ".csv" # type: ignore
        if self.log_file:
            write_data(self.log_file, log_data, self.flag_print_info)
        if self.flag_print_info:
            print("[mobiman_drl_config::Config::set_mobiman_occupancy_obs_config] n_mobiman_occupancy_obs: " + str(self.n_mobiman_occupancy_obs))
            print("[mobiman_drl_config::Config::set_mobiman_goal_obs_config] mobiman_occupancy_obs_frame_id: " + str(self.mobiman_occupancy_obs_frame_id))
            print("[mobiman_drl_config::Config::set_mobiman_goal_obs_config] mobiman_occupancy_obs_dim_dt: " + str(self.mobiman_occupancy_obs_dim_dt))

    '''
    NUA TODO: 
    '''
    def set_selfcoldistance_config(self, n_selfcoldistance):
        self.n_selfcoldistance = n_selfcoldistance
        log_data = []
        log_data.append(["n_selfcoldistance", self.n_selfcoldistance])
        #log_file = self.data_folder_path + self.training_log_name + ".csv" # type: ignore
        if self.log_file:
            write_data(self.log_file, log_data, self.flag_print_info)
        if self.flag_print_info:
            print("[mobiman_drl_config::Config::set_selfcoldistance_config] n_selfcoldistance: " + str(self.n_selfcoldistance))

    '''
    NUA TODO: 
    '''
    def set_extcoldistance_base_config(self, n_extcoldistance_base):
        self.n_extcoldistance_base = n_extcoldistance_base
        log_data = []
        log_data.append(["n_extcoldistance_base", self.n_extcoldistance_base])
        #log_file = self.data_folder_path + self.training_log_name + ".csv" # type: ignore
        if self.log_file:
            write_data(self.log_file, log_data, self.flag_print_info)
        if self.flag_print_info:
            print("[mobiman_drl_config::Config::set_extcoldistance_base_config] n_extcoldistance_base: " + str(self.n_extcoldistance_base))

    '''
    NUA TODO: 
    '''
    def set_extcoldistance_arm_config(self, n_extcoldistance_arm):
        self.n_extcoldistance_arm = n_extcoldistance_arm
        log_data = []
        log_data.append(["n_extcoldistance_arm", self.n_extcoldistance_arm])
        #log_file = self.data_folder_path + self.training_log_name + ".csv" # type: ignore
        if self.log_file:
            write_data(self.log_file, log_data, self.flag_print_info)
        if self.flag_print_info:
            print("[mobiman_drl_config::Config::set_selfcoldistance_config] n_extcoldistance_arm: " + str(self.n_extcoldistance_arm))