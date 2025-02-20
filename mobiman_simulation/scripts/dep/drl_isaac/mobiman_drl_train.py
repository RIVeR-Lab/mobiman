#### LAST UPDATE: 2023.06.09
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
##
## Tensorboard view (DEPRECATED):
## ~/.local/share/ov/pkg/isaac_sim-2022.2.1/python.sh ~/.local/share/ov/pkg/isaac_sim-2022.2.1/tensorboard --logdir ./
##
## Real time nvidia-smi:
## watch -n0.1 nvidia-smi
##
## Train Commands (DEPRECATED):
## cd ~/.local/share/ov/pkg/isaac_sim-2022.2.1/DRL_Isaac_lib/
## ~/.local/share/ov/pkg/isaac_sim-2022.2.1/python.sh train_d.py

from tabnanny import verbose
from env import Isaac_envs
from stable_baselines3 import PPO, DQN
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor
from stable_baselines3.common.callbacks import CheckpointCallback
import torch as th
from torch import nn
import gym

'''
DESCRIPTION: TODO...
'''
class CustomCombinedExtractor(BaseFeaturesExtractor):
    def __init__(self, observation_space: gym.spaces.Dict):
        
        # We do not know features-dim here before going over all the items,
        # so put something dummy for now. PyTorch requires calling
        # nn.Module.__init__ before adding modules
        super(CustomCombinedExtractor, self).__init__(observation_space, features_dim=1)

        extractors_policy = {}

        total_concat_size = 0
        # We need to know size of the output of this extractor,
        # so go over all the spaces and compute output feature sizes
        for key, subspace in observation_space.spaces.items():
            # if key == "image_depth":
            #     extractors_policy[key] = nn.Sequential(nn.Conv2d(1, 32, kernel_size=8, stride=4, padding=0),
            #                                            nn.ReLU(),
            #                                            nn.Conv2d(32, 64, kernel_size=4, stride=2, padding=0),
            #                                            nn.ReLU(),
            #                                            nn.Conv2d(64, 64, kernel_size=3, stride=1, padding=0),
            #                                            nn.ReLU(),
            #                                            nn.Flatten(),
            #                                            nn.Linear(9216, 256),
            #                                            nn.ReLU(),
            #                                            nn.Linear(256, 64),
            #                                            nn.ReLU(),
            #                                            nn.Linear(64, 4),)
            #     total_concat_size += 4

            # if key == "h_raleted":
            #     extractors_policy[key] = nn.Sequential(nn.Flatten(),
            #                                            nn.Linear(10, 256),
            #                                            nn.ReLU(),
            #                                            nn.Linear(256, 256),
            #                                            nn.ReLU(),
            #                                            nn.Linear(256, 4),
            #                                            nn.ReLU(),)
            #     total_concat_size += 4

            if key == "IR_raleted":
                extractors_policy[key] = nn.Sequential(nn.Flatten(),)
                                                    #    nn.Linear(24, 256),
                                                    #    nn.ReLU(),
                                                    #    nn.Linear(256, 128),
                                                    #    nn.ReLU(),
                                                    #    nn.Linear(128, 64),
                                                    #    nn.ReLU(),
                                                    #    nn.Linear(64, 32),
                                                    #    nn.ReLU(),
                                                    #    nn.Linear(32, 4),
                                                    #    nn.ReLU(),)
                total_concat_size += 12
            
            if key == "pos_raleted":
                extractors_policy[key] = nn.Sequential(nn.Flatten(),)
                #                                        nn.Linear(2, 256),
                #                                        nn.ReLU(),
                #                                        nn.Linear(256, 64),
                #                                        nn.ReLU(),
                #                                        nn.Linear(64, 1),
                #                                        nn.ReLU(),)
                total_concat_size += 2

            if key == "vel_raleted":
                extractors_policy[key] = nn.Sequential(nn.Flatten(),)
                                                    #    nn.Linear(2, 256),
                                                    #    nn.ReLU(),
                                                    #    nn.Linear(256, 64),
                                                    #    nn.ReLU(),
                                                    #    nn.Linear(64, 1),
                                                    #    nn.ReLU(),)
                total_concat_size += 2

            # if key == "mapita":
            #     extractors_policy[key] = nn.Sequential(nn.Conv2d(1, 8, kernel_size=2, stride=1, padding=0),
            #                                            nn.ReLU(),
            #                                            nn.Conv2d(8, 1, kernel_size=2, stride=1, padding=0),
            #                                            nn.ReLU(),
            #                                            nn.Flatten(),
            #                                            nn.Linear(9, 64),
            #                                            nn.ReLU(),
            #                                            nn.Linear(64, 16),
            #                                            nn.ReLU(),
            #                                            nn.Linear(16, 4),)
            #     total_concat_size += 4

        self.extractors_policy = nn.ModuleDict(extractors_policy)

        # Update the features dim manually
        self._features_dim = total_concat_size

    def forward(self, observations) -> th.Tensor:
        encoded_tensor_list_policy = []

        # self.extractors contain nn.Modules that do all the processing.
        # policy
        for key, extractor in self.extractors_policy.items():
            encoded_tensor_list_policy.append(extractor(observations[key]))
        
        # Return a (B, self._features_dim) PyTorch tensor, where B is batch dimension.
        self.policy_net = th.cat(encoded_tensor_list_policy, dim=1)
        
        return self.policy_net

'''
DESCRIPTION: TODO...
'''
if __name__ == '__main__':

    ## ROS NODE
    #rospy.init_node('mobiman_drl_train', anonymous=True, log_level=rospy.WARN)
    
    ### NUA TODO: Read from a .csv file
    ## USER DEFINED PARAMETERS
    #log_path_rel = rospy.get_param('log_path', "")
    log_path_rel = "dataset/drl_training_data/"
    save_freq = 5000
    #max_episode_length = rospy.get_param('max_episode_length', "")
    max_episode_length = 1000
    total_timesteps = 3000
    name_prefix = "mobiman_policy_checkpoint"
    policy = "MultiInputPolicy"
    
    #robot_name = "jetbot"
    robot_name = "jackal_jaco"

    ## LOCAL PARAMETERS
    #rospack = rospkg.RosPack()
    #mobiman_path = rospack.get_path('mobiman_simulation') + "/"
    mobiman_path = "/home/akmandor/ros_workspaces/mobiman_ws/src/mobiman/mobiman_simulation/"
    log_path = mobiman_path + log_path_rel

    ## PRINT INFO
    print("[mobiman_drl_train::__main__] log_path_rel: " + log_path_rel)
    print("[mobiman_drl_train::__main__] log_path: " + log_path)
    print("[mobiman_drl_train::__main__] save_freq: " + str(save_freq))
    print("[mobiman_drl_train::__main__] max_episode_length: " + str(max_episode_length))
    print("[mobiman_drl_train::__main__] total_timesteps: " + str(total_timesteps))
    print("[mobiman_drl_train::__main__] name_prefix: " + name_prefix)
    print("[mobiman_drl_train::__main__] policy: " + policy)
    print("[mobiman_drl_train::__main__] robot_name: " + robot_name)

    # set headles to false to visualize training
    print("[mobiman_drl_train::__main__] BEFORE Isaac_envs")
    my_env = Isaac_envs(headless=False, 
                        max_episode_length=max_episode_length, 
                        robot_name=robot_name)
    print("[mobiman_drl_train::__main__] AFTER Isaac_envs")

    print("[mobiman_drl_train::__main__] BEFORE policy_kwargs")
    policy_kwargs = dict(features_extractor_class=CustomCombinedExtractor, 
                        activation_fn=th.nn.ReLU, 
                        net_arch=[512, 512, 512],)
    print("[mobiman_drl_train::__main__] AFTER policy_kwargs")
    
    print("[mobiman_drl_train::__main__] BEFORE CheckpointCallback")
    checkpoint_callback = CheckpointCallback(save_freq=10000, save_path=log_path, name_prefix=name_prefix)
    print("[mobiman_drl_train::__main__] AFTER CheckpointCallback")

    '''
    model = PPO(policy,
                my_env,
                policy_kwargs=policy_kwargs,
                verbose=1,
                n_steps=1000,
                batch_size=1000,
                learning_rate=0.00015,
                gamma=0.9995,
                device="cuda",
                ent_coef=0,
                vf_coef=0.5,
                max_grad_norm=10,
                clip_range=1,
                tensorboard_log=log_dir,)
    '''
  
    print("[mobiman_drl_train::__main__] BEFORE model")
    model = DQN(policy,
                my_env,
                policy_kwargs=policy_kwargs,
                verbose=1,
                buffer_size=800000,
                learning_starts=1000, 
                learning_rate=0.0003, #0.00015
                exploration_fraction=0.35,
                device="cuda",
                tensorboard_log=log_path,)
    print("[mobiman_drl_train::__main__] AFTER model")

    print("[mobiman_drl_train::__main__] START learn")
    model.learn(total_timesteps=total_timesteps, callback=[checkpoint_callback])
    print("[mobiman_drl_train::__main__] END learn")

    model.save(log_path + "/mobiman_policy")

    #print("[mobiman_drl_train::__main__] DEBUG INF")
    #while 1:
    #    continue

    my_env.close()
