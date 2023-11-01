#### LAST UPDATE: 2023.06.09
##
#### AUTHOR: 
## Neset Unver Akmandor (NUA)
##
#### E-MAIL: 
## akmandor.n@northeastern.edu
##
#### REFERENCE: 
##  https://github.com/Lauqz/Easy_DRL_Isaac_Sim             
##  NVIDIA
##
#### DESCRIPTION: TODO...

from env_procedural_v4_d import JetBotEnv
from stable_baselines3 import DQN

policy_path = "./mlp_policy/DQN_24/jetbot_policy.zip"

my_env = JetBotEnv(headless=False)
model = DQN.load(policy_path)

for _ in range(30):
    obs = my_env.reset()
    done = False
    while not done:
        actions, _ = model.predict(observation=obs, deterministic=True)
        obs, reward, done, info = my_env.step(actions)
        my_env.render()

my_env.close()
