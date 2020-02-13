<<<<<<< HEAD
"""
Testing and visualizing trained model on the mujoco environment
"""
#export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libGLEW.so

import os
#import mujoco_py
import gym
import numpy as np
import matplotlib.pyplot as plt
from arm_mjpy import arm_env_mj

from stable_baselines import SAC

from stable_baselines.sac.policies import MlpPolicy
from stable_baselines.common.vec_env import DummyVecEnv
from stable_baselines.bench import Monitor
from stable_baselines.results_plotter import load_results, ts2xy
from stable_baselines.ppo2 import PPO2

#8, 10, 13, 16, 17

#****** BEST ******

# model = SAC.load("logs_tune/exp14/14/model.pkl")

#******************


model = SAC.load("logs_tune/exp15/4/model.pkl")
# model = SAC.load("logs_tune/exp14/14/best_model.pk1")
#model = PPO2.load("logs/ppo_log5best_model.pk1")
#model = SAC.load("ARM_SAC2")

envmj = arm_env_mj()
env = DummyVecEnv([lambda:envmj for i in range(1)])
_ = env.reset()

obs = envmj.reset()


#Recurrent
state = None
done = [False for i in range(1)]

envmj.set_viewer()
while True:
    action, state = model.predict(obs)
    obs, rewards, done, info = envmj.step(action)
    envmj.viewer.render()
    if done:
        envmj.print_pos_rads()
        obs = envmj.reset()
=======
"""
Testing and visualizing trained model on the mujoco environment
"""
#export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libGLEW.so

import os
#import mujoco_py
import gym
import numpy as np
import matplotlib.pyplot as plt
from arm_mjpy import arm_env_mj

from stable_baselines import SAC, DDPG

from stable_baselines.sac.policies import MlpPolicy
from stable_baselines.common.vec_env import DummyVecEnv
from stable_baselines.bench import Monitor
from stable_baselines.results_plotter import load_results, ts2xy
from stable_baselines.ppo2 import PPO2

#8, 10, 13, 16, 17

#****** BEST ******

# model = SAC.load("logs_tune/exp14/14/model.pkl")

#******************


model = SAC.load("logs_tune/exp15/4/model.pkl")
# model = SAC.load("logs_tune/exp14/14/best_model.pk1")
#model = PPO2.load("logs/ppo_log5best_model.pk1")
#model = SAC.load("ARM_SAC2")

envmj = arm_env_mj()
env = DummyVecEnv([lambda:envmj for i in range(1)])
_ = env.reset()

obs = envmj.reset()


#Recurrent
state = None
done = [False for i in range(1)]

envmj.set_viewer()
while True:
    action, state = model.predict(obs)
    obs, rewards, done, info = envmj.step(action)
    envmj.viewer.render()
    if done:
        envmj.print_pos_rads()
        obs = envmj.reset()
>>>>>>> 8b4769fefa4495bd747e3aca54fa1e1841844409
