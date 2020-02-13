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

# model = SAC.load("logs/log12/ARM_SAC12.pkl")

# model = PPO2.load("ppo2_1.pkl")
model = PPO2.load("logs/log_lr32/best_model.pk1")
#model = SAC.load("ARM_SAC2")

envmj = arm_env_mj()
env = DummyVecEnv([lambda:envmj for i in range(1)])
_ = env.reset()

obs = env.reset()


#Recurrent
state = None
done = [False for i in range(1)]

envmj.set_viewer()
while True:
    action, state = model.predict(obs)
    obs, rewards, done, info = env.step(action)
    envmj.viewer.render()
