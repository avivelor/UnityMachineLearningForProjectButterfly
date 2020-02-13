"""
Stable baselines implementation of SAC applied to the ARM
"""

import os
#import mujoco_py
import gym
import numpy as np
import matplotlib.pyplot as plt
from arm_mjpy import arm_env_mj

from stable_baselines import SAC

import sys
import gym
import random
import csv
import time
import numpy as np
import keyboard

from numpy import random
from Arm_Env import arm_env

###### ARM environment ###############
envmj = arm_env_mj()
# env = env.make()
envmj.reset()






print("Beginning setup...")
env = arm_env.ArmEnv()
print('Making')
env.make()
print('Finished make')
# state = env.reset()
state, reward, done, info    = env.step([0,0])

action = [0,0]

obs = envmj.reset()
envmj.set_viewer()
i = 0
while True:
    i += 1

    state, reward, done, info    = env.step([0,0])

    obs, rewards, dones, info = envmj.step([0,0])
    qpos = np.array([state[2],state[0]])
    # qpos = np.array([state[3],state[1]])

    print(qpos)
    # qpos = np.array([0,0])
    qvel = np.array([0,0])
    envmj.set_state(qpos,qvel)
    envmj.viewer.render()


###here
