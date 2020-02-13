"""
Stable baselines implementation of SAC applied to the ARM
"""

import os
#import mujoco_py
import gym
import numpy as np
import matplotlib.pyplot as plt
from arm_mjpy import arm_env_mj

from stable_baselines import SAC, DDPG

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
envmj = arm_env_mj(xml_file='arm_params.xml')
# env = env.make()
envmj.reset()






print("Beginning setup...")
#env = arm_env.ArmEnv()
print('Making')
#env.make()
print('Finished make')
# state = env.reset()
#state, reward, done, info    = env.step([0,0])

fold = 'mj_data'
filename = fold + '/drop_testv2_8.csv'

if not os.path.isdir(fold):
    os.mkdir(fold)

with open(filename,'a', newline = '') as f:
      writer = csv.writer(f)
      writer.writerow(['z,y,force'])


action = 0

obs = envmj.reset()
envmj.set_viewer()
i = 0

# Init state for drop\

# qpos = np.array([1.2,0])
qpos = np.array([.40,0])

print(qpos)
qvel = np.array([0,0])
envmj.set_state(qpos,qvel)

while i<1000:
    i += 1

    #state, reward, done, info    = env.step([0,0])

    obs, rewards, dones, info = envmj.step([0,0])
    #print
    with open(filename,'a', newline = '') as f:
        writer = csv.writer(f)
        writer.writerow([obs[0]*180/np.pi, obs[2]*180/np.pi , 0])
    envmj.viewer.render()


###here
#z2,z1,force
#0.0,0.0,0.0




4
state_size = env.observation_space.shape[0]
print('Observation Shape: ',state_size)

action_size = env.action_space
print('Action Shape: ', action_size)

print('\n\nBegin Data Gathering:\n')

t = time.time()


strength = 1
reward_total = 0

while(True):

        #Do an environment step


    t = time.time()

    action1 = 0
    action2 = 0
    try:

        #Joint 1
        if keyboard.is_pressed('w'):
            action1 = 1*strength
        if keyboard.is_pressed('s'):
            action1 = -1*strength
        if keyboard.is_pressed('w') and keyboard.is_pressed('s'):
            action1 = 0

        #Joint 2
        if keyboard.is_pressed('d'):
            action2 = 1*strength
        if keyboard.is_pressed('a'):
            action2 = -1*strength
        if keyboard.is_pressed('a') and keyboard.is_pressed('d'):
            action2 = 0

        if keyboard.is_pressed('z'):
            print('Strength ={}%'.format(100*strength))
            strength += .1

        if keyboard.is_pressed('x'):
            strength -= .1

            print('Strength ={}%'.format(100*strength))

        if strength >= 1:
                strength = 1
        if strength <= 0:
                strength = 0

    except:
        pass

    action = np.array([action1,action2])

    next_state, reward, done, info = env.step(action)
    reward_total += reward
    # env.saveData('data/pid_follow_4.csv')



    if env.steps %25 == 0:
            print('**********************************')
            print('Actions =\t',action)
            print('Pitch Pos =\t',state[0])
            print('Pitch Goal =\t',state[1])
            print('Roll Pos =\t',state[2])
            print('Roll Goal =\t',state[3])
            print('Reward Total =\t',reward_total)

    # print('State', state[0])


    # with open(filename,'a', newline = '') as File:
        # writer = csv.writer(File)

        # writer.writerow([state[0]] + [action] + [next_state[0]])

    if done:
        state = env.reset()
        reward_total = 0

    state = next_state
