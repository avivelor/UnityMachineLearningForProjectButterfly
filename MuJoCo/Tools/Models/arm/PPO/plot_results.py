"""
Plotting data from training sessions of stable_baselines
Mainly those that used Monitor
"""

import os
#import mujoco_py
import gym
import numpy as np
import matplotlib.pyplot as plt
# from arm_mjpy import arm_env_mj

from stable_baselines import SAC, DDPG

from stable_baselines.sac.policies import MlpPolicy
from stable_baselines.common.vec_env import DummyVecEnv
from stable_baselines.bench import Monitor
from stable_baselines.results_plotter import load_results, ts2xy
from stable_baselines.ppo2 import PPO2



def movingAverage(values,window):
    """
    Smooth values by doing a moving average
    :param values: (numpy array)
    :param window: (int)
    :return: (numpy array)
    """
    weights = np.repeat(1.0, window) / window
    return np.convolve(values, weights, 'valid')

def plot_results(log_folder, title='Learning Curve'):
    """
    Plot the results
    :param log_folder: (str) the save location of the results to plot
    :param title: (str) the title of the task to plot
    """
    x, y = ts2xy(load_results(log_folder), 'timesteps')
    y = movingAverage(y,window=50)
    #Truncate x
    x = x[len(x)-len(y):]

    fig = plt.figure(title)
    plt.plot(x,y)
    plt.xlabel('Number of Timesteps')
    plt.ylabel('Rewards')
    plt.title(title + " Smoothed" + log_folder)
    plt.show()

# plot_results('logs/log_lr_')
for i in range(7):

    log_dir = "logs/log_lr3" + str(i)

    plot_results(log_dir)
