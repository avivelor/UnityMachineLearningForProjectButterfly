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

from stable_baselines import SAC

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
    plt.title(title + " Smoothed")
    plt.show()

def load_data(log_folder):
    """
    Loads result
    :param log_folder: (str) save location of the training results
    """
    x, y = ts2xy(load_results(log_folder), 'timesteps')
    # y = movingAverage(y,window=50)
    #Truncate x
    x = x[len(x)-len(y):]

    return (x,y)

def load_all_data(log_folder,count):
    """
    Loads number of results from a log directory
    :param log_folder: (str) save location of the training results
    :param count: (int) number of logs to load in format of log_folder + num
    """
    X = []
    Y = []
    for i in range(count):
        try:
            x,y = load_data(log_dir + str(i))
            X.append(x)
            Y.append(y)
        except:
            continue
    X = np.array(X)
    Y = np.array(Y)
    return (X,Y)

def plot_all_results(log_folder,count):
    """
    Loads number of results from a log directory, plotting mean and std
    :param log_folder: (str) save location of the training results
    :param count: (int) number of logs to load in format of log_folder + num
    """
    X,Y = load_all_data(log_folder,count)
    ymean = Y.mean(axis=0)
    ystd = Y.std(axis=0)
    fig, ax = plt.subplots(2, 1)
    ax[0].plot(X.mean(axis=0),ymean)
    ax[0].fill_between(X[0], ymean+ystd, ymean-ystd, facecolor='green',alpha = .3, interpolate=True)
    ax[1].plot(X.T,Y.T)

    plt.show()

def plot_one_by_one(log_folder,count):
    """
    Plots results of each folder individually
    :param log_folder: (str) save location of the training results
    :param count: (int) number of logs to load in format of log_folder + num
    """

    for i in range(30):
        plot_results(log_folder + str(i), title='Learning Curve {}'.format(i))


log_dir = "logs_tune/exp15/"
# plot_results(log_dir)
plot_all_results(log_dir,8)
plot_one_by_one(log_dir,30)
# x,y = load_all_data(log_dir,30)
=======
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
    plt.title(title + " Smoothed")
    plt.show()

def load_data(log_folder):
    """
    Loads result
    :param log_folder: (str) save location of the training results
    """
    x, y = ts2xy(load_results(log_folder), 'timesteps')
    # y = movingAverage(y,window=50)
    #Truncate x
    x = x[len(x)-len(y):]

    return (x,y)

def load_all_data(log_folder,count):
    """
    Loads number of results from a log directory
    :param log_folder: (str) save location of the training results
    :param count: (int) number of logs to load in format of log_folder + num
    """
    X = []
    Y = []
    for i in range(count):
        try:
            x,y = load_data(log_dir + str(i))
            X.append(x)
            Y.append(y)
        except:
            continue
    X = np.array(X)
    Y = np.array(Y)
    return (X,Y)

def plot_all_results(log_folder,count):
    """
    Loads number of results from a log directory, plotting mean and std
    :param log_folder: (str) save location of the training results
    :param count: (int) number of logs to load in format of log_folder + num
    """
    X,Y = load_all_data(log_folder,count)
    ymean = Y.mean(axis=0)
    ystd = Y.std(axis=0)
    fig, ax = plt.subplots(2, 1)
    ax[0].plot(X.mean(axis=0),ymean)
    ax[0].fill_between(X[0], ymean+ystd, ymean-ystd, facecolor='green',alpha = .3, interpolate=True)
    ax[1].plot(X.T,Y.T)

    plt.show()

def plot_one_by_one(log_folder,count):
    """
    Plots results of each folder individually
    :param log_folder: (str) save location of the training results
    :param count: (int) number of logs to load in format of log_folder + num
    """

    for i in range(30):
        plot_results(log_folder + str(i), title='Learning Curve {}'.format(i))


log_dir = "logs_tune/exp15/"
# plot_results(log_dir)
plot_all_results(log_dir,8)
plot_one_by_one(log_dir,30)
# x,y = load_all_data(log_dir,30)
>>>>>>> 8b4769fefa4495bd747e3aca54fa1e1841844409
