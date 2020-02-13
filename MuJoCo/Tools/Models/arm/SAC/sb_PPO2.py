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
from stable_baselines.common.policies import MlpPolicy,MlpLnLstmPolicy
from stable_baselines.common.vec_env import SubprocVecEnv
from stable_baselines.bench import Monitor
from stable_baselines.results_plotter import load_results, ts2xy
from stable_baselines.ppo2 import PPO2

###### Monitoring Training ######
best_mean_reward, n_steps = -np.inf, 0

n_cpu = 8

#Define a Callback function
def callback(_local, _globals):
    """
    Callback called at each step (DQN and others) or after n steps
    (ACER or PPO2)
    :param _locals: (dict)
    :param_globals: (dict)
    """

    global n_steps, best_mean_reward
    # print(n_steps)
    #Print stats every 1000 calls
    if(n_steps+1)%5==0:
        #Evaluate policy performance
        x,y = ts2xy(load_results(log_dir), 'timesteps')

        if len(x) > 0:
            mean_reward = np.nanmean(y[-100:])
            print('Last 5 rewards',y[-5:])
            print(x[-1], 'timesteps')
            print("Best mean reward: {:.4f} - Last mean reward per episode:{:.4f}".format(best_mean_reward, mean_reward))

            #Save the new best model compared to the old model
            if mean_reward > best_mean_reward:
                best_mean_reward = mean_reward
                print("Saving new best model")
                _local['self'].save(log_dir + 'best_model.pk1')
    n_steps +=1
    return True

#Create log dir
log_dir = "logs/ppo_log2"
os.makedirs(log_dir, exist_ok=True)

###### ARM environment ###############
envmj = [arm_env_mj() for i in range(n_cpu - 1)]
env_m = arm_env_mj()
# env = env.make()
# envmj.reset()
temp = Monitor(env_m, log_dir, allow_early_resets = True)
envmj.append(temp)
env = SubprocVecEnv([lambda: i for i in envmj])

model = PPO2(MlpLnLstmPolicy, env, verbose=1)
print('Starting Learning')
model.learn(total_timesteps=5000000, callback = callback)
model.save("ppo2_1")
# model = HER('MlpPolicy', env, SAC, n_sampled_goal=4,
#             goal_selection_strategy='future',
#             verbose=1, buffer_size=int(1e6),
#             learning_rate=1e-3,
#             gamma=0.95, batch_size=256,
#             policy_kwargs=dict(layers=[256, 256, 256]))


# model.save('ARM_SAC2')

###### Gym environment #####

#env = gym.make('Acrobot-v1')
# env = Monitor(env, log_dir, allow_early_resets = True)
#env = DummyVecEnv([lambda: env])

#model = PPO2(MlpPolicy, env, verbose = 1)
#model.learn(total_timesteps=20000, callback=callback)
#env.render()


########## Plotting helpers ##########

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
    plt.title(title + "Smoothed")
    plt.show()

# plot_results(log_dir)
print('Done')
# print('Presenting Results!')
#
# obs = envmj.reset()
# envmj.set_viewer()
# while True:
#     action, _states = model.predict(obs)
#     obs, rewards, dones, info = env.step(action)
#     envmj.viewer.render()
