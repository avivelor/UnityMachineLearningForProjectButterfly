"""
Stable baselines implementation of SAC applied to the ARM
"""

import os
#import mujoco_py
import gym
import tensorflow as tf
import numpy as np
import matplotlib.pyplot as plt
from arm_mjpy import arm_env_mj

from stable_baselines import SAC, DDPG

from stable_baselines.sac.policies import MlpPolicy, LnMlpPolicy
from stable_baselines.common.vec_env import DummyVecEnv
from stable_baselines.bench import Monitor
from stable_baselines.results_plotter import load_results, ts2xy
from stable_baselines.ppo2 import PPO2

import ray
from ray import tune
from ray.tune import Trainable, run
from ray.tune.schedulers import PopulationBasedTraining

import threading
import multiprocessing
from multiprocessing import Pool, Process, TimeoutError
import time
import os
from itertools import product, repeat



###### Monitoring Training ######
best_mean_reward, n_steps = -np.inf, 0

#Define a Callback function
def callback(_local, _globals):
    """
    Callback called at each step (DQN and others) or after n steps
    (ACER or PPO2)
    :param _locals: (dict)
    :param_globals: (dict)
    """

    global n_steps, best_mean_reward
    log_dir = _local['self'].log_name
    #Print stats every 1000 calls
    if(n_steps+1)%500==0:
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


print('Starting Learning')
# model.learn(total_timesteps=200000, callback=callback)
# model.save('logs/log13/ARM_SAC')

def f(lr,num_trials, policy_kwargs,dir='logs_tune/exptest'):
    """Takes in dict of params, trains network and finds optimal hyperparameters
    """
    #global lr
    # lr = .01
    def get_learning_rate(num):
        #print(.001*num)
        return .001*num


    log_dir = dir + str(num_trials)
    print("Starting ",log_dir)


    os.makedirs(log_dir, exist_ok=True)

    steps = 1000000


    envmj = []
    # envmj = [arm_env_mj() for i in range(n_cpu - 1)]

    env_m = arm_env_mj()
    temp = Monitor(env_m, log_dir, allow_early_resets = True)
    print('Made env and monitor')
    #envmj.append(temp)
    # env = SubprocVecEnv([lambda: i for i in envmj])
    #env = DummyVecEnv([lambda: i for i in envmj])(log_dir + "/model
    env = DummyVecEnv([lambda: temp])
    print('Made Dummy')
    #gen = lambda i: i for i in np.linspace(.001,.000001,steps)
    model = SAC(LnMlpPolicy, env, verbose=0, learning_rate=get_learning_rate, policy_kwargs=policy_kwargs, batch_size = 128)

    model.log_name = log_dir + '/'

    f= open(log_dir + '/notes.txt',"w+")
    f.write('num_trials={}\n'.format(num_trials))
    f.write('lr={}\n'.format(lr))
    f.write('arch={}\n'.format(policy_kwargs))

    # model.log_num = log_num

    print('Starting Learning')
    model.learn(total_timesteps=steps, callback=callback)

    # model.learn(total_timesteps=10000000, callback = callback)
    model.save(log_dir + "/model")
    return 'DONE'


if __name__ == '__main__':
    lr = .001
    num_trials = 6
    # make args for parallel process tasks (0th task gets arg1[0], arg2[0])

    #arg3...

    # os.makedirs('logs_tune', exist_ok=True)
    log_dir = 'logs_tune/exp15'
    os.makedirs(log_dir, exist_ok=True)
    log_dir += '/'


    #Args for
    lr = [lr]*num_trials
    num_trials = len(lr)
    num = np.arange(num_trials)
    log_arr = [log_dir for i in range(num_trials)]
    policy_arr = [
                  dict(act_fun=tf.nn.sigmoid, layers=[128,64]),
                  dict(act_fun=tf.nn.sigmoid, layers=[128,64]),
                  dict(act_fun=tf.nn.sigmoid, layers=[128,64]),
                  dict(act_fun=tf.nn.sigmoid, layers=[128,64]),
                  dict(act_fun=tf.nn.sigmoid, layers=[128,64]),
                  dict(act_fun=tf.nn.sigmoid, layers=[128,64]),
                  ]



    # make as many workers as cpu cores on the machine
    workers = multiprocessing.cpu_count()
    #workers = 1
    #workers = 1

    #for i in num:
        #f(lr,i,dir=log_dir)


    with multiprocessing.Pool(processes=workers) as pool:
         #give the workers a batch of tasks
        # this blocks the program until tasks finished
        results = pool.starmap(f, zip(lr, num, policy_arr,log_arr))

#        print(results)
###### Gym environment #####

#env = gym.make('Acrobot-v1')
# env = Monitor(env, log_dir, allow_early_resets = True)
#env = DummyVecEnv([lambda: env])

#model = PPO2(MlpPolicy, env, verbose = 1)
#model.learn(total_timesteps=20000, callback=callback)
#env.render()


########## Plotting helpers ##########


# plot_results(log_dir)
print('Done')
#print('Presenting Results!')

#envmj.set_viewer()
#while True:#
#    action, _states = model.predict(obs)
#    obs, rewards, dones, info = env.step(action)
#    envmj.viewer.render()
