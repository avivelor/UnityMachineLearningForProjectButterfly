"""
Stable baselines implementation of SAC applied to the ARM
"""

import os
#import mujoco_py
import gym
import ray
import numpy as np
import matplotlib.pyplot as plt

from arm_mjpy import arm_env_mj

from ray import tune
from ray.tune import run, sample_from
from ray.tune.schedulers import PopulationBasedTraining



from stable_baselines import SAC, DDPG
from stable_baselines.common.policies import MlpPolicy,MlpLnLstmPolicy
from stable_baselines.common.vec_env import SubprocVecEnv, DummyVecEnv
from stable_baselines.bench import Monitor
from stable_baselines.results_plotter import load_results, ts2xy
from stable_baselines.ppo2 import PPO2


#Multi
import threading
import multiprocessing
from multiprocessing import Pool, Process, TimeoutError
import time
import os
from itertools import product, repeat

###### Monitoring Training ######
best_mean_reward, n_steps = -np.inf, 0

n_cpu = 1

#Define a Callback function
def callback(_locals, _globals):
    """
    Callback called at each step (DQN and others) or after n steps
    (ACER or PPO2)
    :param _locals: (dict)
    :param_globals: (dict)
    """

    global n_steps, best_mean_reward, log_dir
    # print(n_steps)
    #Print stats every 1000 calls
    log_dir_t = log_dir + str(_locals['self'].log_num)

    if(n_steps+1)%5==0:
        #Evaluate policy performance
        x,y = ts2xy(load_results(log_dir_t), 'timesteps')

        if len(x) > 0:
            mean_reward = np.nanmean(y[-100:])
            print('Last 5 rewards',y[-5:])
            print(x[-1], 'timesteps')
            print("Best mean reward: {:.4f} - Last mean reward per episode:{:.4f}".format(best_mean_reward, mean_reward))

            #Save the new best model compared to the old model
            if mean_reward > best_mean_reward:
                best_mean_reward = mean_reward
                print("Saving new best model")
                _locals['self'].save(log_dir_t + '/best_model.pk1')
    n_steps +=1
    return True

log_dir = "logs/log_lr2_"
# log_num = 1
def train_ppo2(log_num, learning_rate):
    global log_dir
    log_dir_t = log_dir + str(log_num)
    os.makedirs(log_dir_t, exist_ok=True)



    envmj = []
    # envmj = [arm_env_mj() for i in range(n_cpu - 1)]

    env_m = arm_env_mj()
    temp = Monitor(env_m, log_dir_t, allow_early_resets = True)
    envmj.append(temp)
    # env = SubprocVecEnv([lambda: i for i in envmj])
    env = DummyVecEnv([lambda: i for i in envmj])

    model = PPO2(MlpLnLstmPolicy, env, verbose=0,nminibatches=1)

    model.log_num = log_num

    print('Starting Learning')
    model.learn(total_timesteps=10000000, callback = callback)
    model.save(log_dir + "/ppo2")


#Create log dir


#Do multiprocessing
if __name__ == '__main__':
    def explore(config):
        # ensure we collect enough timesteps to do sgd
        if config["train_batch_size"] < config["sgd_minibatch_size"] * 2:
            config["train_batch_size"] = config["sgd_minibatch_size"] * 2
        # ensure we run at least one sgd iter
        if config["num_sgd_iter"] < 1:
            config["num_sgd_iter"] = 1
        return config


    pbt = PopulationBasedTraining(
        time_attr="time_total_s",
        metric="episode_reward_mean",
        mode="max",
        perturbation_interval=120,
        resample_probability=0.25,
        # Specifies the mutations of these hyperparams
        hyperparam_mutations={
            "lambda": lambda: random.uniform(0.9, 1.0),
            "clip_param": lambda: random.uniform(0.01, 0.5),
            "lr": [1e-3, 5e-4, 1e-4, 5e-5, 1e-5],
            "num_sgd_iter": lambda: random.randint(1, 30),
            "sgd_minibatch_size": lambda: random.randint(128, 16384),
            "train_batch_size": lambda: random.randint(2000, 160000),
        },
        custom_explore_fn=explore)

    ray.init()


    run(
        "PPO",
        name="pbt_humanoid_test",
        scheduler=pbt,
        **{
            "env": "Humanoid-v1",
            "num_samples": 8,
            "config": {
                "kl_coeff": 1.0,
                "num_workers": 8,
                "num_gpus": 1,
                "model": {
                    "free_log_std": True
                },
                # These params are tuned from a fixed starting value.
                "lambda": 0.95,
                "clip_param": 0.2,
                "lr": 1e-4,
                # These params start off randomly drawn from a set.
                "num_sgd_iter": sample_from(
                    lambda spec: random.choice([10, 20, 30])),
                "sgd_minibatch_size": sample_from(
                    lambda spec: random.choice([128, 512, 2048])),
                "train_batch_size": sample_from(
                    lambda spec: random.choice([10000, 20000, 40000]))
            },
        })



    # make args for parallel process tasks (0th task gets arg1[0], arg2[0])

    learning_rate = [.01,.008,.005,.002,.001,.0005,.0001]
    log_num = range(len(learning_rate))
    # arg2 = range(10)
    #arg3...


    # make as many workers as cpu cores on the machine
    workers = multiprocessing.cpu_count()

    with multiprocessing.Pool(processes=workers) as pool:
        # give the workers a batch of tasks
        # this blocks the program until tasks finished
        results = pool.starmap(train_ppo2, zip(log_num, learning_rate))

    print(results)



###### ARM environment ###############


# env = env.make()
# envmj.reset()



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
