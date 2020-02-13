import mujoco_py
import gym
import numpy as np
import os

from gym import utils
from gym.envs.mujoco import mujoco_env
from mujoco_py import MjViewer
from gym import error, spaces
from os import path

#In case its not visually rendering
#export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libGLEW.so


def convert_observation_to_space(observation):
    if isinstance(observation, dict):
        space = spaces.Dict(OrderedDict([
            (key, convert_observation_to_space(value))
            for key, value in observation.items()
        ]))
    elif isinstance(observation, np.ndarray):
        low = np.full(observation.shape, -float('inf'))
        high = np.full(observation.shape, float('inf'))
        space = spaces.Box(low, high, dtype=observation.dtype)
    else:
        raise NotImplementedError(type(observation), observation)

    return space




class arm_env_mj(mujoco_env.MujocoEnv, utils.EzPickle):
    def __init__(self,
                 xml_file='../SAC/arm.xml',
                 forward_reward_weight=1.0,
                 ctrl_cost_weight=0.1,
                 reset_noise_scale=0.1,
                 exclude_current_positions_from_observation=True,
                 rgb_rendering_tracking=True):
        utils.EzPickle.__init__(**locals())

        self._forward_reward_weight = forward_reward_weight

        self._ctrl_cost_weight = ctrl_cost_weight

        self._reset_noise_scale = reset_noise_scale

        self._exclude_current_positions_from_observation = (
            exclude_current_positions_from_observation)

        fullpath = xml_file
        if not path.exists(fullpath):
            raise IOError("File %s does not exist" % fullpath)

        ###### Variables ###########################
        self.steps = 0
        self.max_steps = 1000
        self.y_lower= 0
        self.y_upper = 1
        self.z_lower = -.5
        self.z_upper = .8


        self.set_goals()



        self.frame_skip = 1
        self.model = mujoco_py.load_model_from_path(fullpath)
        self.sim = mujoco_py.MjSim(self.model)
        self.data = self.sim.data
        self.viewer = None
        # self.viewer =  MjViewer(self.sim)
        self.rgb_rendering_tracking = rgb_rendering_tracking
        self._viewers = {}

        self.metadata = {
            'render.modes': ['human', 'rgb_array', 'depth_array'],
            'video.frames_per_second': int(np.round(1.0 / self.dt))
        }

        self.init_qpos = self.sim.data.qpos.ravel().copy()
        self.init_qvel = self.sim.data.qvel.ravel().copy()

        self._set_action_space()

        action = self.action_space.sample()
        observation, _reward, done, _info = self.step(action)
        assert not done

        self._set_observation_space(observation)

        self.seed()





    def _set_action_space(self):
        bounds = self.model.actuator_ctrlrange.copy()
        low, high = bounds.T
        self.action_space = spaces.Box(low=low, high=high, dtype=np.float32)
        print('Action space:',self.action_space)
        return self.action_space

    def _set_observation_space(self, observation):
        self.observation_space = convert_observation_to_space(observation)
        print('Observation space:', self.observation_space)
        return self.observation_space

    def set_viewer(self):
        self.viewer =  MjViewer(self.sim)



    def control_cost(self, action):
        control_cost = self._ctrl_cost_weight * np.sum(np.square(action))
        return control_cost

    def step(self, action):
        self.steps += 1
        pos_before =  self.sim.data.qpos
        self.do_simulation(action, self.frame_skip)
        pos_after = self.sim.data.qpos
        self.velocity = ((pos_before - pos_after)
                      / self.dt)
        # print(pos_before,pos_after,self.velocity)

        ctrl_cost = self.control_cost(action)

        forward_reward = self._forward_reward_weight

        observation = self._get_obs()

        reward = 0

        # reward = forward_reward - ctrl_cost
        # reward_y = abs(observation[2] - observation[3])
        # reward_z = abs(observation[0] - observation[1])
        reward_y = abs(observation[3])
        reward_z = abs(observation[1])

        reward += -float(reward_y + reward_z)

        t_reward = reward
        # print('Reward',reward)
        vel =  self.sim.data.qvel.flat.copy()
        reward -= (abs(vel[0]) + abs(vel[1]))/2
        # print('Vel Reward',(abs(vel[0]) + abs(vel[1]))/10)
        #if self.steps%100==0:
        # print('Velocity', vel)
        done = False
        info = {
            'reward_run': forward_reward,
            'reward_ctrl': -ctrl_cost
        }

        if self.steps > self.max_steps:
            done = True


        return observation, reward, done, info

    def _get_obs(self):
        position = self.sim.data.qpos.flat.copy()
        velocity = self.sim.data.qvel.flat.copy()


        #################z,zg,y,yg #############################################
        observation = np.array((position[1],(self.goalz - position[1]),position[0], \
                                (self.goaly - position[0]), \
                                velocity[0],velocity[1])).ravel()
        return observation


    def set_goals(self):
        self.goaly = np.random.uniform(low=self.y_lower, high=self.y_upper)
        self.goalz = np.random.uniform(low=self.z_lower, high=self.z_upper)

    def reset_model(self):
        ####### Set goal positions #############################################
        self.set_goals()
        print('Reset goal to(y,z):',self.goaly/np.pi*180,self.goalz/np.pi*180)


        noise_low = -self._reset_noise_scale
        noise_high = self._reset_noise_scale

        qpos = self.init_qpos + self.np_random.uniform(
            low=noise_low, high=noise_high, size=self.model.nq)
        qvel = self.init_qvel + self._reset_noise_scale * self.np_random.randn(
            self.model.nv)

        self.set_state(qpos, qvel)

        observation = self._get_obs()

        self.steps = 0
        return observation

    def viewer_setup(self):
        for key, value in DEFAULT_CAMERA_CONFIG.items():
            if isinstance(value, np.ndarray):
                getattr(self.viewer.cam, key)[:] = value
            else:
                setattr(self.viewer.cam, key, value)




################### An example of a control loop using the mujoco environment ##

if __name__ == "__main__":
    env = arm_env_mj()
    env.reset()

    t = 0

    while(True):
        s,a,r,i = env.step([np.sin(t/200),np.cos(t/100)])
        t += 1
        env.viewer.render()
        if t > 10000:
            break
# mj_path, _ = mujoco_py.utils.discover_mujoco()
# xml_path = os.path.join(mj_path, 'model/arm', 'arm.xml')
# model = mujoco_py.load_model_from_path(xml_path)
# sim = mujoco_py.MjSim(model)
#
# print(sim.data.qpos)
# sim.step()
#
#
# print(sim.data.qpos)
# viewer = MjViewer(sim)
#
# t = 0
#
# while(True):
#     sim.data.ctrl[0] = 1
#     sim.data.ctrl[1] = 1
#     t += 1
#     sim.step()
#     viewer.render()
#
#     if t> 1000:
#         break
