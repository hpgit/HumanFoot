# from rl.core import Env
import pydart2 as pydart
import numpy as np
from math import exp, pi
from PyCommon.modules.Math import mmMath as mm
from random import randrange, random
import gym
import gym.spaces
from gym.utils import seeding

import PyCommon.modules.Resource.ysMotionLoader as yf
from DartDeep.pd_controller import PDController

from multiprocessing import Pool


def exp_reward_term(w, exp_w, v0, v1):
    norm = np.linalg.norm(v0 - v1)
    return w * exp(-exp_w * norm * norm)


class HpDartEnv(gym.Env):
    def __init__(self, env_name='walk', env_slaves=1):
        self.world = pydart.World(1./1200., "../data/woody_with_ground_v2.xml")
        self.world.control_skel = self.world.skeletons[1]
        self.skel = self.world.skeletons[1]
        self.Kp, self.Kd = 400., 40.
        self.pdc = PDController(self.skel, self.world.time_step(), 400., 40.)

        self.env_name = env_name

        self.ref_motion = None  # type: ym.Motion

        if env_name == 'walk':
            self.ref_motion = yf.readBvhFile("../data/woody_walk_normal.bvh")[40:]
        elif env_name == 'spiral_walk':
            self.ref_motion = yf.readBvhFile("../data/wd2_spiral_walk_normal05.bvh")
        elif env_name == 'walk_spin':
            self.ref_motion = yf.readBvhFile("../data/wd2_2foot_walk_turn2.bvh")
        elif env_name == 'jump':
            self.ref_motion = yf.readBvhFile("../data/wd2_jump0.bvh")[164:280]
        elif env_name == 'walk_fast':
            self.ref_motion = yf.readBvhFile("../data/wd2_WalkForwardVFast00.bvh")

        elif env_name == 'walk_left_90':
            self.ref_motion = yf.readBvhFile("../data/walk_left_90degree.bvh")
        elif env_name == 'walk_left_45':
            self.ref_motion = yf.readBvhFile("../data/walk_left_45degree.bvh")
        elif env_name == 'walk_pick':
            self.ref_motion = yf.readBvhFile("../data/wd2_pick_walk_1.bvh")
        elif env_name == 'walk_u_turn':
            self.ref_motion = yf.readBvhFile("../data/wd2_u-turn.bvh")[25:214]
            self.ref_motion.translateByOffset([0., 0.03, 0.])

        elif env_name == 'walk_sukiko':
            self.ref_motion = yf.readBvhFile('../data/wd2_WalkSukiko00.bvh')

        elif env_name == 'jump_whole':
            self.ref_motion = yf.readBvhFile("../data/wd2_jump0.bvh")[315:966]
        elif env_name == 'walk_u_turn_whole':
            self.ref_motion = yf.readBvhFile("../data/wd2_u-turn.bvh")
            self.ref_motion.translateByOffset([0., 0.03, 0.])

        self.ref_world = pydart.World(1./1200., "../data/woody_with_ground_v2.xml")
        self.ref_skel = self.ref_world.skeletons[1]
        # self.step_per_frame = round((1./self.world.time_step()) / self.ref_motion.fps)
        self.step_per_frame = 40

        self.rsi = True

        self.w_p = 0.65
        self.w_v = 0.1
        self.w_e = 0.15
        self.w_c = 0.1

        self.exp_p = 2.
        self.exp_v = 0.1
        self.exp_e = 40.
        self.exp_c = 10.

        self.body_num = self.skel.num_bodynodes()
        self.idx_e = [self.skel.bodynode_index('LeftFoot'), self.skel.bodynode_index('RightFoot'),
                      self.skel.bodynode_index('LeftForeArm'), self.skel.bodynode_index('RightForeArm')]
        self.body_e = list(map(self.skel.body, self.idx_e))
        self.ref_body_e = list(map(self.ref_skel.body, self.idx_e))
        self.motion_len = len(self.ref_motion)
        self.motion_time = len(self.ref_motion) / self.ref_motion.fps

        self.time_offset = 0.

        state_num = 1 + (3*3 + 4) * self.body_num
        action_num = self.skel.num_dofs() - 6

        state_high = np.array([np.finfo(np.float32).max] * state_num)
        action_high = np.array([pi*10./2.] * action_num)

        self.action_space = gym.spaces.Box(-action_high, action_high, dtype=np.float32)
        self.observation_space = gym.spaces.Box(-state_high, state_high, dtype=np.float32)

        self.viewer = None

        self.phase_frame = 0

    def state(self):
        pelvis = self.skel.body(0)
        p_pelvis = pelvis.world_transform()[:3, 3]
        R_pelvis = pelvis.world_transform()[:3, :3]

        phase = min(1., (self.world.time() + self.time_offset)/self.motion_time)
        state = [phase]

        p = np.array([np.dot(R_pelvis.T, body.to_world() - p_pelvis) for body in self.skel.bodynodes]).flatten()
        R = np.array([mm.rot2quat(np.dot(R_pelvis.T, body.world_transform()[:3, :3])) for body in self.skel.bodynodes]).flatten()
        v = np.array([np.dot(R_pelvis.T, body.world_linear_velocity()) for body in self.skel.bodynodes]).flatten()
        w = np.array([np.dot(R_pelvis.T, body.world_angular_velocity())/20. for body in self.skel.bodynodes]).flatten()

        state.extend(p)
        state.extend(R)
        state.extend(v)
        state.extend(w)

        return np.asarray(state).flatten()

    def reward(self):
        # current_frame = min(len(self.ref_motion)-1, int((self.world.time() + self.time_offset) * self.ref_motion.fps))
        current_time = self.world.time() + self.time_offset
        self.ref_skel.set_positions(self.ref_motion.get_q_by_time(current_time))
        self.ref_skel.set_velocities(self.ref_motion.get_dq_dart_by_time(current_time))

        p_e_hat = np.asarray([body.world_transform()[:3, 3] for body in self.ref_body_e]).flatten()
        p_e = np.asarray([body.world_transform()[:3, 3] for body in self.body_e]).flatten()

        return exp_reward_term(self.w_p, self.exp_p, self.skel.q, self.ref_skel.q) \
              + exp_reward_term(self.w_v, self.exp_v, self.skel.dq, self.ref_skel.dq) \
              + exp_reward_term(self.w_e, self.exp_e, p_e, p_e_hat) \
              + exp_reward_term(self.w_c, self.exp_c, self.skel.com(), self.ref_skel.com())

    def is_done(self):
        if self.skel.com()[1] < 0.4:
            # print('fallen')
            return True
        elif True in np.isnan(np.asarray(self.skel.q)) or True in np.isnan(np.asarray(self.skel.dq)):
            # print('nan')
            return True
        elif self.world.time() + self.time_offset > self.motion_time:
            # print('timeout')
            return True
        return False

    def step(self, _action):
        """Run one timestep of the environment's dynamics.
        Accepts an action and returns a tuple (observation, reward, done, info).

        # Arguments
            action (object): An action provided by the environment.

        # Returns
            observation (object): Agent's observation of the current environment.
            reward (float) : Amount of reward returned after previous action.
            done (boolean): Whether the episode has ended, in which case further step() calls will return undefined results.
            info (dict): Contains auxiliary diagnostic information (helpful for debugging, and sometimes learning).
        """
        action = np.hstack((np.zeros(6), _action/10.))

        next_frame_time = self.world.time() + self.time_offset + self.world.time_step() * self.step_per_frame
        self.ref_skel.set_positions(self.ref_motion.get_q_by_time(next_frame_time))
        self.ref_skel.set_velocities(self.ref_motion.get_dq_dart_by_time(next_frame_time))
        for i in range(self.step_per_frame):
            self.skel.set_forces(self.skel.get_spd(self.ref_skel.q + action, self.world.time_step(), self.Kp, self.Kd))
            # self.skel.set_forces(self.pdc.compute_flat(self.ref_skel.q + action))
            self.world.step()
        return tuple([self.state(), self.reward(), self.is_done(), dict()])

    def continue_from_now_by_phase(self, phase):
        self.phase_frame = round(phase * (self.motion_len-1))
        skel_pelvis_offset = self.skel.joint(0).position_in_world_frame() - self.ref_motion[self.phase_frame].getJointPositionGlobal(0)
        skel_pelvis_offset[1] = 0.
        self.ref_motion.translateByOffset(skel_pelvis_offset)
        self.time_offset = - self.world.time() + (self.phase_frame / self.ref_motion.fps)

    def reset(self):
        """
        Resets the state of the environment and returns an initial observation.

        # Returns
            observation (object): The initial observation of the space. Initial reward is assumed to be 0.
        """
        self.world.reset()
        self.continue_from_now_by_phase(random() if self.rsi else 0.)
        self.skel.set_positions(self.ref_motion.get_q(self.phase_frame))
        dq = self.ref_motion.get_dq_dart(self.phase_frame)
        self.skel.set_velocities(dq)

        return self.state()

    def render(self, mode='human', close=False):
        """Renders the environment.
        The set of supported modes varies per environment. (And some
        environments do not support rendering at all.)

        # Arguments
            mode (str): The mode to render with.
            close (bool): Close all open renderings.
        """
        if self.viewer is None:
            from gym.envs.classic_control import rendering
            self.viewer = rendering.Viewer(500,500)
            self.viewer.set_bounds(-2.2, 2.2, -2.2, 2.2)
            # rod = rendering.make_capsule(1, .2)
            # rod.set_color(.8, .3, .3)
            # rod.add_attr(self.pole_transform)
            # self.viewer.add_geom(rod)
            self.body_transform = list()
            self.ref_body_transform = list()
            for i in range(self.body_num):
                axle = rendering.make_circle(.05)
                axle.set_color(0, 0, 0)
                self.body_transform.append(rendering.Transform())
                axle.add_attr(self.body_transform[i])
                self.viewer.add_geom(axle)

            for i in range(self.body_num):
                axle = rendering.make_circle(.05)
                axle.set_color(1, 0, 0)
                self.ref_body_transform.append(rendering.Transform())
                axle.add_attr(self.ref_body_transform[i])
                self.viewer.add_geom(axle)

        for i in range(self.body_num):
            self.body_transform[i].set_translation(self.skel.body(i).world_transform()[:3, 3][0]-1., self.skel.body(i).world_transform()[:3, 3][1])
            self.ref_body_transform[i].set_translation(self.ref_skel.body(i).world_transform()[:3, 3][0]-1., self.ref_skel.body(i).world_transform()[:3, 3][1])

        return self.viewer.render(return_rgb_array = mode=='rgb_array')

    def close(self):
        """Override in your subclass to perform any necessary cleanup.
        Environments will automatically close() themselves when
        garbage collected or when the program exits.
        """
        pass

    def seed(self, seed=None):
        """Sets the seed for this env's random number generator(s).

        # Returns
            Returns the list of seeds used in this env's random number generators
        """
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        return [seed]

    def GetState(self, param):
        return self.state()

    def GetStates(self):
        return [self.state()]

    def Resets(self, rsi):
        self.rsi = rsi
        self.reset()

    def Reset(self, rsi, idx):
        self.rsi = rsi
        self.reset()

    def Steps(self, actions):
        return self.step(actions[0])

    def IsTerminalState(self, j):
        return self.is_done()

    def GetReward(self, j):
        return self.reward()

    def IsTerminalStates(self):
        return [self.is_done()]
