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


def exp_reward_term(w, exp_w, v):
    norm = np.linalg.norm(v)
    return w * exp(-exp_w * norm * norm)


class HpDartEnv(gym.Env):
    def __init__(self, rnn_len):
        skel_file = '../data/cmu_with_ground.xml'
        self.world = pydart.World(1./1200., skel_file)
        self.world.control_skel = self.world.skeletons[1]
        self.skel = self.world.skeletons[1]
        self.Kp, self.Kd = 800., 40.
        self.pdc = PDController(self.skel, self.world.time_step(), 400., 40.)

        self.ref_motion = yf.readBvhFile("../data/cmu_tpose.bvh")[:rnn_len]
        self.ref_motion_len = rnn_len

        self.ref_world = pydart.World(1./1200., skel_file)
        self.ref_skel = self.ref_world.skeletons[1]
        self.step_per_frame = 40

        self.rsi = True

        # self.w_g = 0.3
        # self.w_p = 0.65 * .7
        # self.w_v = 0.1 * .7
        # self.w_e = 0.15 * .7
        # self.w_c = 0.1 * .7

        self.w_p = 0.65
        self.w_v = 0.1
        self.w_e = 0.15
        self.w_c = 0.1

        self.exp_g = 2.5
        self.exp_p = 2.
        self.exp_v = 0.1
        self.exp_e = 40.
        self.exp_c = 10.

        self.body_num = self.skel.num_bodynodes()
        self.idx_e = [self.skel.bodynode_index('LeftFoot'), self.skel.bodynode_index('RightFoot'),
                      self.skel.bodynode_index('LeftHand'), self.skel.bodynode_index('RightHand')]
        self.body_e = list(map(self.skel.body, self.idx_e))
        self.ref_body_e = list(map(self.ref_skel.body, self.idx_e))
        self.motion_len = len(self.ref_motion)
        self.motion_time = len(self.ref_motion) / self.ref_motion.fps

        state_num = 2 + (3*3 + 4) * self.body_num + (3 + 4) * self.body_num
        action_num = self.skel.num_dofs() - 6

        state_high = np.array([np.finfo(np.float32).max] * state_num)
        action_high = np.array([pi*10./2.] * action_num)

        self.action_space = gym.spaces.Box(-action_high, action_high, dtype=np.float32)
        self.observation_space = gym.spaces.Box(-state_high, state_high, dtype=np.float32)

        self.viewer = None

        self.phase_frame = 0

        self.goals_in_world_frame = list()
        self.goal = np.zeros(2)

        self.prev_ref_q = np.zeros(self.ref_skel.num_dofs())
        self.prev_ref_dq = np.zeros(self.ref_skel.num_dofs())
        self.prev_ref_p_e_hat = np.asarray([body.world_transform()[:3, 3] for body in self.ref_body_e]).flatten()
        self.prev_ref_com = self.ref_skel.com()
        self.prev_goal = np.zeros(2)

    def state(self):
        state = [self.goal[0], self.goal[1]]

        pelvis = self.skel.body(0)
        p_pelvis = pelvis.world_transform()[:3, 3]
        R_pelvis = pelvis.world_transform()[:3, :3]

        ref_pelvis = self.ref_skel.body(0)
        ref_p_pelvis = ref_pelvis.world_transform()[:3, 3]
        ref_R_pelvis = ref_pelvis.world_transform()[:3, :3]

        # deep mimic style
        p = np.array([np.dot(R_pelvis.T, body.to_world() - p_pelvis) for body in self.skel.bodynodes]).flatten()
        R = np.array([mm.rot2quat(np.dot(R_pelvis.T, body.world_transform()[:3, :3])) for body in self.skel.bodynodes]).flatten()
        v = np.array([np.dot(R_pelvis.T, body.world_linear_velocity()) for body in self.skel.bodynodes]).flatten()
        w = np.array([np.dot(R_pelvis.T, body.world_angular_velocity())/20. for body in self.skel.bodynodes]).flatten()
        ref_p = np.array([np.dot(ref_R_pelvis.T, body.to_world() - ref_p_pelvis) for body in self.ref_skel.bodynodes]).flatten()
        ref_R = np.array([mm.rot2quat(np.dot(ref_R_pelvis.T, body.world_transform()[:3, :3])) for body in self.ref_skel.bodynodes]).flatten()

        state.extend(p)
        state.extend(R)
        state.extend(v)
        state.extend(w)
        state.extend(ref_p)
        state.extend(ref_R)

        # soohwan style
        # p = 0.8 * np.array([np.dot(R_pelvis.T, body.to_world() - p_pelvis) for body in self.skel.bodynodes]).flatten()
        # v = 0.2 * np.array([np.dot(R_pelvis.T, body.world_linear_velocity()) for body in self.skel.bodynodes]).flatten()
        # _R = 2. * self.skel.positions()
        # _w = .2 * self.skel.velocities()
        # R = np.append(_R[:3], _R[6:]).flatten()
        # w = np.append(_w[:3], _w[6:]).flatten()
        # _ref_R = 2. * self.prev_ref_q
        # ref_R = np.append(_ref_R[:3], _ref_R[6:]).flatten()

        # state.extend(p)
        # state.extend(R)
        # state.extend(v)
        # state.extend(w)
        # state.extend(ref_R)

        return np.asarray(state).flatten()

    def reward(self):
        p_e = np.asarray([body.world_transform()[:3, 3] for body in self.body_e]).flatten()

        q_diff = np.asarray(self.skel.position_differences(self.prev_ref_q, self.skel.q))
        dq_diff = np.asarray(self.skel.velocity_differences(self.prev_ref_dq, self.skel.dq))

        reward = 0.
        reward += exp_reward_term(self.w_p, self.exp_p, np.concatenate((q_diff[:3], q_diff[6:])))
        reward += exp_reward_term(self.w_v, self.exp_v, np.concatenate((dq_diff[:3], dq_diff[6:])))
        reward += exp_reward_term(self.w_e, self.exp_e, p_e - self.prev_ref_p_e_hat)
        reward += exp_reward_term(self.w_c, self.exp_c, self.skel.com() - self.prev_ref_com)
        # reward += exp_reward_term(self.w_g, self.exp_g, self.prev_goal)
        return reward

    def is_done(self):
        if self.skel.com()[1] < 0.4:
            # print('fallen')
            return True
        elif True in np.isnan(np.asarray(self.skel.q)) or True in np.isnan(np.asarray(self.skel.dq)):
            # print('nan')
            return True
        # elif self.world.time() + self.time_offset > self.motion_time:
        elif self.phase_frame == self.ref_motion_len-1:
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

        for i in range(self.step_per_frame):
            self.skel.set_forces(self.skel.get_spd(self.ref_skel.q + action, self.world.time_step(), self.Kp, self.Kd))
            # self.skel.set_forces(self.pdc.compute_flat(self.ref_skel.q + action))
            self.world.step()

        self.phase_frame += 1
        self.update_ref_skel()
        self.update_goal_in_local_frame()

        # now
        # state:
        # simul[phase_frame-1] : 'current' char pose
        # goal[phase_frame] : 'current' goal
        # motion[phase_frame] : 'next' motion pose

        # reward:
        # similarity btw current char pose and (previous goal, and current motion pose)

        return tuple([self.state(), self.reward(), self.is_done(), dict()])

    def update_goal_in_local_frame(self):
        self.prev_goal = self.goal.copy()
        body_transform = self.skel.body(0).world_transform()
        goal_vector_in_world_frame = self.goals_in_world_frame[self.phase_frame] - body_transform[:3, 3]
        goal_vector_in_world_frame[1] = 0.
        radius = mm.length(goal_vector_in_world_frame)
        unit_goal_vector_in_world_frame = mm.normalize(goal_vector_in_world_frame)
        root_x_in_world_plane = body_transform[:3, 0]
        root_x_in_world_plane[1] = 0.
        unit_root_x_in_world_plane = mm.seq2Vec3(mm.normalize(root_x_in_world_plane))
        unit_root_z_in_world_plane = mm.cross(unit_root_x_in_world_plane, mm.unitY())
        # angle = atan2(np.dot(unit_root_x_in_world_plane, unit_goal_vector_in_world_frame), np.dot(unit_root_z_in_world_plane, unit_goal_vector_in_world_frame))

        self.goal = radius * np.array([np.dot(unit_root_x_in_world_plane, unit_goal_vector_in_world_frame), np.dot(unit_root_z_in_world_plane, unit_goal_vector_in_world_frame)])

    def update_ref_skel(self):
        self.prev_ref_q = self.ref_skel.positions()
        self.prev_ref_dq = self.ref_skel.velocities()
        self.prev_ref_p_e_hat = np.asarray([body.world_transform()[:3, 3] for body in self.ref_body_e]).flatten()
        self.prev_ref_com = self.ref_skel.com()

        self.ref_skel.set_positions(self.ref_motion.get_q(self.phase_frame))
        self.ref_skel.set_velocities(self.ref_motion.get_dq_dart(self.phase_frame))

    def reset(self):
        """
        Resets the state of the environment and returns an initial observation.

        # Returns
            observation (object): The initial observation of the space. Initial reward is assumed to be 0.
        """
        self.world.reset()

        self.phase_frame = randrange(0, self.ref_motion_len - 2) if self.rsi else 0

        self.skel.set_positions(self.ref_motion.get_q(self.phase_frame))
        self.skel.set_velocities(self.ref_motion.get_dq_dart(self.phase_frame))

        self.ref_skel.set_positions(self.ref_motion.get_q(self.phase_frame))
        self.ref_skel.set_velocities(self.ref_motion.get_dq_dart(self.phase_frame))

        self.phase_frame += 1
        self.update_ref_skel()
        self.update_goal_in_local_frame()

        return self.state()

    def update_target(self, goals_world, qs):
        self.goals_in_world_frame = goals_world
        for i in range(self.ref_motion_len):
            self.ref_motion[i].set_q(qs[i])
        self.ref_motion.updateGlobalT()

    def render(self, mode='human', close=False):
        """Renders the environment.
        The set of supported modes varies per environment. (And some
        environments do not support rendering at all.)

        # Arguments
            mode (str): The mode to render with.
            close (bool): Close all open renderings.
        """
        pass

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

    def IsTerminalStates(self):
        return [self.is_done()]

    def GetReward(self, j):
        return self.reward()

    def GetRewards(self):
        return [self.reward()]

