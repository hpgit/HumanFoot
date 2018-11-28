import os
import numpy as np
from math import exp, pi, log
from PyCommon.modules.Math import mmMath as mm
from random import randrange, random
import gym
import gym.spaces
from gym.utils import seeding

import PyCommon.modules.Resource.ysMotionLoader as yf
from PyCommon.modules.Simulator import csVpWorld as cvw
from PyCommon.modules.Simulator import csVpModel as cvm
from PyCommon.modules.ArticulatedBody import ysControl as yct
from PyCommon.modules.Util import ysPythonEx as ype

from FootDeep.sh_v2 import mtInitialize as mit


def exp_reward_term(w, exp_w, v):
    norm = np.linalg.norm(v)
    return w * exp(-exp_w * norm * norm)


class HpVpEnv(gym.Env):
    def __init__(self, env_name='walk'):
        motionFile = 'wd2_tiptoe_zygote.bvh'
        motion, mcfg, wcfg, stepsPerFrame, config, frame_rate = mit.create_biped(motionFile, SEGMENT_FOOT_MAG=0.01, SEGMENT_FOOT_RAD=0.008)
        self.world = cvw.VpWorld(wcfg)
        self.world.SetGlobalDamping(0.999)
        self.skel = cvm.VpControlModel(self.world, motion[0], mcfg)
        self.world.initialize()
        self.skel.initializeHybridDynamics()
        self.world.BackupState()

        self.ref_world = cvw.VpWorld(wcfg)
        self.ref_skel = cvm.VpControlModel(self.ref_world, motion[0], mcfg)
        self.ref_world.initialize()

        self.bodyIDsToCheck = list(range(self.world.getBodyNum()))
        self.mus = [.5]* len(self.bodyIDsToCheck)
        self.Ks, self.Ds = config['Ks'], config['Ds']
        self.Kp, self.Kd = 400., 40.

        self.env_name = env_name

        self.ref_motion = None  # type: ym.Motion
        motion_name = None

        if env_name == 'walk':
            motion_name = "../data/segfoot_wd2_WalkForwardNormal00.bvh"
        elif env_name == 'walk_fast':
            motion_name = "../data/segfoot_wd2_WalkForwardVFast00.bvh"
        elif env_name == 'walk_sukiko':
            motion_name = '../data/segfoot_wd2_WalkSukiko00.bvh'

        SEGMENT_FOOT_MAG = 0.01
        SEGMENT_FOOT_RAD = 0.008
        bvh = yf.readBvhFileAsBvh(motion_name)
        current_path = os.path.dirname(os.path.abspath(__file__))
        partBvhFilePath = current_path + '/../../PyCommon/modules/samples/'
        partBvhFilePath = partBvhFilePath + 'foot_model_real_joint_01.bvh'
        partBvh = yf.readBvhFileAsBvh(partBvhFilePath)
        partSkeleton = partBvh.toJointSkeleton(1., False)
        bvh.replaceJointFromBvh('RightFoot', partBvh, SEGMENT_FOOT_MAG)
        partBvh = yf.readBvhFileAsBvh(partBvhFilePath)
        partBvh.mirror('YZ')
        bvh.replaceJointFromBvh('LeftFoot', partBvh, SEGMENT_FOOT_MAG)

        self.ref_motion = bvh.toJointMotion(1., False)  # type: ym.JointMotion

        if env_name == 'walk':
            # self.ref_motion = self.ref_motion[20:]
            self.ref_motion.translateByOffset([0., -0.03, 0.])
        elif env_name == 'jump':
            self.ref_motion = self.ref_motion[164:280]
        elif env_name == 'walk_u_turn':
            self.ref_motion = self.ref_motion[25:214]
            self.ref_motion.translateByOffset([0., 0.03, 0.])
        elif env_name == 'jump_whole':
            self.ref_motion = self.ref_motion[315:966]
        elif env_name == 'walk_u_turn_whole':
            self.ref_motion.translateByOffset([0., 0.03, 0.])

        self.step_per_frame = stepsPerFrame

        self.rsi = True

        self.w_p = 0.65
        self.w_v = 0.1
        self.w_e = 0.15
        self.w_c = 0.1

        self.exp_p = 2.
        self.exp_v = 0.1
        self.exp_e = 40.
        self.exp_c = 10.

        self.body_num = self.skel.getBodyNum()
        self.idx_e = [self.skel.name2index('LeftFoot'), self.skel.name2index('RightFoot'),
                      self.skel.name2index('LeftForeArm'), self.skel.name2index('RightForeArm')]
        self.motion_len = len(self.ref_motion)
        self.motion_time = len(self.ref_motion) / self.ref_motion.fps

        self.time_offset = 0.

        self.viewer = None

        self.phase_frame = 0

        self.prev_ref_q = np.zeros(self.ref_skel.getTotalDOF())
        self.prev_ref_dq = np.zeros(self.ref_skel.getTotalDOF())
        self.prev_ref_p_e_hat = np.asarray([self.ref_skel.getBodyPositionGlobal(body_idx) for body_idx in self.idx_e]).flatten()
        self.prev_ref_com = self.ref_skel.getCOM()

        # setting for reward
        self.reward_joint = list()
        self.reward_joint.append('j_Hips')
        self.reward_joint.append('j_RightUpLeg')
        self.reward_joint.append('j_RightLeg')
        self.reward_joint.append('j_LeftUpLeg')
        self.reward_joint.append('j_LeftLeg')
        self.reward_joint.append('j_Spine')
        self.reward_joint.append('j_Spine1')
        self.reward_joint.append('j_RightArm')
        self.reward_joint.append('j_RightForeArm')
        self.reward_joint.append('j_LeftArm')
        self.reward_joint.append('j_LeftForeArm')

        # setting for pd gain
        self.foot_joint = list()
        self.foot_joint.append('j_RightFoot')
        self.foot_joint.append('j_RightFoot_foot_0_0')
        self.foot_joint.append('j_RightFoot_foot_0_0_0')
        self.foot_joint.append('j_RightFoot_foot_0_1_0')
        self.foot_joint.append('j_RightFoot_foot_1_0')
        self.foot_joint.append('j_LeftFoot')
        self.foot_joint.append('j_LeftFoot_foot_0_0')
        self.foot_joint.append('j_LeftFoot_foot_0_0_0')
        self.foot_joint.append('j_LeftFoot_foot_0_1_0')
        self.foot_joint.append('j_LeftFoot_foot_1_0')

        state_num = 1 + (3*3 + 4) * self.body_num
        action_num = self.skel.getTotalDOF() - 6 + len(self.foot_joint)

        state_high = np.array([np.finfo(np.float32).max] * state_num)
        action_high = np.array([pi*10.] * action_num)

        self.action_space = gym.spaces.Box(-action_high, action_high, dtype=np.float32)
        self.observation_space = gym.spaces.Box(-state_high, state_high, dtype=np.float32)

    def state(self):
        p_pelvis = self.skel.getBodyPositionGlobal(0)
        R_pelvis = self.skel.getBodyOrientationGlobal(0)

        phase = min(1., (self.world.GetSimulationTime() + self.time_offset)/self.motion_time)
        state = [phase]

        self.skel.getBodyPositionsGlobal()

        p = np.array([np.dot(R_pelvis.T, body_pos - p_pelvis) for body_pos in self.skel.getBodyPositionsGlobal()]).flatten()
        R = np.array([mm.rot2quat(np.dot(R_pelvis.T, body_ori)) for body_ori in self.skel.getBodyOrientationsGlobal()]).flatten()
        v = np.array([np.dot(R_pelvis.T, body_vel) for body_vel in self.skel.getBodyVelocitiesGlobal()]).flatten()
        w = np.array([np.dot(R_pelvis.T, body_angvel)/20. for body_angvel in self.skel.getBodyAngVelocitiesGlobal()]).flatten()

        state.extend(p)
        state.extend(R)
        state.extend(v)
        state.extend(w)

        return np.asarray(state).flatten()

    def reward(self):
        p_e = np.asarray([self.skel.getBodyPositionGlobal(body_idx) for body_idx in self.idx_e]).flatten()

        q_diff = np.asarray(self.skel.position_differences(self.prev_ref_q, self.skel.q))
        dq_diff = np.asarray(self.skel.velocity_differences(self.prev_ref_dq, self.skel.dq))

        q_reward = exp_reward_term(self.w_p, self.exp_p,
                                   np.concatenate(list(q_diff[self.skel.getJointDOFIndexes(self.skel.name2index(joint_name))] for joint_name in self.reward_joint)))
        dq_reward = exp_reward_term(self.w_p, self.exp_p,
                                    np.concatenate(list(dq_diff[self.skel.getJointDOFIndexes(self.skel.name2index(joint_name))] for joint_name in self.reward_joint)))
        ee_reward = exp_reward_term(self.w_e, self.exp_e, p_e - self.prev_ref_p_e_hat)
        com_reward = exp_reward_term(self.w_c, self.exp_c, self.skel.getCOM() - self.prev_ref_com)

        reward = q_reward + dq_reward + ee_reward + com_reward

        return reward

    def is_done(self):
        if self.skel.com()[1] < 0.4:
            # print('fallen')
            return True
        elif True in np.isnan(np.asarray(self.skel.q)) or True in np.isnan(np.asarray(self.skel.dq)):
            # print('nan')
            return True
        elif self.world.GetSimulationTime() + self.time_offset > self.motion_time:
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
        action = np.hstack((np.zeros(6), _action[:self.skel.ndofs-6]/10.))
        Kp_joint = np.asarray([0.0] + [self.Kp] * self.skel.getJointNum())
        Kd_joint = np.asarray([0.0] + [self.Kd] * self.skel.getJointNum())
        for joint_idx in range(len(self.foot_joint)):
            Kp_joint[1+joint_idx] = self.Kd * exp(log(self.Kp) * _action[self.skel.ndofs-6 + joint_idx]/10.)
            Kd_joint[1+joint_idx] = self.Kp * exp(log(self.Kd) * _action[self.skel.ndofs-6 + joint_idx]/20.)

        DOFs = self.skel.getDOFs()
        action_nested = ype.makeNestedList(DOFs)
        th_r = self.ref_skel.getDOFPositions()
        ype.nested(action, action_nested)
        for i in range(1, len(th_r)):
            th_r[i] = np.dot(th_r[i], mm.exp(action_nested[i]))

        th = self.skel.getDOFPositions()
        dth = self.skel.getDOFVelocities()
        ddth_des = yct.getDesiredDOFAccelerations(th_r, th, None, dth, None, Kp_joint, Kd_joint)

        for i in range(self.step_per_frame):
            bodyIDs, contactPositions, contactPositionLocals, contactForces = \
                self.world.calcPenaltyForce(self.bodyIDsToCheck, self.mus, self.Ks, self.Ds)
            self.world.applyPenaltyForce(bodyIDs, contactPositionLocals, contactForces)
            self.skel.setDOFAccelerations(ddth_des)
            self.world.step()

        self.update_ref_skel(False)

        return tuple([self.state(), self.reward(), self.is_done(), dict()])

    def update_ref_skel(self, reset=False):
        if not reset:
            self.prev_ref_q = self.ref_skel.positions()
            self.prev_ref_dq = self.ref_skel.velocities()
            self.prev_ref_p_e_hat = np.asarray([self.ref_skel.getBodyPositionGlobal(body_idx) for body_idx in self.idx_e]).flatten()
            self.prev_ref_com = self.ref_skel.com()

        next_frame_time = self.world.GetSimulationTime() + self.time_offset + self.world.time_step() * self.step_per_frame
        self.ref_skel.set_positions(self.ref_motion.get_q_by_time(next_frame_time))
        self.ref_skel.set_velocities(self.ref_motion.get_dq_dart_by_time(next_frame_time))

        if reset:
            self.prev_ref_q = self.ref_skel.positions()
            self.prev_ref_dq = self.ref_skel.velocities()
            self.prev_ref_p_e_hat = np.asarray([self.ref_skel.getBodyPositionGlobal(body_idx) for body_idx in self.idx_e]).flatten()
            self.prev_ref_com = self.ref_skel.com()

    def continue_from_now_by_phase(self, phase):
        self.phase_frame = round(phase * (self.motion_len-1))
        skel_pelvis_offset = self.skel.getJointPositionGlobal(0) - self.ref_motion[self.phase_frame].getJointPositionGlobal(0)
        skel_pelvis_offset[1] = 0.
        self.ref_motion.translateByOffset(skel_pelvis_offset)
        self.time_offset = - self.world.GetSimulationTime() + (self.phase_frame / self.ref_motion.fps)

    def reset(self):
        """
        Resets the state of the environment and returns an initial observation.

        # Returns
            observation (object): The initial observation of the space. Initial reward is assumed to be 0.
        """
        # self.world.reset()
        self.world.RollbackState()

        self.continue_from_now_by_phase(random() if self.rsi else 0.)

        self.skel.set_q(self.ref_motion.get_q(self.phase_frame))
        self.skel.set_velocities(self.ref_motion.get_dq_dart(self.phase_frame))

        self.update_ref_skel(True)

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
