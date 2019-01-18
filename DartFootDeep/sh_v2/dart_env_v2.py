import os
import pydart2 as pydart
import numpy as np
from math import exp, pi, log
from PyCommon.modules.Math import mmMath as mm
from random import randrange, random
import gym
import gym.spaces
from gym.utils import seeding

import PyCommon.modules.Resource.ysMotionLoader as yf


def exp_reward_term(w, exp_w, v):
    norm = np.linalg.norm(v)
    return w * exp(-exp_w * norm * norm)


def get_joint_dof_range(joint):
    return range(joint.dofs[0].index_in_skeleton(), joint.dofs[0].index_in_skeleton()+joint.num_dofs())


def fix_motion_data_by_foot(motion, skel, SEGMENT_FOOT_RAD):
    """

    :param motion:
    :type motion: ym.JointMotion
    :param skel:
    :type skel: pydart.Skeleton
    :param SEGMENT_FOOT_RAD:
    :type SEGMENT_FOOT_RAD: float
    :return:
    """
    for i in range(len(motion)):
        skel.set_positions(motion.get_q(i))
        min_joint_y = np.inf
        for body in skel.bodynodes:
            for shapenode in body.shapenodes:
                if shapenode.has_collision_aspect():
                    joint_y = np.dot(body.world_transform(), shapenode.relative_transform())[1, 3]
                    if min_joint_y > joint_y:
                        min_joint_y = joint_y

        if min_joint_y < SEGMENT_FOOT_RAD:
            motion[i].translateByOffset((SEGMENT_FOOT_RAD-min_joint_y) * mm.unitY())


class HpDartEnv(gym.Env):
    def __init__(self, env_name='walk'):
        self.world = pydart.World(1./1200., "../data/wd2_seg.xml")
        self.world.control_skel = self.world.skeletons[1]
        self.world.disable_recording()
        self.world.collision_auto_update = False
        self.skel = self.world.skeletons[1]
        self.Kp, self.Kd = 400., 40.

        self.env_name = env_name

        self.ref_motion = None  # type: ym.Motion
        motion_name = None

        if env_name == 'walk':
            motion_name = "../data/segfoot_wd2_WalkForwardNormal00.bvh"
        elif env_name == 'walk_repeated':
            motion_name = "../data/segfoot_wd2_WalkForwardNormal00_REPEATED.bvh"
        elif env_name == 'walk_fast':
            motion_name = "../data/segfoot_wd2_WalkForwardVFast00.bvh"
        elif env_name == 'walk_sukiko':
            motion_name = '../data/segfoot_wd2_WalkSukiko00.bvh'
        elif env_name == 'walk_u_turn':
            motion_name = '../data/segfoot_wd2_u-turn_edit.bvh'
        elif env_name == '1foot_contact_run':
            motion_name = '../data/segfoot_wd2_1foot_contact_run2_edit.bvh'
        elif env_name == 'round_girl':
            motion_name = '../data/segfoot_wd2_boxing_round_round_girl1_edit.bvh'
        elif env_name == 'fast_2foot_hop':
            motion_name = '../data/segfoot_wd2_fast_2foot_hop_edit.bvh'
        elif env_name == 'slow_2foot_hop':
            motion_name = '../data/segfoot_wd2_slow_2foot_hop_edit.bvh'
        elif env_name == 'long_broad_jump':
            motion_name = '../data/segfoot_wd2_long_broad_jump_edit.bvh'
        elif env_name == 'short_broad_jump':
            motion_name = '../data/segfoot_wd2_short_broad_jump_edit.bvh'
        elif env_name == 'n_kick':
            motion_name = '../data/segfoot_wd2_n_kick_edit.bvh'
        elif env_name == 'jump':
            motion_name = '../data/wd2_jump0_edited.bvh'

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
        if env_name == 'walk_repeated':
            # self.ref_motion = self.ref_motion[20:]
            self.ref_motion.translateByOffset([0., -0.03, 0.])
        elif env_name == 'jump':
            self.ref_motion = self.ref_motion[164:280]
            self.ref_motion.translateByOffset([0., -0.06, 0.])
        elif env_name == 'walk_u_turn':
            self.ref_motion = self.ref_motion[25:214]
            self.ref_motion.translateByOffset([0., -0.09, 0.])
        elif env_name == 'jump_whole':
            self.ref_motion = self.ref_motion[315:966]
        elif env_name == 'walk_u_turn_whole':
            self.ref_motion.translateByOffset([0., 0.03, 0.])

        elif env_name == '1foot_contact_run':
            self.ref_motion.translateByOffset([0., -0.09, 0.])

        elif env_name == 'round_girl':
            self.ref_motion = self.ref_motion[505:658]
            self.ref_motion.translateByOffset([0., -0.01, 0.])

        elif env_name == 'fast_2foot_hop':
            self.ref_motion.translateByOffset([0., -0.09, 0.])
        elif env_name == 'slow_2foot_hop':
            self.ref_motion.translateByOffset([0., -0.09, 0.])
        elif env_name == 'long_broad_jump':
            self.ref_motion.translateByOffset([0., -0.09, 0.])
        elif env_name == 'short_broad_jump':
            self.ref_motion.translateByOffset([0., -0.09, 0.])
        elif env_name == 'n_kick':
            self.ref_motion.translateByOffset([0., -0.09, 0.])

        self.ref_world = pydart.World(1./1200., "../data/wd2_seg.xml")
        self.ref_skel = self.ref_world.skeletons[1]
        self.step_per_frame = round((1./self.world.time_step()) / self.ref_motion.fps)
        # self.step_per_frame = 40

        fix_motion_data_by_foot(self.ref_motion, self.ref_skel, SEGMENT_FOOT_RAD)

        self.rsi = True

        # self.w_p = 0.65
        self.w_p = 0.50
        self.w_v = 0.1
        self.w_e = 0.10
        self.w_c = 0.1
        self.w_t = 0.1
        self.w_y_ankle = 0.1

        self.exp_p = 2.
        self.exp_v = 0.1
        self.exp_e = 40.
        self.exp_c = 10.
        self.exp_t = 20.
        self.exp_y_ankle = 5.

        # soohwan style
        # self.w_p = 0.15
        # self.w_v = 0.05
        # self.w_c = 0.4
        # self.w_c_v = 0.05
        # self.w_e = 0.2
        # self.w_e_ori = 0.05
        #
        # self.exp_p = 2.
        # self.exp_v = 20.
        # self.exp_c = .3
        # self.exp_c_v = 2.
        # self.exp_e = 2.
        # self.exp_e_ori = 2.

        self.body_num = self.skel.num_bodynodes()
        self.idx_e = [self.skel.bodynode_index('LeftFoot'), self.skel.bodynode_index('RightFoot'),
                      self.skel.bodynode_index('LeftForeArm'), self.skel.bodynode_index('RightForeArm')]
        self.ref_body_e = list(map(self.ref_skel.body, self.idx_e))
        self.ankle_joint_names = ['j_LeftFoot', 'j_RightFoot']
        self.motion_len = len(self.ref_motion)
        self.motion_time = len(self.ref_motion) / self.ref_motion.fps

        self.time_offset = 0.

        self.viewer = None

        self.phase_frame = 0

        self.prev_ref_q = np.zeros(self.ref_skel.num_dofs())
        self.prev_ref_dq = np.zeros(self.ref_skel.num_dofs())
        self.prev_ref_p_e_hat = np.asarray([body.world_transform()[:3, 3] for body in self.ref_body_e]).flatten()
        self.prev_ref_p_e_ori_hat = [body.world_transform()[:3, :3] for body in self.ref_body_e]
        self.prev_ref_com = self.ref_skel.com()
        self.prev_ref_com_vel = self.ref_skel.com_velocity()
        self.prev_ref_com_spatial_vel = self.ref_skel.com_spatial_velocity()
        self.prev_ref_torso_ori = self.ref_skel.body('Spine').world_transform()[:3, :3]
        self.prev_ref_ankle_joint_y_pos = np.asarray([joint.get_world_frame_after_transform()[1, 3] for joint in [self.ref_skel.joint(joint_name) for joint_name in self.ankle_joint_names]]).flatten()

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
        action_num = self.skel.num_dofs() - 6 + len(self.foot_joint)

        state_high = np.array([np.finfo(np.float32).max] * state_num)
        action_high = np.array([pi*10.] * action_num)

        self.action_space = gym.spaces.Box(-action_high, action_high, dtype=np.float32)
        self.observation_space = gym.spaces.Box(-state_high, state_high, dtype=np.float32)

        self.force_done = False

    def state(self):
        pelvis = self.skel.body(0)
        p_pelvis = pelvis.world_transform()[:3, 3]
        R_pelvis = pelvis.world_transform()[:3, :3]

        phase = min(1., (self.world.time() + self.time_offset)/self.motion_time)
        if self.env_name == 'walk_repeated':
            if self.phase_frame <= 156:
                phase = self.phase_frame / 156.
            else:
                phase = ((self.phase_frame - 113) % 43 + 113)/156.

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
        body_e = list(map(self.skel.body, self.idx_e))

        p_e = np.asarray([body.world_transform()[:3, 3] for body in body_e]).flatten()

        p_e_ori = [body.world_transform()[:3, :3] for body in body_e]
        # p_e_ori_diff = np.asarray([mm.logSO3(np.dot(p_e_ori[idx].T, self.prev_ref_p_e_ori_hat[idx])) for idx in range(len(p_e_ori))]).flatten()

        q_diff = np.asarray(self.skel.position_differences(self.prev_ref_q, self.skel.q))
        dq_diff = np.asarray(self.skel.velocity_differences(self.prev_ref_dq, self.skel.dq))

        rewards = list()

        # q reward
        # rewards.append(exp_reward_term(self.w_p, self.exp_p,
        #                            np.concatenate(list(q_diff[get_joint_dof_range(self.skel.joint(joint_name))] for joint_name in self.reward_joint))))
        rewards.append(exp_reward_term(self.w_p, self.exp_p, q_diff))

        # dq reward
        # rewards.append(exp_reward_term(self.w_v, self.exp_v,
        #                             np.concatenate(list(dq_diff[get_joint_dof_range(self.skel.joint(joint_name))] for joint_name in self.reward_joint))))
        rewards.append(exp_reward_term(self.w_v, self.exp_v, dq_diff))

        # end effector reward
        rewards.append(exp_reward_term(self.w_e, self.exp_e, p_e - self.prev_ref_p_e_hat))

        # end effector orientation reward
        # rewards.append(exp_reward_term(self.w_e_ori, self.exp_e_ori, p_e_ori_diff))

        # com reward
        rewards.append(exp_reward_term(self.w_c, self.exp_c, self.skel.com() - self.prev_ref_com))

        # com_vel reward
        # rewards.append(exp_reward_term(self.w_c_v, self.exp_c_v, self.skel.com_velocity() - self.prev_ref_com_vel))
        # rewards.append(exp_reward_term(self.w_c_v, self.exp_c_v, self.skel.com_spatial_velocity() - self.prev_ref_com_spatial_vel))

        # torso reward
        torso_ori = self.skel.body('Spine').world_transform()[:3, :3]
        torso_ori_diff = np.asarray(mm.logSO3(np.dot(torso_ori.T, self.prev_ref_torso_ori)))
        rewards.append(exp_reward_term(self.w_t, self.exp_t, torso_ori_diff))

        # ankle y position reward
        # ankle_joint_y_pos = np.asarray([joint.get_world_frame_after_transform()[1, 3] for joint in [self.skel.joint(joint_name) for joint_name in self.ankle_joint_names]]).flatten()
        # rewards.append(exp_reward_term(self.w_y_ankle, self.exp_y_ankle, ankle_joint_y_pos - self.prev_ref_ankle_joint_y_pos))

        return sum(rewards)/(len(self.ref_motion) + self.ref_motion.fps)

    def is_done(self):
        if self.force_done:
            return True
        if (self.env_name == 'walk' or self.env_name == 'walk_repeated') and self.skel.com()[1] < 0.55:
            # print('fallen')
            return True
        elif self.skel.com()[1] < 0.4:
            # print('fallen')
            return True
        elif True in np.isnan(np.asarray(self.skel.q)) or True in np.isnan(np.asarray(self.skel.dq)):
            # print('nan')
            return True
        elif self.world.time() + self.time_offset > self.motion_time + 1.:
            # print('timeout')
            return True
        elif self.world.time() > 10.:
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
        Kp_vector = np.asarray([0.0] * 6 + [self.Kp] * (self.skel.ndofs - 6))
        Kd_vector = np.asarray([0.0] * 6 + [self.Kd] * (self.skel.ndofs - 6))
        for joint_idx in range(len(self.foot_joint)):
            for dof_idx in get_joint_dof_range(self.skel.joint(self.foot_joint[joint_idx])):
                Kp_vector[dof_idx] = self.Kp * exp(log(self.Kp) * _action[self.skel.ndofs-6 + joint_idx]/10.)
                Kd_vector[dof_idx] = self.Kd * exp(log(self.Kd) * _action[self.skel.ndofs-6 + joint_idx]/20.)

        for i in range(self.step_per_frame):
            # tau = self.skel.get_spd(self.ref_skel.q + action, self.world.time_step(), self.Kp, self.Kd)
            tau = self.skel.get_spd_extended(self.ref_skel.q + action, self.world.time_step(), Kp_vector, Kd_vector)
            # tau = self.skel.get_simple_spd_extended(self.ref_skel.q + action, self.world.time_step(), mass_mat_eig, Kp_vector, Kd_vector)
            self.skel.set_forces(tau)
            self.world.step()

        self.update_ref_skel(False)
        self.phase_frame += 1

        return tuple([self.state(), self.reward(), self.is_done(), {'kp': Kp_vector, 'kd': Kd_vector}])

    def update_ref_skel(self, reset=False):
        if not reset:
            self.prev_ref_q = self.ref_skel.positions()
            self.prev_ref_dq = self.ref_skel.velocities()
            self.prev_ref_p_e_hat = np.asarray([body.world_transform()[:3, 3] for body in self.ref_body_e]).flatten()
            self.prev_ref_p_e_ori_hat = [body.world_transform()[1, :3] for body in self.ref_body_e]
            self.prev_ref_com = self.ref_skel.com()
            self.prev_ref_com_vel = self.ref_skel.com_velocity()
            self.prev_ref_com_spatial_vel = self.ref_skel.com_spatial_velocity()
            self.prev_ref_torso_ori = self.ref_skel.body('Spine').world_transform()[:3, :3]
            self.prev_ref_ankle_joint_y_pos = np.asarray([joint.get_world_frame_after_transform()[1, 3] for joint in [self.ref_skel.joint(joint_name) for joint_name in self.ankle_joint_names]]).flatten()

        next_frame_time = self.world.time() + self.time_offset + self.world.time_step() * self.step_per_frame
        self.ref_skel.set_positions(self.ref_motion.get_q_by_time(next_frame_time))
        self.ref_skel.set_velocities(self.ref_motion.get_dq_dart_by_time(next_frame_time))

        if reset:
            self.prev_ref_q = self.ref_skel.positions()
            self.prev_ref_dq = self.ref_skel.velocities()
            self.prev_ref_p_e_hat = np.asarray([body.world_transform()[:3, 3] for body in self.ref_body_e]).flatten()
            self.prev_ref_p_e_ori_hat = [body.world_transform()[:3, :3] for body in self.ref_body_e]
            self.prev_ref_com = self.ref_skel.com()
            self.prev_ref_com_vel = self.ref_skel.com_velocity()
            self.prev_ref_com_spatial_vel = self.ref_skel.com_spatial_velocity()
            self.prev_ref_torso_ori = self.ref_skel.body('Spine').world_transform()[:3, :3]
            self.prev_ref_ankle_joint_y_pos = np.asarray([joint.get_world_frame_after_transform()[1, 3] for joint in [self.ref_skel.joint(joint_name) for joint_name in self.ankle_joint_names]]).flatten()

    def continue_from_now_by_phase(self, phase):
        self.continue_from_frame(round(phase*(self.motion_len-1)))
        # self.phase_frame = round(phase * (self.motion_len-1))
        # skel_pelvis_offset = self.skel.joint(0).position_in_world_frame() - self.ref_motion[self.phase_frame].getJointPositionGlobal(0)
        # skel_pelvis_offset[1] = 0.
        # self.ref_motion.translateByOffset(skel_pelvis_offset)
        # self.time_offset = - self.world.time() + (self.phase_frame / self.ref_motion.fps)

    def continue_from_frame(self, frame):
        self.phase_frame = frame
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

        # self.continue_from_now_by_phase(random() if self.rsi else 0.)
        self.continue_from_frame(randrange(self.motion_len) if self.rsi else 0)

        self.skel.set_positions(self.ref_motion.get_q(self.phase_frame))
        self.skel.set_velocities(self.ref_motion.get_dq_dart(self.phase_frame))

        self.update_ref_skel(True)

        self.force_done = False

        return self.state()

    def hard_reset(self):
        self.world = pydart.World(1./1200., "../data/wd2_seg.xml")
        self.world.control_skel = self.world.skeletons[1]
        self.skel = self.world.skeletons[1]
        return self.reset()

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
