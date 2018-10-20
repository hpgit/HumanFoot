# from rl.core import Env
import pydart2 as pydart
import numpy as np
from math import exp, pi, cos, sin, acos
from PyCommon.modules.Math import mmMath as mm
from random import randrange, random
import gym
import gym.spaces
from gym.utils import seeding

import PyCommon.modules.Resource.ysMotionLoader as yf
from PyCommon.modules.dart.dart_ik import DartIk
from DartDeep.pd_controller import PDController
from DartDeepRNN.rnn.RNNController import RNNController
from DartDeepRNN.util.Pose2d import Pose2d
from DartDeepRNN.util.Util import v_len


def exp_reward_term(w, exp_w, v0, v1):
    norm = np.linalg.norm(v0 - v1)
    return w * exp(-exp_w * norm * norm)


class HpDartEnv(gym.Env):
    def __init__(self, env_name='walk'):
        self.skel_file_name = '../data/cmu_with_ground.xml'
        self.world = pydart.World(1./1200., self.skel_file_name)
        self.world.control_skel = self.world.skeletons[1]
        self.skel = self.world.skeletons[1]
        self.Kp, self.Kd = 400., 40.
        self.pdc = PDController(self.skel, self.world.time_step(), 400., 40.)

        self.env_name = env_name

        self.ref_world = pydart.World(1./1200., self.skel_file_name)
        self.ref_skel = self.ref_world.skeletons[1]
        self.ref_prev_q = self.ref_skel.positions()
        self.step_per_frame = 40

        self.rnn = RNNController(env_name)
        self.RNN_MOTION_SCALE = 0.01
        self.rnn_joint_list = ["Head", "Hips", "LHipJoint", "LeftArm", "LeftFoot", "LeftForeArm", "LeftHand",
                               "LeftLeg", "LeftShoulder", "LeftToeBase", "LeftUpLeg", "LowerBack", "Neck", "Neck1",
                               "RHipJoint", "RightArm", "RightFoot","RightForeArm","RightHand",
                               "RightLeg","RightShoulder","RightToeBase","RightUpLeg",
                               "Spine","Spine1"]
        self.rnn_target_update_prob = 1./150.
        self.ik = DartIk(self.ref_skel)

        self.rsi = True

        self.w_g = 0.3
        self.w_p = 0.65 * .7
        self.w_v = 0.1 * .7
        self.w_e = 0.15 * .7
        self.w_c = 0.1 * .7

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

        self.time_offset = 0.
        self.step_num = 0

        state_num = 2 + (3*3 + 4) * self.body_num
        action_num = self.skel.num_dofs() - 6

        state_high = np.array([np.finfo(np.float32).max] * state_num)
        action_high = np.array([pi*10./2.] * action_num)

        self.action_space = gym.spaces.Box(-action_high, action_high, dtype=np.float32)
        self.observation_space = gym.spaces.Box(-state_high, state_high, dtype=np.float32)

        self.viewer = None

        self.phase_frame = 0
        self.goal = np.zeros(2)
        self.goal_in_world_frame = np.zeros(3)

        self.first = True

    def state(self):
        pelvis = self.skel.body(0)
        p_pelvis = pelvis.world_transform()[:3, 3]
        R_pelvis = pelvis.world_transform()[:3, :3]

        state = [self.goal[0]/5., self.goal[1]/5.]

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
        p_e_hat = np.asarray([body.world_transform()[:3, 3] for body in self.ref_body_e]).flatten()
        p_e = np.asarray([body.world_transform()[:3, 3] for body in self.body_e]).flatten()

        return exp_reward_term(self.w_p, self.exp_p, self.skel.q, self.ref_skel.q) \
              + exp_reward_term(self.w_v, self.exp_v, self.skel.dq, self.ref_skel.dq) \
              + exp_reward_term(self.w_e, self.exp_e, p_e, p_e_hat) \
              + exp_reward_term(self.w_c, self.exp_c, self.skel.com(), self.ref_skel.com()) #\
              # + exp_reward_term(self.w_g, self.exp_g, self.skel.body(0).to_world(), self.goal_in_world_frame)

    def is_done(self):
        if self.skel.com()[1] < 0.4:
            # print('fallen')
            return True
        elif True in np.isnan(np.asarray(self.skel.q)) or True in np.isnan(np.asarray(self.skel.dq)):
            # print('nan')
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
        action = np.hstack((np.zeros(6), _action/10.))
        self.get_rnn_ref_pose_step()
        for i in range(self.step_per_frame):
            # self.skel.set_forces(self.skel.get_spd(self.ref_skel.q + action, self.world.time_step(), self.Kp, self.Kd))
            self.skel.set_forces(self.pdc.compute_flat(self.ref_skel.q + action))
            self.world.step()

        return_val = tuple([self.state(), self.reward(), self.is_done(), dict()])

        if random() < self.rnn_target_update_prob:
            self.sample_target()
        else:
            self.update_goal_in_local_frame()

        return return_val

    def reset(self):
        """
        Resets the state of the environment and returns an initial observation.

        # Returns
            observation (object): The initial observation of the space. Initial reward is assumed to be 0.
        """
        self.world.reset()
        if self.first:
            self.sample_target()
        self.get_rnn_ref_pose_step()
        self.skel.set_positions(self.ref_skel.positions())
        self.skel.set_velocities(self.ref_skel.velocities())
        self.first = False

        return self.state()

    def sample_target(self):
        angle = 2. * pi * random()
        radius = 5. * random()

        body_transform = self.skel.body(0).world_transform()
        root_x_in_world_plane = body_transform[:3, 0]
        root_x_in_world_plane[1] = 0.
        unit_root_x_in_world_plane = mm.seq2Vec3(mm.normalize(root_x_in_world_plane))
        unit_root_z_in_world_plane = mm.cross(unit_root_x_in_world_plane, mm.unitY())
        self.goal_in_world_frame = body_transform[:3, 3] + radius * (sin(angle) * unit_root_x_in_world_plane + cos(angle) * unit_root_z_in_world_plane)
        self.goal_in_world_frame[1] = 0.

        self.goal = radius * np.array([sin(angle), cos(angle)])

    def update_goal_in_local_frame(self):
        body_transform = self.skel.body(0).world_transform()
        goal_vector_in_world_frame = self.goal_in_world_frame - body_transform[:3, 3]
        goal_vector_in_world_frame[1] = 0.
        radius = mm.length(goal_vector_in_world_frame)
        unit_goal_vector_in_world_frame = mm.normalize(goal_vector_in_world_frame)
        root_x_in_world_plane = body_transform[:3, 0]
        root_x_in_world_plane[1] = 0.
        unit_root_x_in_world_plane = mm.seq2Vec3(mm.normalize(root_x_in_world_plane))
        unit_root_z_in_world_plane = mm.cross(unit_root_x_in_world_plane, mm.unitY())
        # angle = atan2(np.dot(unit_root_x_in_world_plane, unit_goal_vector_in_world_frame), np.dot(unit_root_z_in_world_plane, unit_goal_vector_in_world_frame))

        self.goal = radius * np.array([np.dot(unit_root_x_in_world_plane, unit_goal_vector_in_world_frame), np.dot(unit_root_z_in_world_plane, unit_goal_vector_in_world_frame)])

    def get_rnn_ref_pose_step(self):
        self.ref_prev_q = self.ref_skel.q
        p = self.goal_in_world_frame

        target = Pose2d([p[0]/self.RNN_MOTION_SCALE, -p[2]/self.RNN_MOTION_SCALE])
        target = self.rnn.pose.relativePose(target)
        target = target.p
        t_len = v_len(target)
        if t_len > 80:
            ratio = 80/t_len
            target[0] *= ratio
            target[1] *= ratio

        contacts, points, angles, orientations, root_orientation = self.rnn.step(target)

        for j in range(len(self.ref_skel.joints)):
            if j == 0:
                joint = self.ref_skel.joints[j]  # type: pydart.FreeJoint
                joint_idx = self.rnn_joint_list.index(joint.name)
                hip_angles = mm.logSO3(np.dot(root_orientation, orientations[joint_idx]))
                # hip_angles = mm.logSO3(root_orientation)
                joint.set_position(np.array([hip_angles[0], hip_angles[1], hip_angles[2], points[0][0], points[0][1], points[0][2]]))
                continue
            joint = self.ref_skel.joints[j]  # type: pydart.BallJoint
            joint_idx = self.rnn_joint_list.index(joint.name)
            joint.set_position(angles[joint_idx*3:joint_idx*3+3])

        self.ik.clean_constraints()
        self.ik.add_joint_pos_const('LeftForeArm', np.asarray(points[10]))
        self.ik.add_joint_pos_const('LeftHand', np.asarray(points[2]))
        self.ik.add_joint_pos_const('LeftLeg', np.asarray(points[11]))
        self.ik.add_joint_pos_const('LeftFoot', np.asarray(points[3]))
        if contacts[0] > 0.8 and False:
            body_transform = self.ref_skel.body('LeftFoot').transform()[:3, :3]
            angle = acos(body_transform[1, 1])
            body_ori = np.dot(body_transform, mm.rotX(-angle))
            self.ik.add_orientation_const('LeftFoot', body_ori)

        self.ik.add_joint_pos_const('RightForeArm', np.asarray(points[12]))
        self.ik.add_joint_pos_const('RightHand', np.asarray(points[5]))
        self.ik.add_joint_pos_const('RightLeg', np.asarray(points[13]))
        self.ik.add_joint_pos_const('RightFoot', np.asarray(points[6]))
        self.ik.solve()

        foot_joint_ori = mm.exp(self.ref_skel.joint('LeftFoot').position())
        self.ref_skel.joint('LeftFoot').set_position(mm.logSO3(np.dot(foot_joint_ori, np.dot(mm.rotX(-.6), mm.rotZ(.4)))))
        foot_joint_ori = mm.exp(self.ref_skel.joint('RightFoot').position())
        self.ref_skel.joint('RightFoot').set_position(mm.logSO3(np.dot(foot_joint_ori, np.dot(mm.rotX(-.6), mm.rotZ(-.4)))))

        if not self.first:
            dq = 30. * self.ref_skel.position_differences(self.ref_skel.positions(), self.ref_prev_q)
            self.ref_skel.set_velocities(dq)

    def render(self, mode='human', close=False):
        """Renders the environment.
        The set of supported modes varies per environment. (And some
        environments do not support rendering at all.)

        # Arguments
            mode (str): The mode to render with.
            close (bool): Close all open renderings.
        """
        return

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
