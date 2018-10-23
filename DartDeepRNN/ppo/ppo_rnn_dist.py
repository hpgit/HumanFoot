# from rl.core import Agent
import numpy as np
from PyCommon.modules.Math import mmMath as mm
from math import pi, cos, acos, sin

import pydart2 as pydart
from DartDeepRNN.ppo.dart_cmu_env_v1 import HpDartEnv
from PyCommon.modules.dart.dart_ik import DartIk
from DartDeepRNN.rnn.RNNController import RNNController
from DartDeepRNN.util.Pose2d import Pose2d
from DartDeepRNN.util.Util import v_len

from itertools import count
import random
import time
import os

from multiprocessing import Process, Pipe

# from PyCommon.modules.NeuralNet.TorchBase import *
# import torch
# from torch import optim

import torch
from torch import nn, optim
import torch.nn.functional as F
import torchvision.transforms as T
from collections import namedtuple, deque
import random


MultiVariateNormal = torch.distributions.Normal
temp = MultiVariateNormal.log_prob
MultiVariateNormal.log_prob = lambda self, val: temp(self, val).sum(-1, keepdim=True)

temp2 = MultiVariateNormal.entropy
MultiVariateNormal.entropy = lambda self: temp2(self).sum(-1)
MultiVariateNormal.mode = lambda self: self.mean


class Model(nn.Module):
    def __init__(self, num_states, num_actions, hidden_layer):
        super(Model, self).__init__()

        hidden_layer_size1 = hidden_layer[0]
        hidden_layer_size2 = hidden_layer[1]
        hidden_layer_size3 = hidden_layer[2]

        '''Policy Mean'''
        self.policy_fc1 = nn.Linear(num_states		  , hidden_layer_size1)
        self.policy_fc2 = nn.Linear(hidden_layer_size1, hidden_layer_size2)
        self.policy_fc3 = nn.Linear(hidden_layer_size2, hidden_layer_size3)
        self.policy_fc4 = nn.Linear(hidden_layer_size3, num_actions)
        '''Policy Distributions'''
        self.log_std = nn.Parameter(torch.zeros(num_actions))

        '''Value'''
        self.value_fc1 = nn.Linear(num_states		  ,hidden_layer_size1)
        self.value_fc2 = nn.Linear(hidden_layer_size1 ,hidden_layer_size2)
        self.value_fc3 = nn.Linear(hidden_layer_size2 ,hidden_layer_size3)
        self.value_fc4 = nn.Linear(hidden_layer_size3 ,1)

        self.initParameters()

    def initParameters(self):
        '''Policy'''
        if self.policy_fc1.bias is not None:
            self.policy_fc1.bias.data.zero_()

        if self.policy_fc2.bias is not None:
            self.policy_fc2.bias.data.zero_()

        if self.policy_fc3.bias is not None:
            self.policy_fc3.bias.data.zero_()

        if self.policy_fc4.bias is not None:
            self.policy_fc4.bias.data.zero_()
        torch.nn.init.xavier_uniform_(self.policy_fc1.weight)
        torch.nn.init.xavier_uniform_(self.policy_fc2.weight)
        torch.nn.init.xavier_uniform_(self.policy_fc3.weight)
        torch.nn.init.xavier_uniform_(self.policy_fc4.weight)
        '''Value'''
        if self.value_fc1.bias is not None:
            self.value_fc1.bias.data.zero_()

        if self.value_fc2.bias is not None:
            self.value_fc2.bias.data.zero_()

        if self.value_fc3.bias is not None:
            self.value_fc3.bias.data.zero_()

        if self.value_fc4.bias is not None:
            self.value_fc4.bias.data.zero_()
        torch.nn.init.xavier_uniform_(self.value_fc1.weight)
        torch.nn.init.xavier_uniform_(self.value_fc2.weight)
        torch.nn.init.xavier_uniform_(self.value_fc3.weight)
        torch.nn.init.xavier_uniform_(self.value_fc4.weight)

    def forward(self, x):
        '''Policy'''
        p_mean = F.relu(self.policy_fc1(x))
        p_mean = F.relu(self.policy_fc2(p_mean))
        p_mean = F.relu(self.policy_fc3(p_mean))
        p_mean = self.policy_fc4(p_mean)

        p = MultiVariateNormal(p_mean, self.log_std.exp())
        '''Value'''
        v = F.relu(self.value_fc1(x))
        v = F.relu(self.value_fc2(v))
        v = F.relu(self.value_fc3(v))
        v = self.value_fc4(v)

        return p,v


Episode = namedtuple('Episode', ('s', 'a', 'r', 'value', 'logprob'))


class EpisodeBuffer(object):
    def __init__(self):
        self.data = []

    def push(self, *args):
        self.data.append(Episode(*args))

    def get_data(self):
        return self.data


Transition = namedtuple('Transition', ('s', 'a', 'logprob', 'TD', 'GAE'))


class ReplayBuffer(object):
    def __init__(self, buff_size=10000):
        super(ReplayBuffer, self).__init__()
        self.buffer = deque(maxlen=buff_size)

    def sample(self, batch_size):
        index_buffer = [i for i in range(len(self.buffer))]
        indices = random.sample(index_buffer, batch_size)
        return indices, self.np_buffer[indices]

    def push(self, *args):
        self.buffer.append(Transition(*args))

    def clear(self):
        self.buffer.clear()


def worker(rnn_len, proc_num, state_sender, result_sender, action_receiver, reset_receiver, motion_receiver):
    """

    :type rnn_len: int
    :type proc_num: int
    :type result_sender: Connection
    :type state_sender: Connection
    :type action_receiver: Connection
    :type reset_receiver: Connection
    :type motion_receiver: Connection
    :return:
    """

    # reset variable
    # 0 : go on (no reset)
    # 1 : soft reset ( w/o motion change )
    # 2 : hard reset ( with motion change )

    env = HpDartEnv(rnn_len)

    state = None
    while True:
        reset_flag = reset_receiver.recv()
        if reset_flag == 1:
            state = env.reset()
        elif reset_flag == 2:
            goals, qs = motion_receiver.recv()
            env.update_target(goals, qs)
            state = env.reset()

        state_sender.send(state)
        action = action_receiver.recv()
        state, reward, is_done, _ = env.step(action)
        result_sender.send((reward, is_done))


class PPO(object):
    def __init__(self, env_name, num_slaves=1, eval_print=True, eval_log=True, visualize_only=False):
        np.random.seed(seed=int(time.time()))
        self.env_name = env_name
        self.rnn_len = 150
        self.env = HpDartEnv(self.rnn_len)
        self.num_slaves = num_slaves
        self.num_state = self.env.observation_space.shape[0]
        self.num_action = self.env.action_space.shape[0]
        self.num_epochs = 10
        self.num_evaluation = 0
        self.num_training = 0

        self.gamma = 0.99
        self.lb = 0.95
        self.clip_ratio = 0.2

        self.buffer_size = 2048
        self.batch_size = 128
        self.replay_buffer = ReplayBuffer(10000)

        self.total_episodes = []

        self.model = Model(self.num_state, self.num_action, (256, 256, 128)).float()
        self.optimizer = optim.Adam(self.model.parameters(), lr=7E-4)
        # self.optimizer = optim.Adam(self.model.parameters(), lr=5E-3)
        self.w_entropy = 0.0

        self.saved = False
        self.save_directory = self.env_name + '_' + 'model_'+time.strftime("%m%d%H%M") + '/'
        if not self.saved and not os.path.exists(self.save_directory) and not visualize_only:
            os.makedirs(self.save_directory)
            self.saved = True

        self.log_file = None
        if not visualize_only:
            self.log_file = open(self.save_directory + 'log.txt', 'w')
        self.eval_print = eval_print
        self.eval_log = eval_log

        self.state_sender = []  # type: list[Connection]
        self.result_sender = []  # type: list[Connection]
        self.state_receiver = []  # type: list[Connection]
        self.result_receiver = []  # type: list[Connection]
        self.action_sender = []  # type: list[Connection]
        self.reset_sender = []  # type: list[Connection]
        self.motion_sender = []  # type: list[Connection]
        self.envs = []  # type: list[Process]

        self.ik_world = pydart.World(1./1200., '../data/cmu_with_ground.xml')
        self.ik_skel = self.ik_world.skeletons[1]
        self.rnn = RNNController(env_name)
        self.RNN_MOTION_SCALE = 0.01
        self.rnn_joint_list = ["Head", "Hips", "LHipJoint", "LeftArm", "LeftFoot", "LeftForeArm", "LeftHand",
                               "LeftLeg", "LeftShoulder", "LeftToeBase", "LeftUpLeg", "LowerBack", "Neck", "Neck1",
                               "RHipJoint", "RightArm", "RightFoot","RightForeArm","RightHand",
                               "RightLeg","RightShoulder","RightToeBase","RightUpLeg",
                               "Spine","Spine1"]
        self.rnn_target_update_prob = 2./self.rnn_len
        self.ik = DartIk(self.ik_skel)
        self.goal_in_world_frame = np.zeros(3)

        self.qs = list()
        # self.replace_motion_num = self.rnn_len//5
        self.replace_motion_num = 1
        # self.sample_target()
        self.goal_in_world_frame = np.array((5., 0., 0.))

        # for i in range(6):
        # for i in range(1):
        for i in range(20):
            # for warming
            self.get_rnn_ref_pose_step()

        # for i in range(10):
        #     self.qs.append((np.zeros(3), self.ik_skel.positions()))

        for i in range(self.rnn_len-0):
            root_body_pos = self.ik_skel.body(0).to_world()
            root_body_pos[1] = 0.
            to_goal_len = mm.length(self.goal_in_world_frame - root_body_pos)
            # if random.random() < 2./self.rnn_len or to_goal_len < 0.1:
            if to_goal_len < 0.1:
                self.sample_target()
            self.get_rnn_ref_pose_step()
            self.qs.append((self.goal_in_world_frame, self.ik_skel.positions()))

        self.init_envs()

    def init_envs(self):
        for slave_idx in range(self.num_slaves):
            s_s, s_r = Pipe()
            r_s, r_r = Pipe()
            a_s, a_r = Pipe()
            reset_s, reset_r = Pipe()
            motion_s, motion_r = Pipe()
            p = Process(target=worker, args=(self.rnn_len, slave_idx, s_s, r_s, a_r, reset_r, motion_r))
            self.state_sender.append(s_s)
            self.result_sender.append(r_s)
            self.state_receiver.append(s_r)
            self.result_receiver.append(r_r)
            self.action_sender.append(a_s)
            self.reset_sender.append(reset_s)
            self.motion_sender.append(motion_s)
            self.envs.append(p)
            p.start()

    def envs_get_states(self, terminated):
        states = []
        for recv_idx in range(len(self.state_receiver)):
            if terminated[recv_idx]:
                states.append([0.] * self.num_state)
            else:
                states.append(self.state_receiver[recv_idx].recv())
        return states

    def envs_send_actions(self, actions, terminated):
        for i in range(len(self.action_sender)):
            if not terminated[i]:
                self.action_sender[i].send(actions[i])

    def envs_get_status(self, terminated):
        status = []
        for recv_idx in range(len(self.result_receiver)):
            if terminated[recv_idx]:
                status.append((0., True))
            else:
                status.append(self.result_receiver[recv_idx].recv())
        return zip(*status)

    def envs_resets(self, reset_flag):
        for i in range(len(self.reset_sender)):
            self.reset_sender[i].send(reset_flag)

    def envs_reset(self, i, reset_flag):
        self.reset_sender[i].send(reset_flag)

    def envs_send_rnn_motion(self):
        for motion_sender in self.motion_sender:
            motion_sender.send(zip(*self.qs))
        goals, qs = zip(*self.qs)
        self.env.update_target(goals, qs)

    def sample_target(self):
        angle = 2. * pi * random.random()
        radius = 4. * random.random() + 1.

        self.goal_in_world_frame = self.ik_skel.body(0).to_world() + radius * (cos(angle) * mm.unitX() - sin(angle) * mm.unitZ())
        self.goal_in_world_frame[1] = 0.

    def get_rnn_ref_pose_step(self):
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

        for j in range(len(self.ik_skel.joints)):
            if j == 0:
                joint = self.ik_skel.joints[j]  # type: pydart.FreeJoint
                joint_idx = self.rnn_joint_list.index(joint.name)
                hip_angles = mm.logSO3(np.dot(root_orientation, orientations[joint_idx]))
                # hip_angles = mm.logSO3(root_orientation)
                joint.set_position(np.array([hip_angles[0], hip_angles[1], hip_angles[2], points[0][0], points[0][1], points[0][2]]))
                continue
            joint = self.ik_skel.joints[j]  # type: pydart.BallJoint
            joint_idx = self.rnn_joint_list.index(joint.name)
            joint.set_position(angles[joint_idx*3:joint_idx*3+3])

        self.ik.clean_constraints()
        self.ik.add_joint_pos_const('Hips', np.asarray(points[0]))

        self.ik.add_joint_pos_const('LeftForeArm', np.asarray(points[10]))
        self.ik.add_joint_pos_const('LeftHand', np.asarray(points[2]))
        self.ik.add_joint_pos_const('LeftLeg', np.asarray(points[11]))
        self.ik.add_joint_pos_const('LeftFoot', np.asarray(points[3]))
        if contacts[0] > 0.8 and False:
            body_transform = self.ik_skel.body('LeftFoot').transform()[:3, :3]
            angle = acos(body_transform[1, 1])
            body_ori = np.dot(body_transform, mm.rotX(-angle))
            self.ik.add_orientation_const('LeftFoot', body_ori)

        self.ik.add_joint_pos_const('RightForeArm', np.asarray(points[12]))
        self.ik.add_joint_pos_const('RightHand', np.asarray(points[5]))
        self.ik.add_joint_pos_const('RightLeg', np.asarray(points[13]))
        self.ik.add_joint_pos_const('RightFoot', np.asarray(points[6]))

        self.ik.add_joint_pos_const('Neck1', np.asarray(points[17]))

        self.ik.solve()

        foot_joint_ori = mm.exp(self.ik_skel.joint('LeftFoot').position())
        self.ik_skel.joint('LeftFoot').set_position(mm.logSO3(np.dot(foot_joint_ori, np.dot(mm.rotX(-.6), mm.rotZ(.4)))))
        foot_joint_ori = mm.exp(self.ik_skel.joint('RightFoot').position())
        self.ik_skel.joint('RightFoot').set_position(mm.logSO3(np.dot(foot_joint_ori, np.dot(mm.rotX(-.6), mm.rotZ(-.4)))))

        left_foot = self.ik_skel.body('LeftFoot')

        if (left_foot.to_world([0.05, -0.045, 0.1125])[1] < 0. or left_foot.to_world([-0.05, -0.045, 0.1125])[1] < 0.) \
                and (left_foot.to_world([0.05, -0.045, -0.1125])[1] < 0. or left_foot.to_world([-0.05, -0.045, -0.1125])[1] < 0.):

            left_toe_pos1 = left_foot.to_world([0.05, -0.045, +0.1125])
            left_toe_pos1[1] = 0.
            left_toe_pos2 = left_foot.to_world([-0.05, -0.045, +0.1125])
            left_toe_pos2[1] = 0.

            left_heel_pos1 = left_foot.to_world([0.05, -0.045, -0.1125])
            left_heel_pos1[1] = 0.
            left_heel_pos2 = left_foot.to_world([-0.05, -0.045, -0.1125])
            left_heel_pos2[1] = 0.

            self.ik.clean_constraints()
            self.ik.add_position_const('LeftFoot', left_toe_pos1, np.array([0.05, -0.045, +0.1125]))
            self.ik.add_position_const('LeftFoot', left_toe_pos2, np.array([-0.05, -0.045, +0.1125]))
            self.ik.add_position_const('LeftFoot', left_heel_pos1, np.array([0.05, -0.045, -0.1125]))
            self.ik.add_position_const('LeftFoot', left_heel_pos2, np.array([-0.05, -0.045, -0.1125]))
            self.ik.solve()

        right_foot = self.ik_skel.body('RightFoot')

        if (right_foot.to_world([0.05, -0.045, 0.1125])[1] < 0. or right_foot.to_world([-0.05, -0.045, 0.1125])[1] < 0.) \
                and (right_foot.to_world([0.05, -0.045, -0.1125])[1] < 0. or right_foot.to_world([-0.05, -0.045, -0.1125])[1] < 0.):

            right_toe_pos1 = right_foot.to_world([0.05, -0.045, +0.1125])
            right_toe_pos1[1] = 0.
            right_toe_pos2 = right_foot.to_world([-0.05, -0.045, +0.1125])
            right_toe_pos2[1] = 0.

            right_heel_pos1 = right_foot.to_world([0.05, -0.045, -0.1125])
            right_heel_pos1[1] = 0.
            right_heel_pos2 = right_foot.to_world([-0.05, -0.045, -0.1125])
            right_heel_pos2[1] = 0.

            self.ik.clean_constraints()
            self.ik.add_position_const('RightFoot', right_toe_pos1, np.array([0.05, -0.045, +0.1125]))
            self.ik.add_position_const('RightFoot', right_toe_pos2, np.array([-0.05, -0.045, +0.1125]))
            self.ik.add_position_const('RightFoot', right_heel_pos1, np.array([0.05, -0.045, -0.1125]))
            self.ik.add_position_const('RightFoot', right_heel_pos2, np.array([-0.05, -0.045, -0.1125]))
            self.ik.solve()

    def generate_rnn_motion(self):
        return
        del self.qs[:self.replace_motion_num]
        for i in range(self.replace_motion_num):
            # goal, 'next' pose
            root_body_pos = self.ik_skel.body(0).to_world()
            root_body_pos[1] = 0.
            to_goal_len = mm.length(self.goal_in_world_frame - root_body_pos)
            # if random.random() < 2./self.rnn_len or to_goal_len < 0.1:
            if to_goal_len < 0.1:
                self.sample_target()
            self.get_rnn_ref_pose_step()
            self.qs.append((self.goal_in_world_frame.copy(), self.ik_skel.positions()))

    def SaveModel(self):
        torch.save(self.model.state_dict(), self.save_directory + str(self.num_evaluation) + '.pt')

    def LoadModel(self, model_path):
        self.model.load_state_dict(torch.load(model_path))

    def ComputeTDandGAE(self):
        self.replay_buffer.clear()
        for epi in self.total_episodes:
            data = epi.get_data()
            size = len(data)
            states, actions, rewards, values, logprobs = zip(*data)

            values = np.concatenate((values, np.zeros(1)), axis=0)
            advantages = np.zeros(size)
            ad_t = 0

            for i in reversed(range(len(data))):
                delta = rewards[i] + values[i + 1] * self.gamma - values[i]
                ad_t = delta + self.gamma * self.lb * ad_t
                advantages[i] = ad_t

            TD = values[:size] + advantages
            for i in range(size):
                self.replay_buffer.push(states[i], actions[i], logprobs[i], TD[i], advantages[i])

    def GenerateTransitions(self):
        del self.total_episodes[:]
        states = [None] * self.num_slaves
        actions = [None] * self.num_slaves
        rewards = [None] * self.num_slaves
        states_next = [None] * self.num_slaves
        episodes = [None] * self.num_slaves
        for j in range(self.num_slaves):
            episodes[j] = EpisodeBuffer()

        self.envs_resets(2)
        self.envs_send_rnn_motion()
        # self.env.Resets(False)

        local_step = 0
        terminated = [False] * self.num_slaves
        # print('Generate Transtions...')

        percent = 0
        while True:
            # update states
            states = self.envs_get_states(terminated)
            # states = self.env.GetStates()

            # new_percent = local_step*10//self.buffer_size
            # if (new_percent == percent) is not True:
            # percent = new_percent
            # print('{}0%'.format(percent))
            a_dist, v = self.model(torch.tensor(states).float())
            actions = a_dist.sample().detach().numpy()
            logprobs = a_dist.log_prob(torch.tensor(actions).float()).detach().numpy().reshape(-1)
            values = v.detach().numpy().reshape(-1)

            self.envs_send_actions(actions, terminated)
            rewards, is_done = self.envs_get_status(terminated)
            # self.env.Steps(actions)
            # rewards, is_done = self.env.GetRewards(), self.env.IsTerminalStates()

            for j in range(self.num_slaves):
                if terminated[j]:
                    continue

                nan_occur = np.any(np.isnan(states[j])) or np.any(np.isnan(actions[j]))
                if not nan_occur:
                    episodes[j].push(states[j], actions[j], rewards[j], values[j], logprobs[j])
                    local_step += 1

                # if episode is terminated
                if is_done[j] or nan_occur:
                    if not is_done[j] and nan_occur:
                        self.print('!!!!!!!!!!!!!!!!!!!!!!!!exception')
                    # push episodes
                    self.total_episodes.append(episodes[j])

                    # if data limit is exceeded, stop simulations
                    if local_step < self.buffer_size:
                        episodes[j] = EpisodeBuffer()
                        self.envs_reset(j, 1)
                        # self.env.Reset(True, j)
                    else:
                        terminated[j] = True
                else:
                    self.envs_reset(j, 0)
                    pass

            if local_step >= self.buffer_size:
                if all(terminated):
                    break
        self.generate_rnn_motion()

    # print('Done!')
    def OptimizeModel(self):
        # print('Optimize Model...')
        self.ComputeTDandGAE()
        all_transitions = np.array(self.replay_buffer.buffer)

        for _ in range(self.num_epochs):
            np.random.shuffle(all_transitions)
            for i in range(len(all_transitions) // self.batch_size):
                transitions = all_transitions[i * self.batch_size:(i + 1) * self.batch_size]
                batch = Transition(*zip(*transitions))

                stack_s = np.vstack(batch.s).astype(np.float32)
                stack_a = np.vstack(batch.a).astype(np.float32)
                stack_lp = np.vstack(batch.logprob).astype(np.float32)
                stack_td = np.vstack(batch.TD).astype(np.float32)
                stack_gae = np.vstack(batch.GAE).astype(np.float32)

                a_dist, v = self.model(torch.tensor(stack_s).float())
                '''Critic Loss'''
                loss_critic = ((v - torch.tensor(stack_td).float()).pow(2)).mean()

                '''Actor Loss'''
                ratio = torch.exp(a_dist.log_prob(torch.tensor(stack_a).float()) - torch.tensor(stack_lp).float())
                stack_gae = (stack_gae - stack_gae.mean()) / (stack_gae.std() + 1E-5)
                surrogate1 = ratio * torch.tensor(stack_gae).float()
                surrogate2 = torch.clamp(ratio, min=1.0 - self.clip_ratio, max=1.0 + self.clip_ratio) * torch.tensor(stack_gae).float()
                loss_actor = - torch.min(surrogate1, surrogate2).mean()

                '''Entropy Loss'''
                loss_entropy = - self.w_entropy * a_dist.entropy().mean()

                loss = loss_critic + loss_actor + loss_entropy
                # loss = loss_critic + loss_actor
                self.optimizer.zero_grad()
                loss.backward(retain_graph=True)
                for param in self.model.parameters():
                    param.grad.data.clamp_(-0.5, 0.5)
                self.optimizer.step()

    # print('Done!')
    def Train(self):
        self.GenerateTransitions()
        self.OptimizeModel()
        self.num_training += 1

    def Evaluate(self):
        self.num_evaluation += 1
        total_reward = 0
        total_step = 0
        self.env.Resets(True)
        self.env.Reset(False, 0)

        for t in count():
            states = self.env.GetStates()
            for j in range(len(states)):
                if np.any(np.isnan(states[j])):
                    self.print("state warning!!!!!!!! start")

            action_dist, _ = self.model(torch.tensor(states).float())
            actions = action_dist.loc.detach().numpy()
            for j in range(len(actions)):
                if np.any(np.isnan(actions[j])):
                    self.print("action warning!!!!!!!!" + str(t))

            self.env.Steps(actions)

            for j in range(len(states)):
                if np.any(np.isnan(states[j])):
                    self.print("state warning!!!!!!!!"+str(t))

            if not self.env.IsTerminalState(0):
                total_step += 1
                reward = self.env.GetReward(0)
                total_reward += reward
            else:
                break

        self.print('noise : {:.3f}'.format(self.model.log_std.exp().mean()))
        if total_step is not 0:
            self.print('Epi reward : {:.2f}, Step reward : {:.2f} Total step : {}'
                  .format(total_reward, total_reward / total_step, total_step))
        else:
            self.print('bad..')
        return total_reward, total_step

    def print(self, s):
        if self.eval_print:
            print(s)
        if self.eval_log:
            self.log_file.write(s+"\n")

import matplotlib
import matplotlib.pyplot as plt
plt.ion()


def Plot(y,title,num_fig=1,ylim=True):
    global glob_count_for_plt

    plt.figure(num_fig)
    plt.clf()
    plt.title(title)
    plt.plot(y)
    plt.show()
    if ylim:
        plt.ylim([0,1])
    plt.pause(0.001)

import argparse


if __name__ == "__main__":
    import sys
    pydart.init()
    tic = time.time()
    ppo = None  # type: PPO
    if len(sys.argv) < 2:
        ppo = PPO('walk', 4)
    else:
        ppo = PPO(sys.argv[1], int(sys.argv[2]))

    if len(sys.argv) > 3:
        print("load {}".format(sys.argv[3]))
        ppo.LoadModel(sys.argv[3])

    # parser = argparse.ArgumentParser()
    # parser.add_argument('-m','--model',help='actor model directory')
    # args =parser.parse_args()
    # if args.model is not None:
    #     print("load {}".format(args.model))
    #     ppo.LoadModel(args.model)
    rewards = []
    steps = []
    # print('num states: {}, num actions: {}'.format(ppo.env.GetNumState(),ppo.env.GetNumAction()))

    max_avg_steps = 0

    for i in range(50000):
    # for i in range(1):
        ppo.Train()
        print('# {}'.format(i+1))
        ppo.log_file.write('# {}'.format(i+1) + "\n")
        reward, step = ppo.Evaluate()
        rewards.append(reward)
        steps.append(step)
        if i % 10 == 0 or max_avg_steps < step:
            ppo.SaveModel()
            if max_avg_steps < step:
                max_avg_steps = step

        # Plot(np.asarray(rewards), 'reward', 1, False)
        print("Elapsed time : {:.2f}s".format(time.time() - tic))
        ppo.log_file.write("Elapsed time : {:.2f}s".format(time.time() - tic) + "\n")
        ppo.log_file.flush()
