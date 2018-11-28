import numpy as np

import pydart2 as pydart
from DartFootDeep.sh_v3.dart_env_v3 import HpDartEnv

from itertools import count
import time
import os

from multiprocessing import Process, Pipe, Manager, Lock
from threading import Timer
import copy

import torch
from torch import optim
from DartFootDeep.sh_v3.TorchNN import Model, Episode, EpisodeBuffer, Transition, ReplayBuffer


DEBUG = False


def worker(env_num, episode_buffer_size, total_episodes, terminal, episodes_lock, terminal_lock, run_lock, netparam_receiver):
    """

    :type env_num: int
    :type episode_buffer_size: int
    :type total_episodes: list
    :type terminal: list
    :type episodes_lock: Lock
    :type terminal_lock: Lock
    :type run_lock: Lock
    :type netparam_receiver: Connection
    :return:
    """
    env = HpDartEnv()

    net_param_state_dict = netparam_receiver.recv()
    env.model.load_state_dict(net_param_state_dict)
    if DEBUG:
        run_lock.acquire()
        state_dict = env.model.state_dict()
        print('Env {} state dict:'.format(env_num))
        print(state_dict)
        run_lock.release()

    episode_done = False

    while True:
        run_lock.acquire()
        if DEBUG:
            print('Env {} start'.format(env_num))
        run_lock.release()

        env.run_edisode()
        episodes_lock.acquire()
        try:
            local_step = sum(epi.get_size() for epi in total_episodes)
            if DEBUG:
                print('Env{}, epi {} befor'.format(env_num, local_step))
            episode_done = local_step > episode_buffer_size

            if not episode_done:
                total_episodes.append(copy.deepcopy(env.episode))

            local_step = sum(epi.get_size() for epi in total_episodes)
            if DEBUG:
                print('Env{}, epi {} after'.format(env_num, local_step))
            episode_done = local_step > episode_buffer_size
        finally:
            episodes_lock.release()

        if episode_done:
            terminal_lock.acquire()
            terminal[env_num] = True
            terminal_lock.release()

            net_param_state_dict = netparam_receiver.recv()
            env.model.load_state_dict(net_param_state_dict)
        else:
            continue


class PPO(object):
    def __init__(self, env_name, num_slaves=1, eval_print=True, eval_log=True, visualize_only=False):
        np.random.seed(seed=int(time.time()))
        self.env_name = env_name
        self.env = HpDartEnv(env_name)
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

        self.model = Model(self.num_state, self.num_action).float()
        self.optimizer = optim.Adam(self.model.parameters(), lr=7E-4)
        self.w_entropy = 0.0

        self.sum_return = 0.
        self.num_episode = 0

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

        self.param_sender = []  # type: list[Connection]
        self.param_receiver = []  # type: list[Connection]
        self.envs = []  # type: list[Process]

        self.manager = Manager()
        self.episodes_lock = Lock()
        self.terminal_lock = Lock()
        self.run_lock = Lock()
        self.total_episodes = self.manager.list()
        self.terminal = self.manager.list()
        for _ in range(self.num_slaves):
            self.terminal.append(False)

        self.init_envs()

    def init_envs(self):
        for slave_idx in range(self.num_slaves):
            a_s, a_r = Pipe()
            p = Process(target=worker, args=(slave_idx, self.buffer_size, self.total_episodes, self.terminal, self.episodes_lock, self.terminal_lock, self.run_lock, a_r))
            self.param_sender.append(a_s)
            self.param_receiver.append(a_r)
            self.envs.append(p)
            p.start()

    def envs_update_models(self):
        state_dict = self.model.state_dict()
        if DEBUG:
            print(state_dict)
        for j in range(self.num_slaves):
            self.param_sender[j].send(state_dict)

    def SaveModel(self):
        torch.save(self.model.state_dict(), self.save_directory + str(self.num_evaluation) + '.pt')

    def LoadModel(self, model_path):
        self.model.load_state_dict(torch.load(model_path))

    def GenerateTransitions(self):
        del self.total_episodes[:]

        self.terminal_lock.acquire()
        for j in range(self.num_slaves):
            self.terminal[j] = False
        self.terminal_lock.release()

        self.run_lock.release()

        while True:
            Timer(1., lambda: None).start()
            self.terminal_lock.acquire()
            if all(self.terminal):
                self.terminal_lock.release()
                break
            else:
                self.terminal_lock.release()

    def ComputeTDandGAE(self):
        self.replay_buffer.clear()
        self.sum_return = 0.
        for epi in self.total_episodes:
            data = epi.get_data()
            size = len(data)
            states, actions, rewards, values, logprobs = zip(*data)

            values = np.concatenate((values, np.zeros(1)), axis=0)
            advantages = np.zeros(size)
            ad_t = 0

            for i in reversed(range(len(data))):
                self.sum_return += rewards[i]
                delta = rewards[i] + values[i + 1] * self.gamma - values[i]
                ad_t = delta + self.gamma * self.lb * ad_t
                advantages[i] = ad_t

            TD = values[:size] + advantages
            for i in range(size):
                self.replay_buffer.push(states[i], actions[i], logprobs[i], TD[i], advantages[i])
        self.num_episode = len(self.total_episodes)

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
        self.run_lock.acquire()
        self.envs_update_models()
        self.GenerateTransitions()
        self.OptimizeModel()
        self.num_training += 1

    def Evaluate(self):
        self.num_evaluation += 1
        self.print('noise : {:.3f}'.format(self.model.log_std.exp().mean()))
        self.print('Avg return : {:.2f}'.format(self.sum_return / self.num_episode))
        return self.sum_return/self.num_episode, 0

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
        ppo = PPO('walk', 2)
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
    max_avg_reward = 0.

    for i in range(50000):
        ppo.Train()
        print('# {}'.format(i+1))
        ppo.log_file.write('# {}'.format(i+1) + "\n")
        reward, step = ppo.Evaluate()
        rewards.append(reward)
        steps.append(step)
        if i % 10 == 0 or max_avg_steps < step or max_avg_reward*1.05 < reward:
            ppo.SaveModel()
            if max_avg_steps < step:
                max_avg_steps = step
            if max_avg_reward * 1.05 < reward:
                max_avg_reward = reward

        # Plot(np.asarray(rewards),'reward',1,False)
        print("Elapsed time : {:.2f}s".format(time.time() - tic))
        ppo.log_file.write("Elapsed time : {:.2f}s".format(time.time() - tic) + "\n")
        ppo.log_file.flush()
