import random
from collections import namedtuple
from collections import deque

import torch
from torch import nn
import torch.nn.functional as F


MultiVariateNormal = torch.distributions.Normal
temp = MultiVariateNormal.log_prob
MultiVariateNormal.log_prob = lambda self, val: temp(self, val).sum(-1, keepdim=True)

temp2 = MultiVariateNormal.entropy
MultiVariateNormal.entropy = lambda self: temp2(self).sum(-1)
MultiVariateNormal.mode = lambda self: self.mean


class Model(nn.Module):
    def __init__(self, num_states, num_actions, hidden_layer_size, init_log_sigma=0.):
        super(Model, self).__init__()
        self.layer_size = [num_states]
        self.layer_size.extend(list(hidden_layer_size))

        self.policy_fc = list()
        self.value_fc = list()

        # policy mean
        for i in range(len(self.layer_size)-1):
            self.policy_fc.append(nn.Linear(self.layer_size[i], self.layer_size[i+1]))
        self.policy_fc.append(nn.Linear(self.layer_size[-1], num_actions))

        # Policy Distributions
        self.log_std = nn.Parameter(init_log_sigma * torch.ones(num_actions))

        # Value
        for i in range(len(self.layer_size)-1):
            self.value_fc.append(nn.Linear(self.layer_size[i], self.layer_size[i+1]))
        self.value_fc.append(nn.Linear(self.layer_size[-1], 1))

        self.initParameters()

    def initParameters(self):
        # policy
        for i in range(len(self.policy_fc)):
            if self.policy_fc[i].bias is not None:
                self.policy_fc[i].bias.data.zero_()

        for i in range(len(self.policy_fc)):
            torch.nn.init.xavier_uniform_(self.policy_fc[i].weight)

        # value
        for i in range(len(self.value_fc)):
            if self.value_fc[i].bias is not None:
                self.value_fc[i].bias.data.zero_()

        for i in range(len(self.value_fc)):
            torch.nn.init.xavier_uniform_(self.value_fc[i].weight)

    def forward(self, x):
        # policy
        p_mean = F.relu(self.policy_fc[0](x))
        for i in range(1, len(self.policy_fc)-1):
            p_mean = F.relu(self.policy_fc[i](p_mean))
        p_mean = self.policy_fc[-1](p_mean)

        p = MultiVariateNormal(p_mean, self.log_std.exp())

        # value
        v = F.relu(self.value_fc[0](x))
        for i in range(1, len(self.value_fc)-1):
            v = F.relu(self.value_fc[i](v))
        v = self.value_fc[-1](v)

        return p, v


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
    def __init__(self, buff_size = 10000):
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
