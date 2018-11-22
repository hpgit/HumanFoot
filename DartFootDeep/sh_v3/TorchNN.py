import torch
from torch import nn
import torch.nn.functional as F

from collections import namedtuple
from collections import deque
import random

MultiVariateNormal = torch.distributions.Normal
temp = MultiVariateNormal.log_prob
MultiVariateNormal.log_prob = lambda self, val: temp(self, val).sum(-1, keepdim=True)

temp2 = MultiVariateNormal.entropy
MultiVariateNormal.entropy = lambda self: temp2(self).sum(-1)
MultiVariateNormal.mode = lambda self: self.mean


class Model(nn.Module):
    def __init__(self, num_states, num_actions):
        super(Model, self).__init__()

        hidden_layer_size1 = 256
        hidden_layer_size2 = 128

        '''Policy Mean'''
        self.policy_fc1 = nn.Linear(num_states		  , hidden_layer_size1)
        self.policy_fc2 = nn.Linear(hidden_layer_size1, hidden_layer_size2)
        self.policy_fc3 = nn.Linear(hidden_layer_size2, num_actions)
        '''Policy Distributions'''
        self.log_std = nn.Parameter(torch.zeros(num_actions))

        '''Value'''
        self.value_fc1 = nn.Linear(num_states		  ,hidden_layer_size1)
        self.value_fc2 = nn.Linear(hidden_layer_size1 ,hidden_layer_size2)
        self.value_fc3 = nn.Linear(hidden_layer_size2 ,1)

        self.initParameters()

    def initParameters(self):
        '''Policy'''
        if self.policy_fc1.bias is not None:
            self.policy_fc1.bias.data.zero_()

        if self.policy_fc2.bias is not None:
            self.policy_fc2.bias.data.zero_()

        if self.policy_fc3.bias is not None:
            self.policy_fc3.bias.data.zero_()
        torch.nn.init.xavier_uniform_(self.policy_fc1.weight)
        torch.nn.init.xavier_uniform_(self.policy_fc2.weight)
        torch.nn.init.xavier_uniform_(self.policy_fc3.weight)
        '''Value'''
        if self.value_fc1.bias is not None:
            self.value_fc1.bias.data.zero_()

        if self.value_fc2.bias is not None:
            self.value_fc2.bias.data.zero_()

        if self.value_fc3.bias is not None:
            self.value_fc3.bias.data.zero_()
        torch.nn.init.xavier_uniform_(self.value_fc1.weight)
        torch.nn.init.xavier_uniform_(self.value_fc2.weight)
        torch.nn.init.xavier_uniform_(self.value_fc3.weight)

    def forward(self, x):
        # policy
        p_mean = F.relu(self.policy_fc1(x))
        p_mean = F.relu(self.policy_fc2(p_mean))
        p_mean = self.policy_fc3(p_mean)

        p = MultiVariateNormal(p_mean, self.log_std.exp())

        # value
        v = F.relu(self.value_fc1(x))
        v = F.relu(self.value_fc2(v))
        v = self.value_fc3(v)

        return p,v


Episode = namedtuple('Episode', ('s', 'a', 'r', 'value', 'logprob'))


class EpisodeBuffer(object):
    def __init__(self):
        self.data = []

    def push(self, *args):
        self.data.append(Episode(*args))

    def extend(self, data):
        self.data.extend(data)

    def get_data(self):
        return self.data

    def get_size(self):
        return len(self.data)


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
