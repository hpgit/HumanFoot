import tensorflow as tf
import numpy as np
from DartDeep.dart_env_v2_1 import HpDartEnv
import pydart2
from itertools import count
from collections import deque
from random import random, sample
from multiprocessing import Process, Pipe
from copy import deepcopy


class Replay(deque):
    def sample(self, batch_size):
        return [self[i] for i in sample(range(0, len(self)), batch_size)]


class Episode(list):
    pass


class HpPPO(object):
    def __init__(self, session, env_name='walk', num_slaves=1):
        self.sess = session
        self.env = HpDartEnv(env_name)
        self.num_slaves = num_slaves

        self.num_state = self.env.observation_space.shape[0]
        self.num_action = self.env.action_space.shape[0]
        self.action_bound = [self.env.action_space.low, self.env.action_space.high]

        self.num_train = 0

        self.layer_size = [128, 64]
        self.error_mag = 0.1
        self.num_epoches = 10
        # self.sample_size = 256
        self.sample_size = 2048
        self.batch_size = 128
        self.gamma = 0.95
        self.td_lambda = 0.95
        self.clip_ratio = 0.2

        # set memory and episodes
        self.replay_buffer = Replay()
        self.total_episodes = list()  # type: list[Episode]

        # set varialbles
        with tf.variable_scope('state'):
            self.state = tf.placeholder(tf.float32, shape=[None, self.num_state])

        with tf.variable_scope('action'):
            self.action = tf.placeholder(tf.float32, shape=[None, self.num_action])

        with tf.variable_scope('target_value'):
            self.y = tf.placeholder(tf.float32, shape=[None, 1])

        with tf.variable_scope('advantages'):
            self.advantages = tf.placeholder(tf.float32, shape=[None, 1])

        # build networks
        self.value = self.build_value_net()
        self.actor, self.actor_param = self.build_actor_net('actor_net', trainable=True)
        self.actor_old, self.actor_old_param = self.build_actor_net('actor_old', trainable=False)
        self.syn_old_pi = [oldp.assign(p) for p, oldp in zip(self.actor_param, self.actor_old_param)]
        self.sample_op = tf.clip_by_value(tf.squeeze(self.actor.sample(1), axis=0), self.action_bound[0], self.action_bound[1])

        # set loss function
        with tf.variable_scope('critic_loss'):
            self.adv = self.y - self.value
            self.critic_loss = tf.reduce_mean(tf.square(self.adv))

        with tf.variable_scope('actor_loss'):
            ratio = self.actor.prob(self.action) / self.actor_old.prob(self.action)
            self.actor_loss = tf.reduce_mean(tf.minimum(ratio * self.advantages, tf.clip_by_value(ratio, 1. - self.clip_ratio, 1. + self.clip_ratio)))

        # set optimizer
        self.value_step_size = 1e-2
        self.value_optimizer = tf.train.AdamOptimizer(self.value_step_size)
        self.train_critic = self.value_optimizer.minimize(self.critic_loss)

        self.policy_step_size = 1e-4
        self.policy_optimizer = tf.train.AdamOptimizer(self.policy_step_size)
        self.train_policy = self.value_optimizer.minimize(self.actor_loss)

        # for evaluation
        self.num_eval = 0

        # for multiprocessing
        self.state_sender = []  # type: list[Connection]
        self.result_sender = []  # type: list[Connection]
        self.state_receiver = []  # type: list[Connection]
        self.result_receiver = []  # type: list[Connection]
        self.action_sender = []  # type: list[Connection]
        self.reset_sender = []  # type: list[Connection]
        self.motion_sender = []  # type: list[Connection]
        self.envs = []  # type: list[Process]

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

    def build_value_net(self):
        # build networks
        with tf.variable_scope('value_net'):
            value_dl1 = tf.contrib.layers.fully_connected(inputs=self.state,
                                                          num_outputs=self.layer_size[0],
                                                          activation_fn=tf.nn.relu,
                                                          scope='value_dl1')

            value_dl2 = tf.contrib.layers.fully_connected(inputs=value_dl1,
                                                          num_outputs=self.layer_size[1],
                                                          activation_fn=tf.nn.relu,
                                                          scope='value_dl2')

            value = tf.contrib.layers.fully_connected(inputs=value_dl2,
                                                      num_outputs=1,
                                                      activation_fn=None,
                                                      scope='value')

            return value

    def build_actor_net(self, scope, trainable):
        with tf.variable_scope(scope):
            actor_dl1 = tf.contrib.layers.fully_connected(inputs=self.state,
                                                          num_outputs=self.layer_size[0],
                                                          activation_fn=tf.nn.relu,
                                                          trainable=trainable,
                                                          scope='dl1')

            actor_dl2 = tf.contrib.layers.fully_connected(inputs=actor_dl1,
                                                          num_outputs=self.layer_size[1],
                                                          activation_fn=tf.nn.relu,
                                                          trainable=trainable,
                                                          scope='dl2')

            mu = tf.contrib.layers.fully_connected(inputs=actor_dl2,
                                                   num_outputs=self.num_action,
                                                   activation_fn=None,
                                                   trainable=trainable,
                                                   scope='mu')

            sigma = tf.contrib.layers.fully_connected(inputs=actor_dl2,
                                                      num_outputs=self.num_action,
                                                      activation_fn=tf.nn.softplus,
                                                      trainable=trainable,
                                                      scope='sigma')
            # sigma = tf.convert_to_tensor(0.1 * np.ones(self.num_action), dtype=np.float32)

            actor_dist = tf.contrib.distributions.Normal(loc=mu, scale=sigma)

            param = tf.get_collection(tf.GraphKeys.GLOBAL_VARIABLES, scope)

            return actor_dist, param

    def get_action(self, s):
        return self.sess.run(self.sample_op, feed_dict={self.state: s[np.newaxis, :]})

    def get_v(self, s):
        if s.ndim < 2:
            s = s[np.newaxis, :]
        return self.sess.run(self.value, feed_dict={self.state: s})[0, 0]

    def train(self):
        self.generate_transitions()
        self.optimize_model()
        self.num_train += 1

    def generate_transitions(self):
        del self.total_episodes[:]
        episodes = [Episode() for _ in range(self.num_slaves)]
        terminated = [False for _ in range(self.num_slaves)]

        self.env.Resets(True)
        # self.envs_resets(1)
        local_step = 0

        while True:
            states = self.env.GetStates()
            # states = self.envs_get_states(terminated)

            actions = np.asarray(self.get_action(states))
            values = self.get_v(states)
            logprobs = self.actor.prob(actions)

            # self.envs_send_actions(actions, terminated)
            # rewards, is_done = self.envs_get_status(terminated)

            __, reward, is_done, info = self.env.step(actions.flatten())
            rewards = [reward]
            is_dones = [is_done]
            for j in range(self.num_slaves):
                if terminated[j]:
                    continue

                nan_occur = np.any(np.isnan(states[j])) or np.any(np.isnan(actions[j]))
                if not nan_occur:
                    episodes[j].append((states[j], actions[j], rewards[j], values[j], logprobs[j]))

                if is_dones[j] or nan_occur:
                    self.total_episodes.append(deepcopy(episodes[j]))

                    if local_step < self.sample_size:
                        episodes[j] = Episode()
                        # self.envs_reset(j, 1)
                        self.env.reset()
                    else:
                        terminated[j] = True
                else:
                    # self.envs_reset(j, 0)
                    pass

            if local_step >= self.sample_size and all(terminated):
                break


    def optimize_model(self):
        self.compute_td_gae()

        for _ in range(self.num_epoches):
            transitions = self.replay_buffer.sample(self.batch_size)
            batch = list(zip(*transitions))

            td = batch[3]


            self.update_value()
            self.update_policy()

    def compute_td_gae(self):
        for epi in self.total_episodes:
            len_epi = len(epi)
            states, actions, rewards, values, logprobs = zip(*epi)

            values = np.concatenate((values, np.zeros(1)), axis=0)
            advantages = np.zeros(len_epi)
            ad_t = 0

            for i in reversed(range(len_epi)):
                delta = rewards[i] + values[i + 1] * self.gamma - values[i]
                ad_t = delta + self.gamma * self.td_lambda * ad_t
                advantages[i] = ad_t

            TD = values[:len_epi] + advantages
            for i in range(len_epi):
                self.replay_buffer.append((states[i], actions[i], logprobs[i], TD[i], advantages[i]))

    def update_value(self):
        pass

    def update_policy(self):
        pass

    def evaluate(self):
        self.num_eval += 1
        total_reward = 0
        total_step = 0
        self.env.Reset(False, 0)

        state = self.env.GetState(0)

        for t in count():
            action = np.asarray(self.actor.mean().eval(feed_dict={ppo.state: [state]})).flatten()
            state, reward, is_done, info = self.env.step(action)
            if is_done:
                break
            else:
                total_step += 1
                total_reward += reward

        # print('noise: {:.3f}'.format(self.actor.stddev().eval(feed_dict={ppo.state: [state]})))
        print('noise: ', self.actor.stddev().eval(feed_dict={ppo.state: [state]}))
        if total_step > 0:
            print('Epi reward : {:.2f}, Step reward : {:.2f} Total step : {}'
                       .format(total_reward, total_reward / total_step, total_step))
        else:
            print('bad')
        return total_reward, total_step


def worker(env_name, proc_num, state_sender, result_sender, action_receiver, reset_receiver, motion_receiver):
    """

    :type env_name: str
    :type proc_num: int
    :type result_sender: Connection
    :type action_receiver: Connection
    :return:
    """

    # reset variable
    # 0 : go on (no reset)
    # 1 : soft reset ( w/o motion change )
    # 2 : hard reset ( with motion change )

    env = HpDartEnv(env_name)

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


if __name__ == '__main__':
    pydart2.init()
    with tf.Session() as sess:
        ppo = HpPPO(sess, 'walk')
        sess.run(tf.global_variables_initializer())
        # print(sigma.eval())
        # print(actor_dist.sample(1).eval(feed_dict={state: np.zeros((1, num_state))}))
        # print(ppo.actor.mean().eval(feed_dict={ppo.state: np.zeros((1, ppo.num_state))}))
        ppo.train()
        ppo.evaluate()
