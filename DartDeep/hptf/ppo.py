import tensorflow as tf
import numpy as np
from DartDeep.dart_env_v3 import HpDartEnv
import pydart2


class Memory:
    pass


class Episode:
    pass


class HpPPO:
    def __init__(self, session, env_name='walk'):
        self.sess = session
        self.env = HpDartEnv(env_name, 1)

        self.num_state = self.env.observation_space.shape[0]
        self.num_action = self.env.action_space.shape[0]
        self.action_bound = [self.env.action_space.low, self.env.action_space.high]

        self.num_train = 0

        self.layer_size = [128, 64]
        self.error_mag = 0.1
        self.num_epoches = 10
        self.sample_size = 4096
        self.batch_size = 256
        self.gamma = 0.95
        self.td_lambda = 0.95
        self.clip_ratio = 0.2

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
            ratio = self.actor.prob(self.action) / self.actor.prob(self.action)
            self.actor_loss = tf.reduce_mean(tf.minimum(ratio * self.advantages, tf.clip_by_value(ratio, 1. - self.clip_ratio, 1. + self.clip_ratio)))

        # set optimizer
        self.value_step_size = 1e-2
        self.value_optimizer = tf.train.AdamOptimizer(self.value_step_size)
        self.train_critic = self.value_optimizer.minimize(self.critic_loss)

        self.policy_step_size = 1e-4
        self.policy_optimizer = tf.train.AdamOptimizer(self.policy_step_size)
        self.train_policy = self.value_optimizer.minimize(self.actor_loss)

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

            # sigma = tf.contrib.layers.fully_connected(inputs=dl2,
            #                                           num_outputs=self.num_action,
            #                                           activation_fn=tf.nn.softplus,
            #                                           trainable=trainable,
            #                                           scope='sigma')
            sigma = tf.convert_to_tensor(0.1 * np.ones(self.num_action), dtype=np.float32)

            actor_dist = tf.contrib.distributions.Normal(loc=mu, scale=sigma)

            param = tf.get_collection(tf.GraphKeys.GLOBAL_VARIABLES, scope)

            return actor_dist, param

    def get_action(self, s):
        return self.sess.run(self.sample_op, feed_dict={self.state: s[np.newaxis, :]})

    def train(self):
        self.generate_transitions()
        self.optimize_model()
        self.num_train += 1

    def generate_transitions(self):
        self.env.Resets(True)
        for _ in range(self.sample_size):
            state = self.env.GetState(0)
            action = self.get_action(state)
            self.env.step(action)
            pass

    def optimize_model(self):
        for _ in range(self.num_epoches):
            self.sample_batch()

            self.update_value()
            self.compute_td()

            self.update_policy()
            self.compute_gae()

    def compute_td(self):
        pass


if __name__ == '__main__':
    pydart2.init()
    ppo = HpPPO()
    with tf.Session() as sess:
        sess.run(tf.global_variables_initializer())
        # print(sigma.eval())
        # print(actor_dist.sample(1).eval(feed_dict={state: np.zeros((1, num_state))}))
        print(ppo.actor.mean().eval(feed_dict={ppo.state: np.zeros((1, ppo.num_state))}))
