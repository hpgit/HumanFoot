import tensorflow as tf
from DartDeep.dart_env import HpDartEnv
from DartDeep.tf.ppo_tf import *
from DartDeep.tf.worker import Worker
import pydart2



tf.reset_default_graph()
global_episodes = tf.Variable(0, dtype=tf.int32, name='global_episodes', trainable=False)

summary_writer = tf.summary.FileWriter('./summary_log/' + 'Walking')

pydart2.init()
env = HpDartEnv()
chief = Worker('Walking', env, summary_writer, global_episodes,
               visualize=False,
               gamma=0.95, batch_size=128, a_lr=5e-5, c_lr=1e-2)

with tf.Session() as sess:
    saver = tf.train.Saver(max_to_keep=5)
    sess.run(tf.global_variables_initializer())
    chief.ppo.load_model(sess, saver)
    chief.process(sess, saver)