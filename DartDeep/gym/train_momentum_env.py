#!/usr/bin/env python3
import numpy as np
from DartDeep.dart_env import HpDartEnv
from baselines.common.cmd_util import common_arg_parser
from baselines import bench, logger

# import os
# os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'

import pydart2 as pydart


def train(num_timesteps, seed):
    from baselines.common.vec_env.vec_normalize import VecNormalize
    from baselines.ppo2 import ppo2
    import tensorflow as tf
    from baselines.common.vec_env.dummy_vec_env import DummyVecEnv
    # ncpu = 1
    # config = tf.ConfigProto(allow_soft_placement=True,
    #                         intra_op_parallelism_threads=ncpu,
    #                         inter_op_parallelism_threads=ncpu)
    #
    # tf.Session(config=config).__enter__()

    def make_env():
        env = HpDartEnv()
        env = bench.Monitor(env, logger.get_dir(), allow_early_resets=True)
        return env

    env = DummyVecEnv([make_env])
    # env = VecNormalize(env)

    # set_global_seeds(seed)
    model = ppo2.learn(network='mlp', env=env,
                       nsteps=2048, nminibatches=128,
                       lam=0.95, gamma=0.95, noptepochs=10, log_interval=1,
                       ent_coef=0.0, vf_coef=0.5, max_grad_norm=0.5,
                       lr=3e-4,
                       cliprange=0.2,
                       save_interval=0,
                       total_timesteps=num_timesteps)

    load_path = logger.get_dir() + '../log/checkpoints/04096'

    # model = ppo2.learn(policy=policy, env=env, nsteps=2048, nminibatches=32,
    #                    lam=0.95, gamma=0.99, noptepochs=10, log_interval=1,
    #                    ent_coef=0.0,
    #                    lr=3e-4,
    #                    cliprange=0.2, save_interval=512,
    #                    total_timesteps=num_timesteps, load_path=load_path)

    return model, env


def main():
    from time import strftime
    pydart.init()
    args = common_arg_parser().parse_args()
    logger.configure(dir='./log'+strftime("%Y%m%d%H%M")+'/')
    model, env = train(num_timesteps=10000000, seed=args.seed)

    logger.log("Running trained model")
    obs = np.zeros((env.num_envs,) + env.observation_space.shape)
    obs[:] = env.reset()
    while True:
        actions = model.step(obs)
        res = env.step(actions)
        obs[:] = res[0]
        done = res[2]
        if done[0]:
            break

        # env.render()


if __name__ == '__main__':
    main()
