#!/usr/bin/env python3
import numpy as np
from momentum_env import MomentumEnv
from baselines.common.cmd_util import mujoco_arg_parser
from baselines import bench, logger

# import os
# os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'

import pydart2 as pydart


def load(env_id, num_timesteps, seed):
    from baselines.common import set_global_seeds
    from baselines.common.vec_env.vec_normalize import VecNormalize
    from baselines.ppo2 import ppo2
    from baselines.ppo2.policies import MlpPolicy
    import gym
    import tensorflow as tf
    from baselines.common.vec_env.dummy_vec_env import DummyVecEnv
    ncpu = 1
    config = tf.ConfigProto(allow_soft_placement=True,
                            intra_op_parallelism_threads=ncpu,
                            inter_op_parallelism_threads=ncpu)
    tf.Session(config=config).__enter__()

    def make_env():
        env = MomentumEnv()
        env.model.set_deep_ext_force(fm=0., fv=np.array([0., 0., 1.]), ts=5., te=5.1)
        env = bench.Monitor(env, logger.get_dir(), allow_early_resets=True)
        return env

    env = DummyVecEnv([make_env])
    env = VecNormalize(env)

    set_global_seeds(seed)

    # Model setting
    policy = MlpPolicy
    nsteps = 2048
    model = ppo2.Model(policy=policy, ob_space=env.observation_space, ac_space=env.action_space,
                       nbatch_act=env.num_envs, nbatch_train=(env.num_envs * nsteps) // 32, nsteps=nsteps,
                       ent_coef=0.0, vf_coef=0.5, max_grad_norm=0.5)

    # Model loading
    load_path = logger.get_dir() + "/checkpoints/04096"
    model.load(load_path)

    return model, env


def main():
    pydart.init()
    args = mujoco_arg_parser().parse_args()
    logger.configure(dir="./log/")
    model, env = load(args.env, num_timesteps=10000000, seed=args.seed)

    logger.log("Running trained model")
    obs = np.zeros((env.num_envs,) + env.observation_space.shape)
    obs[:] = env.reset()
    while True:
        actions = model.step(obs)
        res = env.step(actions[0])
        obs[:] = res[0]
        done = res[2]
        if done[0]:
            break

        # env.render()


if __name__ == '__main__':
    main()
