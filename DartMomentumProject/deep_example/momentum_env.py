import math
import gym
from gym import spaces, logger
from gym.utils import seeding
import numpy as np
from mtInitialize import create_biped
from PyCommon.modules.Simulator.csDartModel import DartModel
from PyCommon.modules.Math import mmMath as mm


class DeepDartModel(DartModel):
    def __init__(self, wcfg, posture, mcfg, isContainGround=True):
        super(DeepDartModel, self).__init__(wcfg, posture, mcfg, isContainGround)
        self.fm = 200.
        self.fv = np.array([0., 0., 1.])
        self.ts = 5.
        self.te = 5.1

        self.f = self.fm * self.fv

        self.action = np.zeros_like(self.reset_q)

        self.Kt = 10.
        self.Dt = .0

        self.body_foot_R = self.getBody('RightFoot')
        self.body_foot_L = self.getBody('LeftFoot')

        foot_center = self.body_foot_R.com() + self.body_foot_L.com()
        foot_center[1] = self.getCOM()[1]
        self.com_des = foot_center

        self.w_q = 1.
        self.w_com = 1.
        self.w_foot = 1.
        self.w_foot_ori = 1.

        self.w_q_exp = 2.
        self.w_com_exp = 10.
        self.w_foot_exp = 40.
        self.w_foot_ori_exp = 40.

    def get_deep_state(self):
        com = self.getCOM()
        body_num = self.getBodyNum()
        state = [self.getBody(i).to_world() - com for i in range(body_num)]
        v = [self.getBody(i).world_angular_velocity() for i in range(body_num)]
        state.extend(v)
        return np.asarray(state).flatten()

    def get_deep_num_state(self):
        return self.getBodyNum() * 6

    def get_deep_num_action(self):
        return self.skeleton.num_dofs()

    def set_deep_action(self, a):
        self.action = a

    def set_deep_ext_force(self, fm, fv, ts, te):
        self.fm, self.fv, self.ts, self.te = fm, fv, ts, te
        self.f = self.fm * self.fv

    def deep_step(self, step_num=30):
        ddth_des_flat = self.Kt * (self.get_q() - self.reset_q - self.action.flatten()) - self.Dt * self.get_dq()
        # ddth_des_flat = self.Kt * (self.get_q() - self.reset_q) - self.Dt * self.get_dq()
        # ddth_des_flat[:6] = np.zeros(6)
        # print(self.world.time())
        for i in range(step_num):
            self.skeleton.set_accelerations(ddth_des_flat)
            # self.skeleton.set_forces(ddth_des_flat)

            if self.ts <= self.world.time() <= self.te:
                self.applyPenaltyForce([0], [np.zeros(3)], [self.f])
            self.world.step()
        print(self.body_foot_L.world_transform()[:3, :3])

    def get_deep_reward(self):
        r_q = mm.square_sum(self.get_q() - self.reset_q)
        r_com = mm.square_sum(self.getCOM() - self.com_des)
        r_foot = mm.square_sum([self.body_foot_L.to_world()[1], self.body_foot_R.to_world()[1], 0.])
        r_foot_ori = mm.square_sum([self.body_foot_L.world_transform(), self.body_foot_R.to_world()[1], 0.])
        return self.w_com * math.exp(-self.w_com_exp * r_com) + self.w_q * math.exp(-self.w_q_exp * r_q) + self.w_foot * math.exp(-self.w_foot_exp * r_foot)

    def is_terminal_state(self):
        if self.getCOM()[1] < 0.1:
            return True
        elif True in np.isnan(np.asarray(self.get_q())):
            return True
        elif self.world.time() > 10.:
            return True

        return False


class MomentumEnv(gym.Env):
    metadata = {
        'render.modes': ['human', 'rgb_array'],
        'video.frames_per_second': 50
    }

    def __init__(self):
        self.episode_num = 0
        motion, mcfg, wcfg, stepsPerFrame, config, frame_rate = create_biped()
        self.model = DeepDartModel(wcfg, motion[0], mcfg, True)
        self.model.initializeHybridDynamics()

        state_num = self.model.get_deep_num_state()
        action_num = self.model.get_deep_num_action()

        state_high = np.array([1.] * state_num)
        action_high = np.array([np.finfo(np.float32).max] * action_num)

        self.action_space = spaces.Box(-action_high, action_high)
        self.observation_space = spaces.Box(-state_high, state_high)

        self.seed()

        self.episode_times = np.array([0. for i in range(32)])

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):
        # assert self.action_space.contains(action), "%r (%s) invalid"%(action, type(action))
        self.model.set_deep_action(action)
        # print ("Before,", self.env.GetState(0))
        self.model.deep_step()
        # print(self.env.GetReward(0))
        # print ("After,", self.env.GetState(0))
        # print(self.env.IsTerminalState(0))
        return self.model.get_deep_state(), self.model.get_deep_reward(), self.model.is_terminal_state(), {}

    def test_step(self):
        self.model.deep_step()
        return self.model.get_deep_state(), self.model.get_deep_reward(), self.model.is_terminal_state(), {}

    def reset(self):
        # self.model.initializeHybridDynamics()
        self.episode_times[self.episode_num % 32] = self.model.world.time()
        if self.episode_num % 32 == 31:
            print(np.max(self.episode_times))
        #     self.model.WriteRecords("./output_momentum/"+str(self.episode_num))
        # self.env.WriteRecords("")
        self.episode_num += 1
        self.model.reset()
        return self.model.get_deep_state()

    def render(self, mode='human'):
        return 1

    def close(self):
        if self.viewer:
            self.viewer.close()

    def time(self):
        return self.model.world.time()