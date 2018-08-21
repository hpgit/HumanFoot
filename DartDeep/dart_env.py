from rl.core import Env
import pydart2 as pydart
import numpy as np
import PyCommon.modules.Resource.ysMotionLoader as yf
from math import exp
# import gym


class Box(gym.Space):
    """
    A box in R^n.
    I.e., each coordinate is bounded.

    Example usage:
    self.action_space = spaces.Box(low=-10, high=10, shape=(1,))
    """
    def __init__(self, low=None, high=None, shape=None, dtype=None):
        """
        Two kinds of valid input:
            Box(low=-1.0, high=1.0, shape=(3,4)) # low and high are scalars, and shape is provided
            Box(low=np.array([-1.0,-2.0]), high=np.array([2.0,4.0])) # low and high are arrays of the same shape
        """
        if shape is None:
            assert low.shape == high.shape
            shape = low.shape
        else:
            assert np.isscalar(low) and np.isscalar(high)
            low = low + np.zeros(shape)
            high = high + np.zeros(shape)
        if dtype is None:  # Autodetect type
            if (high == 255).all():
                dtype = np.uint8
            else:
                dtype = np.float32
            logger.warn("gym.spaces.Box autodetected dtype as %s. Please provide explicit dtype." % dtype)
        self.low = low.astype(dtype)
        self.high = high.astype(dtype)
        gym.Space.__init__(self, shape, dtype)

    def sample(self):
        return gym.spaces.np_random.uniform(low=self.low, high=self.high + (0 if self.dtype.kind == 'f' else 1), size=self.low.shape).astype(self.dtype)
    def contains(self, x):
        return x.shape == self.shape and (x >= self.low).all() and (x <= self.high).all()

    def to_jsonable(self, sample_n):
        return np.array(sample_n).tolist()
    def from_jsonable(self, sample_n):
        return [np.asarray(sample) for sample in sample_n]

    def __repr__(self):
        return "Box" + str(self.shape)
    def __eq__(self, other):
        return np.allclose(self.low, other.low) and np.allclose(self.high, other.high)

class PDController:
    """
    :type h : float
    :type skel : pydart.Skeleton
    :type qhat : np.array
    """
    def __init__(self, skel, h, Kp, Kd, weightMap=None):
        self.h = h
        self.skel = skel
        # ndofs = self.skel.ndofs
        # self.qhat = self.skel.q
        # self.Kp = np.diagflat([0.0] * 6 + [Kp] * (ndofs - 6))
        # self.Kd = np.diagflat([0.0] * 6 + [Kd] * (ndofs - 6))
        self.setKpKd(Kp, Kd, weightMap)
        self.preoffset = 0.0

    def setKpKd(self, Kp, Kd, weightMap=None):
        ndofs = self.skel.ndofs

        if weightMap is None:
            self.Kp = np.diagflat([Kp] * ndofs)
            self.Kd = np.diagflat([Kd] * ndofs)
        else:
            Kp_list = [Kp] * ndofs
            Kd_list = [Kd] * ndofs
            for j in range(self.skel.num_joints()):
                joint = self.skel.joint(j)  # type: pydart.Joint
                for d in range(joint.num_dofs()):
                    dof = joint.dofs[d]  # type:  pydart.Dof
                    Kp_list[dof.index_in_skeleton()] = Kp * weightMap[joint.name[2:]]
                    Kd_list[dof.index_in_skeleton()] = Kd * weightMap[joint.name[2:]]
            self.Kp = np.diagflat(Kp_list)
            self.Kd = np.diagflat(Kd_list)

    def compute_flat(self, qhat, robustPD=True):
        skel = self.skel

        if robustPD:
            invM = np.linalg.inv(skel.M + self.Kd * self.h)
            p = -self.Kp.dot(skel.q + skel.dq * self.h - qhat)
            d = -self.Kd.dot(skel.dq)
            qddot = invM.dot(-skel.c + p + d + skel.constraint_forces())
            tau = p + d - self.Kd.dot(qddot) * self.h
        else:
            tau = self.Kp.dot(qhat - skel.q) - self.Kd.dot(skel.dq)
        tau[0:6] = np.zeros(6)

        return tau


def exp_reward_term(w, exp_w, v0, v1):
    norm = np.linalg.norm(v0 - v1)
    return w * exp(-exp_w * norm * norm)

class HpDartEnv(Env):
    def __init__(self):
        self.world = pydart.World(1./1200., "../assets/woody_with_ground.xml")
        self.world.control_skel = self.world.skeletons[1]
        self.pdc = PDController(self.world.skel, self.world.time_step(), 200., 20.)
        self.skel = self.world.skeletons[1]

        self.ref_motion = yf.readBvhFile("woody_walk_normal.bvh", 0.01)

        self.w_p = 0.65
        self.w_v = 0.1
        self.w_e = 0.15
        self.w_c = 0.1

        self.exp_p = 2.
        self.exp_v = 0.1
        self.exp_e = 40.
        self.exp_c = 10.

    def state(self):
        return

    def reward(self):
        current_frame = int(self.world.time()*30)
        return exp_reward_term(self.w_p, self.exp_p, self.skel.q, self.ref_motion[current_frame].get_q())

    def is_done(self):
        return self.world.skeletons[1].com()[1] < 0.5

    def step(self, action):
        """Run one timestep of the environment's dynamics.
        Accepts an action and returns a tuple (observation, reward, done, info).

        # Arguments
            action (object): An action provided by the environment.

        # Returns
            observation (object): Agent's observation of the current environment.
            reward (float) : Amount of reward returned after previous action.
            done (boolean): Whether the episode has ended, in which case further step() calls will return undefined results.
            info (dict): Contains auxiliary diagnostic information (helpful for debugging, and sometimes learning).
        """
        for i in range(40):
            self.pdc.compute_flat(action)
            self.world.step()
        return (self.state(), self.reward(), self.is_done(), dict())

    def reset(self):
        """
        Resets the state of the environment and returns an initial observation.

        # Returns
            observation (object): The initial observation of the space. Initial reward is assumed to be 0.
        """
        pass

    def render(self, mode='human', close=False):
        """Renders the environment.
        The set of supported modes varies per environment. (And some
        environments do not support rendering at all.)

        # Arguments
            mode (str): The mode to render with.
            close (bool): Close all open renderings.
        """
        pass

    def close(self):
        """Override in your subclass to perform any necessary cleanup.
        Environments will automatically close() themselves when
        garbage collected or when the program exits.
        """
        pass


if __name__ == '__main__':
    pydart.init()
    HpDartEnv()
