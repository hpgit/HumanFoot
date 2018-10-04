import numpy as np

from PyCommon.modules.Motion import ysMotion as ym
from PyCommon.modules.Motion import hpMotionBlend as hmb
from PyCommon.modules.Math import mmMath as mm
from annoy import AnnoyIndex


def joint_poses_btw_posture_in_plane_by_joint_pos(posture, posture_base):
    """

    :type posture: ym.JointPosture
    :type posture_base: ym.JointPosture
    :return:
    """

    align_transform = hmb.get_plane_align_transform_by_posture(posture, posture_base)
    position = [mm.affine_pos(align_transform, posture.getPosition(i)) for i in range(1, posture.skeleton.getElementNum())]
    position.insert(0, np.array((posture.getPosition(0)[1], )))

    return np.concatenate(position)


BLEND_FRAME = 10


class MotionTransition(object):
    def __init__(self, _from, _from_idx, _to, _to_idx, dist):
        self.motion_to = _to
        self.motion_to_idx = _to_idx
        self.motion_from = _from
        self.motion_from_idx = _from_idx
        self.dist = dist


class MotionTransitionPool(object):
    def __init__(self):
        self.motion_to_idx_begin = np.inf
        self.motion_to_idx_end = -1
        self.motion_from_idx_begin = np.inf
        self.motion_from_idx_end = -1
        self.transition = []
        self.min_idx = -1
        self.min_dist = np.inf

    def check_transition_enter(self, transition):
        """

        :type transition: MotionTransition
        :return:
        """
        if self.motion_to_idx_begin - 7 < transition.motion_to_idx < self.motion_to_idx_end + 7:
            if self.motion_from_idx_begin - 7 < transition.motion_from_idx < self.motion_from_idx_end + 7:
                return True
        return False

    def add_transition(self, transition):
        """

        :type transition: MotionTransition
        :return:
        """
        self.transition.append(transition)
        # print(transition.motion_from_idx, transition.motion_to_idx)

        if self.motion_from_idx_begin > transition.motion_from_idx:
            self.motion_from_idx_begin = transition.motion_from_idx

        if self.motion_from_idx_end < transition.motion_from_idx:
            self.motion_from_idx_end = transition.motion_from_idx

        if self.motion_to_idx_begin > transition.motion_to_idx:
            self.motion_to_idx_begin = transition.motion_to_idx

        if self.motion_to_idx_end < transition.motion_to_idx:
            self.motion_to_idx_end = transition.motion_to_idx

        if self.min_dist > transition.dist:
            self.min_dist = transition.dist
            self.min_idx = len(self.transition) - 1

    def get_best_transition(self):
        """

        :rtype: MotionTransition
        """
        return self.transition[self.min_idx]


class MotionGraph(object):
    def __init__(self):
        self.is_built = False
        self.transition = []  # type: list[MotionTransition]
        self.transition_processed = []  # type: list[MotionTransition]
        self.motions = []  # type: list[ym.JointMotion]
        self.distance = None  # type: np.ndarray

    def add_transition(self, _transition):
        self.transition.append(_transition)

    def generate_motion(self, start_motion_idx, start_motion_time_offset, motion_time):
        pass

    def add_motion(self, motion):
        self.motions.append(motion)

    def build(self):
        self.init()
        self.find_nn()
        # self.calc_whole_dist()
        self.prune_contact()
        self.prune_local_maxima()
        self.prune_likelihood()
        self.prune_dead_end()

        self.is_built = True

    def init(self):
        """
        init transition data structure
        :return:
        """
        pass

    def find_nn(self, near_neigh=20):
        dim = joint_poses_btw_posture_in_plane_by_joint_pos(self.motions[0][0], self.motions[0][0]).shape[0]

        size = sum(map(len, self.motions))

        self.distance = np.zeros((size, size))
        data = []
        for i in range(len(self.motions)):
            for j in range(len(self.motions[i])):
                data.append(joint_poses_btw_posture_in_plane_by_joint_pos(self.motions[i][j], self.motions[0][0]))

        t = AnnoyIndex(dim, metric='euclidean')
        for i in range(size):
            t.add_item(i, data[i])

        t.build(20)

        for i in range(size):
            res, dist = t.get_nns_by_vector(data[i], near_neigh, include_distances=True)

            for j in range(near_neigh):
                if abs(i-res[j]) > 10:
                    self.distance[i, res[j]] = dist[j]
                    # TODO:
                    self.add_transition(MotionTransition(0, i, 0, res[j], dist[j]))
                    # print(i, res[j], dist[j])

    def calc_whole_dist(self):
        size = sum(map(len, self.motions))
        self.distance = np.zeros((size, size))
        data = []
        for i in range(len(self.motions)):
            for j in range(len(self.motions[i])):
                data.append(joint_poses_btw_posture_in_plane_by_joint_pos(self.motions[i][j], self.motions[0][0]))

        for i in range(len(data)):
            for j in range(i, len(data)):
                self.distance[i, j] = np.linalg.norm(data[i] - data[j])
                self.distance[j, i] = self.distance[i, j]
                print(i, j, self.distance[i, j])

    def prune_contact(self):
        pass

    def prune_likelihood(self):
        processed_index = []
        for i in range(len(self.transition_processed)):
            if self.transition_processed[i].dist > 0.3:
                processed_index.append(i)

        prune_num = len(processed_index)
        processed_index.reverse()
        for j in processed_index:
            self.transition_processed.pop(j)
        print('prune by likelihood: ', prune_num)

    def prune_local_maxima(self):
        prune_num = 0
        while self.transition:
            pool = MotionTransitionPool()
            pool.add_transition(self.transition.pop(0))
            processed_index = []
            for i in range(len(self.transition)):
                if pool.check_transition_enter(self.transition[i]):
                    pool.add_transition(self.transition[i])
                    processed_index.append(i)
            # print(pool.motion_from_idx_begin, pool.motion_from_idx_end, pool.motion_to_idx_begin, pool.motion_to_idx_end)
            transition = pool.get_best_transition()
            # print(transition.dist, transition.motion_from_idx, transition.motion_to_idx)
            # print(transition.motion_from_idx, transition.motion_to_idx)

            prune_num += len(processed_index) - 1

            processed_index.reverse()
            for j in processed_index:
                self.transition.pop(j)
            self.transition_processed.append(pool.get_best_transition())
        print('prune by local maxima: ', prune_num)

    def prune_dead_end(self):
        pass


