import math

MAX_SCAN_DEPTH = 180
MAX_MOTIONS = 40
MAX_BUFFER_SIZE = 36

M_UNIT_LEN = 10.


class FlowEntity:
    def __init__(self):
        self.id = 0
        self.value = 0.
        self.prev = None  # type: FlowEntity
        self.next = None  # type: FlowEntity
        self.action = None  # type: Action


class FlowEntityHead:
    def __init__(self):
        self.num = 0
        self.entity = None  # type: FlowEntity

        # segmentation
        self.is_break_point = True

        # strongly connected component
        self.scc_component = 0
        self.scc_low = 0
        self.scc_number = 0
        self.scc_visited = False
        self.scc_in_stack = False


class FlowGraph:
    def __init__(self):
        self.size = 0
        self.motion_frames = None  # type: PmLinearMotion
        self.velocity_frames = None  # type: PmVectorAray

        self.flow_graph = None  # type: list[FlowEntityHead]

        self.distance_matrix = None  # type: np.ndarray

        self.local_coordinate = False
        self.variance = .1
        self.min_jump = 60
        self.threshold = 1e-5
        self.pelvis_weight = 0.03

        self.angle_fixed = False
        self.angle_tolerance = math.pi

        self.scc_seed = 0
        self.scc_index = list()  # type: list[int]
        self.scc_size = 0

        self.transition_at_contact_change = True
        self.transition_at_left_heel_strike = True
        self.transition_at_left_toe_off = False
        self.transition_at_right_heel_strike = False
        self.transition_at_right_toe_off = False

        self.acc_neck_dist = list()  # type: list[float]
        self.unit_end_i = list()  # type: list[int]
        self.unit_size = 0

    def init(self, n_motions, motion_list):
        # Concatenate motion clips
        # motion_frames = new PmLinearMotion(manager->human );
        self.motion_frames = PmLinearMotion( motion_list[0].getBody() )
        for k in range(n_motions):
            self.motion_frames.concat(*(motion_list[k]))

        print("Number of motion clips = %d".format(n_motions))
        print("Number of motion frames = %d".format(self.motion_frames.getSize()) )

        self.size = self.motion_frames.getSize()

        self.flow_graph = [FlowEntityHead() for i in range(self.size)]

        return self.size

    def build(self, n_motions, motion_list):
        print('initialize the flow graph...')

        buffer1, buffer2, buffer3 = [0.]*self.size, [0.]*self.size, [0.]*self.size
        total, prune1, prune2, prune3 = 0, 0, 0, 0

        # Estimate Velocity
        self.velocity_frames = PmVectorArray()
        self.velocity_frames.setSize(self.size)

        v = PmVector()

        j = 0
        for k in range(n_motions):
            for i in range(motion_list[k].getSize()):
                motion_list[j].getPositionVelocity(i, v)
                self.velocity_frames.setVector(j, v)
                j += 1

        # Construct Flow Graph
        for j in range(self.size):
            buffer1[j] = 0.
            buffer2[j] = 0.
            buffer3[j] = 0.

        i, j = 0, 0
        for k in range(n_motions):
            j += 1
            l = 1
            while l < motion_list[k].getSize() - MAX_BUFFER_SIZE:
                if i == j-1:
                    buffer1[j] = 1.
                elif abs(i-j) > self.min_jump:
                    buffer1[j] = self.distance(i, j-1)
                else:
                    buffer1[j] = 0.
                j += 1

            while l < motion_list[k].getSize():
                l += 1
                j += 1

        i, j = 1, 0
        for k in range(n_motions):
            j += 1
            l = 1
            while l < motion_list[k].getSize() - MAX_BUFFER_SIZE:
                if i == j-1:
                    buffer2[j] = 1.
                elif abs(i-j) > self.min_jump:
                    buffer2[j] = self.distance(i, j-1)
                else:
                    buffer2[j] = 0.
                j += 1

            while l < motion_list[k].getSize():
                l += 1
                j += 1

        for i in range(self.size):
            if i % 100 == 0:
                print('%d frames processed...'.format(i))

            j = 0
            for k in range(n_motions):
                j += 1
                l = 1
                while l < motion_list[k].getSize() - MAX_BUFFER_SIZE:
                    if i == j-1:
                        buffer3[j] = 1.
                    elif abs(i-j) > self.min_jump:
                        buffer3[j] = self.distance(i, j-1)
                    else:
                        buffer3[j] = 0.
                    j += 1

                while l < motion_list[k].getSize():
                    l += 1
                    j += 1

            while self.flow_graph[i].entity is not None:
                self.removeEntity(i, self.flow_graph[i].entity.id)

            for j in range(self.size-1):
                total += 1
                if buffer2[j] > 0.:
                    prune1 += 1
                if buffer2[j] > self.threshold:
                    prune2 += 1
                if buffer2[j] > self.threshold \
                        and buffer2[j] > buffer2[j-1] \
                        and buffer2[j] >= buffer2[j+1] \
                        and buffer2[j] >= buffer1[j] \
                        and buffer2[j] >= buffer3[j] \
                        and buffer2[j] >= buffer1[j-1] \
                        and buffer2[j] >= buffer3[j-1] \
                        and buffer2[j] >= buffer1[j-1] \
                        and buffer2[j] >= buffer3[j-1]:
                    prune3 += 1
                    self.setValue(i-1, j, buffer2[j])

            for j in range(self.size):
                buffer1[j] = buffer2[j]
                buffer2[j] = buffer3[j]

            # Evaluate probability
            sum = 0.
            e = self.flow_graph[i].entity
            while e is not None:
                sum += e.value
                e = e.next

            e = self.flow_graph[i].entity
            while e is not None:
                e.value /= sum
                e = e.next

        print('The flow graph is constructed.')
        print('The total number of edges: %d'.format(total))
        print('pruning by contact: %d'.format(prune1))
        print('pruning by likelihood: %d'.format(prune2))
        print('pruning by local maxima: %d'.format(prune3))

        return self.size

    def getSize(self):
        return self.size

    def getNumTransitions(self):
        n = 0
        for i in range(self.getSize()):
            n += self.flow_graph[i].num
        return n

    def getFrame(self, i):
        return self.motion_frames.getPostures(i)

    def getConstraint(self, i):
        return self.motion_frames.getConstraint(i)

    def getFrames(self):
        return self.motion_frames

    def addEntity(self, x, y, v, prior):
        """

        :param x:
        :type x: int
        :param y:
        :type y: int
        :param v:
        :type v: float
        :param prior:
        :type prior: FlowEntity | None
        :return:
        """
        e = FlowEntity()
        e.id = y
        e.value = v

        if prior is not None:
            if prior.next is not None:
                prior.next.prev = e

            e.next = prior.next
            e.prev = prior
            prior.next = e
        else:
            if self.flow_graph[x].entity is not None:
                self.flow_graph[x].entity.prev = e

            e.next = self.flow_graph[x].entity
            e.prev = None
            self.flow_graph[x].entity = e

        self.flow_graph[x].num += 1


    def removeEntity(self, x, y):
        e = self.flow_graph[x].entity
        while e is not None:
            if e.id == y:
                if e.next is not None:
                    e.next.prev = e.prev
                if e.prev is not None:
                    e.prev.next = e.next
                if self.flow_graph[x].entity is e:
                    self.flow_graph[x].entity = e.next

                self.flow_graph[x].num -= 1
                return

            e = e.next

    def getValue(self, x, y):
        e = self.flow_graph[x].entity
        while e is not None:
            if e.id == y:
                return e.value
            elif e.id > y:
                return 0.
            e = e.next

    def setValue(self, x, y, v):
        if v == 0.:
            self.removeEntity(x, y)
        elif self.flow_graph[x].entity is None or \
                (self.flow_graph[x].entity is not None and self.flow_graph[x].entity.id > y):
            self.addEntity(x, y, v, None)
        else:
            e = self.flow_graph[x].entity
            while e is not None:
                if e.id == y:
                    e.value = v
                    return
                elif e.next is None or (e.next is not None and e.next.id > y):
                    self.addEntity(x, y, v, e)
                    return

    def distance(self, f1, f2):
        if f1 == f2:
            # same frames
            return 1
        #TODO:
        # check distance_matrix set value
        d = self.distance_matrix.getValue(f1, f2)
        if d == 0:
            # not included in the knn
            return 0
        else:
            return math.exp(-d/self.variance)

    def segmentation(self):
        self.flow_graph[0].is_break_point = False
        self.flow_graph[self.getSize() - 1].is_break_point = False

        for i in range(1, self.getSize()-1):
            e0 = self.motion_frames.getKineticEnergy(i-1)
            e1 = self.motion_frames.getKineticEnergy(i)
            e2 = self.motion_frames.getKineticEnergy(i+1)

            if e0 > e1 and e1 < e2:
                self.flow_graph[i].is_break_point = True
            else:
                self.flow_graph[i].is_break_point = False

    def setLocalCoordinate(self, lc):
        self.local_coordinate = lc

    def setAngleFixed(self, af):
        self.angle_fixed = af

    def setAngleTolerance(self, t):
        self.angle_tolerance = t

    def setMinJump(self, j):
        self.min_jump = j

    def setThreshold(self, t):
        self.threshold = t

    def setPelvisWeight(self, w):
        self.pelvis_weight = w

    def setVariance(self, v):
        """

        :param v:
        :type v: float
        :return:
        """
        self.variance = v

    def setPelvisConstraints(self, speed):
        self.motion_frames.setPelvisConstraints(speed)

    def setHandConstraints(self, palm_speed, hand_speed):
        self.motion_frames.setHandConstraints(palm_speed, hand_speed)

    def setFootConstraints(self, ts, th, _as, ah):
        self.motion_frames.setFootConstraints(ts, th, _as, ah)

    def setTransitionAtContactChange(self, b):
        self.transition_at_contact_change = b

    def setTransitionAtLeftHeelStrike(self, b):
        self.transition_at_left_heel_strike = b

    def setTransitionAtLeftToeOff(self, b ):
        self.transition_at_left_toe_off = b

    def setTransitionAtRightHeelStrike(self, b):
        self.transition_at_right_heel_strike = b

    def setTransitionAtRightToeOff(self, b ):
        self.transition_at_right_toe_off = b

    def getNextPosture(self, p_cur, p_next, next_frame, calib=None):
        # TODO:
        #if calib is not None, calib should be call by reference
        p_prev = self.getFrame(next_frame - 1)

        t1 = p_cur.getGlobalTransf(PELVIS)
        t2 = p_prev.getGlobalTransf(PELVIS)

        t1 = PlaneProject(p_cur)
        t2 = PlaneProject(p_prev)

        calib = t2.inverse() * t1

        if self.angle_fixed:
            calib = translate_transf(calib.getTranslation())

        p_next.setTransf(PELVIS, p_next.getTransf(PELVIS) * calib)

    def getNextTransf(self, p_cur, next_frame):
        p_prev = self.getFrame(next_frame - 1).getGlobalTransf(PELVIS)
        p_next = self.getFrame(next_frame).getGlobalTransf(PELVIS)

        t1 = PlaneProject(p_cur)
        t2 = PlaneProject(p_prev)

        calib = t2.inverse() * t1

        if self.angle_fixed:
            calib = translate_transf(calib.getTranslation())

        return p_next * calib

    # used for scc
    StartTime = 0
    stack = list()
    visit_count = 0

    # strongly connected component
    def stronglyConnectedComponents(self):
        self.StartTime = 0

        del self.stack[:]

        for i in range(self.size):
            self.flow_graph[i].scc_in_stack = False
            self.flow_graph[i].scc_visited = False

        max_cluster_seed = 0
        max_cluster_size = 0

        for i in range(self.size):
            if not self.flow_graph[i].scc_visited:
                self.visit_count = 0
                self.SCC(i)

                if self.visit_count > max_cluster_size:
                    max_cluster_size = self.visit_count
                    max_cluster_seed = self.scc_seed

        return max_cluster_seed

    def SCC(self, v):
        self.StartTime += 1

        self.flow_graph[v].scc_low = self.StartTime
        self.flow_graph[v].scc_number = self.StartTime

        self.flow_graph[v].scc_visited = True
        self.flow_graph[v].scc_in_stack = True

        self.stack.append(v)

        self.visit_count += 1

        e = self.flow_graph[v].entity
        while e is not None:
            if not self.flow_graph[e.id].scc_visited:
                self.SCC(e.id)
                self.flow_graph[v].scc_low = min(self.flow_graph[v].scc_low, self.flow_graph[e.id].scc_low)
            elif self.flow_graph[e.id].scc_number < self.flow_graph[v].scc_number \
                    and self.flow_graph[e.id].scc_in_stack:
                self.flow_graph[v].scc_low = min(self.flow_graph[v].scc_low, self.flow_graph[e.id].scc_number)
            e = e.next

        if self.flow_graph[v].scc_low == self.flow_graph[v].scc_number:
            while self.stack and self.flow_graph[self.stack[-1]].scc_number >= self.flow_graph[v].scc_number:
                if v != self.stack[-1]:
                    self.scc_seed = v
                self.flow_graph[self.stack[-1]].scc_in_stack = False
                self.flow_graph[self.stack[-1]].scc_component = v
                self.stack.pop()


    def preventDeadLock(self, seed):
        self.scc_index = [0] * self.getSize()
        self.scc_size = 0

        for i in range(self.size):
            if self.flow_graph[i].scc_component != seed:
                self.flow_graph[i].scc_component = -1
            else:
                self.scc_index[self.scc_size] = i
                self.scc_size += 1

            e = self.flow_graph[i].entity
            while e is not None:
                if self.flow_graph[e.id].scc_component != seed:
                    self.removeEntity(i, e.id)
                    e = self.flow_graph[i].entity
                else:
                    e = e.next

            # normalize
            sum = 0.
            e = self.flow_graph[i].entity
            while e is not None:
                sum += e.value
                e = e.next

            e = self.flow_graph[i].entity
            while e is not None:
                e.value /= sum
                e = e.next

    # terrain
    def evaluateConnectivity(self):
        pass

    # File IO
    def open(self, filename):
        with open(filename, 'r') as file:
            for i in range(self.getSize()):
                for j in range(self.getSize()):
                    self.setValue(i, j, 0)

            num_of_data = int(file.read())

            for i in range(num_of_data):
                row, col, value = int(file.read()), int(file.read()), float(file.read())
                self.setValue(row, col, value)

            for i in range(self.size):
                sum = 0.
                e = self.flow_graph[i].entity
                while e is not None:
                    sum += e.value
                    e = e.next

                e = self.flow_graph[i].entity
                while e is not None:
                    e.value /= sum
                    e = e.next

    def save(self, filename):
        with open(filename, 'w') as file:
            num_of_data = 0
            for i in range(self.getSize()):
                num_of_data += self.flow_graph[i].num

            file.write('%d\n'.format(num_of_data))

            for i in range(self.getSize()):
                e = self.flow_graph[i].entity
                while e is not None:
                    file.write('%d %d %lf\n'.format(i, e.id, e.value))
                    e = e.next

    def vectorize(self, filename):
        dim = 0
        with open(filename, 'w') as file:
            for i in range(self.motion_frames.getSize()):
                dim = 0
                p = self.motion_frames.getPostures(i)
                v = self.velocity_frames.getVector(i)

                t = PlaneProject(p.getGlobalTransf(0)).inverse()

                file.write('%f'.format(p.getGlobalPosition(0).y()))
                dim += 1

                for j in range(PM_HUMAN_NUM_LINKS):
                    if self.motion_frames.getMask() & MaskBit(j):
                        a = p.getGlobalPosition(j)
                        file.write('%f %f %f'.format(a.x(), a.y(), a.z()))
                        dim += 3

                file.write('\n')

                if i % 100 == 0:
                    print(i, '/', self.motion_frames.getSize())

        print()

        return dim





