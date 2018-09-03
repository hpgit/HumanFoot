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
        self.visited = False
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

    def init(self, int, motion_list):
        # Concatenate motion clips
        # motion_frames = new PmLinearMotion(manager->human );
        motion_frames = PmLinearMotion(motion_list[0]->getBody() )
        for (int k=0; k < n_motions; k++)
            motion_frames->concat(*(motion_list[k]));

        printf("Number of motion clips = %d\n", n_motions);
        printf("Number of motion frames = %d\n", motion_frames->getSize() );

        size = motion_frames->getSize();

        flow_graph = new
        FlowEntityHead[size];

        return size;
        pass

    def build(self, int, motion_list):
        pass

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

    def distance(self, int, int):
        pass

    def segmentation(self):
        pass

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

    def getNextPosture(self, PmPosture , PmPosture , int):
        pass

    def getNextPosture(self, PmPosture , PmPosture , int, transf):
        pass

    def getNextTransf(self, transf, int):
        pass

    # strongly connected component
    def stronglyConnectedComponents(self):
        pass

    def SCC(self, int):
        pass

    def preventDeadLock(self, int):
        pass

    # terrain
    def evaluateConnectivity(self):
        pass

    # File IO
    def open(self, char):
        pass

    def save(self, char):
        pass

    def vectorize(self, char):
        pass

