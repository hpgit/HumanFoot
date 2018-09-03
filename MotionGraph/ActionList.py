class Action:
    def __init__(self):
        self.id = 0
        self.start = 0
        self.end = 0
        self.transition = None  # type: FlowEntity

    def getLength(self):
        return self.end - self.start


class ActionList:
    def __init__(self):
        self.num_actions = 0
        self.size = 0
        self.actions = list()  # type: list[Action]
        self.graph = None  # type: FlowGraph

    def __getitem__(self, item):
        assert isinstance(item, int)
        return self.actions[item]

    def setNumActions(self, n):
        if n > self.size:
            if self.size > 0:
                del self.actions[:]

            self.actions = [Action() for i in range(n)]
            self.size = n

        self.num_actions = n

    def getNumActions(self):
        return self.num_actions

    def initialize(self, g):
        self.graph = g
        graph = self.graph

        num_branches = 0

        for i in range(graph.getSize()):
            if graph.flow_graph[i].num > 1:
                num_branches += graph.flow_graph[i].num

        self.setNumActions(num_branches)

        j = 0

        for i in range(graph.getSize()):
            if graph.flow_graph[i].num > 1:
                e = graph.flow_graph[i].entity
                while e is not None:
                    f = e
                    while graph.flow_graph[f.id].num < 2:
                        f.action = self.actions[j]
                        f = graph.flow_graph[f.id].entity

                    self.actions[j].start = e.id
                    self.actions[j].end = f.id
                    self.actions[j].transition = e
                    j += 1

                    if e.id == f.id:
                        print('A short segment (%d, %d) detected'.format(i, e.id))

                    e = e.next

        for i in range(self.num_actions):
            self.actions[i].id = i

        print("# of branches is %d (%d)".format(num_branches, j))

    def instantiate(self, n, m):
        action = self.actions[n]
        m.setSize(action.getLength())

        for i in range(action.getLength()):
            m.setPosture(i, self.graph.getFrame(i + action.start))
