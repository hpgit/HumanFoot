import PyCommon.modules.Resource.ysMotionLoader as yf
from MotionGraph.FlowGraph import FlowGraph
from MotionGraph.FrameDistance import AllPointKNearest

def build_graph():
    n_motions = 1
    motion_files = ['wd2_jump0.bvh']
    # motions = yf.readBvhFile("data/woody_walk_normal.bvh")[40:]

    motions = []

    for i in range(n_motions):
        # motions.append(PmLinearMotion(human))
        # motions[i].openAMC(motion_files[i], 1, 1)
        motions.append(yf.readBvhFileAsBvh(motion_files[i]).toPmLinearMotion(1., False))
    skeleton = motions[0][0].skeleton

    # for i in range(skeleton.getElementNum()):
    #     print(skeleton.getElement(i))

    # create FlowGraph
    fGraph = FlowGraph()
    fGraph.init(n_motions, motions)
    fGraph.setLocalCoordinate(True)
    fGraph.setThreshold(1e-10)
    fGraph.setVariance(2.)
    # fGraph.setMinJump(10)
    fGraph.setMinJump(60)
    fGraph.setPelvisWeight(0.)

    # calculate distances between two frames
    pointSetFile = 'point_set.txt'
    distFile = 'dist.txt'
    poseDimension = fGraph.vectorize(pointSetFile)

    # frame distance
    fGraph.distance_matrix = AllPointKNearest(fGraph.getSize(), poseDimension, fGraph.getSize()//20, pointSetFile, distFile)

    # build up Motion-Graph
    fGraph.build(n_motions, motions)

    # seed = fGraph.stronglyConnectedComponents()
    # fGraph.preventDeadLock(seed)

    return fGraph


if __name__ == '__main__':
    fGraph = build_graph()
    motionGraphFile = 'mg.txt'
    fGraph.save(motionGraphFile)
