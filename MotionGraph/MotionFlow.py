import PyCommon.modules.Resource.ysMotionLoader as yf
from MotionGraph.FlowGraph import FlowGraph

if __name__ == '__main__':
    n_motions = 1
    motion_files = ['wd2_jump0.bvh']
    motions = yf.readBvhFile("../data/woody_walk_normal.bvh")[40:]

    # create FlowGraph

    fGraph = FlowGraph()

