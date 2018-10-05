from fltk import Fl
from PyCommon.modules.GUI import hpSimpleViewer as hsv
from PyCommon.modules.Renderer import ysRenderer as yr
from PyCommon.modules.Resource import ysMotionLoader as yf
from PyCommon.modules.Motion import hpMotionGraph as hmg
import numpy as np

from graphviz import Digraph

import pydart2 as pydart


def main():
    def get_info(x):
        temp = x.split()
        return tuple([int(temp[0]), int(temp[1]), float(temp[2])])

    motion_idx = [None, None]

    pydart.init()
    world = [pydart.World(1./1200., "../DartDeep/data/woody_with_ground_v2.xml") for _ in range(2)]

    # viewer settings
    rd_contact_positions = [None]
    rd_contact_forces = [None]
    viewer = hsv.hpSimpleViewer(rect=(0, 0, 1200, 800), viewForceWnd=False)
    viewer.doc.addRenderer('motionModel[0]', yr.DartRenderer(world[0], (255, 240, 255)))
    viewer.doc.addRenderer('motionModel[1]', yr.DartRenderer(world[1]))

    motion_state = [0]

    def callback_0(_):
        motion_state[0] = 0

    def callback_1(_):
        motion_state[0] = 1

    def callback_2(_):
        motion_state[0] = 2

    viewer.objectInfoWnd.addBtn('0', callback_0)
    viewer.objectInfoWnd.addBtn('1', callback_1)
    viewer.objectInfoWnd.addBtn('2', callback_2)

    def postCallback(frame):
        for i in range(2):
            world[i].skeletons[1].set_positions(fGraph.motion_frames.get_q(motion_info[frame][i]))
        print(frame, motion_info[frame])

    # viewer.setPreFrameCallback_Always(postCallback)
    viewer.setMaxFrame(total_num-1)
    viewer.startTimer(1./30.)
    viewer.show()

    Fl.run()


def main2():
    import PyCommon.modules.Motion.ysMotionAnalysis as yma
    pydart.init()
    world = [pydart.World(1./1200., "../DartDeep/data/woody_with_ground_v2.xml") for _ in range(2)]

    motion_files = ['wd2_jump0.bvh']
    motion_files = ['woody_walk_normal.bvh']
    motion_files = ['walk_left_90degree.bvh']
    motion = yf.readBvhFileAsBvh(motion_files[0]).toPmLinearMotion(1., False)
    # 70, 112, 156 right heel strike
    mg = hmg.MotionGraph()
    mg.add_motion(motion)
    mg.build()
    total_num = len(mg.transition_processed)

    # viewer settings
    rd_contact_positions = [None]
    rd_contact_forces = [None]
    viewer = hsv.hpSimpleViewer(rect=(0, 0, 1200, 800), viewForceWnd=False)
    viewer.doc.addRenderer('motionModel[0]', yr.DartRenderer(world[0], (255, 240, 255)))
    viewer.doc.addRenderer('motionModel[1]', yr.DartRenderer(world[1]))

    motion_state = [0]

    def callback_0(_):
        motion_state[0] = 0

    def callback_1(_):
        motion_state[0] = 1

    def callback_2(_):
        motion_state[0] = 2

    viewer.objectInfoWnd.addBtn('0', callback_0)
    viewer.objectInfoWnd.addBtn('1', callback_1)
    viewer.objectInfoWnd.addBtn('2', callback_2)

    hRef = 0.1
    vRef = 0.4
    lc = yma.getElementContactStates(motion, 'LeftFoot', hRef, vRef)
    rc = yma.getElementContactStates(motion, 'RightFoot', hRef, vRef)
    lindex = motion[0].skeleton.getElementIndex('LeftFoot')
    rindex = motion[0].skeleton.getElementIndex('RightFoot')

    def postCallback(frame):
        transition = mg.transition_processed[frame]
        world[0].skeletons[1].set_positions(motion.get_q(transition.motion_from_idx))
        world[1].skeletons[1].set_positions(motion.get_q(transition.motion_to_idx))
        print(frame, transition.motion_from_idx, transition.motion_to_idx, transition.dist)
        print(motion.getPosition(lindex))
        print(lc[transition.motion_from_idx], rc[transition.motion_from_idx])
        print(lc[transition.motion_to_idx], rc[transition.motion_to_idx])

    viewer.setPreFrameCallback_Always(postCallback)
    viewer.setMaxFrame(total_num-1)
    viewer.startTimer(1./30.)
    viewer.show()

    all_node_set = set()
    dot = Digraph()
    for i in range(len(mg.transition_processed)):
        all_node_set.add(mg.transition_processed[i].motion_from_idx)
        all_node_set.add(mg.transition_processed[i].motion_to_idx)

    all_node_list = list(all_node_set)
    all_node_list.sort()

    for i in all_node_list:
        dot.node(str(i))

    all_edge_set = set()
    for i in range(len(all_node_list)-1):
        all_edge_set.add((all_node_list[i], all_node_list[i+1]))

    for i in range(len(mg.transition_processed)):
        all_edge_set.add((mg.transition_processed[i].motion_from_idx, mg.transition_processed[i].motion_to_idx))

    for i in list(all_edge_set):
        dot.edge(str(i[0]), str(i[1]))

    # print(dot.source)
    dot.render('test', view=True)
    dot.pipe()

    Fl.run()


if __name__ == '__main__':
    # main()
    main2()
    # from dynamicgraphviz.graph.directedgraph import DirectedGraph
    # from dynamicgraphviz.gui.graphDrawer import GraphDrawer
    #
    # g = DirectedGraph()
    #
    # v1 = g.add_node()
    # v2 = g.add_node()
    #
    # a1 = g.add_arc(v1, v2)
    #
    # gd = GraphDrawer(g)
    # gd.place_nodes(doanimate=True)
    # gd.pause()
    #

