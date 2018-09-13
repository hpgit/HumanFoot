from fltk import Fl
from PyCommon.modules.GUI import hpSimpleViewer as hsv
from PyCommon.modules.Renderer import ysRenderer as yr
from PyCommon.modules.Resource import ysMotionLoader as yf
from MotionGraph.MotionFlow import build_graph
import numpy as np

import pydart2 as pydart


def main():
    fGraph = build_graph()

    def get_info(x):
        temp = x.split()
        return tuple([int(temp[0]), int(temp[1]), float(temp[2])])

    motion_idx = [None, None]

    with open('mg.txt', 'r') as f:
        s = f.read().splitlines(False)
        total_num = int(s[0])

        motion_info = list(map(get_info, s[1:]))

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

    viewer.setPreFrameCallback_Always(postCallback)
    viewer.setMaxFrame(total_num-1)
    viewer.startTimer(1./30.)
    viewer.show()

    Fl.run()


if __name__ == '__main__':
    main()
