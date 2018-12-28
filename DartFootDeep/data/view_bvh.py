from fltk import Fl
from PyCommon.modules.GUI import hpSimpleViewer as hsv
from PyCommon.modules.Renderer import ysRenderer as yr
from PyCommon.modules.Resource import ysMotionLoader as yf
import numpy as np

from random import randrange


def main(filenames):
    motion = []
    for filename in filenames:
        motion.append(yf.readBvhFile(filename))

    # viewer settings
    rd_contact_positions = [None]
    rd_contact_forces = [None]
    viewer = hsv.hpSimpleViewer(title='Bvh Viewer', rect=(0, 0, 1200, 800), viewForceWnd=False)
    # viewer.doc.addRenderer('MotionModel', yr.DartRenderer(ppo.env.ref_world, (150,150,255), yr.POLYGON_FILL))
    for motion_idx in range(len(motion)):
        viewer.doc.addRenderer('motion'+str(motion_idx), yr.JointMotionRenderer(motion[motion_idx], (randrange(256), randrange(256), randrange(256))))

    def preCallback(frame):
        for _motion in motion:
            _motion.frame = min(frame, len(_motion)-1)

    viewer.setPostFrameCallback_Always(preCallback)
    viewer.setMaxFrame(max([len(_motion)-1 for _motion in motion]))
    viewer.startTimer(1./30.)
    viewer.show()

    Fl.run()


if __name__ == '__main__':
    # main(['wd2_1foot_contact_run2.bvh', 'wd2_1foot_contact_run2_edit.bvh'])
    main(['wd2_1foot_contact_run2_edit.bvh', 'wd2_fast_2foot_hop_edit.bvh', 'wd2_slow_2foot_hop_edit.bvh', 'wd2_long_broad_jump_edit.bvh', 'wd2_short_broad_jump_edit.bvh', 'wd2_n_kick_edit.bvh', 'wd2_boxing_round_round_girl1_edit.bvh'])
    # main(['segfoot_wd2_n_kick_edit.bvh'])

    # main('../../data/woody2/Motion/Balancing/wd2_1foot_contact_run2.bvh')
    # main('../../data/woody2/Motion/Balancing/wd2_fast_2foot_hop.bvh')
    # main('../../data/woody2/Motion/Balancing/wd2_slow_2foot_hop.bvh')
    # main('../../data/woody2/Motion/Balancing/wd2_long_broad_jump.bvh')
    # main('../../data/woody2/Motion/Balancing/wd2_short_broad_jump.bvh')
    # main('../../data/woody2/Motion/Balancing/wd2_n_kick.bvh')

    # main('../../data/woody2/Motion/samsung_boxing/round/boxing_round_round_girl1.bvh')[505:658]


