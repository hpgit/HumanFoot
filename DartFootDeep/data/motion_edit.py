from PyCommon.modules.Resource import ysMotionLoader as yf
import numpy as np

import os


def editing1(filename, scale=1.):
    bvh = yf.readBvhFileAsBvh(filename)
    for joint in bvh.joints:
        if 'dummy' in joint.name.lower():
            bvh.remove_joint_element(joint.name, preseve_child_offset=True)
        # elif 'toes' in joint.name.lower() and not('effector' in joint.name.lower()):
        #     bvh.remove_joint_element(joint.name)
        # elif 'hand' in joint.name.lower() and not('effector' in joint.name.lower()):
        #     bvh.remove_joint_element(joint.name)
    if bvh.joints[1].name == 'RightUpLeg':
        bvh.swap_joint('RightUpLeg', 'LeftUpLeg')

    bvh.set_scale(scale)
    bvh.joints[0].offset = np.zeros(3)
    bvh.writeBvhFile('./ori_motion/'+os.path.splitext(os.path.basename(filename))[0]+'_edit.bvh')


if __name__ == '__main__':
    editing1('./ori_motion/wd2_1foot_contact_run2.bvh')
    editing1('./ori_motion/wd2_fast_2foot_hop.bvh')
    editing1('./ori_motion/wd2_slow_2foot_hop.bvh')
    editing1('./ori_motion/wd2_long_broad_jump.bvh')
    editing1('./ori_motion/wd2_short_broad_jump.bvh')
    editing1('./ori_motion/wd2_n_kick.bvh')
    editing1('./ori_motion/wd2_boxing_round_round_girl1.bvh', 2.54)
