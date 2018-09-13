from fltk import Fl
import torch
from DartDeep.sh_v3.ppo_v3 import PPO
from PyCommon.modules.GUI import hpSimpleViewer as hsv
from PyCommon.modules.Renderer import ysRenderer as yr
import numpy as np

import pydart2 as pydart


def main():
    MOTION_ONLY = False

    pydart.init()

    env_name = 'walk'

    ppo = PPO(env_name, 1, visualize_only=True)
    ppo.env.visualize = True
    if not MOTION_ONLY:
        ppo.LoadModel('model/' + env_name + '.pt')
    ppo.env.Resets(False)
    ppo.env.ref_skel.set_positions(ppo.env.ref_motion.get_q_by_time(ppo.env.time_offset))

    # viewer settings
    rd_contact_positions = [None]
    rd_contact_forces = [None]
    dart_world = ppo.env.world
    viewer = hsv.hpSimpleViewer(rect=(0, 0, 1200, 800), viewForceWnd=False)
    viewer.doc.addRenderer('MotionModel', yr.DartRenderer(ppo.env.ref_world, (150,150,255), yr.POLYGON_FILL))

    motion_state = [0]

    if not MOTION_ONLY:
        viewer.doc.addRenderer('controlModel', yr.DartRenderer(dart_world, (255,240,255), yr.POLYGON_FILL))
        viewer.doc.addRenderer('contact', yr.VectorsRenderer(rd_contact_forces, rd_contact_positions, (255,0,0)))

        def callback_0(_):
            motion_state[0] = 0

        def callback_1(_):
            motion_state[0] = 1

        def callback_2(_):
            motion_state[0] = 2

        viewer.objectInfoWnd.addBtn('0', callback_0)
        viewer.objectInfoWnd.addBtn('1', callback_1)
        viewer.objectInfoWnd.addBtn('2', callback_2)

    def preCallback(frame):
        ppo.env.ref_skel.set_positions(ppo.env.ref_motion.get_q(frame))

    def simulateCallback(frame):
        print(frame)
        state = ppo.env.GetState(0)
        action_dist, _ = ppo.model(torch.tensor(state.reshape(1, -1)).float())
        action = action_dist.loc.detach().numpy()
        ppo.env.visualize_select_motion = motion_state[0]
        res = ppo.env.Steps(action)
        # print(res[0][0])
        # res = ppo.env.Steps(np.zeros_like(action))
        # print(frame, ppo.env.ref_skel.current_frame, ppo.env.world.time()*ppo.env.ref_motion.fps)
        # print(frame, res[0][0])
        # if res[0][0] > 0.46:
        #     ppo.env.continue_from_now_by_phase(0.2)
        # if res[2]:
        #     print(frame, 'Done')
        #     ppo.env.reset()

        # contact rendering
        contacts = ppo.env.world.collision_result.contacts
        del rd_contact_forces[:]
        del rd_contact_positions[:]
        for contact in contacts:
            rd_contact_forces.append(contact.f/1000.)
            rd_contact_positions.append(contact.p)

    if MOTION_ONLY:
        viewer.setPreFrameCallback_Always(preCallback)
        viewer.setMaxFrame(ppo.env.motion_len-1)
    else:
        viewer.setSimulateCallback(simulateCallback)
        viewer.setMaxFrame(3000)
    # viewer.setMaxFrame(len(ppo.env.ref_motion)-1)
    viewer.startTimer(1./30.)
    viewer.show()

    Fl.run()


if __name__ == '__main__':
    main()
