from fltk import Fl
import torch
from DartDeepRNN.ppo.ppo_rnn_dist import PPO
from PyCommon.modules.GUI import hpSimpleViewer as hsv
from PyCommon.modules.Renderer import ysRenderer as yr
import numpy as np

import pydart2 as pydart


def main():
    MOTION_ONLY = False

    pydart.init()

    env_name = 'walk'

    ppo = PPO(env_name, 0, visualize_only=True)
    # if not MOTION_ONLY:
    #     ppo.LoadModel('model/' + env_name + '.pt')
    ppo.envs_send_rnn_motion()
    ppo.env.Resets(False)
    # ppo.replace_motion_num = ppo.rnn_len
    # ppo.env.ref_skel.set_positions(ppo.env.ref_motion.get_q(ppo.env.phase_frame))

    # viewer settings
    rd_contact_positions = [None]
    rd_contact_forces = [None]
    rd_target_position = [None]
    rd_com = [None]

    dart_world = ppo.env.world
    viewer = hsv.hpSimpleViewer(rect=[0, 0, 1280+300, 720+1+55], viewForceWnd=False)
    viewer.doc.addRenderer('MotionModel', yr.DartRenderer(ppo.env.ref_world, (150,150,255), yr.POLYGON_FILL))
    viewer.doc.addRenderer('target', yr.PointsRenderer(rd_target_position, (0, 255, 0)))
    viewer.doc.addRenderer('motion', yr.JointMotionRenderer(ppo.env.ref_motion))
    if not MOTION_ONLY:
        viewer.doc.addRenderer('controlModel', yr.DartRenderer(dart_world, (255,240,255), yr.POLYGON_FILL))
        viewer.doc.addRenderer('contact', yr.VectorsRenderer(rd_contact_forces, rd_contact_positions, (255,0,0)))
        viewer.doc.addRenderer('CM_plane', yr.PointsRenderer(rd_com, (0, 0, 255)))

    def preCallback(frame):
        ppo.env.ref_motion.frame = frame

    def simulateCallback(frame):
        del rd_target_position[:]
        del rd_com[:]
        rd_target_position.append(ppo.env.goals_in_world_frame[ppo.env.phase_frame])
        com = ppo.env.skel.com()
        com[1] = 0.
        rd_com.append(com)

        state = ppo.env.GetState(0)
        action_dist, _ = ppo.model(torch.tensor(state.reshape(1, -1)).float())
        action = action_dist.loc.detach().numpy()
        res = ppo.env.Steps(action)
        # res = [False, False, False]
        print(res[1])

        # res = ppo.env.Steps(np.zeros_like(action))
        # print(frame, ppo.env.ref_skel.current_frame, ppo.env.world.time()*ppo.env.ref_motion.fps)
        # print(frame, res[0][0])
        # if res[0][0] > 0.46:
        #     ppo.env.continue_from_now_by_phase(0.2)
        # print(ppo.env.goal)
        if res[2]:
            print(frame, 'Done')
            ppo.generate_rnn_motion()
            ppo.envs_send_rnn_motion()
            ppo.env.reset()

        # contact rendering
        contacts = ppo.env.world.collision_result.contacts
        del rd_contact_forces[:]
        del rd_contact_positions[:]
        for contact in contacts:
            rd_contact_forces.append(contact.f/1000.)
            rd_contact_positions.append(contact.p)

    viewer.setPreFrameCallback_Always(preCallback)
    viewer.setSimulateCallback(simulateCallback)
    viewer.setMaxFrame(3000)
    viewer.startTimer(1./30.)
    viewer.show()

    Fl.run()


if __name__ == '__main__':
    main()
