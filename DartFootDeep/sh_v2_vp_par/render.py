import os
from fltk import Fl
import torch
from DartFootDeep.sh_v2_vp_par.ppo_v2_single_thread import PPO
from PyCommon.modules.GUI import hpSimpleViewer as hsv
from PyCommon.modules.Renderer import ysRenderer as yr
import numpy as np

from PyCommon.modules.Simulator import csVpWorld as cvw


def main():
    MOTION_ONLY = True
    CURRENT_CHECK = False

    cvw.vp_init()

    env_name = 'walk'

    ppo = PPO(env_name, 1, visualize_only=True)
    env = ppo.env.envs[0]

    if not MOTION_ONLY and not CURRENT_CHECK:
        ppo.LoadModel('model/' + env_name + '.pt')
    elif not MOTION_ONLY and CURRENT_CHECK:
        env_model_dir = []
        for dir_name in sorted(os.listdir()):
            if 'walk' in dir_name:
                env_model_dir.append(dir_name)

        pt_names = os.listdir(env_model_dir[-1])
        pt_names.pop(pt_names.index('log.txt'))
        pt_names.sort(key=lambda f: int(os.path.splitext(f)[0]))
        ppo.LoadModel(env_model_dir[-1]+'/'+pt_names[-1])

    env.Resets(False)
    env.ref_skel.set_q(env.ref_motion.get_q(env.phase_frame))

    # viewer settings
    rd_contact_positions = [None]
    rd_contact_forces = [None]
    viewer = hsv.hpSimpleViewer(rect=(0, 0, 1200, 800), viewForceWnd=False)
    viewer.doc.addRenderer('MotionModel', yr.VpModelRenderer(env.ref_skel, (150,150,255), yr.POLYGON_FILL))
    if not MOTION_ONLY:
        viewer.doc.addRenderer('controlModel', yr.VpModelRenderer(env.skel, (255,240,255), yr.POLYGON_FILL))
        viewer.doc.addRenderer('contact', yr.VectorsRenderer(rd_contact_forces, rd_contact_positions, (255,0,0)))

    def postCallback(frame):
        env.ref_skel.set_positions(env.ref_motion.get_q(frame))

    def simulateCallback(frame):
        state = env.GetState(0)
        action_dist, _ = ppo.model(torch.tensor(state.reshape(1, -1)).float())
        action = action_dist.loc.detach().numpy()
        res = env.Steps(action)
        # res = ppo.env.Steps(np.zeros_like(action))
        # print(frame, ppo.env.Ref_skel.current_frame, ppo.env.world.time()*ppo.env.ref_motion.fps)
        # print(frame, res[0][0])
        # if res[0][0] > 0.46:
        #     ppo.env.continue_from_now_by_phase(0.2)
        if res[2]:
            print(frame, 'Done')
            env.reset()

        # contact rendering
        del rd_contact_forces[:]
        del rd_contact_positions[:]
        # for contact in contacts:
        #     rd_contact_forces.append(contact.f/1000.)
        #     rd_contact_positions.append(contact.p)

    if MOTION_ONLY:
        viewer.setPostFrameCallback_Always(postCallback)
        viewer.setMaxFrame(len(env.ref_motion)-1)
    else:
        viewer.setSimulateCallback(simulateCallback)
        viewer.setMaxFrame(3000)
    viewer.startTimer(1./30.)
    viewer.show()

    Fl.run()


if __name__ == '__main__':
    main()
