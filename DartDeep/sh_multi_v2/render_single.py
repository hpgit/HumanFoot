from fltk import Fl
# import torch
import numpy as np
from DartDeep.sh_multi_v2.ppo_multi_v2 import PPO_MULTI
from PyCommon.modules.GUI import hpSimpleViewer as hsv
from PyCommon.modules.Renderer import ysRenderer as yr

import pydart2 as pydart


def main():
    MOTION_ONLY = False

    pydart.init()

    env_name = 'multi'

    ppo = PPO_MULTI(env_name, 0, visualize_only=True)
    if not MOTION_ONLY:
        ppo.LoadModel('model/param.pt')
    ppo.env.specify_motion_num(0)

    ppo.env.Resets(True)

    # viewer settings
    rd_contact_positions = [None]
    rd_contact_forces = [None]
    dart_world = ppo.env.world

    viewer = hsv.hpSimpleViewer(rect=(0, 0, 1200, 800), viewForceWnd=False)
    # viewer = hsv.hpSimpleViewer(rect=[0, 0, 960+300, 1+1080+55], viewForceWnd=False)
    viewer.doc.addRenderer('MotionModel', yr.DartRenderer(ppo.env.ref_world, (150,150,255), yr.POLYGON_FILL))
    viewer.doc.addRenderer('controlModel', yr.DartRenderer(dart_world, (255,240,255), yr.POLYGON_FILL))
    viewer.doc.addRenderer('contact', yr.VectorsRenderer(rd_contact_forces, rd_contact_positions, (255,0,0)))

    viewer.setMaxFrame(3000)
    cameraTargets = [None] * (viewer.getMaxFrame()+1)

    def preCallback(frame):
        # ppo.env.ref_skel.set_positions(ppo.env.ref_motion.get_q(frame))
        q = ppo.env.ref_motion.get_q(frame)
        q_noise = np.random.normal(q, 0.05*np.ones_like(q))
        q_noise[3:6] = q[3:6]
        # ppo.env.ref_skel.set_positions(q_noise)
        ppo.env.ref_skel.set_positions(q)

    def simulateCallback(frame):
        state = ppo.env.GetState(0)
        action_dist, _ = ppo.model(state.reshape(1, -1))
        action = action_dist.loc.detach().numpy()
        res = ppo.env.Steps(action)
        if res[2]:
            print(frame, 'Done')
            ppo.env.reset()

        # contact rendering
        contacts = ppo.env.world.collision_result.contacts
        del rd_contact_forces[:]
        del rd_contact_positions[:]
        for contact in contacts:
            rd_contact_forces.append(contact.f/1000.)
            rd_contact_positions.append(contact.p)

    def postFrameCallback_Always(frame):
        if cameraTargets[frame] is None:
            cameraTargets[frame] = np.asarray(dart_world.skel.body(0).to_world())
        viewer.setCameraTarget(cameraTargets[frame])

    if MOTION_ONLY:
        viewer.setPreFrameCallback_Always(preCallback)
    else:
        viewer.setSimulateCallback(simulateCallback)

    viewer.startTimer(1/ppo.env.ref_motion.fps)
    viewer.show()

    Fl.run()


if __name__ == '__main__':
    main()
