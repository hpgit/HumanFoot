from fltk import Fl
import torch
from DartDeep.sh_v2.ppo_v2 import PPO
from PyCommon.modules.GUI import hpSimpleViewer as hsv
from PyCommon.modules.Renderer import ysRenderer as yr

import pydart2 as pydart


def main():
    MOTION_ONLY = False

    pydart.init()

    env_name = 'multi'

    ppo = PPO(env_name, 1, visualize_only=True)
    if not MOTION_ONLY:
        ppo.LoadModel('model/' + env_name + '.pt')
    ppo.env.specify_motion_num(1)

    ppo.env.Resets(False)

    # viewer settings
    rd_contact_positions = [None]
    rd_contact_forces = [None]
    dart_world = ppo.env.world
    viewer = hsv.hpSimpleViewer(rect=(0, 0, 1200, 800), viewForceWnd=False)
    viewer.doc.addRenderer('MotionModel', yr.DartRenderer(ppo.env.ref_world, (150,150,255), yr.POLYGON_FILL))
    viewer.doc.addRenderer('controlModel', yr.DartRenderer(dart_world, (255,240,255), yr.POLYGON_FILL))
    viewer.doc.addRenderer('contact', yr.VectorsRenderer(rd_contact_forces, rd_contact_positions, (255,0,0)))

    def preCallback(frame):
        ppo.env.ref_skel.set_positions(ppo.env.ref_motion.get_q(frame))

    def simulateCallback(frame):
        state = ppo.env.GetState(0)
        action_dist, _ = ppo.model(torch.tensor(state.reshape(1, -1)).float())
        action = action_dist.loc.detach().numpy()
        res = ppo.env.Steps(action)

        # contact rendering
        contacts = ppo.env.world.collision_result.contacts
        del rd_contact_forces[:]
        del rd_contact_positions[:]
        for contact in contacts:
            rd_contact_forces.append(contact.f/1000.)
            rd_contact_positions.append(contact.p)

    if MOTION_ONLY:
        viewer.setPreFrameCallback_Always(preCallback)
    else:
        viewer.setSimulateCallback(simulateCallback)
    viewer.setMaxFrame(len(ppo.env.ref_motion)-1)
    viewer.startTimer(1/ppo.env.ref_motion.fps)
    viewer.show()

    Fl.run()


if __name__ == '__main__':
    main()
