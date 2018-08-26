from fltk import Fl
import torch
from DartDeep.keras.ppo import PPO
from PyCommon.modules.GUI import hpSimpleViewer as hsv
from PyCommon.modules.Renderer import ysRenderer as yr

import pydart2 as pydart

def main():

    pydart.init()

    env_name = 'walk'

    ppo = PPO(env_name, 1)
    ppo.LoadModel('model/' + env_name + '.pt')
    ppo.env.Resets(False)

    dart_world = ppo.env.world
    viewer = hsv.hpSimpleViewer(rect=(0, 0, 1200, 800), viewForceWnd=False)
    viewer.doc.addRenderer('MotionModel', yr.DartRenderer(ppo.env.ref_world, (150,150,255), yr.POLYGON_FILL))
    viewer.doc.addRenderer('controlModel', yr.DartRenderer(dart_world, (255,240,255), yr.POLYGON_FILL))

    def preCallback(frame):
        ppo.env.ref_skel.set_positions(ppo.env.ref_motion.get_q(frame))

    def simulateCallback(frame):
        state = ppo.env.GetState(0)
        action_dist, _ = ppo.model(torch.tensor(state.reshape(1, -1)).float())
        action = action_dist.loc.detach().numpy()
        res = ppo.env.Steps(action)

    # viewer.setPreFrameCallback_Always(preCallback)
    viewer.setSimulateCallback(simulateCallback)
    viewer.setMaxFrame(len(ppo.env.ref_motion)-1)
    viewer.startTimer(1/30.)
    viewer.show()

    Fl.run()


if __name__ == '__main__':
    main()
