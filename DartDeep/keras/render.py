from fltk import Fl
import torch
from DartDeep.keras.ppo import PPO
from PyCommon.modules.GUI import hpSimpleViewer as hsv
from PyCommon.modules.Renderer import ysRenderer as yr

import pydart2 as pydart

def main():

    pydart.init()

    ppo = PPO('jump', 1)
    ppo.LoadModel('model/1473.pt')
    ppo.env.Resets(False)

    dart_world = ppo.env.world
    viewer = hsv.hpSimpleViewer(rect=(0, 0, 1000, 800), viewForceWnd=False)
    viewer.doc.addRenderer('MotionModel', yr.DartRenderer(ppo.env.ref_world, (150,150,255), yr.POLYGON_FILL))
    viewer.doc.addRenderer('controlModel', yr.DartRenderer(dart_world, (255,240,255), yr.POLYGON_FILL))

    def simulateCallback(frame):
        state = ppo.env.GetState(0)
        action_dist, _ = ppo.model(torch.tensor(state.reshape(1, -1)).float())
        action = action_dist.loc.detach().numpy()
        res = ppo.env.Steps(action)

    viewer.setSimulateCallback(simulateCallback)
    viewer.setMaxFrame(len(ppo.env.ref_motion))
    viewer.startTimer(1/30.)
    viewer.show()

    Fl.run()


if __name__ == '__main__':
    main()
