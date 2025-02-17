import os
from fltk import *

#os.chdir('''D:/Research/2009_BipedSimulation/Tools''')
#os.chdir('''D:/Research/2009.2_BipedSimulation/Code/Momentum/Tools''')

import sys
if '..' not in sys.path:
    sys.path.append('..')
import PyCommon.modules.Resource.ysMotionLoader as yf
import PyCommon.modules.Renderer.ysRenderer as yr
import PyCommon.modules.GUI.ysSimpleViewer as ysv
import PyCommon.modules.Motion.ysMotion as ym

def usuage(scriptName):
    print('Usage: %s bvh_file'%scriptName)

class BvhViewer(ysv.SimpleViewer):
    def __init__(self, rect=None, title='BvhViewer'):
        ysv.SimpleViewer.__init__(self, rect, title)
    def open(self, bvhFilePath):
        fl_cursor(FL_CURSOR_WAIT)
        
        self.initialize()
        self.record(False)
        
        jointMotion = yf.readBvhFile(bvhFilePath, 1.)
        tpose = jointMotion[0].getTPose()
        
        print(jointMotion[0].skeleton)
        print(jointMotion.fps, 'fps')
        
        self.doc.addRenderer('jointMotion_BOX', yr.JointMotionRenderer(jointMotion, (0,127,255), yr.LINK_SOLIDBOX), False)
        self.doc.addRenderer('jointMotion_LINE', yr.JointMotionRenderer(jointMotion, (0,127,255), yr.LINK_LINE), True)
        self.doc.addRenderer('jointMotion_BONE', yr.JointMotionRenderer(jointMotion, (0,127,255), yr.LINK_BONE), False)
        # self.doc.addRenderer('tpose', yr.JointMotionRenderer(ym.JointMotion([tpose]), (255,0,0), yr.LINK_BONE), True)
        self.doc.addObject('jointMotion', jointMotion)
        
        title = '%s - BvhViewer'%os.path.basename(bvhFilePath)
        self.label(title)
        self.iconlabel(title)
        
        # self.setTimeInterval(1./jointMotion.fps)
        self.startTimer(1./jointMotion.fps)

        fl_cursor(FL_CURSOR_DEFAULT)
    
def dnd_handle(event):
    global bvhViewer
    if event == FL_DND_DRAG:
        return 1
    if event == FL_DND_RELEASE:
        cl = Fl.event_text()
        ln = Fl.event_length()
        bvhViewer.open(cl)
        return 1
    return 0

bvhViewer = None    

if __name__ == "__main__":
    if len(sys.argv) > 2:
        usuage(os.path.basename(sys.argv[0]))
    else:
        if len(sys.argv) == 2:
            bvhFilePath = sys.argv[1]
        else:
            bvhFilePath = None
        
        bvhViewer = BvhViewer()
        bvhViewer.record(False)

        bvhFilePath = '../../../data/woody2/Motion/Balancing/wd2_1_kick.bvh'
        bvhFilePath = '../../../data/woody2/Motion/Balancing/wd2_1_kick.bvh'
        bvhFilePath = '../../../data/woody2/Motion/Balancing/wd2_1_kick.bvh'
        bvhFilePath = '../../../data/woody2/Motion/Balancing/wd2_1_kick.bvh'
        bvhFilePath = '../../../data/woody2/Motion/Balancing/wd2_1_kick.bvh'
        bvhFilePath = '../../../data/woody2/Motion/Balancing/wd2_1_kick.bvh'
        bvhFilePath = '../../../data/woody2/Motion/Balancing/wd2_1foot_contact_run.bvh'
        bvhFilePath = '../../../data/woody2/Motion/Balancing/wd2_slow_2foot_hop.bvh'
        bvhFilePath = '../../../data/woody2/Motion/Balancing/wd2_1foot_contact_run2.bvh'
        bvhFilePath = '../../../data/woody2/Motion/Balancing/wd2_falling_down.bvh'
        bvhFilePath = '../../../MomentumProject/woddy2_jump1.bvh'
        bvhFilePath = '../../../MomentumProject/MotionFile/cmu/15_07_15_07.bvh'

        if bvhFilePath is not None:
            bvhViewer.open(bvhFilePath)
            
        bvhViewer.startTimer((1/30.)/1.4)
        bvhViewer.show()
        
        Fl.add_handler(dnd_handle)
        Fl.run()    
