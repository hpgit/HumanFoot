from fltk import *

import sys
if '..' not in sys.path:
    sys.path.append('..')
import Motion.ysMotion as ym
import GUI.ysBaseUI as ybu
import GUI.ysViewer3 as yv3
import GUI.tree as tree

# EVENTS
EV_addRenderer           = 0
EV_setRendererVisible    = 1
EV_addObject             = 2
EV_selectObjectElement   = 3
EV_selectObject          = 4

class SimpleSetting(ybu.BaseSettings):
    def __init__(self):
        ybu.BaseSettings.__init__(self)
        self.camera = yv3.Camera().__dict__
        self.ortho = False
        self.viewMode = yv3.VIEW_PERSPECTIVE
        self.prevRotX = 0
        self.prevRotY = 0
#        self.infoWndIdx = 0
    def setToApp(self, window):
        ybu.BaseSettings.setToApp(self, window)
        window.motionViewWnd.glWindow.camera.__dict__ = self.camera
        window.motionViewWnd.glWindow.projectionOrtho = self.ortho
        window.motionViewWnd.glWindow.viewMode = self.viewMode
        window.motionViewWnd.glWindow.prevRotX = self.prevRotX
        window.motionViewWnd.glWindow.prevRotY = self.prevRotY
#        window.objectInfoWnd.currentChildWndIdx = self.infoWndIdx
    def getFromApp(self, window):
        ybu.BaseSettings.getFromApp(self, window)
        self.camera = window.motionViewWnd.glWindow.camera.__dict__
        self.ortho = window.motionViewWnd.glWindow.projectionOrtho
        self.viewMode = window.motionViewWnd.glWindow.viewMode
        self.prevRotX = window.motionViewWnd.glWindow.prevRotX
        self.prevRotY = window.motionViewWnd.glWindow.prevRotY
#        self.infoWndIdx = window.objectInfoWnd.currentChildWndIdx

class SimpleViewer(ybu.BaseWnd):
    def __init__(self, rect=None, title='SimpleViewer'):
        ybu.BaseWnd.__init__(self, rect, title, SimpleSetting())
        self.doc = SimpleDoc()
        self.begin()
        panelWidth = 180
        self.motionViewWnd = MotionViewWnd(0, 0, self.w()-panelWidth, self.h(), self.doc)
        t = .3
        self.renderersWnd = RenderersWnd(self.w()-panelWidth, 0, panelWidth, int(self.h()*t), self.doc)
        self.objectInfoWnd = ObjectInfoWnd(self.w()-panelWidth, int(self.h()*t), panelWidth, int(self.h()*(1-t)), self.doc)
        self.end()
        self.resizable(self.motionViewWnd)
        self.size_range(600, 400)
    def startTimer(self, timeInterval):
        self.motionViewWnd.startTimer(timeInterval)
    def endTimer(self):
        self.motionViewWnd.endTimer()
    def show(self):
        ybu.BaseWnd.show(self)
        self.motionViewWnd.show()
    def setPreFrameCallback(self, callback):
        self.motionViewWnd.preFrameCallback = callback
    def setPreFrameCallback_Always(self, callback):
        self.motionViewWnd.preFrameCallback_Always = callback
    def setSimulateCallback(self, callback):
        self.motionViewWnd.simulateCallback = callback
    def setPostFrameCallback(self, callback):
        self.motionViewWnd.postFrameCallback = callback
    def setPostFrameCallback_Always(self, callback):
        self.motionViewWnd.postFrameCallback_Always = callback
    def setExtraDrawCallback(self, callback):
        self.motionViewWnd.glWindow.extraDrawCallback = callback
#    def setRecSimulObjs(self, objs):
#        self.motionViewWnd.setRecSimulObjs(objs)
    def getMaxFrame(self):
        return self.motionViewWnd.getMaxFrame()
    def setMaxFrame(self, maxFrame):
        self.motionViewWnd.setMaxFrame(maxFrame)
    def record(self, bRec):
        self.motionViewWnd.record(bRec)
    def play(self):
        self.motionViewWnd.play()
    def setCurrentFrame(self, frame):
        self.motionViewWnd.setCurrentFrame(frame)
    def getCurrentFrame(self):
        return self.motionViewWnd.getCurrentFrame()
    def setCameraTarget(self, targetPos):
        self.motionViewWnd.glWindow.camera.center[0] = targetPos[0]
        self.motionViewWnd.glWindow.camera.center[2] = targetPos[2]
    def initialize(self):
        self.doc.initialize()
        self.motionViewWnd.initialize()
        
class SimpleDoc(ybu.Subject):
    def __init__(self):
        ybu.Subject.__init__(self)
        
        self.rendererNames = []
        self.rendererMap = {}
        self.renderersVisible = {}

        self.motionNames = []
        self.motionMap = {}
        self.motionSystem = ym.MotionSystem()
        
        self.objectNames = []
        self.objectMap = {}
        self.selectedObject = None
    def initialize(self):
        self.removeAllRenderers()
        self.removeAllObjects()
    def removeAllRenderers(self):
        del self.rendererNames[:]
        self.rendererMap.clear()
        self.renderersVisible.clear()
        self.notify(EV_addRenderer)
    def removeAllObjects(self):
        del self.objectNames[:]
        self.objectMap.clear()
        self.motionSystem.removeAllMotions()
    def addRenderer(self, name, renderer, visible=True):
        self.rendererNames.append(name)
        self.rendererMap[name] = renderer
        self.renderersVisible[name] = visible
        self.notify(EV_addRenderer)
    def setRendererVisible(self, name, visible):
        self.renderersVisible[name] = visible
        self.notify(EV_setRendererVisible)
    def getVisibleRenderers(self):
        ls = []
        for name, renderer in self.rendererMap.items():
            if self.renderersVisible[name]:
                ls.append(renderer)
        return ls
    def addObject(self, name, object):
        self.objectNames.append(name)
        self.objectMap[name] = object
        if isinstance(object, ym.Motion):
            self.motionSystem.addMotion(object)
        self.notify(EV_addObject)
    def selectObjectElement(self, element):
        for renderer in self.rendererMap.values():
            renderer.selectedElement = element 
        self.notify(EV_selectObjectElement)
    def selectObject(self, objectName):
        self.selectedObject = self.objectMap[objectName]
        self.notify(EV_selectObject)
    
class MotionViewWnd(yv3.MotionViewer, ybu.Observer):
    def __init__(self, x, y, w, h, doc):
        yv3.MotionViewer.__init__(self, x, y, w, h)
        self.doc = doc
        self.doc.attach(self)
    def update(self, ev, doc):
        if ev==EV_addRenderer or ev==EV_setRendererVisible:
            self.setRenderers(doc.getVisibleRenderers())
        elif ev==EV_addObject:
            self.setMotionSystem(doc.motionSystem)
            self.setStateObjects(doc.objectMap.values())
        self.glWindow.redraw()

class RenderersWnd(Fl_Window, ybu.Observer):
    def __init__(self, x, y, w, h, doc):
        Fl_Window.__init__(self, x, y, w, h)
        self.doc = doc
        self.doc.attach(self)
        self.box(FL_PLASTIC_UP_BOX)
        self.begin()
        self.rx = 5; self.ry = 5; self.rw = w-10; self.rh = h-10
        self.renderersChk = Fl_Check_Browser(self.rx,self.ry,self.rw,self.rh,'')
        self.renderersChk.type(FL_MULTI_BROWSER)
#        self.renderersChk.callback(self.onClickBrowser)
        self.end()
    def update(self, ev, doc):
        if ev==EV_addRenderer or ev==EV_setRendererVisible:
            self.renderersChk.clear()
            for name in doc.rendererNames:
                self.renderersChk.add(name, doc.renderersVisible[name])
    def onClickBrowser(self, x, y):
        i = (y-2)/16
        if i>=0 and i<self.renderersChk.nitems():
            self.doc.setRendererVisible(self.renderersChk.text(i+1), not self.renderersChk.checked(i+1))
    def handle(self, event):
        if event == FL_PUSH:
            x = Fl.event_x()
            y = Fl.event_y()
            if x>=self.rx and x<=self.rx+self.rw and y>=self.ry and y<=self.ry+self.rh:
                self.onClickBrowser(x-self.rx, y-self.ry)
        return Fl_Window.handle(self, event)
    
    def resize(self, x, y, w, h):
        self.renderersChk.size(self.renderersChk.w(), h-10)
        Fl_Window.resize(self, x, y, w, h)


class ObjectInfoWnd(Fl_Window, ybu.Observer):
    def __init__(self, x, y, w, h, doc):
        Fl_Window.__init__(self, x, y, w, h)
        self.doc = doc
        self.doc.attach(self)
#        self.box(FL_PLASTIC_UP_BOX)
        self.begin()
        self.objectNames = Fl_Choice(5,5,w-10,20,'')
        self.objectNames.align(FL_ALIGN_LEFT)
        self.objectNames.value(0)
        self.objectNames.callback(self.onChangeObjectName)
        self.motionSkeletonWnd = MotionSkeletonWnd(0, 25, self.w(), self.h()-25, self.doc)
        self.end()
        self.childWnds = [self.motionSkeletonWnd]
        for cw in self.childWnds:
            cw.hide()
        self.currentChildWndIdx = 0
        self.resizable(self.motionSkeletonWnd)
    def update(self, ev, doc):
        if ev==EV_addObject:
            self.objectNames.clear()
            for objectName in doc.objectNames:
                idx = self.objectNames.add(objectName)
        elif ev==EV_selectObject:
            if isinstance(self.doc.selectedObject, ym.Motion):
                self.currentChildWndIdx = 0
            elif isinstance(self.doc.selectedObject, yms.Mesh):
                self.currentChildWndIdx = 1
            for i in range(len(self.childWnds)):
                if i == self.currentChildWndIdx:
                    self.childWnds[i].show()
                else:
                    self.childWnds[i].hide()
    def onChangeObjectName(self, ptr):
        self.doc.selectObject(ptr.text(ptr.value()))
        self.doc.notify(EV_selectObject)


class MotionSkeletonWnd(Fl_Window, ybu.Observer):
    def __init__(self, x, y, w, h, doc):
        Fl_Window.__init__(self, x, y, w, h)
        self.doc = doc
        self.doc.attach(self)
#        self.box(FL_PLASTIC_UP_BOX)        
        self.begin()
        self.tree= tree.Fl_Tree(5,5,w-10,h-10,'')
        self.tree.align(FL_ALIGN_TOP)
        self.tree.on_select = self.onSelectTree
        self.end()
        self.resizable(self.tree)
    def update(self, ev, doc):
        if ev==EV_selectObject:
            if isinstance(self.doc.selectedObject, ym.Motion):
                self.selectMotion(self.doc.selectedObject)
    def selectMotion(self, motion):
        self.tree.clearTree()
        posture = motion[0]
        if isinstance(posture, ym.JointPosture):
            rootJoint = posture.skeleton.root
            rootNode = self.tree.append(rootJoint.name, rootJoint)
            rootNode.open()
            self._buildJointTree(rootJoint, rootNode)
    def _buildJointTree(self, parentJoint, parentNode):
        for joint in parentJoint.children:
            node = parentNode.append(joint.name, joint)
            node.open()
            self._buildJointTree(joint, node)
    def onSelectTree(self, node):
        self.doc.selectObjectElement(node.payload)

