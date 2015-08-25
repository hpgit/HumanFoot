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
        panelWidth = 280
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
    def GetForce(self):
        mag = self.objectInfoWnd.GetForceMag()
        force = self.motionViewWnd.glWindow.GetForce()
        return [mag*force[0], mag*force[1], mag*force[2]]
    def ResetForce(self):
        self.motionViewWnd.glWindow.ResetForce()
    def GetParam(self):
        return self.objectInfoWnd.GetParam()
        
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
    def removeRenderer(self, name, update = False):
        for index in range(len(self.rendererNames)):
            if self.rendererNames[index] == name :
                del self.rendererNames[index]
                break        
        del self.rendererMap[name]
        del self.renderersVisible[name]
        if update == True:
            self.notify(EV_addRenderer)
    def getRenderer(self, name):
        return self.rendererMap[name]
    def showRenderer(self, name, visible):
        self.renderersVisible[name] = visible
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
         
initKt = 50
maxKt = 1000
initKk = 35
#initKk = 21#300

initKl = 37
initKh = 35
initKs = 50
        
initBk = 0.0#3.5
initBt = 1.0
initBl = 0.1
initBh = 0.1
initBs = 0

initFm = 100.0
maxFm = 1000

initCM = 100.0
maxCM = 200

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
                   
        self.labelKt = Fl_Value_Input(80, 30, 60, 10, 'Tracking')
        self.labelKt.when(FL_WHEN_ENTER_KEY)
        self.labelKt.callback(self.onEnterLabelKt)
        self.labelKt.value(initKt)
        
        self.sliderKt = Fl_Hor_Nice_Slider(10, 42, 250, 10)
        self.sliderKt.bounds(0, 1000)
        self.sliderKt.value(initKt*(1000/maxKt))
        self.sliderKt.step(1)
        self.sliderKt.callback(self.onChangeSliderKt)
                
        offset = 30
        index = 1
        self.labelKk = Fl_Value_Input(80, 30+offset*index, 60, 10, 'Support Kt')
        self.labelKk.when(FL_WHEN_ENTER_KEY)
        self.labelKk.callback(self.onEnterLabelKk)
        self.labelKk.value(initKk)
        
        self.sliderKk = Fl_Hor_Nice_Slider(10, 42+offset*index, 250, 10)
        self.sliderKk.bounds(0, 1000)
        self.sliderKk.value(initKk)
        self.sliderKk.step(1)
        self.sliderKk.callback(self.onChangeSliderKk)
        index += 1

        self.labelKl = Fl_Value_Input(80, 30+offset*index, 60, 10, 'Linear    ')
        self.labelKl.when(FL_WHEN_ENTER_KEY)
        self.labelKl.callback(self.onEnterLabelKl)
        self.labelKl.value(initKl)
        
        self.sliderKl = Fl_Hor_Nice_Slider(10, 42+offset*index, 250, 10)
        self.sliderKl.bounds(0, 10000)
        self.sliderKl.value(initKl*10)
        self.sliderKl.step(1)
        self.sliderKl.callback(self.onChangeSliderKl)
        index += 1
        
        self.labelKh = Fl_Value_Input(80, 30+offset*index, 60, 10, 'Angular  ')
        self.labelKh.when(FL_WHEN_ENTER_KEY)
        self.labelKh.callback(self.onEnterLabelKh)
        self.labelKh.value(initKh)
        
        self.sliderKh = Fl_Hor_Nice_Slider(10, 42+offset*index, 250, 10)
        self.sliderKh.bounds(0, 10000)
        self.sliderKh.value(initKh)
        self.sliderKh.step(1)
        self.sliderKh.callback(self.onChangeSliderKh)
        index += 1
        
        self.labelKs = Fl_Value_Input(80, 30+offset*index, 60, 10, 'SoftConst')
        self.labelKs.when(FL_WHEN_ENTER_KEY)
        self.labelKs.callback(self.onEnterLabelKs)
        self.labelKs.value(initKs)
        
        self.sliderKs = Fl_Hor_Nice_Slider(10, 42+offset*index, 250, 10)
        self.sliderKs.bounds(0, 1000)
        self.sliderKs.value(initKs)
        self.sliderKs.step(1)
        self.sliderKs.callback(self.onChangeSliderKs)
        index += 1
        
        self.labelBt = Fl_Value_Input(80, 30+offset*index, 60, 10, 'Tracking-W')
        self.labelBt.when(FL_WHEN_ENTER_KEY)
        self.labelBt.callback(self.onEnterLabelBt)
        self.labelBt.value(initBt)
        
        self.sliderBt = Fl_Hor_Nice_Slider(10, 42+offset*index, 250, 10)
        self.sliderBt.bounds(0, 100)
        self.sliderBt.value(initBt*10)
        self.sliderBt.step(1)
        self.sliderBt.callback(self.onChangeSliderBt)
        index += 1

        self.labelBl = Fl_Value_Input(80, 30+offset*index, 60, 10, 'Linear-W  ')
        self.labelBl.when(FL_WHEN_ENTER_KEY)
        self.labelBl.callback(self.onEnterLabelBl)
        self.labelBl.value(initBl)
        
        self.sliderBl = Fl_Hor_Nice_Slider(10, 42+offset*index, 250, 10)
        self.sliderBl.bounds(0, 1000)
        self.sliderBl.value(initBl*10)
        self.sliderBl.step(1)
        self.sliderBl.callback(self.onChangeSliderBl)
        index += 1
        
        self.labelBh = Fl_Value_Input(80, 30+offset*index, 60, 10, 'Angular-W')
        self.labelBh.when(FL_WHEN_ENTER_KEY)
        self.labelBh.callback(self.onEnterLabelBh)
        self.labelBh.value(initBh)
        
        self.sliderBh = Fl_Hor_Nice_Slider(10, 42+offset*index, 250, 10)
        self.sliderBh.bounds(0, 1000)
        self.sliderBh.value(initBh*10)
        self.sliderBh.step(1)
        self.sliderBh.callback(self.onChangeSliderBh)
        index += 1
        '''
        self.labelBs = Fl_Value_Input(80, 30+offset*index, 60, 10, 'SoftConst-W')
        self.labelBs.when(FL_WHEN_ENTER_KEY)
        self.labelBs.callback(self.onEnterLabelBs)
        self.labelBs.value(initBs)
        
        self.sliderBs = Fl_Hor_Nice_Slider(10, 42+offset*index, 250, 10)
        self.sliderBs.bounds(0, 100)
        self.sliderBs.value(initBs)
        self.sliderBs.step(1)
        self.sliderBs.callback(self.onChangeSliderBs)
        index += 1
        '''
        self.labelFm = Fl_Value_Input(80, 30+offset*index, 60, 10, 'Force   ')
        self.labelFm.when(FL_WHEN_ENTER_KEY)
        self.labelFm.callback(self.onEnterLabelFm)
        self.labelFm.value(initFm)
        
        self.sliderFm = Fl_Hor_Nice_Slider(10, 42+offset*index, 250, 10)
        self.sliderFm.bounds(0, maxFm)
        self.sliderFm.value(initFm)
        self.sliderFm.step(1)
        self.sliderFm.callback(self.onChangeSliderFm)
        index += 1
        
        self.labelCM = Fl_Value_Input(80, 30+offset*index, 60, 10, 'COM  F-B')
        self.labelCM.when(FL_WHEN_ENTER_KEY)
        self.labelCM.callback(self.onEnterLabelCM)
        self.labelCM.value(initCM-100)

        self.sliderCM = Fl_Hor_Nice_Slider(10, 42+offset*index, 250, 10)
        self.sliderCM.bounds(0, maxCM)
        self.sliderCM.value(initCM)
        self.sliderCM.step(1)
        self.sliderCM.callback(self.onChangeSliderCM)
        index += 1


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
        
    def onChangeSliderKt(self, ptr):  
        self.labelKt.value(int(ptr.value())/(1000/maxKt))
    def onEnterLabelKt(self, ptr):        
        self.sliderKt.value(int(ptr.value())*(1000/maxKt))
                
    def onChangeSliderKl(self, ptr):
        self.labelKl.value(int(ptr.value())/10.0)
    def onEnterLabelKl(self, ptr):        
        self.sliderKl.value(int(ptr.value())*10)
        
    def onChangeSliderKh(self, ptr):
        self.labelKh.value(int(ptr.value()))
    def onEnterLabelKh(self, ptr):        
        self.sliderKh.value(int(ptr.value()))
        
    def onChangeSliderKs(self, ptr):
        self.labelKs.value(int(ptr.value()))
    def onEnterLabelKs(self, ptr):        
        self.sliderKs.value(int(ptr.value()))
        
    def onChangeSliderKk(self, ptr):
        self.labelKk.value(int(ptr.value()))
    def onEnterLabelKk(self, ptr):        
        self.sliderKk.value(int(ptr.value()))
                
    def onChangeSliderBt(self, ptr):
        self.labelBt.value(int(ptr.value())/10.0)
    def onEnterLabelBt(self, ptr):        
        self.sliderBt.value(int(ptr.value())*10)

    def onChangeSliderBl(self, ptr):
        self.labelBl.value(int(ptr.value())/10.0)
    def onEnterLabelBl(self, ptr):        
        self.sliderBl.value(int(ptr.value())*10)
        
    def onChangeSliderBh(self, ptr):
        self.labelBh.value(int(ptr.value())/10.0)
    def onEnterLabelBh(self, ptr):        
        self.sliderBh.value(int(ptr.value())*10)
        
    def onChangeSliderBs(self, ptr):
        self.labelBs.value(int(ptr.value()))
    def onEnterLabelBs(self, ptr):        
        self.sliderBs.value(int(ptr.value()))
        
    def onChangeSliderFm(self, ptr):
        self.labelFm.value(int(ptr.value()))
    def onEnterLabelFm(self, ptr):        
        self.sliderFm.value(int(ptr.value()))
        
    def onChangeSliderCM(self, ptr):
        self.labelCM.value(int(ptr.value())-100)
    def onEnterLabelCM(self, ptr):        
        self.sliderCM.value(int(ptr.value())+100)        

    def GetParam(self):
        return self.labelKt.value(), self.labelKk.value(), self.labelKl.value(), self.labelKh.value(), self.labelKs.value(), self.labelBt.value(), self.labelBl.value(), self.labelBh.value(), self.labelCM.value()
    def GetForceMag(self):
        return self.labelFm.value()
    
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

