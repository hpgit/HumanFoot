import ysSimpleViewer_ori as ysvOri
import GUI.ysBaseUI as ybu
import fltk


class hpSimpleViewer(ysvOri.SimpleViewer):
    def __init__(self, rect=None, title='hpSimpleViewer'):
        ybu.BaseWnd.__init__(self, rect, title, ysvOri.SimpleSetting())
        self.doc = ysvOri.SimpleDoc()
        self.begin()
        panelWidth = 280
        cForceHeight = 200
        self.motionViewWnd = hpMotionViewWnd(0, 0, self.w()-panelWidth, self.h()-cForceHeight, self.doc)
        t = .3
        self.renderersWnd = ysvOri.RenderersWnd(self.w()-panelWidth, 0, panelWidth, int(self.h()*t), self.doc)
        self.objectInfoWnd = hpObjectInfoWnd(self.w()-panelWidth, int(self.h()*t), panelWidth, int(self.h()*(1-t)), self.doc)
        self.cForceWnd = hpContactForceGraphWnd(0, self.h()-cForceHeight, self.w()-panelWidth-40, cForceHeight, self.doc)
        self.end()
        self.resizable(self.motionViewWnd)
        self.resizable(self.cForceWnd)
        self.size_range(600, 400)

        self.cForceWnd.viewer = self
        self.motionViewWnd.cForceWnd = self.cForceWnd
    pass


class hpMotionViewWnd(ysvOri.MotionViewWnd):
    def goToFrame(self, frame):
        super(hpMotionViewWnd, self).goToFrame(frame)
        self.cForceWnd.redraw()

    def onTimer(self):
        if self.playing:
            self.frame += 1
            if self.frame > self.maxFrame:
                self.frame = 0
            self.onFrame(self.frame)
            self.cForceWnd.redraw()

        if self.timeInterval:
            fltk.Fl.repeat_timeout(self.timeInterval, self.onTimer)

class hpObjectInfoWnd(ysvOri.ObjectInfoWnd):
    def __init__(self, x, y, w, h, doc):
        self.valObjects = dict()
        self.valObjOffset = 30
        super(hpObjectInfoWnd, self).__init__(x, y, w, h, doc)

    def update(self, ev, doc):
        super(hpObjectInfoWnd, self).update(ev, doc)

    def addValObjects(self, obj):
        self.valObjects[obj.name] = obj
        pass

    def getValobject(self, name):
        pass

    def getValObjects(self):
        return self.valObjects.values()

    def getVals(self):
        return (v.value() for v in self.valObjects.values())

    def getVal(self, name):
        try:
            return self.valObjects[name].value()
        except Exception, e:
            print e
            return 0

    def add1DSlider(self, name, minVal, maxVal, valStep, initVal):
        self.begin()
        slider = fltk.Fl_Hor_Value_Slider(10, self.valObjOffset, 250, 18, name)
        slider.bounds(minVal, maxVal)
        slider.value(initVal)
        slider.step(valStep)
        slider.name = name
        self.end()
        self.addValObjects(slider)
        self.valObjOffset += 40

    def add3DSlider(self, name, minVal, maxVal, valStep, initVal):
        self.begin()

        self.end()
        pass


class hpContactForceGraphWnd(fltk.Fl_Widget, ybu.Observer):
    def __init__(self, x, y, w, h, doc):
        self.doc = doc
        self.doc.attach(self)
        super(hpContactForceGraphWnd, self).__init__(x, y, w, h)
        self.data = []
        self.dataLength = 0
        self.dataSetName = []
        self.dataSetColor = []

        self.dataCheckBtn = []
        self.dataCheckBtnOffset = 0

        self.curFrame = -1
        self.viewer = None

    def update(self, ev, doc):
        if ev == ysvOri.EV_addObject:
            self.dataLength = doc.motionSystem.getMaxFrame()
        self.redraw()

    def addDataSet(self, name, color):
        self.data.append([0.] * (self.dataLength+1))
        self.dataSetName.append(name)
        self.dataSetColor.append(color)
        checkBox = fltk.Fl_Check_Button(self.x(), self.y() + self.dataCheckBtnOffset, 30, 40, name)
        checkBox.value(True)
        checkBox.callback(self.checkBtnCallback)
        self.dataCheckBtn.append(checkBox)
        self.dataCheckBtnOffset += 40
        self.redraw()

    def addData(self, name, val):
        dataIdx = self.dataSetName.index(name)
        self.data[dataIdx].append(val)
        self.redraw()

    def insertData(self, name, valIdx, val):
        dataIdx = self.dataSetName.index(name)
        self.data[dataIdx][valIdx] = val
        self.redraw()

    def draw(self):
        fltk.fl_draw_box(fltk.FL_FLAT_BOX, 40+self.x(), self.y(), self.w()-40, self.h(), fltk.fl_rgb_color(192, 192, 192))
        ratio = float(self.w()-40)/self.dataLength
        for dataIdx in range(len(self.data)):
            if self.dataCheckBtn[dataIdx].value():
                for valIdx in range(1, self.dataLength-1):
                    fltk.fl_color(self.dataSetColor[dataIdx])
                    fltk.fl_line(40+self.x()+int(ratio * (valIdx-1)), int(self.y()+self.h() - self.data[dataIdx][valIdx-1]/2.)-3,
                                 40+self.x()+int(ratio * valIdx), int(self.y()+self.h() - self.data[dataIdx][valIdx]/2.)-3)


        frame = self.viewer.getCurrentFrame()
        if frame >-1:
            fltk.fl_color(fltk.FL_BLUE)
            fltk.fl_line(40+self.x()+int(ratio * frame), int(self.y()+self.h())-3,
                         40+self.x()+int(ratio * frame), int(self.y()-3))

    def checkBtnCallback(self, ptr):
        self.redraw()
        pass


