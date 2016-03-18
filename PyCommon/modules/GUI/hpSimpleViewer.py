import ysSimpleViewer_ori as ysvOri
import GUI.ysBaseUI as ybu
import fltk
import cPickle
import OpenGL.GL as gl
from PIL import Image as im
import numpy as np

class hpSimpleViewer(ysvOri.SimpleViewer):
    def __init__(self, rect=None, title='hpSimpleViewer'):
        ybu.BaseWnd.__init__(self, rect, title, ysvOri.SimpleSetting())
        self.title = title
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
        # self.resizable(self.cForceWnd)
        self.size_range(600, 400)

        self.cForceWnd.viewer = self
        self.motionViewWnd.cForceWnd = self.cForceWnd
        self.objectInfoWnd.viewer = self


class hpMotionViewWnd(ysvOri.MotionViewWnd):
    def __init__(self, x, y, w, h, doc):
        ysvOri.MotionViewWnd.__init__(self, x, y, w, h, doc)
        self.mov = False

    def goToFrame(self, frame):
        super(hpMotionViewWnd, self).goToFrame(frame)
        self.cForceWnd.redraw()

    def onTimer(self):
        if self.playing:
            self.frame += 1
            if self.frame > self.maxFrame:
                self.frame = 0
            self.onFrame(self.frame)
            if self.mov:
                self.dump(self, "_movtmp/tmp"+str(self.frame)+".png")

        if self.timeInterval:
            fltk.Fl.repeat_timeout(self.timeInterval, self.onTimer)

    def dump(self, ptr, outfile="output.png"):
        gl.glPixelStorei(gl.GL_PACK_ALIGNMENT, 1)
        gl.glReadBuffer(gl.GL_BACK_LEFT)
        image = None
        if self.w() > 1000:
            image = np.array(255*gl.glReadPixelsf(0, 0, 2000, 2000, gl.GL_RGB))
        else:
            image = np.array(255*gl.glReadPixelsf(0, 0, 1000, 1000, gl.GL_RGB))
        #  image = [img_line.flatten() for img_line in image]

        img = im.new('RGB', (self.w(), self.h()-56))
        pix = img.load()
        for i in range(self.w()):
            for j in range(56, self.h()):
                pix[i, j-56] = tuple(image[self.h()-j, i])
        img.save(outfile, "PNG")
        # f = open('image.png', 'wb')
        # w = png.Writer(self.h()-10, self.w())
        # w.write(f, image)
        # f.close()

    def dumpMov(self, ptr):
        if self.mov:
            self.mov = False
        else:
            self.mov = True

        # import os
        # os.mkdir("_movtmp")
        # for i in range(110, 120):
        #     self.onFrame(i)
        #
        #     self.dump(ptr, "_movtmp/tmp"+str(self.frame)+".png")


class hpObjectInfoWnd(ysvOri.ObjectInfoWnd):
    def __init__(self, x, y, w, h, doc):
        super(hpObjectInfoWnd, self).__init__(x, y, w, h, doc)
        self.valObjects = dict()
        self.valObjOffset = 30

        self.begin()
        saveBtn = fltk.Fl_Button(10, self.valObjOffset, 80, 20, 'param save')
        saveBtn.callback(self.save)
        loadBtn = fltk.Fl_Button(100, self.valObjOffset, 80, 20, 'param load')
        loadBtn.callback(self.load)
        self.end()
        self.valObjOffset += 40

        # super(hpObjectInfoWnd, self).__init__(x, y, w, h, doc)

    def update(self, ev, doc):
        super(hpObjectInfoWnd, self).update(ev, doc)

    def addValObjects(self, obj):
        self.valObjects[obj.name] = obj
        pass

    def getValobject(self, name):
        return self.valObjects[name]
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

    def getNameAndVals(self):
        objValDict = dict()
        for k, v in self.valObjects.iteritems():
            objValDict[k] = v.value()
        return objValDict

    def addBtn(self, name, callback):
        self.begin()
        btn = fltk.Fl_Button(10, self.valObjOffset, 80, 20, name)
        btn.callback(callback)
        self.end()
        self.valObjOffset += 40

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

    def save(self, obj):
        f = file(self.viewer.title+'.param', 'w')
        cPickle.dump(self.getNameAndVals(), f)
        f.close()

    def load(self, obj):
        filefile = fltk.Fl_File_Chooser('.', '*.param', fltk.FL_SINGLE, 'load parameter file')
        filefile.show()
        while filefile.shown():
            fltk.Fl.wait()
        if filefile.count() == 1:
            # f = file(self.viewer.title+'param', 'r')
            f = file(filefile.value(), 'r')
            objVals = cPickle.load(f)
            f.close()
            for k, v in objVals.iteritems():
                if k in self.valObjects.keys():
                    self.valObjects[k].value(v)



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

        self.zoomslider = fltk.Fl_Hor_Value_Slider(self.x()+self.w() - 280, self.y()+20, 250, 18, "zoom")
        self.zoomslider.bounds(1, 10)
        self.zoomslider.value(1)
        self.zoomslider.step(1)
        self.zoomslider.callback(self.checkBtnCallback)

        self.yzoomslider = fltk.Fl_Hor_Value_Slider(self.x()+self.w() - 280, self.y()+60, 250, 18, "y zoom")
        self.yzoomslider.bounds(1, 20)
        self.yzoomslider.value(1)
        self.yzoomslider.step(1)
        self.yzoomslider.callback(self.checkBtnCallback)

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
        ratio *= self.zoomslider.value()
        dataRatio = 1./self.yzoomslider.value()
        for dataIdx in range(len(self.data)):
            if self.dataCheckBtn[dataIdx].value():
                for valIdx in range(1, self.dataLength-1):
                    fltk.fl_color(self.dataSetColor[dataIdx])
                    fltk.fl_line(40+self.x()+int(ratio * (valIdx-1)), int(self.y()+self.h() - self.data[dataIdx][valIdx-1]*dataRatio)-3,
                                 40+self.x()+int(ratio * valIdx), int(self.y()+self.h() - self.data[dataIdx][valIdx]*dataRatio)-3)


        frame = self.viewer.getCurrentFrame()
        if frame >-1:
            fltk.fl_color(fltk.FL_BLUE)
            fltk.fl_line(40+self.x()+int(ratio * frame), int(self.y()+self.h())-3,
                         40+self.x()+int(ratio * frame), int(self.y()-3))

        self.zoomslider.redraw()
        self.yzoomslider.redraw()

    def checkBtnCallback(self, ptr):
        self.redraw()
        pass


