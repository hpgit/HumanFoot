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
        self.motionViewWnd = ysvOri.MotionViewWnd(0, 0, self.w()-panelWidth, self.h()-cForceHeight, self.doc)
        t = .3
        self.renderersWnd = ysvOri.RenderersWnd(self.w()-panelWidth, 0, panelWidth, int(self.h()*t), self.doc)
        self.objectInfoWnd = hpObjectInfoWnd(self.w()-panelWidth, int(self.h()*t), panelWidth, int(self.h()*(1-t)), self.doc)
        self.cForceWnd = hpContactForceGraphWnd(40, self.h()-cForceHeight, self.w()-panelWidth-40, cForceHeight, self.doc)
        self.end()
        self.resizable(self.motionViewWnd)
        self.size_range(600, 400)
    pass


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


class hpContactForceGraphWnd(fltk.Fl_Chart):
    def __init__(self, x, y, w, h, doc):
        self.doc = doc
        super(hpContactForceGraphWnd, self).__init__(x, y, w, h)
        self.type(fltk.FL_LINE_CHART)

    def update(self, ev, doc):
        if ev == ysvOri.EV_addObject:
            self.maxsize(doc.motionSystem.getMaxFrame())
        self.glWindow.redraw()
    def addData(self, val):
        fltk.fl_color(fltk.FL_BLACK)
        # fltk.fl_line
        #
        # int x1 = x(),       y1 = y();
        # int x2 = x()+w()-1, y2 = y()+h()-1;
        # fl_line(x1, y1, x2, y2);
        # fl_line(x1, y2, x2, y1);




