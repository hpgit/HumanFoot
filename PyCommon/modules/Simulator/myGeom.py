import sys
import math
import numpy as np
import itertools

from PyCommon.modules.pyVirtualPhysics import *

def _getVerticesGlobal(pGeom, verticesLocal):
    verticesGlobal = []
    for i in range(len(verticesLocal)):
        verticesGlobal.append(pGeom.GetGlobalFrame()*verticesLocal[i])
    return verticesGlobal

def _getVerticesLocal(pGeom, verticesGlobal):
    verticesLocal = []
    for i in range(len(verticesGlobal)):
        verticesLocal.append( Inv(pGeom.GetGlobalFrame()) * verticesGlobal[i])
    return verticesLocal


class MyBox(vpBox):
    def __init__(self, size):
        vpBox.__init__(self, size)
        self._verticesLocal = []
        data = self.GetHalfSize()
        for perm in itertools.product([1, -1], repeat=3):
            self._verticesLocal.append(Vec3(perm[0]*data[0], perm[1]*data[1], perm[2]*data[2]))

    def getVerticesLocal(self):
        return self._verticesLocal

    def getVerticesGlobal(self):
        return _getVerticesGlobal(self, self._verticesLocal)

    def GetType(self):
        return 'M'


class MyFoot3(vpCapsule):
    def __init__(self, radius, height):
        vpCapsule.__init__(self, radius, height)
        self._verticesLocal = []
        self._r = radius
        self._h = height

    def getContactVertices(self):
        verticesGlobal = self.getContactVerticesGlobal()
        del self._verticesLocal[:]
        globalFrame = self.GetGlobalFrame()
        for i in range(len(verticesGlobal)):
            self._verticesLocal.append(Inv(globalFrame)*verticesGlobal[i])
        return self._verticesLocal

    def getContactVerticesGlobal(self):
        verticesGlobal = []
        contact_segment = 0
        l = self._h / 2. - self._r
        r = -self._h / 2. + self._r

        center = []
        # //scalar depth[2];
        center.append(self.GetGlobalFrame()* Vec3(0, 0, l))
        center.append(self.GetGlobalFrame()* Vec3(0, 0, r))

        verticesGlobal.append(center[0] + Vec3(0, -self._r, 0))
        verticesGlobal.append(center[1] + Vec3(0, -self._r, 0))

        r_contact = 0.0
        if contact_segment != 0:
            for j in range(2):
                if(center[j][1] - self._r < 0.0):
                    r_contact = math.sqrt( self._r * self._r - center[j][1]*center[j][1])
                    for i in range(contact_segment):
                        contactPoint_0 = center[j][0] + r_contact * math.cos(float(i)/contact_segment * 2 * math.pi)
                        contactPoint_2 = center[j][2] + r_contact * math.sin(float(i)/contact_segment * 2 * math.pi)
                        contactPoint = Vec3(contactPoint_0, 0., contactPoint_2)
                        verticesGlobal.append(contactPoint)

        return verticesGlobal

    def getVerticesLocal(self):
        return self.getContactVertices()

    def getVerticesGlobal(self):
        return self.getContactVerticesGlobal()


class MyFoot4(MyFoot3):
    def __init__(self, radius, height):
        MyFoot3.__init__(self, radius, height)

    def getContactVerticesGlobal(self):
        verticesGlobal = []
        contact_segment = 0
        l = self._h / 2. - self._r

        center = []
        # //scalar depth[2];
        center.append(self.GetGlobalFrame()* Vec3(0, 0, l))

        verticesGlobal.append(center[0] + Vec3(0, -self._r, 0))

        r_contact = 0.0
        if(contact_segment != 0):
            for j in range(1):
                if(center[j][1] - self._r < 0.0):
                    r_contact = math.sqrt( self._r * self._r - center[j][1]*center[j][1])
                    for i in range(contact_segment):
                        contactPoint_0 = center[j][0] + r_contact * math.cos(float(i)/contact_segment * 2 * math.pi)
                        contactPoint_2 = center[j][2] + r_contact * math.sin(float(i)/contact_segment * 2 * math.pi)
                        contactPoint = Vec3(contactPoint_0, 0., contactPoint_2)
                        verticesGlobal.append(contactPoint)

        return verticesGlobal

class MyFoot5(MyFoot3):
    def __init__(self, radius, height):
        MyFoot3.__init__(self, radius, height)

    def getContactVerticesGlobal(self):
        return []



'''
class MyBox : public vpBox
{
private:
	vector<Vec3> _verticesLocal;
public:
	MyBox(const Vec3 &size);
	virtual const vector<Vec3>& getVerticesLocal() { return _verticesLocal; }
	virtual const vector<Vec3>& getVerticesGlobal() { return _getVerticesGlobal(this, _verticesLocal); }
};

class MyFoot1: public vpBox
{
private:
	vector<Vec3> _verticesLocal;
public:
	MyFoot1(const Vec3 &size);
	virtual const vector<Vec3>& getVerticesLocal() { return _verticesLocal; }
	virtual const vector<Vec3>& getVerticesGlobal() { return _getVerticesGlobal(this, _verticesLocal); }
	virtual void GetShape(char *type, scalar *data)const { vpBox::GetShape(type, data); type[0] = 'M';}
};

class MyFoot2: public vpBox
{
private:
	vector<Vec3> _verticesLocal;
public:
	MyFoot2(const Vec3 &size);
	virtual const vector<Vec3>& getVerticesLocal() { return _verticesLocal; }
	virtual const vector<Vec3>& getVerticesGlobal() { return _getVerticesGlobal(this, _verticesLocal); }
	virtual void GetShape(char *type, scalar *data) const { vpBox::GetShape(type, data); type[0] = 'M';}
};

class MyFoot3 : public vpCapsule
{
private:
	vector<Vec3> _verticesLocal;
public:
	scalar _r, _h;
	MyFoot3(scalar radius, scalar height);
	//virtual const vector<Vec3>& getVerticesLocal() const { return _verticesLocal; }
	//virtual const vector<Vec3>& getVerticesGlobal() const { return _getVerticesGlobal(this, _verticesLocal); }
	virtual const vector<Vec3>& getVerticesLocal() 
	{ 
		vector<Vec3> verticesGlobal;
		getContactVertices(_verticesLocal, verticesGlobal);
		return _verticesLocal; 
	}
	virtual const vector<Vec3>& getVerticesGlobal() 
	{
		vector<Vec3> verticesGlobal;
		getContactVertices(_verticesLocal, verticesGlobal);
		return _getVerticesGlobal(this, _verticesLocal); 
	}
	void getContactVertices(vector<Vec3>& verticesLocal, vector<Vec3>& verticesGlobal);
	void getContactVerticesGlobal(vector<Vec3>& verticesGlobal);
	virtual void GetShape(char *type, scalar *data) const { vpCapsule::GetShape(type, data); type[0] = 'C'; }
};

class MyFoot4 : public MyFoot3
{
private:
	vector<Vec3> _verticesLocal;
public:
	MyFoot4(scalar radius, scalar height):MyFoot3(radius, height){}
	virtual const vector<Vec3>& getVerticesLocal() 
	{ 
		vector<Vec3> verticesGlobal;
		getContactVertices(_verticesLocal, verticesGlobal);
		return _verticesLocal; 
	}
	virtual const vector<Vec3>& getVerticesGlobal() 
	{
		vector<Vec3> verticesGlobal;
		getContactVertices(_verticesLocal, verticesGlobal);
		return _getVerticesGlobal(this, _verticesLocal); 
	}
	void getContactVertices(vector<Vec3>& verticesLocal, vector<Vec3>& verticesGlobal);
	void getContactVerticesGlobal(vector<Vec3>& verticesGlobal);
	virtual void GetShape(char *type, scalar *data) const { vpCapsule::GetShape(type, data); type[0] = 'C'; }
};

class MyFoot5 : public MyFoot3
{
private:
    vector<Vec3> _verticesLocal;
public:
	MyFoot5(scalar radius, scalar height):MyFoot3(radius, height){_verticesLocal.clear();}
	virtual const vector<Vec3>& getVerticesLocal()
	{
		// vector<Vec3> verticesGlobal;
		// getContactVertices(_verticesLocal, verticesGlobal);
		return _verticesLocal;
	}
	virtual const vector<Vec3>& getVerticesGlobal()
	{
		// vector<Vec3> verticesGlobal;
		// verticesGlobal.clear();
		return _verticesLocal;
	}
	void getContactVertices(vector<Vec3>& verticesLocal, vector<Vec3>& verticesGlobal);
	void getContactVerticesGlobal(vector<Vec3>& verticesGlobal);
	virtual void GetShape(char *type, scalar *data) const { vpCapsule::GetShape(type, data); type[0] = 'C'; }
};

class MyShin: public vpBox
{
private:
	vector<Vec3> _verticesLocal;
public:
	MyShin(const Vec3 &size);
	virtual const vector<Vec3>& getVerticesLocal() { return _verticesLocal; }
	virtual const vector<Vec3>& getVerticesGlobal() { return _getVerticesGlobal(this, _verticesLocal); }
	virtual void GetShape(char *type, scalar *data) const { vpBox::GetShape(type, data); type[0] = 'Q';}
};
'''