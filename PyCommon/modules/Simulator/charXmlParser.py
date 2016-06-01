import sys
sys.path.append('../../..')
import PyCommon.modules.Math.mmMath as mm
import xml.etree.ElementTree as et
# from xml.dom import minidom

import numpy as np

from PyCommon.modules.pyVirtualPhysics import *


def test(filename='../../../FootProject/test.xml'):
    tree = et.parse(filename)

    # get skeleton root and name
    root = tree.getroot()
    charName = root.attrib['name']

    # bodies = dict()
    bodies = list()

    # joints = dict()
    joints = list()

    # bodies
    for child in root.findall('body'):
        bodies.append(child.attrib['name'])
        print(child.attrib['name'])

        # transformation
        print('transformation', map(float, child.find('transformation').text.split()))

        # inertia
        print('inertia', child.find('inertia').find('mass').text)

        # visualization_shape
        for shape in child.findall('visualization_shape'):
            print(shape.find('transformation').text)
            print(shape.find('geometry')[0].tag)
            print(shape.find('geometry')[0][0].text)

        # collision_shape
        for shape in child.findall('collision_shape'):
            print(shape.find('transformation').text)
            print(shape.find('geometry')[0].tag)
            print(shape.find('geometry')[0][0].text)

    # joints
    for child in root.findall('joint'):
        print(child.attrib['name'])
        if child.find('transformation') is not None:
            print('transformation', map(float, child.find('transformation').text.split()))
        print(child.find('parent').text + ' - ' + child.find('child').text)


def transToSE3(trans):
    v = Vec3(trans[0], trans[1], trans[2])
    length = mm.length(trans[3:])
    if length < LIE_EPS:
        return SE3(v)
    R = mm.exp(trans[3:]/length, length)

    return SE3(R[0, 0], R[1, 0], R[2, 0], R[1, 0], R[1, 1], R[1, 2], R[2, 0], R[2, 1], R[2, 2], trans[0], trans[1], trans[2])


if __name__ == '__main__':
    # test()
    tree = et.parse('../../../FootProject/test.xml')
    root = tree.getroot()

    world = vpWorld()
    world.SetNumThreads(3)

    # bodies
    bodies = dict()
    for child in root.findall('body'):
        body = vpBody()
        world.AddBody(body)
        bodies[child.attrib['name']] = body
        body.m_szName = child.attrib['name']

        # transformation
        # print('transformation', map(float, child.find('transformation').text.split()))
        trans = np.array(map(float, child.find('transformation').text.split()))
        bodyFrame = transToSE3(trans)
        body.SetFrame(bodyFrame)

        # inertia
        mass = float(child.find('inertia').find('mass').text)

        # visualization_shape
        for shape in child.findall('visualization_shape'):
            geomTrans = np.array(map(float, child.find('transformation').text.split()))
            geomType = shape.find('geometry')[0].tag
            geomSize = np.array(map(float, shape.find('geometry')[0][0].text.split()))

        # collision_shape
        # class			 vpSphere;		// S
        # class			 vpBox;			// B
        # class			 vpCapsule;		// C
        # class			 vpPlane;		// P
        # class			 vpCylinder;	// L
        # class			 vpTorus;		// T

        for shape in child.findall('collision_shape'):
            geomTrans = np.array(map(float, child.find('transformation').text.split()))
            geomFrame = transToSE3(geomTrans)
            geomType = shape.find('geometry')[0].tag
            geomSize = np.array(map(float, shape.find('geometry')[0][0].text.split()))

            geom = None

            if geomType in ('sphere', 'ellipsoid'):
                geom = vpSphere()
                geom.SetRadius(geomSize[0])
            elif geomType == 'box':
                geom = vpBox()
                geom.SetSize(Vec3(geomSize[0], geomSize[1], geomSize[2]))
            elif geomType == 'capsule':
                geom = vpCapsule()
                geom.SetSize(geomSize[0], geomSize[1])
            elif geomType == 'plane':
                geom = vpPlane()
                geom.SetNormal(Vec3(geomSize[0], geomSize[1], geomSize[2]))
            elif geomType == 'cylinder':
                geom = vpCylinder()
                geom.SetSize(geomSize[0], geomSize[1])
            elif geomType == 'torus':
                geom = vpTorus()
                geom.SetSize(geomSize[0], geomSize[1])
            else:
                continue

            bodies[child.attrib['name']].AddGeometry(geom, geomFrame)

    # joints
    joints = dict()
    for child in root.findall('joint'):
        parentName = child.find('parent').text
        childName = child.find('child').text
        if child.attrib['type'] == 'free':
            world.AddBody(bodies[childName])
            continue
        joint = vpBJoint()
        joint.SetDamping(Inertia(1.))
        jointName = child.attrib['name']
        joints[jointName] = joint


        trans = np.array([0., 0., 0., 0., 0., 0.])
        if child.find('transformation') is not None:
            trans = np.array(map(float, child.find('transformation').text.split()))

        bodies[parentName].SetJoint(joint, transToSE3(trans))
        TchildTojoint = Inv(bodies[childName].GetFrame()) * bodies[parentName].GetFrame() * transToSE3(trans)
        bodies[childName].SetJoint(joint, TchildTojoint)

    world.SetGravity(Vec3(0., -9.8, 0.))
    world.SetTimeStep(0.001)
    world.SetGlobalDamping(0.999)
    world.Initialize()

    for i in range(10):
        print(bodies['root'].GetFrame())
        # print(bodies['root'].GetGeometry(0).GetShape(0))
        # print([str(bodies[body].GetFrame()) for body in bodies])
        world.StepAhead()
    # print(world.GetBody(0).GetFrame().GetPosition())
