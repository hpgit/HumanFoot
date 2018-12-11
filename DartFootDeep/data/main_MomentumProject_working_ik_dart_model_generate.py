import copy
import numpy as np

import sys
if '../..' not in sys.path:
    sys.path.append('../..')
from PyCommon.modules.Math import mmMath as mm
from PyCommon.modules.Simulator import csVpWorld as cvw
from PyCommon.modules.Simulator import csVpModel as cvm
from MomentumProject.foot_example_segfoot_constraint import mtInitialize as mit


g_initFlag = 0
forceShowTime = 0

JsysPre = 0
JsupPreL = 0
JsupPreR = 0
JconstPre = 0

contactChangeCount = 0
contactChangeType = 0
contact = 0
maxContactChangeCount = 30

preFootCenter = [None]

DART_CONTACT_ON = False
SKELETON_ON = True


def main():
    # np.set_printoptions(precision=4, linewidth=200)
    np.set_printoptions(precision=5, threshold=np.inf, suppress=True, linewidth=3000)

    motionFile = 'wd2_tiptoe_zygote.bvh'
    # motion, mcfg, wcfg, stepsPerFrame, config, frame_rate = mit.create_biped(motionFile, SEGMENT_FOOT_RAD=0.008)
    motion, mcfg, wcfg, stepsPerFrame, config, frame_rate = mit.create_biped(motionFile, SEGMENT_FOOT_MAG=0.01, SEGMENT_FOOT_RAD=0.008)
    # motion, mcfg, wcfg, stepsPerFrame, config, frame_rate = mit.create_biped()
    # motion, mcfg, wcfg, stepsPerFrame, config = mit.create_jump_biped()

    vpWorld = cvw.VpWorld(wcfg)
    vpWorld.SetGlobalDamping(0.999)
    motionModel = cvm.VpMotionModel(vpWorld, motion[0], mcfg)
    controlModel = cvm.VpControlModel(vpWorld, motion[0], mcfg)
    # controlModel_shadow_for_ik = cvm.VpControlModel(vpWorld, motion[0], mcfg)
    vpWorld.initialize()
    controlModel.initializeHybridDynamics()

    # controlToMotionOffset = (1.5, -0.02, 0)
    controlToMotionOffset = (1.5, 0., 0)
    controlModel.translateByOffset(controlToMotionOffset)
    # controlModel_shadow_for_ik.set_q(controlModel.get_q())
    # controlModel_shadow_for_ik.computeJacobian(0, np.array([0., 0., 0.]))

    wcfg_ik = copy.deepcopy(wcfg)
    vpWorld_ik = cvw.VpWorld(wcfg_ik)
    controlModel_ik = cvm.VpControlModel(vpWorld_ik, motion[0], mcfg)
    vpWorld_ik.initialize()
    controlModel_ik.set_q(np.zeros_like(controlModel.get_q()))
    print('<?xml version="1.0" ?>')
    print('<skel version="1.0">')
    print('<world name="world 1">')
    print('    <physics>')
    print('        <time_step>0.001</time_step>')
    print('        <gravity>0 -9.81 0</gravity>')
    print('        <collision_detector>fcl_mesh</collision_detector>')
    print('    </physics>')
    print('    <skeleton name="grount skeleton">')
    print('        <mobile>false</mobile>')
    print('        <body name="ground">')
    print('            <transformation>0 -0.025 0 0 0 0</transformation>')
    print('            <visualization_shape name=" - visual - 0">')
    print('                <transformation>0.0 0.0 0.0 0.0 0.0 0.0 </transformation>')
    print('                <geometry>')
    print('                    <box>')
    print('                        <size>10000.0 0.05 10000.0</size>')
    print('                    </box>')
    print('                </geometry>')
    print('            </visualization_shape>')
    print('            <collision_shape name=" - collision - 0">')
    print('                <transformation>0.0 0.0 0.0 0.0 0.0 0.0 </transformation>')
    print('                <geometry>')
    print('                    <box>')
    print('                        <size>100.0 0.05 100.0</size>')
    print('                    </box>')
    print('                </geometry>')
    print('            </collision_shape>')
    print('        </body>')
    print('        <joint name="joint 1" type="free">')
    print('            <parent>world</parent>')
    print('            <child>ground</child>')
    print('        </joint>')
    print('    </skeleton>')
    print('    <skeleton name="dartModel">')

    for i in range(controlModel.getJointNum()):
        body_name = controlModel_ik.index2name(i)
        print('        <body name="'+body_name+'">')
        print('            <transformation>'+str(np.concatenate((controlModel_ik.getBodyPositionGlobal(i), mm.logSO3(controlModel_ik.getBodyOrientationGlobal(i)))))[1:-1]+'</transformation>')
        print('            <inertia>')
        print('                <mass>' + str(controlModel_ik.getBodyMass(i))+'</mass>')
        print('                <offset>'+str(controlModel_ik.getBodyLocalCom(i))[1:-1]+'</offset>')
        print('            </inertia>')
        geom_types = controlModel_ik.getBodyGeomsType(i)
        geom_sizes = controlModel_ik.getBodyGeomsSize(i)
        geom_local_frames = controlModel_ik.getBodyGeomsLocalFrame(i)
        geom_count = 0
        for j in range(len(geom_types)):
            if geom_types[j] == 'B':
                print('            <visualization_shape name="'+body_name+' - visual shape '+str(geom_count)+'">')
                print('                <transformation>'+str(np.concatenate((geom_local_frames[j][:3, 3], mm.logSO3(geom_local_frames[j][:3, :3]))))[1:-1]+'</transformation>')
                print('                <geometry>')
                print('                    <box>')
                print('                        <size>'+str(geom_sizes[j])[1:-1]+'</size>')
                print('                    </box>')
                print('                </geometry>')
                print('            </visualization_shape>')
                geom_count += 1
            elif geom_types[j] in ('C', 'D', 'E', 'F'):
                print('            <visualization_shape name="'+body_name+' - visual shape '+str(geom_count)+'">')
                print('                <transformation>'+str(np.concatenate((geom_local_frames[j][:3, 3], mm.logSO3(geom_local_frames[j][:3, :3]))))[1:-1]+'</transformation>')
                print('                <geometry>')
                print('                    <capsule>')
                print('                        <radius>'+str(geom_sizes[j][0])+'</radius>')
                print('                        <height>'+str(geom_sizes[j][1]-2.*geom_sizes[j][0])+'</height>')
                print('                    </capsule>')
                print('                </geometry>')
                print('            </visualization_shape>')
                geom_count += 1

        geom_count = 0
        for j in range(len(geom_types)):
            if geom_types[j] == 'B':
                print('            <collision_shape name="'+body_name+' - collision shape '+str(geom_count)+'">')
                print('                <transformation>'+str(np.concatenate((geom_local_frames[j][:3, 3], mm.logSO3(geom_local_frames[j][:3, :3]))))[1:-1]+'</transformation>')
                print('                <geometry>')
                print('                    <box>')
                print('                        <size>'+str(geom_sizes[j])[1:-1]+'</size>')
                print('                    </box>')
                print('                </geometry>')
                print('            </collision_shape>')
                geom_count += 1
            if geom_types[j] in ('C', 'D'):
                print('            <collision_shape name="'+body_name+' - collision shape '+str(geom_count)+'">')
                print('                <transformation>'+str(np.concatenate((np.dot(geom_local_frames[j], mm.TransVToSE3((geom_sizes[j][1]/2.-geom_sizes[j][0])*mm.unitZ()))[:3, 3], mm.logSO3(geom_local_frames[j][:3, :3]))))[1:-1]+'</transformation>')
                print('                <geometry>')
                print('                    <sphere>')
                print('                        <radius>'+str(geom_sizes[j][0])+'</radius>')
                print('                    </sphere>')
                print('                </geometry>')
                print('            </collision_shape>')
                geom_count += 1
            if geom_types[j] in ('C', 'F'):
                print('            <collision_shape name="'+body_name+' - collision shape '+str(geom_count)+'">')
                print('                <transformation>'+str(np.concatenate((np.dot(geom_local_frames[j], mm.TransVToSE3(-(geom_sizes[j][1]/2.-geom_sizes[j][0])*mm.unitZ()))[:3, 3], mm.logSO3(geom_local_frames[j][:3, :3]))))[1:-1]+'</transformation>')
                print('                <geometry>')
                print('                    <sphere>')
                print('                        <radius>'+str(geom_sizes[j][0])+'</radius>')
                print('                    </sphere>')
                print('                </geometry>')
                print('            </collision_shape>')
                geom_count += 1
        print('        </body>')

    for i in range(controlModel.getJointNum()):
        name = controlModel_ik.index2name(i)
        inv_bone_R, inv_bone_p = controlModel_ik.getInvBoneT(i)
        if i == 0:
            print('        <joint name="j_'+name+'" type="free">')
            print('            <transformation>'+str(np.concatenate((inv_bone_p, mm.logSO3(inv_bone_R))))[1:-1]+'</transformation>')
            print('            <parent>world</parent>')
            print('            <child>'+name+'</child>')
            print('        </joint>')
        else:
            parent_name = controlModel_ik.index2name(controlModel_ik.getParentIndex(i))
            print('        <joint name="j_'+name+'" type="ball">')
            print('            <transformation>'+str(np.concatenate((inv_bone_p, mm.logSO3(inv_bone_R))))[1:-1]+'</transformation>')
            print('            <parent>'+parent_name+'</parent>')
            print('            <child>'+name+'</child>')
            print('        </joint>')
    print('    </skeleton>')
    print('</world>')
    print('</skel>')


main()
