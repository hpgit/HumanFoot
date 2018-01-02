import numpy as np
import math
from PyCommon.modules.Math import mmMath as mm


def footAdjust(posture_ori, footIdDic, SEGMENT_FOOT_MAG, SEGMENT_FOOT_RAD, baseHeight=0.):
    '''
    :return:
    '''

    def getJointChildPositionGlobal(posture, jointNameOrIdx):
        """

        :type posture: ym.JointPosture
        :type jointNameOrIdx: str | int
        :return: np.array
        """
        idx = jointNameOrIdx
        if type(jointNameOrIdx) == str:
            idx = posture.skeleton.getJointIndex(jointNameOrIdx)
        effectorOffset = posture.skeleton.getJoint(idx).children[0].offset
        return posture.getJointPositionGlobal(idx) + np.dot(posture.getJointOrientationGlobal(idx), effectorOffset)

    def makeTwoContactPos(posture, jointNameOrIdx, isLeftFoot=True, isOutside=True, baseHeight=None):
        """

        :type posture: ym.JointPosture
        :type jointNameOrIdx: str | int
        :return: np.array, np.array
        """
        idx = jointNameOrIdx
        if type(jointNameOrIdx) == str:
            idx = posture.skeleton.getJointIndex(jointNameOrIdx)

        insideOffset = SEGMENT_FOOT_MAG * np.array((0., 0., 2.5))
        outsideOffset = SEGMENT_FOOT_MAG * np.array((1.2, 0., 2.5))
        if isLeftFoot ^ isOutside:
        # if not isOutside:
            # if it is not outside phalange,
            outsideOffset[0] = -1.2 * SEGMENT_FOOT_MAG

        origin = posture.getJointPositionGlobal(idx)
        inside = posture.getJointPositionGlobal(idx, insideOffset)
        outside = posture.getJointPositionGlobal(idx, outsideOffset)

        length = SEGMENT_FOOT_MAG * 2.5

        RotVec1_tmp1 = inside - origin
        RotVec1_tmp2 = inside - origin
        RotVec1_tmp2[1] = 0.
        RotVec1 = np.cross(RotVec1_tmp1, RotVec1_tmp2)
        inner = (origin[1] - SEGMENT_FOOT_RAD)/length

        angle1_1 = math.acos(inner if inner < 1.0 else 1.0)
        if baseHeight is not None:
            angle1_1 = math.acos((origin[1] - (baseHeight + SEGMENT_FOOT_RAD))/length)
        angle1_2 = math.acos((origin[1] - inside[1])/length)
        footRot1 = mm.exp(RotVec1, angle1_1-angle1_2)
        footOri1 = posture.getJointOrientationGlobal(idx)
        posture.setJointOrientationGlobal(idx, np.dot(footRot1, footOri1))

        inside_new = posture.getJointPositionGlobal(idx, insideOffset)
        outside_new_tmp = posture.getJointPositionGlobal(idx, outsideOffset)

        # RotVec2 = inside_new - origin
        width = np.linalg.norm(outside - inside)
        widthVec_tmp = np.cross(RotVec1_tmp1, np.array((0., 1., 0.))) if isLeftFoot ^ isOutside \
            else np.cross(np.array((0., 1., 0.)), RotVec1_tmp1)

        widthVec = width * widthVec_tmp / np.linalg.norm(widthVec_tmp)
        outside_new = inside_new + widthVec

        footRot2 = mm.getSO3FromVectors(outside_new_tmp - inside_new, widthVec)
        footOri2 = posture.getJointOrientationGlobal(idx)
        # print footRot2, footOri2
        newFootOri = np.dot(footRot2, footOri2)
        # posture.setJointOrientationGlobal(idx, np.dot(footRot2, footOri2))

        return newFootOri, inside_new, outside_new

    def makeFourContactPos(posture, jointNameOrIdx, isLeftFoot=True, isOutside=True):
        """

        :type posture: ym.JointPosture
        :type jointNameOrIdx: str | int
        :return: np.array, np.array, np.array
        """
        idx = jointNameOrIdx
        if type(jointNameOrIdx) == str:
            idx = posture.skeleton.getJointIndex(jointNameOrIdx)

        insideOffset = SEGMENT_FOOT_MAG * np.array((0., 0., 2.5))
        outsideOffset = SEGMENT_FOOT_MAG * np.array((1.2, 0., 2.5))
        if isLeftFoot ^ isOutside:
            # if it is not outside phalange,
            outsideOffset[0] = -1.2 * SEGMENT_FOOT_MAG

        origin = posture.getJointPositionGlobal(idx)
        inside = posture.getJointPositionGlobal(idx, insideOffset)
        outside = posture.getJointPositionGlobal(idx, outsideOffset)

        length = SEGMENT_FOOT_MAG * 2.5

        RotVec1_tmp1 = inside - origin
        RotVec1_tmp2 = inside - origin
        RotVec1_tmp2[1] = 0.
        RotVec1 = np.cross(RotVec1_tmp1, RotVec1_tmp2)
        angle1_1 = math.acos((origin[1] - SEGMENT_FOOT_RAD)/length)
        angle1_2 = math.acos((origin[1] - inside[1])/length)
        footRot1 = mm.exp(RotVec1, angle1_1-angle1_2)
        footOri1 = posture.getJointOrientationGlobal(idx)
        posture.setJointOrientationGlobal(idx, np.dot(footRot1, footOri1))

        inside_new = posture.getJointPositionGlobal(idx, insideOffset)
        outside_new_tmp = posture.getJointPositionGlobal(idx, outsideOffset)

        # RotVec2 = inside_new - origin
        width = np.linalg.norm(outside - inside)
        widthVec_tmp = np.cross(RotVec1_tmp1, np.array((0., 1., 0.))) if isLeftFoot ^ isOutside \
            else np.cross(np.array((0., 1., 0.)), RotVec1_tmp1)

        widthVec = width * widthVec_tmp / np.linalg.norm(widthVec_tmp)
        outside_new = inside_new + widthVec

        footRot2 = mm.getSO3FromVectors(outside_new_tmp - inside_new, widthVec)
        footOri2 = posture.getJointOrientationGlobal(idx)
        # print footRot2, footOri2
        posture.setJointOrientationGlobal(idx, np.dot(footRot2, footOri2))
        return

    def getFootSegNormal(posture, jointNameOrIdx, isLeftFoot=True, isOutside=True):
        """

        :type posture: ym.JointPosture
        :type jointNameOrIdx: str | int
        :return: np.array, np.array, np.array
        """
        idx = jointNameOrIdx
        if type(jointNameOrIdx) == str:
            idx = posture.skeleton.getJointIndex(jointNameOrIdx)

        insideOffset = SEGMENT_FOOT_MAG * np.array((0., 0., 2.5))
        outsideOffset = SEGMENT_FOOT_MAG * np.array((1.2, 0., 2.5))
        if isLeftFoot ^ isOutside:
            # if it is not outside phalange,
            outsideOffset[0] = -1.2 * SEGMENT_FOOT_MAG

        origin = posture.getJointPositionGlobal(idx)
        inside = posture.getJointPositionGlobal(idx, insideOffset)
        outside = posture.getJointPositionGlobal(idx, outsideOffset)

        if isLeftFoot ^ isOutside:
            return mm.normalize(-np.cross(inside - origin, outside - origin))
        else:
            return mm.normalize(np.cross(inside - origin, outside - origin))


    # get collision info
    collide = dict()  # type: dict[str, bool]

    for side in ['Left', 'Right']:
        for sideInFoot in ['outside', 'inside']: # outside first!
            isLeftFoot = True if side == 'Left' else False
            isOutside = True if sideInFoot == 'outside' else False
            footPrefix = 'Foot_foot_0_' + ('0' if isOutside else '1')

            collide[side+footPrefix+'_0_Effector'] = \
                getJointChildPositionGlobal(posture_ori, side+footPrefix+'_0')[1] < SEGMENT_FOOT_RAD + baseHeight
            collide[side+footPrefix+'_0'] = \
                posture_ori.getJointPositionGlobal(footIdDic[side+footPrefix+'_0'])[1] < SEGMENT_FOOT_RAD + baseHeight
            collide[side+footPrefix+''] = \
                posture_ori.getJointPositionGlobal(footIdDic[side+footPrefix])[1] < SEGMENT_FOOT_RAD + baseHeight

            if collide[side+footPrefix+'_0_Effector'] and collide[side+footPrefix+'_0'] and collide[side+footPrefix+'']:
                # all segment contact
                footVec = getFootSegNormal(posture_ori, side+footPrefix+'', isLeftFoot=isLeftFoot, isOutside=isOutside)
                footRot = mm.getSO3FromVectors(footVec, np.array((0., 1., 0.)))
                footIdx = posture_ori.skeleton.getJointIndex(side+footPrefix+'')
                footOri = posture_ori.getJointOrientationGlobal(footIdx)
                posture_ori.setJointOrientationGlobal(footIdx, np.dot(footRot, footOri))

            elif collide[side+footPrefix+'_0_Effector'] and collide[side+footPrefix+'_0']:
                # toe fully, phalange partially
                newFootOri, _inside, _outside = makeTwoContactPos(posture_ori, side+footPrefix+'', isLeftFoot=isLeftFoot, isOutside=isOutside, baseHeight=baseHeight)
                posture_ori.setJointOrientationGlobal(footIdDic[side+footPrefix+''], newFootOri)

                footVec = getFootSegNormal(posture_ori, side+footPrefix+'_0', isLeftFoot=isLeftFoot, isOutside=isOutside)
                footRot = mm.getSO3FromVectors(footVec, np.array((0., 1., 0.)))
                footIdx = posture_ori.skeleton.getJointIndex(side+footPrefix+'_0')
                footOri = posture_ori.getJointOrientationGlobal(footIdx)
                posture_ori.setJointOrientationGlobal(footIdx, np.dot(footRot, footOri))

                outsideOffset = np.array((-1., 0., 0.)) if isLeftFoot ^ isOutside else np.array((1., 0., 0.))
                inside_tmp = posture_ori.getJointPositionGlobal(footIdDic[side+footPrefix+'_0'])
                outside_tmp = posture_ori.getJointPositionGlobal(footIdDic[side+footPrefix+'_0'], outsideOffset)
                footRot2 = mm.getSO3FromVectors(outside_tmp - inside_tmp, _outside - _inside)
                posture_ori.setJointOrientationGlobal(footIdx, np.dot(footRot2, np.dot(footRot, footOri)))

            elif collide[side+footPrefix+'_0_Effector']:
                # toe partially
                footPoint = posture_ori.getJointPositionGlobal(footIdDic[side+footPrefix+'_0'])

                newFootOri, _inside, _outside = makeTwoContactPos(posture_ori, side+footPrefix+'', isLeftFoot=isLeftFoot, isOutside=isOutside, baseHeight=footPoint[1]-SEGMENT_FOOT_RAD)
                posture_ori.setJointOrientationGlobal(posture_ori.skeleton.getJointIndex(side+footPrefix+''), newFootOri)
                footVec = getFootSegNormal(posture_ori, side+footPrefix+'_0', isLeftFoot=isLeftFoot, isOutside=isOutside)
                footRot = mm.getSO3FromVectors(footVec, np.array((0., 1., 0.)))
                footIdx = posture_ori.skeleton.getJointIndex(side+footPrefix+'_0')
                footOri = posture_ori.getJointOrientationGlobal(footIdx)
                posture_ori.setJointOrientationGlobal(footIdx, np.dot(footRot, footOri))

                outsideOffset = np.array((-1., 0., 0.)) if isLeftFoot ^ isOutside else np.array((1., 0., 0.))
                inside_tmp = posture_ori.getJointPositionGlobal(footIdDic[side+footPrefix+'_0'])
                outside_tmp = posture_ori.getJointPositionGlobal(footIdDic[side+footPrefix+'_0'], outsideOffset)
                footRot2 = mm.getSO3FromVectors(outside_tmp - inside_tmp, _outside - _inside)
                posture_ori.setJointOrientationGlobal(footIdx, np.dot(footRot2, np.dot(footRot, footOri)))

            elif getJointChildPositionGlobal(posture_ori, side+footPrefix+'_0')[1] < SEGMENT_FOOT_RAD*1.5 + baseHeight:
                # In case of posibility of contact
                # if 1 radius <  toe height < 3/2 radius, this routine is working.
                toeHeight = getJointChildPositionGlobal(posture_ori, side+footPrefix+'_0')[1]
                ratio = (SEGMENT_FOOT_RAD*1.5 +baseHeight - toeHeight)/SEGMENT_FOOT_RAD * 2.

                footPoint = posture_ori.getJointPositionGlobal(footIdDic[side+footPrefix+'_0'])
                newFootOri, _inside, _outside = makeTwoContactPos(posture_ori, side+footPrefix+'', isLeftFoot=isLeftFoot, isOutside=isOutside, baseHeight=footPoint[1]-SEGMENT_FOOT_RAD)

                oldFootOri = posture_ori.getJointOrientationGlobal(footIdDic[side+footPrefix+''])
                posture_ori.setJointOrientationGlobal(footIdDic[side+footPrefix+''], mm.slerp(oldFootOri, newFootOri, ratio))

                oldFootOri2 = posture_ori.getJointOrientationGlobal(footIdDic[side+footPrefix+'_0'])
                footVec = getFootSegNormal(posture_ori, side+footPrefix+'_0', isLeftFoot=isLeftFoot, isOutside=isOutside)
                footRot = mm.getSO3FromVectors(footVec, np.array((0., 1., 0.)))
                footIdx = posture_ori.skeleton.getJointIndex(side+footPrefix+'_0')
                footOri = posture_ori.getJointOrientationGlobal(footIdx)
                posture_ori.setJointOrientationGlobal(footIdx, np.dot(footRot, footOri))

                outsideOffset = np.array((-1., 0., 0.)) if isLeftFoot ^ isOutside else np.array((1., 0., 0.))
                inside_tmp = posture_ori.getJointPositionGlobal(footIdDic[side+footPrefix+'_0'])
                outside_tmp = posture_ori.getJointPositionGlobal(footIdDic[side+footPrefix+'_0'], outsideOffset)
                footRot2 = mm.getSO3FromVectors(outside_tmp - inside_tmp, _outside - _inside)
                posture_ori.setJointOrientationGlobal(footIdx, mm.slerp(oldFootOri2, np.dot(footRot2, np.dot(footRot, footOri)), ratio))

        if True: # back side
            isLeftFoot = True if side == 'Left' else False
            footPrefix = 'Foot_foot_1'

            collide[side+footPrefix+'_0_Effector'] = \
                getJointChildPositionGlobal(posture_ori, side+footPrefix+'_0')[1] < SEGMENT_FOOT_RAD + baseHeight
            collide[side+footPrefix+'_0'] = \
                posture_ori.getJointPositionGlobal(footIdDic[side+footPrefix+'_0'])[1] < SEGMENT_FOOT_RAD + baseHeight

            # if collide[side+footPrefix+'_0_Effector'] and collide[side+footPrefix+'_0']:
            if collide[side+footPrefix+'_0_Effector']:
                # heel contact partially or fully
                heel_idx = footIdDic[side+footPrefix+'_0']
                R_cur = posture_ori.getJointOrientationGlobal(heel_idx)

                insideOffset = SEGMENT_FOOT_MAG * np.array((-.6, 0., 1.2))
                outsideOffset = SEGMENT_FOOT_MAG * np.array((.6, 0., 1.2))

                origin = posture_ori.getJointPositionGlobal(heel_idx)
                inside = posture_ori.getJointPositionGlobal(heel_idx, insideOffset)
                outside = posture_ori.getJointPositionGlobal(heel_idx, outsideOffset)

                # rot_vec = mm.normalize(np.cross(inside - origin, origin - outside if side == 'Left' else outside - origin))
                rot_vec = mm.normalize(np.cross(inside - origin, outside - origin))

                rot_to_y = mm.getSO3FromVectors(rot_vec, mm.unitY())

                posture_ori.setJointOrientationGlobal(heel_idx, np.dot(rot_to_y, R_cur))