from PyCommon.modules.Resource import ysMotionLoader as yf
import PyCommon.modules.Motion.ysSkeletonEdit as yse
from PyCommon.modules.Math import mmMath as mm

bvh = yf.readBvhFileAsBvh('wd2_jump0.bvh')
print(bvh)
bvh.swap_joint('RightUpLeg', 'LeftUpLeg')
print(bvh)
# bvh.writeBvhFile('wd2_jump0_edited.bvh')
motion = bvh.toJointMotion(scale=1., applyRootOffset=False)

yse.rotateJointLocal(motion, 'LeftFoot', mm.rotX(0.4), False)
yse.rotateJointLocal(motion, 'RightFoot', mm.rotX(0.4), False)
yf.writeBvhFile('wd2_jump0_edited.bvh', motion)

