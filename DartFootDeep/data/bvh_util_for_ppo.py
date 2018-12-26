from PyCommon.modules.Resource import ysMotionLoader as yf

bvh = yf.readBvhFileAsBvh('wd2_jump0.bvh')
print(bvh)
bvh.swap_joint('RightUpLeg', 'LeftUpLeg')
bvh.writeBvhFile('wd2_jump0_edited.bvh')

print(bvh)