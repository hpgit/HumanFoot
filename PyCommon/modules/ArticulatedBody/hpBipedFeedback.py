from PyCommon.modules.Motion import ysBipedAnalysis as yba
from PyCommon.modules.Math import mmMath as mm
from PyCommon.modules.Math import ysFunctionGraph as yfg

stitch_func = lambda xx : 1. - yfg.hermite2nd(xx)
#TODO:
if False:
    stf_stabilize_func = yfg.concatenate([yfg.hermite2nd, yfg.one], [c_landing_duration])
match_stl_func = yfg.hermite2nd
swf_placement_func = yfg.hermite2nd
# swf_placement_func = yfg.identity
swf_height_func = yfg.hermite2nd
swf_height_sine_func = yfg.sine
#    stf_balancing_func = yfg.concatenate([yfg.hermite2nd, yfg.one], [c_landing_duration])
stf_balancing_func = yfg.hermite2nd
# stf_balancing_func = yfg.hermite5th

# match stance leg
def match_stance_leg(t, phys_model, motion_edit, frame, cur_gait_state, stanceLegs):
    if cur_gait_state != yba.GaitState.STOP:
        for stanceLegIdx in range(len(stanceLegs)):
            stanceLeg = stanceLegs[stanceLegIdx]
            # stanceFoot = stanceFoots[stanceLegIdx]

            # motion stance leg -> character stance leg as time goes
            R_motion = motion_edit[frame].getJointOrientationGlobal(stanceLeg)
            R_character = phys_model.getJointOrientationGlobal(stanceLeg)
            motion_edit[frame].setJointOrientationGlobal(stanceLeg,
                                                         mm.slerp(R_motion, R_character, match_stl_func(t)))
