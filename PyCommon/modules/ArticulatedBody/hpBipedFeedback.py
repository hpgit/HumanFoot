from PyCommon.modules.Motion import ysBipedAnalysis as yba
from PyCommon.modules.Math import mmMath as mm
from PyCommon.modules.Math import ysFunctionGraph as yfg
from PyCommon.modules.Motion import ysMotionAnalysis as yma

import numpy as np

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


class HpBipedFeedback():
    def __init__(self, phys_model, motion_ori, seginfo):
        self.prev_R_swp = []
        self.phys_model = phys_model
        self.seginfo = seginfo
        self.motion_ori = motion_ori


    def refresh_frame_fbpar_information(self,
                                      Kt,
                                      Dt,
                                      Ks,
                                      Ds,
                                      mu,
                                      c_min_contact_vel,
                                      c_min_contact_time,
                                      c_landing_duration,
                                      c_taking_duration,
                                      c_swf_mid_offset,
                                      c_locking_vel,
                                      c_swf_offset,
                                      K_stp_pos,
                                      c5,
                                      c6,
                                      K_stb_vel,
                                      K_stb_pos,
                                      K_swp_vel_sag,
                                      K_swp_vel_cor,
                                      K_swp_pos_sag,
                                      K_swp_pos_cor,
                                      K_swp_pos_sag_faster
                                      ):
        self.Kt=Kt
        self.Dt=Dt
        self.Ks=Ks
        self.Ds=Ds
        self.mu=mu
        self.c_min_contact_vel=c_min_contact_vel
        self.c_min_contact_time=c_min_contact_time
        self.c_landing_duration=c_landing_duration
        self.c_taking_duration=c_taking_duration
        self.c_swf_mid_offset=c_swf_mid_offset
        self.c_locking_vel=c_locking_vel
        self.c_swf_offset=c_swf_offset
        self.K_stp_pos=K_stp_pos
        self.c5=c5
        self.c6=c6
        self.K_stb_vel=K_stb_vel
        self.K_stb_pos=K_stb_pos
        self.K_swp_vel_sag=K_swp_vel_sag
        self.K_swp_vel_cor=K_swp_vel_cor
        self.K_swp_pos_sag=K_swp_pos_sag
        self.K_swp_pos_cor=K_swp_pos_cor
        self.K_swp_pos_sag_faster=K_swp_pos_sag_faster

    def refresh_frame_seg_information(self, seginfo, seg_index, acc_offset):
        self.cur_state = seginfo[seg_index]['state']
        self.cur_interval = yma.offsetInterval(acc_offset, seginfo[seg_index]['interval'])
        self.stance_legs = seginfo[seg_index]['stanceHips']
        self.swing_legs = seginfo[seg_index]['swingHips']
        self.stance_foots = seginfo[seg_index]['stanceFoots']
        self.swing_foots = seginfo[seg_index]['swingFoots']
        self.swing_knees = seginfo[seg_index]['swingKnees']
        self.ground_height = seginfo[seg_index]['ground_height']
        self.max_stf_push_frame = seginfo[seg_index]['max_stf_push_frame']

    def refresh_frame_dyn_information(self, motion_seg, frame, avg_dCM):
        self.frame = frame
        prev_frame = frame-1 if frame>0 else 0

        # information
        dCM_tar = motion_seg.getJointVelocityGlobal(0, prev_frame)
        CM_tar = motion_seg.getJointPositionGlobal(0, prev_frame)
        stf_tar = motion_seg.getJointPositionGlobal(stanceFoots[0], prev_frame)
        CMr_tar = CM_tar - stf_tar

        # dCM : average velocity of root of controlModel over 1 frame
        dCM = avg_dCM
        CM = self.phys_model.getBody("Hips").com()
        CMreal = self.phys_model.getCOM()
        stf = self.phys_model.getJointPositionGlobal(stanceFoots[0])
        CMr = CM - stf

        # diff_dCM : diff of velocity of COM between current and desired
        diff_dCM = mm.projectionOnPlane(dCM-dCM_tar, (1,0,0), (0,0,1))
        # diff_dCM_axis : perpendicular of diff_dCM
        diff_dCM_axis = np.cross((0,1,0), diff_dCM)
        rd_vec1[0] = diff_dCM
        rd_vecori1[0] = CM_tar

        diff_CMr = mm.projectionOnPlane(CMr-CMr_tar, (1,0,0), (0,0,1))
        diff_CMr_axis = np.cross((0,1,0), diff_CMr)

        direction = mm.normalize2(mm.projectionOnPlane(dCM_tar, (1,0,0), (0,0,1)))
        directionAxis = np.cross((0,1,0), direction)

        diff_dCM_sag, diff_dCM_cor = mm.projectionOnVector2(diff_dCM, direction)
        diff_dCM_sag_axis = np.cross((0,1,0), diff_dCM_sag)
        diff_dCM_cor_axis = np.cross((0,1,0), diff_dCM_cor)

        diff_CMr_sag, diff_CMr_cor = mm.projectionOnVector2(diff_CMr, direction)
        diff_CMr_sag_axis = np.cross((0,1,0), diff_CMr_sag)
        diff_CMr_cor_axis = np.cross((0,1,0), diff_CMr_cor)

        self.t = max((frame-self.cur_interval[0])/float(self.cur_interval[1]-self.cur_interval[0]), 1.)
        t_raw = self.t

    # match stance leg
    def match_stance_leg(self, motion_edit, cur_gait_state, stanceLegs):
        if cur_gait_state != yba.GaitState.STOP:
            for stanceLegIdx in range(len(stanceLegs)):
                stanceLeg = stanceLegs[stanceLegIdx]
                # stanceFoot = stanceFoots[stanceLegIdx]

                # motion stance leg -> character stance leg as time goes
                R_motion = motion_edit[self.frame].getJointOrientationGlobal(stanceLeg)
                R_character = self.phys_model.getJointOrientationGlobal(stanceLeg)
                motion_edit[self.frame].setJointOrientationGlobal(stanceLeg,
                                                             mm.slerp(R_motion, R_character, match_stl_func(t)))

    def swing_foot_placement(self, phys_model, motion_edit, frame, is_extended):
        global prev_R_swp

        t_swing_foot_placement = swf_placement_func(t)

        if is_extended:
            R_swp_sag = prev_R_swp[0][0]
            R_swp_cor = prev_R_swp[0][1]
        else:

            R_swp_sag = mm.I_SO3()
            R_swp_cor = mm.I_SO3()
            # R_swp_sag = np.dot(R_swp_sag, mm.exp(diff_dCM_sag_axis * K_swp_vel_sag * -t_swing_foot_placement))
            # R_swp_cor = np.dot(R_swp_cor, mm.exp(diff_dCM_cor_axis * K_swp_vel_cor * -t_swing_foot_placement))
            # if np.dot(direction, diff_CMr_sag) < 0:
            #     R_swp_sag = np.dot(R_swp_sag, mm.exp(diff_CMr_sag_axis * K_swp_pos_sag * -t_swing_foot_placement))
            # else:
            #     R_swp_sag = np.dot(R_swp_sag, mm.exp(diff_CMr_sag_axis * K_swp_pos_sag_faster * -t_swing_foot_placement))
            # R_swp_cor = np.dot(R_swp_cor, mm.exp(diff_CMr_cor_axis * K_swp_pos_cor * -t_swing_foot_placement))
            R_swp_sag = np.dot(R_swp_sag, mm.clampExp(diff_dCM_sag_axis * K_swp_vel_sag * -t_swing_foot_placement, math.pi/12.))
            R_swp_cor = np.dot(R_swp_cor, mm.clampExp(diff_dCM_cor_axis * K_swp_vel_cor * -t_swing_foot_placement, math.pi/12.))
            if np.dot(direction, diff_CMr_sag) < 0:
                R_swp_sag = np.dot(R_swp_sag, mm.clampExp(diff_CMr_sag_axis * K_swp_pos_sag * -t_swing_foot_placement, math.pi/12.))
            else:
                R_swp_sag = np.dot(R_swp_sag, mm.clampExp(diff_CMr_sag_axis * K_swp_pos_sag_faster * -t_swing_foot_placement, math.pi/12.))
            R_swp_cor = np.dot(R_swp_cor, mm.clampExp(diff_CMr_cor_axis * K_swp_pos_cor * -t_swing_foot_placement, math.pi/12.))

        for i in range(len(swingLegs)):
            swingLeg = swingLegs[i]
            swingFoot = swingFoots[i]

            # save swing foot global orientation
            R_swf = motion_swf_placement[frame].getJointOrientationGlobal(swingFoot)

            # rotate swing leg
            motion_swf_placement[frame].mulJointOrientationGlobal(swingLeg, R_swp_sag)
            motion_swf_placement[frame].mulJointOrientationGlobal(swingLeg, R_swp_cor)

            # restore swing foot global orientation
            motion_swf_placement[frame].setJointOrientationGlobal(swingFoot, R_swf)

            # motion_swf_placement[frame].setJointOrientationGlobal(swingFoot, )

            # hwangpil
            # temporal code.... for heel strike and ankle pushup
            # motion_swf_placement[frame].mulJointOrientationGlobal(swingFoot, mm.exp([0., 0., -0.17*t_swing_foot_placement]))
            # motion_swf_placement[frame].mulJointOrientationGlobal(swingFoot, mm.exp([0.2*t_swing_foot_placement, 0., 0.]))

            # hwangpil
            # foot placement based on difference
            # # CM = dartModel.getBody("Hips").com()
            # swf = dartModel.getJointPositionGlobal(swingFoot)
            # CMr_swf = CM - swf
            #
            # # CM_tar = motion_seg.getJointPositionGlobal(0, prev_frame)
            # swf_tar = motion_seg.getJointPositionGlobal(swingFoot, prev_frame)
            # CMr_swf_tar = CM_tar - swf_tar
            #
            # diff_CMr_swf = mm.projectionOnPlane(CMr_swf-CMr_swf_tar, (1,0,0), (0,0,1))
            #
            # newPosition =  motion_swf_placement[frame].getJointPositionGlobal(swingFoot)
            # # newPosition += (diff_CMr_swf + diff_dCM)*t_swing_foot_placement
            # newPosition += 0.1*diff_CMr_swf * t_swing_foot_placement
            # aik.ik_analytic(motion_swf_placement[frame], swingFoot, newPosition)

            prev_R_swp[0] = (R_swp_sag, R_swp_cor)
