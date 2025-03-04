from fltk import Fl
import os
import torch
from math import exp, log, pi
from DartFootDeep.sh_v2_pd_not.ppo_v2 import PPO
from PyCommon.modules.GUI import hpSimpleViewer as hsv
from PyCommon.modules.Renderer import ysRenderer as yr
import numpy as np

import pydart2 as pydart
from PyCommon.modules.Math import mmMath as mm
from matplotlib import pyplot as plt


def main():
    MOTION_ONLY = False
    CURRENT_CHECK = False
    SKELETON_ON = False
    RSI = False
    PD_PLOT = False

    CAMERA_TRACKING = True

    pydart.init()

    env_name = 'walk'
    env_name = 'walk_repeated'
    # env_name = 'walk_fast'
    # env_name = 'walk_sukiko'
    # env_name = 'walk_u_turn'
    # env_name = '1foot_contact_run'
    # env_name = 'round_girl'
    # env_name = 'fast_2foot_hop'
    # env_name = 'slow_2foot_hop'
    # env_name = 'long_broad_jump'
    # env_name = 'short_broad_jump'
    # env_name = 'n_kick'
    # env_name = 'jump'

    gain_p0 = []
    gain_p1 = []
    gain_p2 = []
    gain_p3 = []
    gain_p4 = []
    MA_DUR = 5
    ma_gain_p0 = []
    ma_gain_p1 = []
    ma_gain_p2 = []
    ma_gain_p3 = []
    ma_gain_p4 = []

    rfoot_contact_ranges = []
    lfoot_contact_ranges = []

    ppo = PPO(env_name, 0, visualize_only=True)
    if not MOTION_ONLY and not CURRENT_CHECK:
        # ppo.LoadModel('model/' + env_name + '.pt')
        ppo.LoadModel('model/' + 'param' + '.pt')
        # ppo.LoadModel('model_test/' + env_name + '.pt')
        # ppo.LoadModel('model_test/' + 'param' + '.pt')
    elif not MOTION_ONLY and CURRENT_CHECK:
        env_model_dir = []
        for dir_name in sorted(os.listdir()):
            if env_name in dir_name:
                env_model_dir.append(dir_name)

        pt_names = os.listdir(env_model_dir[-1])
        pt_names.pop(pt_names.index('log.txt'))
        pt_names.sort(key=lambda f: int(os.path.splitext(f)[0]))
        pt_names.append('1171.pt')
        ppo.LoadModel(env_model_dir[-1]+'/'+pt_names[-1])
        print(env_model_dir[-1]+'/'+pt_names[-1])

    ppo.env.Resets(RSI)
    ppo.env.ref_skel.set_positions(ppo.env.ref_motion.get_q(ppo.env.phase_frame))

    # viewer settings
    rd_contact_positions = [None]
    rd_contact_forces = [None]
    dart_world = ppo.env.world
    skel = dart_world.skeletons[1]
    viewer_w, viewer_h = 960, 1080
    viewer = hsv.hpSimpleViewer(rect=(0, 0, viewer_w+300, 1+viewer_h+55), viewForceWnd=False)
    viewer.doc.addRenderer('MotionModel', yr.DartRenderer(ppo.env.ref_world, (150,150,255), yr.POLYGON_FILL))

    if not MOTION_ONLY:
        control_model_renderer = yr.DartRenderer(dart_world, (255,240,255), yr.POLYGON_FILL)
        viewer.doc.addRenderer('controlModel', control_model_renderer)
        viewer.doc.addRenderer('contact', yr.VectorsRenderer(rd_contact_forces, rd_contact_positions, (255,0,0)))

        def makeEmptyBasicSkeletonTransformDict(init=None):
            Ts = dict()
            Ts['pelvis'] = init
            Ts['spine_ribs'] = init
            Ts['head'] = init
            Ts['thigh_R'] = init
            Ts['shin_R'] = init
            Ts['foot_heel_R'] = init
            Ts['foot_R'] = init
            Ts['heel_R'] = init
            Ts['outside_metatarsal_R'] = init
            Ts['outside_phalanges_R'] = init
            Ts['inside_metatarsal_R'] = init
            Ts['inside_phalanges_R'] = init
            Ts['upper_limb_R'] = init
            Ts['lower_limb_R'] = init
            Ts['thigh_L'] = init
            Ts['shin_L'] = init
            Ts['foot_heel_L'] = init
            Ts['foot_L'] = init
            Ts['heel_L'] = init
            Ts['outside_metatarsal_L'] = init
            Ts['outside_phalanges_L'] = init
            Ts['inside_metatarsal_L'] = init
            Ts['inside_phalanges_L'] = init

            Ts['upper_limb_L'] = init
            Ts['lower_limb_L'] = init

            return Ts
        skeleton_renderer = None
        if SKELETON_ON:
            # skeleton_renderer = yr.BasicSkeletonRenderer(makeEmptyBasicSkeletonTransformDict(np.eye(4)), offset_Y=-0.08)
            # skeleton_renderer = yr.BasicSkeletonRenderer(makeEmptyBasicSkeletonTransformDict(np.eye(4)), color=(230, 230, 230), offset_draw=(0.8, -0.02, 0.))
            skeleton_renderer = yr.BasicSkeletonRenderer(makeEmptyBasicSkeletonTransformDict(np.eye(4)), color=(230, 230, 230), offset_draw=(0., -0.0, 0.))
            viewer.doc.addRenderer('skeleton', skeleton_renderer)

    def postCallback(frame):
        ppo.env.ref_skel.set_positions(ppo.env.ref_motion.get_q(frame-1))
        ppo.env.ref_motion.frame = frame-1

    def simulateCallback(frame):
        state = ppo.env.GetState(0)
        action_dist, v = ppo.model(torch.tensor(state.reshape(1, -1)).float())
        action = action_dist.loc.detach().numpy()
        value = v.detach().numpy()
        res = ppo.env.Steps(action)

        # for gain plotting
        gain_p0.append(res[3]['kp'][ppo.env.skel.dof_index('j_RightFoot_x')])
        gain_p1.append(res[3]['kp'][ppo.env.skel.dof_index('j_RightFoot_foot_0_0_x')])
        gain_p2.append(res[3]['kp'][ppo.env.skel.dof_index('j_RightFoot_foot_0_0_0_x')])
        gain_p3.append(res[3]['kp'][ppo.env.skel.dof_index('j_RightFoot_foot_0_1_0_x')])
        gain_p4.append(res[3]['kp'][ppo.env.skel.dof_index('j_RightFoot_foot_1_0_x')])
        ma_gain_p0.append(sum(gain_p0[-MA_DUR:])/MA_DUR if len(gain_p0) >= MA_DUR else sum(gain_p0)/len(gain_p0))
        ma_gain_p1.append(sum(gain_p1[-MA_DUR:])/MA_DUR if len(gain_p1) >= MA_DUR else sum(gain_p1)/len(gain_p1))
        ma_gain_p2.append(sum(gain_p2[-MA_DUR:])/MA_DUR if len(gain_p2) >= MA_DUR else sum(gain_p2)/len(gain_p2))
        ma_gain_p3.append(sum(gain_p3[-MA_DUR:])/MA_DUR if len(gain_p3) >= MA_DUR else sum(gain_p3)/len(gain_p3))
        ma_gain_p4.append(sum(gain_p4[-MA_DUR:])/MA_DUR if len(gain_p4) >= MA_DUR else sum(gain_p4)/len(gain_p4))

        contacts = ppo.env.world.collision_result.contacts
        if any(['RightFoot' in body.name for body in ppo.env.world.collision_result.contacted_bodies]):
            if rfoot_contact_ranges and rfoot_contact_ranges[-1][1] == frame-1:
                rfoot_contact_ranges[-1][1] = frame
            else:
                rfoot_contact_ranges.append([frame, frame])

        # res = ppo.env.Steps(np.zeros_like(action))
        ppo.env.world.collision_result.update()
        # print(frame, ppo.env.Ref_skel.current_frame, ppo.env.world.time()*ppo.env.ref_motion.fps)
        # print(frame, res[0][0])
        # if res[0][0] > 0.46:
        #     ppo.env.continue_from_now_by_phase(0.2)
        # print(frame, ' '.join(["{:0.1f}".format(400. * exp(log(400.) * rate/10.)) for rate in action[0][ppo.env.skel.ndofs-6:]]))

        # if res[2]:
        #     print(frame, 'Done')
        #     ppo.env.reset()

        contacts = ppo.env.world.collision_result.contacts
        # contact body rendering
        if control_model_renderer is not None:
            skel_idx = dart_world.skel.id
            for body_idx in range(dart_world.skeletons[skel_idx].num_bodynodes()):
                for shape_idx in range(dart_world.skeletons[skel_idx].body(body_idx).num_shapenodes()):
                    control_model_renderer.geom_colors[skel_idx][body_idx][shape_idx] = control_model_renderer.totalColor

            for contact in contacts:
                body_idx, geom_idx = (contact.bodynode_id1, contact.shape_id1) if dart_world.skeletons[contact.skel_id1].body(contact.bodynode_id1).name != 'ground' else (contact.bodynode_id2, contact.shape_id2)
                body = dart_world.skeletons[skel_idx].body(body_idx)
                visual = sum(shapenode.has_visual_aspect() for shapenode in body.shapenodes)
                collision = sum(shapenode.has_collision_aspect() for shapenode in body.shapenodes)
                if visual == collision:
                    control_model_renderer.geom_colors[skel_idx][body_idx][geom_idx-visual] = (255, 0, 0)
                else:
                    control_model_renderer.geom_colors[skel_idx][body_idx][(geom_idx-visual)//2] = (255, 0, 0)

        if PD_PLOT:
            fig = plt.figure(1)
            plt.clf()
            fig.add_subplot(5, 1, 1)
            for rfoot_contact_range in rfoot_contact_ranges:
                plt.axvspan(rfoot_contact_range[0], rfoot_contact_range[1], facecolor='0.5', alpha=0.3)
            plt.ylabel('ankle')
            plt.plot(range(len(ma_gain_p0)), ma_gain_p0)
            fig.add_subplot(5, 1, 2)
            for rfoot_contact_range in rfoot_contact_ranges:
                plt.axvspan(rfoot_contact_range[0], rfoot_contact_range[1], facecolor='0.5', alpha=0.3)
            plt.ylabel('talus')
            plt.plot(range(len(ma_gain_p1)), ma_gain_p1)
            fig.add_subplot(5, 1, 3)
            for rfoot_contact_range in rfoot_contact_ranges:
                plt.axvspan(rfoot_contact_range[0], rfoot_contact_range[1], facecolor='0.5', alpha=0.3)
            plt.ylabel('thumb')
            plt.plot(range(len(ma_gain_p2)), ma_gain_p2)
            fig.add_subplot(5, 1, 4)
            for rfoot_contact_range in rfoot_contact_ranges:
                plt.axvspan(rfoot_contact_range[0], rfoot_contact_range[1], facecolor='0.5', alpha=0.3)
            plt.ylabel('phalange')
            plt.plot(range(len(ma_gain_p3)), ma_gain_p3)
            fig.add_subplot(5, 1, 5)
            for rfoot_contact_range in rfoot_contact_ranges:
                plt.axvspan(rfoot_contact_range[0], rfoot_contact_range[1], facecolor='0.5', alpha=0.3)
            plt.ylabel('heel')
            plt.plot(range(len(ma_gain_p4)), ma_gain_p4)
            plt.show()
            plt.pause(0.001)

        # contact rendering
        contacts = ppo.env.world.collision_result.contacts
        del rd_contact_forces[:]
        del rd_contact_positions[:]
        for contact in contacts:
            rd_contact_forces.append(contact.f/1000.)
            rd_contact_positions.append(contact.p)

        # render skeleton
        if SKELETON_ON:
            Ts = dict()
            Ts['pelvis'] = skel.joint('j_Hips').get_local_transform()
            Ts['thigh_R'] = skel.joint('j_RightUpLeg').get_local_transform()
            Ts['shin_R'] = skel.joint('j_RightLeg').get_local_transform()
            Ts['foot_R'] = skel.joint('j_RightFoot').get_local_transform()
            Ts['foot_heel_R'] = skel.joint('j_RightFoot').get_local_transform()
            Ts['heel_R'] = np.eye(4)
            Ts['outside_metatarsal_R'] = skel.joint('j_RightFoot_foot_0_0').get_local_transform()
            Ts['outside_phalanges_R'] = skel.joint('j_RightFoot_foot_0_0_0').get_local_transform()
            # Ts['inside_metatarsal_R'] = controlModel.getJointTransform(idDic['RightFoot_foot_0_1'])
            Ts['inside_metatarsal_R'] = np.eye(4)
            Ts['inside_phalanges_R'] = skel.joint('j_RightFoot_foot_0_1_0').get_local_transform()
            Ts['spine_ribs'] = skel.joint('j_Spine').get_local_transform()
            Ts['head'] = skel.joint('j_Spine1').get_local_transform()
            Ts['upper_limb_R'] = skel.joint('j_RightArm').get_local_transform()
            # Ts['upper_limb_R'] = np.dot(skel.joint('j_RightArm').get_local_transform(), mm.SO3ToSE3(mm.rotX(pi/6.)))
            Ts['lower_limb_R'] = skel.joint('j_RightForeArm').get_local_transform()
            # Ts['lower_limb_R'] = np.dot(skel.joint('j_RightForeArm').get_local_transform(), mm.SO3ToSE3(mm.rotY(-pi/6.)))
            Ts['thigh_L'] = skel.joint('j_LeftUpLeg').get_local_transform()
            Ts['shin_L'] = skel.joint('j_LeftLeg').get_local_transform()
            Ts['foot_L'] = skel.joint('j_LeftFoot').get_local_transform()
            Ts['foot_heel_L'] = skel.joint('j_LeftFoot').get_local_transform()
            Ts['heel_L'] = np.eye(4)
            Ts['outside_metatarsal_L'] = skel.joint('j_LeftFoot_foot_0_0').get_local_transform()
            Ts['outside_phalanges_L'] = skel.joint('j_LeftFoot_foot_0_0_0').get_local_transform()
            # Ts['inside_metatarsal_L'] = controlModel.getJointTransform(idDic['LeftFoot_foot_0_1'])
            Ts['inside_metatarsal_L'] = np.eye(4)
            Ts['inside_phalanges_L'] = skel.joint('j_LeftFoot_foot_0_1_0').get_local_transform()
            Ts['upper_limb_L'] = skel.joint('j_LeftArm').get_local_transform()
            # Ts['upper_limb_L'] = np.dot(skel.joint('j_LeftArm').get_local_transform(), mm.SO3ToSE3(mm.rotX(pi/6.)))
            Ts['lower_limb_L'] = skel.joint('j_LeftForeArm').get_local_transform()
            # Ts['lower_limb_L'] = np.dot(skel.joint('j_LeftForeArm').get_local_transform(), mm.SO3ToSE3(mm.rotY(pi/6.)))

            skeleton_renderer.appendFrameState(Ts)

    if MOTION_ONLY:
        viewer.setPostFrameCallback_Always(postCallback)
        viewer.setMaxFrame(len(ppo.env.ref_motion)-1)
    else:
        viewer.setSimulateCallback(simulateCallback)
        viewer.setMaxFrame(3000)

        if CAMERA_TRACKING:
            cameraTargets = [None] * (viewer.getMaxFrame()+1)

        def postFrameCallback_Always(frame):
            if CAMERA_TRACKING:
                if cameraTargets[frame] is None:
                    cameraTargets[frame] = ppo.env.skel.body(0).com()
                viewer.setCameraTarget(cameraTargets[frame])

        viewer.setPostFrameCallback_Always(postFrameCallback_Always)

    viewer.startTimer(1./30.)
    viewer.show()

    Fl.run()


if __name__ == '__main__':
    main()
