#!/usr/bin/env python

from poseseq_util import *

import pdb as ipdb

tree = Base.ItemTreeView.instance
rootItem = Base.RootItem.instance

def __update_poseseq_impl(poseseq_items=None, move_offset_pos=None, tmp_offset_pos=None):
    global poseseq_item
    global poseseq

    if poseseq_items is None: poseseq_items = tree.selectedItems(PoseSeqPlugin.PoseSeqItem)
    if type(poseseq_items) is not list: poseseq_items = [poseseq_item]

    if len(poseseq_items) < 1: raise RuntimeError("Please select or pass PoseSeqItems")

    # ipdb.set_trace()

    move_offset_pos = np.zeros(3) if move_offset_pos is None else move_offset_pos
    tmp_offset_pos = np.array([0, 0, 0.1]) if tmp_offset_pos is None else tmp_offset_pos

    # unselect selected items
    for poseseq_item in poseseq_items:
        tree.selectItem(poseseq_item, False)

    for poseseq_item in poseseq_items:
        logger.info("PoseSeqItem: {}".format(poseseq_item.name))
        tree.selectItem(poseseq_item)

        parent_body_item = poseseq_item.getAncestorItems(target_item_types=[BodyPlugin.BodyItem])[0]
        motion = poseseq_item.bodyMotionItem().motion

        poseseq = poseseq_item.poseSeq()
        for pose_ref in poseseq:
            pose = pose_ref.getPose()
            pose_time = pose_ref.time()
            pose_idx = int(pose_time * motion.frameRate)

            logger.info("time: {} ({})".format(pose_time, pose_idx))

            # set current pose from BodyMotion
            motion.frame(pose_idx) >> parent_body_item.body
            parent_body_item.body.calcForwardKinematics()
            parent_body_item.notifyKinematicStateChange()

            # move temporarily
            parent_body_item.body.rootLink.p += tmp_offset_pos + move_offset_pos
            parent_body_item.body.calcForwardKinematics()
            parent_body_item.notifyKinematicStateChange()

            # move to target position
            parent_body_item.body.rootLink.p -= tmp_offset_pos
            parent_body_item.body.calcForwardKinematics()
            parent_body_item.notifyKinematicStateChange()

            # update pose
            print_pose_info(parent_body_item.body, pose, pre_print="before update")
            # poseseq_item.beginEditing() # needless ?
            poseseq.beginPoseModification(pose_ref)
            set_current_body_state_to_pose(parent_body_item, pose)
            poseseq.endPoseModification(pose_ref)
            # poseseq_item.endEditing() # needless ?
            print_pose_info(parent_body_item.body, pose, pre_print="after update")

def move_poseseq_robot(move_offset_pos, poseseq_items=None, **kwargs):
    logger.info("update_poseseq_contact_iklinks()")
    kwargs['move_offset_pos'] = move_offset_pos
    __update_poseseq_impl(poseseq_items, **kwargs)

def update_poseseq_contact_iklinks(poseseq_items=None, **kwargs):
    logger.info("update_poseseq_contact_iklinks()")
    __update_poseseq_impl(poseseq_items, **kwargs)

def update_all_poseseq_contact_iklinks(**kwargs):
    logger.critical('update_all_poseseq_contact_iklinks()')

    global poseseq_items

    poseseq_items = get_all_poseseq_items()
    update_poseseq_contact_iklinks(poseseq_items, **kwargs)

    poseseq_item = poseseq_items[0]
