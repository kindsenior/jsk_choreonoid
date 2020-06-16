#!/usr/bin/env python

import numpy as np

from util import *

def print_pose_info(body, pose, pre_print=""):
    logger.info(pre_print)
    for link_idx, link_info in pose:
        logger.info(" {}: {}".format(body.link(link_idx).name, link_info.p))
        if link_info.isTouching():
            contact_points = link_info.contactPoints()
            logger.info(" touching with {} contact points".format(len(contact_points)))


# similar to PoseSeqViewBase::setCurrentLinkStateToIkLink
def set_current_link_state_to_iklink(link, link_info, collision_link_pairs):

    if link:
        logger.info("set_current_link_state_to_iklink({}, link_info)".format(link.name))
        link_info.p = link.p
        link_info.R = link.R

        contact_points = []
        for collision_link_pair in collision_link_pairs:
            if not collision_link_pair.isSelfCollision():
                logger.info(" {} is touching".format(link.name))
                for collision in collision_link_pair.collisions:
                    contact_points.append(link.R.T.dot(collision.point - link.p))

        if len(contact_points) > 0:
            logger.info(" isTouching: {}, num contact points: {}".format(link_info.isTouching(), len(contact_points)))
            parting_direction = np.array([0,0,1.0])
            link_info.setTouching(parting_direction, contact_points)
    else:
        logger.critical("invalid link")

# similar to PoseSeqViewBase::setCurrentBodyStateToPose
def set_current_body_state_to_pose(body_item, pose):
    logger.info("set_current_body_tate_to_pose()")
    body = body_item.body

    for i in range(pose.numJoints):
        if pose.isJointValid(i):
            joint = body.joint(i)
            q = body.joint(i).q
            if q != pose.jointPosition(i):
                pose.setJointPosition(i, q)

    for link_idx, link_info in pose:
        link = body.link(link_idx)
        collision_link_pairs = body_item.collisionsOfLink(link_idx)
        set_current_link_state_to_iklink(link, link_info, collision_link_pairs)
