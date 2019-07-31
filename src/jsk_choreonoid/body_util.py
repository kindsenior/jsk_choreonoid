# -*-coding:utf-8-*-
from cnoid import Body, BodyPlugin
import numpy as np

# define function
## cnoid.Body.Body (robot)
def links(self):
    num=self.numLinks
    return [self.link(link_i) for link_i in range(num)]
def jointList(self):
    num=self.numJoints
    return [self.joint(joint_i) for joint_i in range(num)]
def angleVector(self,angles=None):
    if not angles is None:
        if self.numJoints != len(angles):
            raise TypeError('length of angles do not agree with self.numJoints')
        for j,joint in enumerate(self.jointList()):
            joint.q=angles[j]
    return np.array([joint.q for joint in self.jointList()])
## cnoid.BodyPlugin.BodyItem (robotItem)
def drawBodyItem(self):
    self.calcForwardKinematics()
    self.notifyKinematicStateChange()
    return True

# add method
## cnoid.Body.Body (robot)
Body.Body.links=links
Body.Body.jointList=jointList
Body.Body.angleVector=angleVector

## cnoid.BodyPlugin.BodyItem (robotItem)
BodyPlugin.BodyItem.draw=drawBodyItem
