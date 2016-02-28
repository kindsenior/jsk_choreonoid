from cnoid.Base import *
from cnoid.BodyPlugin import *
from cnoid.PoseSeqPlugin import *
from cnoid.Util import *

import os
import roslib
import inspect
from pprint import pprint

robot = ItemTreeView.instance().rootItem().childItem().childItem().body()
robotItem = ItemTreeView.instance().rootItem().childItem().childItem()
poseSeqItem = robotItem.childItem()
