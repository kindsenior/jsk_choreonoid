from rtshell import rtls

from cnoid.Base import *
from cnoid.BodyPlugin import *
from cnoid.Body import *
from cnoid.PoseSeqPlugin import *
from cnoid.Util import *

import os
import sys
import roslib
import inspect
from pprint import pprint
from numpy import *

# from hrpsys_choreonoid_tutorials.choreonoid_hrpsys_config import *

# robot = ItemTreeView.instance().rootItem().childItem().childItem().body()
# robotItem = ItemTreeView.instance().rootItem().childItem().childItem()
# poseSeqItem = robotItem.childItem()

# def test():
#     global hrpsys_choreonoid_tutorials_path
#     hrpsys_choreonoid_tutorials_path = os.path.join(roslib.packages.get_pkg_dir('hrpsys_choreonoid_tutorials'),'scripts')
#     sys.path.append(hrpsys_choreonoid_tutorials_path)
#     from jaxon_red_setup import *
#     sys.argv = [os.path.join(hrpsys_choreonoid_tutorials_path,'/jaxon_red_setup.py'), 'JAXON_RED(Robot)0']

#     global hcf
#     hcf = JAXON_RED_HrpsysConfigurator("JAXON_RED")
#     [sys.argv, connect_constraint_force_logger_ports] = hcf.parse_arg_for_connect_ports(sys.argv)
#     hcf.init(sys.argv[1], connect_constraint_force_logger_ports=connect_constraint_force_logger_ports)
#     hcf.startABSTIMP()
