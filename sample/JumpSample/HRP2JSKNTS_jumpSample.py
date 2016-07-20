import yaml
import time

package_path = roslib.packages.get_pkg_dir("jsk_choreonoid")
sample_path = os.path.join(package_path, "sample/JumpSample")

infile = open(os.path.join(package_path, "share/model/HRP2JSKNTS_WITH_3HAND/hrp2jsknts.yaml"))
yaml_data = yaml.load(infile)
yaml_data["modelFile"] = os.path.join(roslib.packages.get_pkg_dir("hrp2_models"),"HRP2JSKNTS_WITH_3HAND/HRP2JSKNTSmain.wrl")
infile.close()

robotFileName = "/tmp/hrp2jsk.yaml"
outFile = open(robotFileName, "w")
outFile.write(yaml.dump(yaml_data))
outFile.close()

worldItem = Item.find("World")

robotItem = BodyItem()
robotItem.load(robotFileName)
worldItem.addChildItem(robotItem)
robot = robotItem.body()
ItemTreeView.instance().checkItem(robotItem)

poseSeqItem = PoseSeqItem()
poseSeqItem.load(str(os.path.join(sample_path, "HRP2JSKNTS_jump.pseq")), robotItem)
robotItem.addChildItem(poseSeqItem)
ItemTreeView.instance().selectItem(poseSeqItem)

floorItem = BodyItem()
floorItem.load(os.path.join(shareDirectory(), "model/misc/floor.wrl"))
worldItem.addChildItem(floorItem)

simulatorItem = AISTSimulatorItem()
worldItem.addChildItem(simulatorItem)
