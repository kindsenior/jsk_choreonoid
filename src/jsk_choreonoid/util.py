#!/usr/bin/env python

from cnoid import Base, Body, BodyPlugin, PoseSeqPlugin


def get_child_items(item, class_type=None):
    if item.childItem is None:
        ret = []
    else:
        ret = get_all_items(item.childItem, class_type, [])
    return ret


def get_all_items(item, class_type, ret=None):
    # prepare buffer
    if ret is None:
        ret = []
    # add self
    if class_type is None or type(item) is class_type:
        ret.append(item)
    # add child
    child = item.childItem
    if child is not None:
        get_all_items(child, class_type, ret)
    # add sibling
    sibling = item.nextItem
    if sibling is not None:
        get_all_items(sibling, class_type, ret)
    return ret


def get_robot(path_or_item):
    if type(path_or_item) is BodyPlugin.BodyItem:
        print "get robot from robotItem"
        return path_or_item.body
    else:
        loader = Body.BodyLoader
        robot = loader.load(str(path_or_item))
        print "load " + str(path_or_item)
        return robot


class World(object):
    itemTreeView = Base.ItemTreeView.instance
    rootItem = Base.RootItem.instance
    is_choreonoid = True if itemTreeView else False

    def __init__(self, model_path=None, worldItem=None):
        if self.is_choreonoid:  # use choreonoid
            # set world item
            if worldItem is None:
                worldItems = get_all_items(Base.ItemTreeView.instance.rootItem, BodyPlugin.WorldItem)
                if len(worldItems) == 0:
                    self.set_worldItem()
                else:
                    self.worldItem = worldItems[0]
                    self.set_child_items(worldItems[0])
            else:
                self.worldItem = worldItem
                self.set_child_items(worldItem)
            # set robot item
            if model_path is not None:
                self.set_robotItem(model_path)
        else:  # not use choreonoid
            if model_path is not None:
                self.robot = get_robot(model_path)

    # util (wrapper)

    @classmethod
    def checkItem(cls, item):
        cls.itemTreeView.checkItem(item)
        print "checked " + item.name

    @classmethod
    def selectItem(cls, item):
        cls.itemTreeView.selectItem(item)
        print "selected " + item.name

    @classmethod
    def add_worldItem(cls):
        worldItem = BodyPlugin.WorldItem()
        cls.rootItem.addChildItem(worldItem)
        print "worldItem add to rootItem"
        return worldItem

    # set item function

    def set_item(self, item, parent=None, name=None):
        # set name
        item_name = name if name else item.name.lower() + "Item"
        print "set " + item_name
        setattr(self, item_name, item)
        # add child to parent
        if parent:
            parent.addChildItem(item)
            print item_name + " add to " + parent.name + "Item"

        return item

    def set_child_items(self, item):
        child_items = get_child_items(item)
        for item in child_items:
            self.set_item(item)

    def set_worldItem(self, name="worldItem"):
        worldItem = self.set_item(self.add_worldItem(), None, name)
        return worldItem

    def set_modelItem(self, model_path, name=None):
        modelItem = self.set_item(BodyPlugin.loadBodyItem(str(model_path)), self.worldItem, name)
        self.checkItem(modelItem)
        return modelItem

    def set_robotItem(self, model_path, name="robot"):
        # set robotItem
        robotItem = self.set_modelItem(model_path,
                                       name + "Item" if name else None)
        # set robot
        robot = get_robot(robotItem)
        setattr(self, name if name else robot.name, robot)
        return robotItem

    def set_poseSeqItem(self, path, parent, name="poseSeqItem"):
        poseSeqItem = PoseSeqPlugin.PoseSeqItem
        poseSeqItem.load(str(path), parent)
        self.set_item(poseSeqItem, parent, name)
        return poseSeqItem

    def set_simulatorItem(self):
        self.set_item(BodyPlugin.AISTSimulatorItem(), self.worldItem, name="simulatorItem")
