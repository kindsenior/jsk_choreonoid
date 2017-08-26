#!/usr/bin/env python

from cnoid import Base, Body, BodyPlugin

def load_model(model_path, use_choreonoid=True):
    if use_choreonoid:
        # set world item
        worldItem = BodyPlugin.WorldItem()
        Base.RootItem.instance().addChildItem(worldItem)
        worldItem.enableCollisionDetection(True)
        # set robot item
        worldItem = Base.Item.find("World")
        robotItem = BodyPlugin.BodyItem()
        robotItem.load(str(model_path))
        worldItem.addChildItem(robotItem)
        Base.ItemTreeView.instance().checkItem(robotItem)
        # set robot
        robot = robotItem.body()
        return robot, robotItem, worldItem
    else:
        loader = Body.BodyLoader()
        robot = loader.load(str(model_path))
        return robot
