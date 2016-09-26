class SimTimeLoop(object):
    """
    This class is used when you wan to use loop program in choreonoid simulation mode.
    This class execute hooked function every time when a sigKinematicStateChanged signal is emmited.
    (sigKinematicStateChanged is a Qt signal emmited every simulation loop in choreonoid.)

    def myFunction(x): print x
    def myFunction2(): print "good bye!"

    # (select JAXON_RED)
    # import cnoid
    # robotItem = cnoid.Base.ItemTreeView.instance().selectedItem(cnoid.BodyPlugin.BodyItem)
    import jsk_choreonoid.CnoidPyUtil as CnoidPyUtil
    stl=CnoidPyUtil.SimTimeLoop(robotItem)
    stl.maxCount=10 # loop 10 times
    stl.addHook(myFunction, ["hellow world!"])
    stl.addHook(myFunction2)
    stl.start() # message is printed on "Message" menue.
    # when you want to stop loop:
    stl.finish()
    """
    def __init__(self,BodyItem):
        """
        robot: BodyItem.Body
        """
        self.hookList=[]
        self.argsList=[]
        self.kwdArgsList=[]
        self.maxCount=-1
        self.BodyItem=BodyItem
        self.sig=None
        self.connection=None
        self.counter=0
    def addHook(self,func,args=[],keyward={}):
        self.hookList.append(func)
        self.argsList.append(args)
        self.kwdArgsList.append(keyward)
    def execute(self):
        self.counter+=1
        if (not self.maxCount == -1) and self.counter >= self.maxCount:
            self.finish()
        [func(*args,**keyward) for (func, args, keyward) in zip(self.hookList, self.argsList, self.kwdArgsList)]
    def start(self):
        self.sig=self.BodyItem.sigKinematicStateChanged()
        self.connection=self.sig.connect(self.execute)
        self.counter=0
    def finish(self):
        self.connection.disconnect()
    def setMaxCount(self,num):
        self.maxCount=num
