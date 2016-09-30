class SimTimeLoop(object):
    """
    This class is used when you wan to use loop program in choreonoid simulation mode.
    This class execute hooked function every time when a sigKinematicStateChanged signal is emmited.
    (sigKinematicStateChanged is a Qt signal emmited every simulation loop in choreonoid.)

    ```
    def myFunction(x,y="?"): print x,y
    import cnoid
    robotItem = cnoid.Base.Item.find("JAXON_RED")
    import jsk_choreonoid.CnoidPyUtil as CnoidPyUtil
    ```

    # How-To 1
    set function and args to CnoidPyUtil.SimTimeLoop.

    ```
    stl=CnoidPyUtil.SimTimeLoop(robotItem,target=myFunction, args=("hellow world",), kwargs={"y":"!"})
    stl.setMaxCount(10) # loop 10 times
    stl.start() # message is printed on "Message" menue.
    # when you want to stop loop:
    stl.finish()
    ```


    # How-To 2
    inherit the class, and override the run method.

    ```
    class myClass(CnoidPyUtil.SimTimeLoop):
        def __init__(self,*args,**kwargs): super(myClass, self).__init__(*args,**kwargs);
        def run(self): print "ok!"
    stl2=myClass(robotItem, maxCount=5)
    stl.finish()
    ```

    """
    def __init__(self, BodyItem, target=None, args=(), kwargs={}, debug=False, name="", maxCount=-1):
        self.__maxCount=maxCount
        self.__BodyItem=BodyItem
        self.__lock=False
        self.__target = target
        self.__args = args
        self.__kwargs = kwargs
        self.__debug=debug
        self.__name=name

    def run (self):
        try:
            if self.__target:
                self.__target(*self.__args, **self.__kwargs)
        except:
            self.finish()
            print "cannot execute target. target:{}, args:{}, kwargs:{}".format(self.__target, self.__args, self.__kwargs)
            raise

    def __execute(self):
        if self.__lock:
            if self.__debug:
                print "time over!"
            return None
        self.__lock=True
        try:
            self.run()
        except:
            self.finish()
            print "error in self.run(). (connection was successfully disconnected)"
            raise
        self.__counter+=1
        if self.__maxCount > 0 and self.__counter >= self.__maxCount:
            self.finish()
        self.__lock=False

    def start(self):
        self.sig=self.__BodyItem.sigKinematicStateChanged()
        self.connection=self.sig.connect(self.__execute)
        self.__counter=0

    def finish(self):
        self.connection.disconnect()

    def setMaxCount(self,num):
        self.__maxCount=num

    def name(self):
        return self.__name
