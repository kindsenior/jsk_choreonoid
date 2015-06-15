VAR=$(patsubst %.0,%, $(shell pkg-config --modversion choreonoid) )

CXXFLAGS += `pkg-config --cflags choreonoid-body-plugin`
CXXFLAGS += -fPIC

TARGET = UtilPlugin
PLUGIN = libCnoid$(TARGET).so
SRC = $(TARGET).o
HEADER = $(TARGET).h
INSTALLED_TARGET=`rospack find choreonoid`/lib/choreonoid-$(VAR)/$(PLUGIN)
HEADER_DIR = $(HOME)/ros/$(ROS_DISTRO)/devel/include/choreonoid-$(VAR)/cnoid/src/$(TARGET)

all: $(INSTALLED_TARGET)

$(PLUGIN): $(SRC)
	g++ -shared -o $(PLUGIN) $(SRC) `pkg-config --libs choreonoid-body-plugin` -lCnoidPoseSeqPlugin -Wl,-rpath='$$ORIGIN' -Wl,-rpath=..'$$ORIGIN/..'	


$(INSTALLED_TARGET): $(PLUGIN)
	mkdir -p `rospack find choreonoid`/lib/choreonoid-$(VAR)
	install -s $(PLUGIN) $(INSTALLED_TARGET)
	install -s $(PLUGIN) `pkg-config --variable=plugindir choreonoid`

clean:
	rm -f *.o *.so $(INSTALLED_TARGET)
