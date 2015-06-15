VAR=$(patsubst %.0,%, $(shell pkg-config --modversion choreonoid) )

CXXFLAGS += `pkg-config --cflags choreonoid-body-plugin`
# CXXFLAGS += `pkg-config --cflags eigen3`
# CXXFLAGS += -I `rospack find choreonoid`/build/choreonoid-1.4.1/src/Collision
# CXXFLAGS += -I `rospack find choreonoid`/build/choreonoid-$(VAR)/src/Collision
CXXFLAGS += -fPIC

TARGET = DynamicsPlugin
PLUGIN = libCnoid$(TARGET).so
SRC = $(TARGET).o Jacobian.o
HEADER = $(TARGET).h Jacobian.h
INSTALLED_TARGET=`rospack find choreonoid`/lib/choreonoid-$(VAR)/$(PLUGIN)
HEADER_DIR = $(HOME)/ros/$(ROS_DISTRO)/devel/include/choreonoid-$(VAR)/cnoid/src/$(TARGET)

all: $(INSTALLED_TARGET)

$(PLUGIN): $(SRC)
	# g++ -shared -o $(PLUGIN) $(SRC) `pkg-config --libs choreonoid-body-plugin` -lCnoidPoseSeqPlugin
	g++ -shared -o $(PLUGIN) $(SRC) `pkg-config --libs choreonoid-body-plugin` -lCnoidPoseSeqPlugin -lCnoidUtilPlugin -Wl,-rpath='$$ORIGIN' -Wl,-rpath=..'$$ORIGIN/..'


$(INSTALLED_TARGET): $(PLUGIN)
	# ln -sf `pwd`/$(PLUGIN) $(INSTALLED_TARGET)
	mkdir -p `rospack find choreonoid`/lib/choreonoid-$(VAR)
	install -s $(PLUGIN) $(INSTALLED_TARGET)
	# mkdir -p $(HEADER_DIR)
	# cp $(HEADER) /usr/local/include/choreonoid-$(VAR)/cnoid/src/$(TARGET)
	# sudo cp $(HEADER) $(HEADER_DIR)
	install -s $(PLUGIN) `pkg-config --variable=plugindir choreonoid`

clean:
	rm -f *.o *.so $(INSTALLED_TARGET)
