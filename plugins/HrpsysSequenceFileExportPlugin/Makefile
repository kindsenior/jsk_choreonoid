VAR=$(patsubst %.0,%, $(shell pkg-config --modversion choreonoid) )

CXXFLAGS += `pkg-config --cflags choreonoid-body-plugin`
# CXXFLAGS += `pkg-config --cflags eigen3`
# CXXFLAGS += -I `rospack find choreonoid`/build/choreonoid-1.4.1/src/Collision
CXXFLAGS += -I `rospack find jsk_choreonoid`/plugins/
CXXFLAGS += -fPIC

PLUGIN = libCnoidHrpsysSequenceFileExportPlugin.so
SRC = HrpsysSequenceFileExportPlugin.o
INSTALLED_TARGET=`rospack find choreonoid`/lib/choreonoid-$(VAR)/$(PLUGIN)

all: $(INSTALLED_TARGET)

$(PLUGIN): $(SRC)
	# g++ -shared -o $(PLUGIN) $(SRC) `pkg-config --libs choreonoid-body-plugin` -lCnoidPoseSeqPlugin -lCnoidDynamicsPlugin -Wl,-rpath=`pkg-config --variable=plugindir choreonoid`
	g++ -shared -o $(PLUGIN) $(SRC) `pkg-config --libs choreonoid-body-plugin` -lCnoidPoseSeqPlugin -lCnoidDynamicsPlugin -lCnoidUtilPlugin -Wl,-rpath='$$ORIGIN' -Wl,-rpath='$$ORIGIN/..'
	# g++ -shared -o $(PLUGIN) $(SRC) `pkg-config --libs choreonoid-body-plugin` -lCnoidPoseSeqPlugin -lCnoidDynamicsPlugin -lCnoidUtilPlugin -Wl,-rpath=. -Wl,-rpath=..

$(INSTALLED_TARGET): $(PLUGIN)
	# ln -sf `pwd`/$(PLUGIN) $(INSTALLED_TARGET)
#	install -s $(PLUGIN) `pkg-config --variable=plugindir choreonoid`
	install -s $(PLUGIN) $(INSTALLED_TARGET)

clean:
	rm -f *.o *.so $(INSTALLED_TARGET)