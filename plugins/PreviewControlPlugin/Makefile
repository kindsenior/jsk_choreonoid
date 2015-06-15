VAR=$(patsubst %.0,%, $(shell pkg-config --modversion choreonoid) )

CXXFLAGS += `pkg-config --cflags choreonoid-body-plugin`
# CXXFLAGS += `pkg-config --cflags eigen3`
CXXFLAGS += `pkg-config --cflags openhrp3.1`
CXXFLAGS += -I `rospack find jsk_choreonoid`/plugins/
CXXFLAGS += -fPIC

PLUGIN = libCnoidPreviewControlPlugin.so
SRC = PreviewControlPlugin.o PreviewController.o
INSTALLED_TARGET=`rospack find choreonoid`/lib/choreonoid-$(VAR)/$(PLUGIN)

all: $(INSTALLED_TARGET)

$(PLUGIN): $(SRC)
	# g++ -shared -o $(PLUGIN) $(SRC) `pkg-config --libs choreonoid-body-plugin` -lCnoidPoseSeqPlugin -lCnoidDynamicsPlugin -Wl,-rpath=`pkg-config --variable=plugindir choreonoid`
	g++ -shared -o $(PLUGIN) $(SRC)  `pkg-config --libs choreonoid-body-plugin` -lCnoidPoseSeqPlugin -lCnoidDynamicsPlugin -Wl,-rpath='$$ORIGIN' -Wl,-rpath=..'$$ORIGIN/..'

$(INSTALLED_TARGET): $(PLUGIN)
	# ln -sf `pwd`/$(PLUGIN) $(INSTALLED_TARGET)
# 	install -s $(PLUGIN) `pkg-config --variable=plugindir choreonoid`
	install -s $(PLUGIN) $(INSTALLED_TARGET)

clean:
	rm -f *.o *.so $(INSTALLED_TARGET)

