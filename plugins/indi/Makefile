PLUGIN_NAME=indi
FW_DIR=../..
BUILD_DIR=$(FW_DIR)/build/plugins

PLUGIN_SRC=$(PLUGIN_NAME).C indi_camera.C indi_client.C indi_client_manager.C indi_device.C indi_device_manager.C indi_focuser.C indi_filter_wheel.C indi_utils.C

PLUGIN_OBJECTS=$(patsubst %.C,%.o,$(PLUGIN_SRC))

CXX=g++
CXXFLAGS=-fPIC -rdynamic -std=c++11 -g -O0 -DBOOST_LOG_DYN_LINK
INCPATH=-I$(FW_DIR) -I/usr/include/libindi
LIBS=

.C.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

#-lindiclient
all: $(PLUGIN_OBJECTS)
	g++ $(PLUGIN_OBJECTS) -shared -Wl,-soname,libat$(PLUGIN_NAME).so $(CXXFLAGS) -lboost_serialization -lindi -lindiclient -o $(BUILD_DIR)/libat$(PLUGIN_NAME).so


clean:
	rm -rf *.o $(BUILD_DIR)/libat$(PLUGIN_NAME).so
