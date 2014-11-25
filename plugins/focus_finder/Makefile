PLUGIN_NAME=focus_finder
FW_DIR=../..
PLUGIN_DIR=$(FW_DIR)/plugins
BUILD_DIR=$(FW_DIR)/build/plugins

PLUGIN_SRC=$(PLUGIN_NAME).C fwhm.C fitgsl.C

PLUGIN_OBJECTS=$(patsubst %.C,%.o,$(PLUGIN_SRC))

CXX=g++
CXXFLAGS=-fPIC -rdynamic -std=c++11 -g -O0 -DBOOST_LOG_DYN_LINK
INCPATH=-I$(FW_DIR) -I/usr/include/libindi -I$(PLUGIN_DIR)
LIBS=

.C.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

#-lindiclient
all: $(PLUGIN_OBJECTS)
	g++ $(PLUGIN_OBJECTS) -shared -Wl,-soname,libat$(PLUGIN_NAME).so $(CXXFLAGS) -lboost_serialization -lindi -o $(BUILD_DIR)/libat$(PLUGIN_NAME).so


clean:
	rm -rf *.o $(BUILD_DIR)/libat$(PLUGIN_NAME).so