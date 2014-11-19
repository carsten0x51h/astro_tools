VPATH=.:ui:ui/gen
BASE_DIR = .
BUILD_DIR = $(BASE_DIR)/build
CXX = g++
INCPATH = -I/usr/share/qt4/mkspecs/linux-g++ -I. -I/usr/include/qt4/Qt -I/usr/include/qt4/QtCore -I/usr/include/qt4/QtGui -I/usr/include/qt4 -Iui -Iui/gen -I/usr/include/qwt-qt4 -I/home/schmica/Apps/boost_1_51_0 -I/usr/include/libindi
LFLAGS  = -std=c++0x -lgsl -lgslcblas -lm -lX11 -lpthread -lCCfits -lqwt-qt4 -L/usr/lib/i386-linux-gnu -lQtGui -lQtCore -g -O0

UI_HEADER = $(patsubst ui/%.hpp,ui_%.hpp,$(wildcard ui/*.hpp))
UI_MOCS = $(patsubst ui/%.hpp,moc_%.C,$(wildcard ui/*.hpp))
UI_SOURCE = $(wildcard ui/*.C) $(wildcard ui/gen/*.C)

ui_%.hpp: %.ui
	uic-qt4 $< -o ui/gen/$(@F)

moc_%.C: %.hpp
	moc-qt4 $< -o ui/gen/$(@F)

ui: $(UI_HEADER) $(UI_MOCS)


.SUFFIXES: .o .C

.C.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"


##################################################################################################################################

all: vcurve_gui_test focus_finder_gui focus_finder_test camera_test_gui camera_exposure_test focuser_test_gui focuser_move_test centroid_test hfd_test fwhm_test



fwhm_test:
	$(CXX) $(INCPATH) fwhm.C fitgsl.C fwhm_test.C ui/fwhm_panel.C ui/gen/moc_fwhm_panel.C -o $(BUILD_DIR)/fwhm_test $(LFLAGS)

hfd_test:
	$(CXX) $(INCPATH) hfd_test.C ui/hfd_panel.C ui/gen/moc_hfd_panel.C -o $(BUILD_DIR)/hfd_test $(LFLAGS)

centroid_test:
	$(CXX) $(INCPATH) centroid_test.C -o $(BUILD_DIR)/centroid_test $(LFLAGS)

#
# Focuser
#
focuser_move_test:
	$(CXX) $(INCPATH) -I/usr/include/libindi focuser_move_test.C -o $(BUILD_DIR)/focuser_move_test $(LFLAGS) -lindiclient -lindi -lz -ldl

focuser_test_gui:
	$(CXX) $(INCPATH) -I/usr/include/libindi focuser_test_gui.C ui/focuser_cntl_panel.C ui/gen/moc_focuser_cntl_panel.C -o $(BUILD_DIR)/focuser_test_gui $(LFLAGS) -lindiclient -lindi -lz -ldl


#
# Camera
#
camera_exposure_test:
	$(CXX) $(INCPATH) -I/usr/include/libindi camera_exposure_test.C -o $(BUILD_DIR)/camera_exposure_test $(LFLAGS) -lindiclient -lindi -lz -ldl

camera_test_gui:
	$(CXX) $(INCPATH) -I/usr/include/libindi camera_test_gui.C ui/camera_cntl_panel.C ui/gen/moc_camera_cntl_panel.C -o $(BUILD_DIR)/camera_test_gui $(LFLAGS) -lindiclient -lindi -lz -ldl


#
# VCurve
#
vcurve_gui_test:
	$(CXX) $(INCPATH) -I/usr/include/libindi vcurve_gui_test.C ui/vcurve_viewer_panel.C ui/gen/moc_vcurve_viewer_panel.C -o $(BUILD_DIR)/vcurve_gui_test $(LFLAGS) -lindiclient -lindi -lz -ldl

