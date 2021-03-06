VPATH=.:ui:ui/gen
BASE_DIR = .
BUILD_DIR = $(BASE_DIR)/build
CXX = g++
CXXFLAGS = -std=c++11 -g -O0 -DBOOST_LOG_DYN_LINK -fPIC

INCPATH_COMMON = -I/usr/include/libindi -I/usr/include/python2.7
INCPATH_GUI = -I/usr/share/qt4/mkspecs/linux-g++ -I. -I/usr/include/qt4 -I/usr/include/qt4/Qt -I/usr/include/qt4/QtCore -I/usr/include/qt4/QtGui -Iui -Iui/gen -I/usr/include/qwt-qt4
INCPATH_ALL = $(INCPATH_COMMON) $(INCPATH_GUI)

LFLAGS_COMMON  = -lCCfits -lcfitsio -lgsl -lgslcblas -lm -lpthread -lX11 -L/usr/lib/i386-linux-gnu -lpython2.7 -rdynamic
LFLAGS_CONSOLE = $(LFLAGS_COMMON)
LFLAGS_GUI = $(LFLAGS_COMMON) -lqwt-qt4 -lQtGui -lQtCore

.C.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH_ALL) -o "$@" "$<"

#
# Qt stuff
#
UI_HEADER = $(patsubst ui/%.hpp,ui_%.hpp,$(wildcard ui/*.hpp))
UI_MOCS = $(patsubst ui/%.hpp,ui/gen/moc_%.C,$(wildcard ui/*.hpp))
UI_SOURCE = $(wildcard ui/*.C) $(wildcard ui/gen/*.C)

ui_%.hpp: %.ui
	uic-qt4 $< -o ui/gen/$(@F)

ui/gen/moc_%.C: %.hpp
	moc-qt4 $< -o ui/gen/$(@F)
	$(CXX) -c $(CXXFLAGS) $(INCPATH_ALL) -o $(patsubst ui/gen/%.C,ui/gen/%.o,ui/gen/$(@F)) ui/gen/$(@F)


#
# Astro tools
#
ASTRO_TOOLS_COMMON_FILES=io_util.C util.C at_plugin_mgr.C astro_tools_app.C tmpl_inst.C at_task.C

ASTRO_TOOLS_CONSOLE_FILES=$(ASTRO_TOOLS_COMMON_FILES) astro_tools_console_app.C
ASTRO_TOOLS_GUI_FILES=$(ASTRO_TOOLS_COMMON_FILES) $(UI_SOURCE) astro_tools_gui_app.C

ASTRO_TOOLS_CONSOLE_OBJECTS=$(patsubst %.C,%.o,$(ASTRO_TOOLS_CONSOLE_FILES))
ASTRO_TOOLS_GUI_OBJECTS=$(patsubst %.C,%.o,$(ASTRO_TOOLS_GUI_FILES))

ASTRO_TOOLS_COMMON_LIBS=-Wl,-whole-archive -lindiclient -Wl,-no-whole-archive -lindi -lz -ldl -lboost_regex -lboost_system -lboost_filesystem -lboost_log -lboost_thread -lboost_log_setup -lpthread -lboost_program_options
ASTRO_TOOLS_CONSOLE_LIBS=$(ASTRO_TOOLS_COMMON_LIBS)
ASTRO_TOOLS_GUI_LIBS=$(ASTRO_TOOLS_COMMON_LIBS)


.SUFFIXES: .o .C

astro_tools: prepare_dirs astro_tools_console astro_tools_gui

prepare_dirs:
	mkdir -p build
	mkdir -p build/plugins

astro_tools_console: $(ASTRO_TOOLS_CONSOLE_OBJECTS)
	$(CXX) $(CXXFLAGS) $(ASTRO_TOOLS_CONSOLE_OBJECTS) -o $(BUILD_DIR)/astro_tools $(LFLAGS_CONSOLE) $(ASTRO_TOOLS_CONSOLE_LIBS) /usr/lib/libindiclient.a

ui: $(UI_HEADER) $(UI_MOCS)

astro_tools_gui_int: $(ASTRO_TOOLS_GUI_OBJECTS)
	$(CXX) $(CXXFLAGS) $(ASTRO_TOOLS_GUI_OBJECTS) -o $(BUILD_DIR)/astro_tools_gui $(LFLAGS_GUI) $(ASTRO_TOOLS_GUI_LIBS)

#
# This is a hack... due to a weird moc dependency. Certain C files are
# generated _after_ the variables are filled and hence the new files are
# missing. Probably there is a mor elegant solution but somehow gmake
# sucks at the moment....
# Maybe see here for a solution: http://makepp.sourceforge.net/1.19/makepp_cookbook.html#Using%20Qt%27s%20moc%20preprocessor
astro_tools_gui:
	make -f Makefile ui
	make -f Makefile astro_tools_gui_int

clean:
	rm -rf *.o  ui/*.o ui/gen/* $(BUILD_DIR)/astro_tools $(BUILD_DIR)/astro_tools_gui
	-find ./ -name '*~' | xargs rm


#
# swig
# -lindimain -lindidriver -lindi
#	
py_if:
	swig -c++ -python -module libastrotoolspythonwrap astrotools.i
	g++ -c -fPIC -I/usr/include/python2.7 -I/usr/include/libindi -I . astrotools_wrap.cxx
	g++ astrotools_wrap.o -shared -o _libastrotoolspythonwrap.so

#Import module: import _libastrotoolspythonwrap
py:
	python2.7

#py_test:
#	g++ -rdynamic -I/usr/include/python2.7 io_util.C python_test.C /usr/lib/libindiclient.a -lindi -lz -lpthread -lindiclient -lpython2.7 -lCCfits -lX11


# shortcut - remove later...
sim_indiserver:
	indiserver -m 500 -vv /usr/bin/indi_simulator_ccd /usr/bin/indi_simulator_focus /usr/bin/indi_simulator_telescope /usr/bin/indi_simulator_wheel

sim_indiserver_dbg:
	indiserver -m 500 -vvv /usr/bin/indi_simulator_ccd /usr/bin/indi_simulator_focus /usr/bin/indi_simulator_telescope /usr/bin/indi_simulator_wheel

#OLD path: /home/devnull/workspace/atik_ccd_src/src
atik_indiserver:
	indiserver -m 500 -v /usr/bin/indi_moonlite_focus /usr/bin/indi_atik_ccd /usr/bin/indi_atik_wheel

eq6_indiserver:
	indiserver -m 500 -v /usr/bin/indi_eqmod_telescope

#/usr/bin/indi_atik_ccd /usr/bin/indi_atik_wheel
full_indiserver:
	indiserver -m 500 -v /usr/bin/indi_moonlite_focus  /usr/bin/indi_eqmod_telescope /usr/bin/indi_joystick /usr/bin/indi_atik_ccd /usr/bin/indi_atik_wheel

sim_find_focus:
	gdb --args ./build/astro_tools focus_find -vv --filter_device="Filter Simulator" --camera_device="CCD Simulator" --exposure_time=1 --focuser_device="Focuser Simulator" --star_select=264,574 --focus_mode=manual --pixel_size=5.4x5.4 --focal_distance=1000
# --seq_record_dir=/home/devnull/workspace/astro_tools/plugins/focus_finder/records2

atik_find_focus:
	gdb --args ./build/astro_tools focus_find --camera_device="Atik 383L+ CCD" --filter_device="ATIK Wheel" --focuser_device="MoonLite" --focuser_device_port=/dev/ttyUSB1 --exposure_time=1 --focus_mode=manual --star_select=display --pixel_size=5.4x5.4 --focal_distance=1000
#--star_select=264,574
#./build/astro_tools focus_find --camera_device="Atik 383L+ CCD" --filter_device="ATIK Wheel" --focuser_device="MoonLite" --focuser_device_port=/dev/ttyUSB2 --exposure_time=3 --focus_mode=manual --star_select=display --pixel_size=5.4x5.4 --focal_distance=1000 -vvv --seq_record_dir=/home/devnull/workspace/astro_tools/plugins/focus_finder/records



#	./build/astro_tools focus_find -v --camera_device="ATIK CCD Atik 383L+" --focuser_device="MoonLite" --focuser_device_port=/dev/ttyUSB1 --exposure_time=1 --star_select=2390,1267 --num_steps_to_determine_direction=300 --steps_to_reach_focus=300 --extrema_fitness_boundary=16 --rough_focus_granularity_steps=50 --fine_focus_granularity_steps=10 --fine_search_range_steps=150 --outer_hfd_radius_px=15 --fine_focus_record_num_curves=1

atik_take_picture:
	./build/astro_tools take_picture -v --device_name="ATIK CCD Atik 383L+" --exposure_time=1



# window_size"].as<unsigned int>();
# num_steps_to_determine_direction"].as<unsigned int>();
# steps_to_reach_focus"].as<unsigned int>();
# extrema_fitness_boundary"].as<unsigned int>();
# outer_hfd_radius_px"].as<unsigned int>();
# rough_focus_max_iter_cnt"].as<unsigned int>();
# etryCnt = cmdLineMap["take_picture_fit_gauss_curve_max_retry_cnt"].as<unsigned int>();
# eMap["debug_show_take_picture_image"].as<unsigned int>();
# cmdLineMap["rough_focus_search_range_perc"].as<unsigned int>();
# cmdLineMap["rough_focus_record_num_curves"].as<unsigned int>();
#  cmdLineMap["rough_focus_granularity_steps"].as<unsigned int>();
# mdLineMap["fine_focus_record_num_curves"].as<unsigned int>();
# cmdLineMap["fine_focus_granularity_steps"].as<unsigned int>();
# neMap["fine_search_range_steps"].as<unsigned int>();
# ["vcurve_fit_eps_abs"].as<unsigned int>();
# ["vcurve_fit_eps_rel"].as<unsigned int>();
