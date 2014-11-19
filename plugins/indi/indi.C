/*****************************************************************************
 *
 *  AstroTools
 *
 *  Copyright(C) 2015 Carsten Schmitt <c.schmitt51h@gmail.com>
 *
 *  This program is free software ; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation ; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY ; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program ; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 ****************************************************************************/

#include "indi.hpp"
#include "at_logging.hpp"
#include "astro_tools_app.hpp"

#include "indi_camera.hpp"
#include "indi_focuser.hpp"
#include "indi_filter_wheel.hpp"

#include "indi_utils.hpp"
#include "util.hpp" // TODO: at_util.hpp?! and AT namespace?!

#include "at_validator.hpp"
#include "indi_validator.hpp"

#include <boost/algorithm/string.hpp>

using namespace std;

namespace AT {
  DEF_Exception(IndiPlugin);

  static const string sDegreeCelsius = "°C"; /* Note: As an option we can use \u2103 = one degree celsius symbol instead */

  template <typename DeviceT, typename ActionT>
  class IndiDeviceActionT {
  public:
    static void performAction(void) {
      const po::variables_map & cmdLineMap = CommonAstroToolsAppT::getCmdLineOptionsMap();
      
      // Check cmdline args - depends
      AT_ASSERT(IndiPlugin, cmdLineMap.count("indi_server") > 0, "Expecting indi_server option being set.");
      AT_ASSERT(IndiPlugin, cmdLineMap.count("device_name") > 0, "Expecting device_name option being set.");
      
      const HostnameAndPortT & hostnameAndPort = cmdLineMap["indi_server"].as<HostnameAndPortT>();
      const string & deviceName = cmdLineMap["device_name"].as<string>();
      
      LOG(info) << "Indi server: " << hostnameAndPort << ", Device name: " << deviceName << endl;
      
      IndiClientT indiClient(hostnameAndPort.getHostname(), hostnameAndPort.getPort(), true);
      
      if (indiClient.isConnected()) {

	DeviceT * device = static_cast<DeviceT*>(indiClient.getDevice(deviceName.c_str(), DeviceT::getType()));
	
	if (! device) {
	  // TODO: Print device name?!
	  stringstream ss;
	  ss << "Invalid device handle for device name '" << deviceName << "' returned.";
	  throw IndiPluginExceptionT(ss.str().c_str());
	}

	// TODO: What if device is already connected?!
	bool forceReConnect = ActionT::setupAction(device, cmdLineMap); // This takes place before connectiong to device..

	if (forceReConnect && device->isConnected()) {
	  LOG(info) << "Setup routine forces device reconnect... Disconnecting device..." << endl;
	  device->disconnect();
	}

	// TODO: Remember if device was connected, ... disconnect if so?! Maybe regulate this behaviour by another switch? "keep state" ?!
	if (! device->isConnected()) {
	  LOG(info) << "Connecting " << DeviceTypeT::asStr(device->getType()) << "..." << endl;
	  device->connect(); // NOTE: Throws on failure
	}
	
	if (! device->isConnected()) {
	  // TODO: Print device name?!
	  stringstream ss;
	  ss << "Could not connect to '" << DeviceTypeT::asStr(device->getType());
	  throw IndiPluginExceptionT(ss.str().c_str());
	}
	
	// Call static member of specialization
	ActionT::performAction(device, cmdLineMap);
	
      } else {
	stringstream ss;
	ss << "Could not connect to INDI client: '" << indiClient << "'." << endl;
	throw IndiPluginExceptionT(ss.str().c_str());
      }

      // NOTE: We only disconnect the camera if it was disconnected previously.
      // if (! wasCameraConnected) {
      // 	LOG(info) << "Disconnecting camera..." << flush;	
      // 	inDevice->disconnect();
      // 	LOG(info) << "DONE." << flush;	
      // }
    }
  }; // end class


  /**
   * Camera
   *
   */
  class CameraTakePictureT : public IndiDeviceActionT<IndiCameraT, CameraTakePictureT> {
  public:
    static bool setupAction(IndiCameraT * inDevice, const po::variables_map & cmdLineMap) { return false; }
    static void performAction(IndiCameraT * inDevice, const po::variables_map & cmdLineMap) {
      LOG(info) << "Begin taking picture..." << endl;
      AT_ASSERT(IndiPlugin, cmdLineMap.count("exposure_time") > 0, "Expecting exposure_time option being set.");
      AT_ASSERT(IndiPlugin, cmdLineMap.count("frame_type") > 0, "Expecting frame_type option being set.");
      AT_ASSERT(IndiPlugin, cmdLineMap.count("binning") > 0, "Expecting binning option being set.");
      AT_ASSERT(IndiPlugin, cmdLineMap.count("output") > 0, "Expecting output option being set.");

      const float exposureTime = cmdLineMap["exposure_time"].as<float>();
      const FrameTypeT::TypeE & frameType = cmdLineMap["frame_type"].as<FrameTypeT::TypeE>();
      const BinningT & binning = cmdLineMap["binning"].as<BinningT>();
      const string & outputFileName = cmdLineMap["output"].as<string>();
      const bool displayPicture = (cmdLineMap.count("display_picture") > 0);

      LOG(info) << ", Exposure time [s]: " << exposureTime
		<< ", Frame type: " << FrameTypeT::asStr(frameType)
		<< ", Binning: " << binning << ", Output: " << outputFileName
		<< ", display picture: " << displayPicture << endl;

      // Get max. resolution
      DimensionT maxRes(inDevice->getMaxResolution().get<0>(), inDevice->getMaxResolution().get<1>());
      FrameT frameSize;
      bool hasIntersection = false;
      
      LOG(debug) << "Max camera resolution: " << maxRes << endl;
      
      if (cmdLineMap.count("frame_size")) {
	hasIntersection = intersect(maxRes, cmdLineMap["frame_size"].as<FrameT>(), & frameSize);
	LOG(debug) << "Frame size (user specified): " << cmdLineMap["frame_size"].as<FrameT>() << endl;
      }
      
      if (! cmdLineMap.count("frame_size") || ! hasIntersection) {
	frameSize = FrameT(0, 0, maxRes.get<0>(), maxRes.get<1>());
	LOG(debug) << "Frame size (default=max res): " << frameSize << endl;
      }
      
      LOG(debug) << "HasIntersection: " << hasIntersection << ", resulting frameSize: " << frameSize << endl;
      
      // Finally, we have all to take a picture.
      CImg<float> img;
      inDevice->takePicture(& img, exposureTime, frameSize, frameType, binning, false /* not compressed - TODO?! */);
      
      // Save picture
      img.save(outputFileName.c_str());
      
      // Display image, if user specified
      if (displayPicture) {
	CImgDisplay disp1(img, "img");
	while(! disp1.is_closed()) { CImgDisplay::wait(disp1); }
      }
    }
  };

  class CameraCoolerEnableT : public IndiDeviceActionT<IndiCameraT, CameraCoolerEnableT> {
  public:
    static bool setupAction(IndiCameraT * inDevice, const po::variables_map & cmdLineMap) { return false; }
    static void performAction(IndiCameraT * inDevice, const po::variables_map & cmdLineMap) {
      AT_ASSERT(IndiPlugin, cmdLineMap.count("timeout") > 0, "Expecting timeout option being set.");
      AT_ASSERT(IndiPlugin, cmdLineMap.count("temperature") > 0, "Expecting temperature option being set.");
      const float timeoutMs = 1000.0 * cmdLineMap["timeout"].as<float>(); // Conversion to ms
      float temperature = cmdLineMap["temperature"].as<float>();

      LOG(info) << "CameraCoolerEnableT - timeoutMs: " << timeoutMs << " [ms]"
		<< " cooler...temperature: " << temperature << sDegreeCelsius << endl;

      // Set temperature
      // TODO: Does non-blocking work? Which one blocks until temp. has been reached?! --> TEST with real camera!
      if (! inDevice->isCoolerEnabled())
      	inDevice->setCoolerEnabled(true, 0 /* do not block */);

      inDevice->setTemperature(temperature, 0 /* do not block */);

      WAIT_MAX_FOR_PRINT(inDevice->isTemperatureReached(), timeoutMs, CommonAstroToolsAppT::isInQuietMode(),
			 "[Temperature: " << inDevice->getTemperature() << "°C]...", "Hit timeout while waiting for cooler.");
    }
  };

  class CameraCoolerDisableT : public IndiDeviceActionT<IndiCameraT, CameraCoolerDisableT> {
  public:
    static bool setupAction(IndiCameraT * inDevice, const po::variables_map & cmdLineMap) { return false; }
    static void performAction(IndiCameraT * inDevice, const po::variables_map & cmdLineMap) {
      AT_ASSERT(IndiPlugin, cmdLineMap.count("timeout") > 0, "Expecting timeout option being set.");
      const float timeoutMs = 1000.0 * cmdLineMap["timeout"].as<float>(); // Conversion to ms

      LOG(info) << "CameraCoolerDisableT - timeoutMs: " << timeoutMs << " [ms]" << endl;

      if (inDevice->isCoolerEnabled())
      	inDevice->setCoolerEnabled(false, 0 /* do not block */);

      WAIT_MAX_FOR_PRINT(inDevice->isTemperatureReached(), timeoutMs, CommonAstroToolsAppT::isInQuietMode(),
			 "[Temperature: " << inDevice->getTemperature() << sDegreeCelsius << "]...", "Hit timeout while waiting for cooler.");
    }
  };

  class CameraCoolerInfoT : public IndiDeviceActionT<IndiCameraT, CameraCoolerInfoT> {
  public:
    static bool setupAction(IndiCameraT * inDevice, const po::variables_map & cmdLineMap) { return false; }
    static void performAction(IndiCameraT * inDevice, const po::variables_map & cmdLineMap) {
      cout << "Cooler is: " << (inDevice->isCoolerEnabled() ? "enabled" : "disabled")
	   << ", state: " << (inDevice->isTemperatureReached() ? "ready" : "busy")
	   << ", temperature: " << inDevice->getTemperature() << " " << sDegreeCelsius << endl;
    }
  };


  /**
   * FilterWheel
   *
   */
  class FilterWheelFilterSelectT : public IndiDeviceActionT<IndiFilterWheelT, FilterWheelFilterSelectT> {
  public:
    static bool setupAction(IndiFilterWheelT * inDevice, const po::variables_map & cmdLineMap) { return false; }
    static void performAction(IndiFilterWheelT * inDevice, const po::variables_map & cmdLineMap) {
      // TODO: Check if device is still connected... somehow driver does not realize if USB was plugged off or power was disabled...
      //       --> We may move this directly into the filter-wheel device class...
      AT_ASSERT(IndiPlugin, cmdLineMap.count("timeout") > 0, "Expecting timeout option being set.");
      AT_ASSERT(IndiPlugin, cmdLineMap.count("filter") > 0, "Expecting filter option being set.");
      
      const float timeoutMs = 1000.0 * cmdLineMap["timeout"].as<float>(); // Conversion to ms
      const string & filter = cmdLineMap["filter"].as<string>();

      LOG(info) << "FilterWheelFilterSelectT - timeoutMs: " << timeoutMs << " [ms]"
		<< ", filter: " << filter << endl;

      size_t pos;

      try {
	pos = std::stoi(filter);
      } catch(...) {
	// TODO: Ok, we try to resolve the pos by name...
	pos = 0; // TODO...
      }

      inDevice->setPos(pos, 0 /* do not block */);

      WAIT_MAX_FOR_PRINT(!inDevice->isMovementInProgess(), timeoutMs, CommonAstroToolsAppT::isInQuietMode(),
			 "[Filter pos: " << inDevice->getPos() << "]...", "Hit timeout while waiting for filter wheel.");
    }
  };

  class FilterWheelFilterInfoT : public IndiDeviceActionT<IndiFilterWheelT, FilterWheelFilterInfoT> {
  public:
    static bool setupAction(IndiFilterWheelT * inDevice, const po::variables_map & cmdLineMap) { return false; }
    static void performAction(IndiFilterWheelT * inDevice, const po::variables_map & cmdLineMap) {
      // TODO: Check if device is still connected... somehow driver does not realize if USB was plugged off or power was disabled...
      //       --> We may move this directly into the filter-wheel device class...
      cout << "FilterWheel is: " << (inDevice->isMovementInProgess() ? "busy" : "ready")
	   << ", position: " << inDevice->getPos() << "/" << inDevice->getNumSlots() << endl;
    }
  };


  /**
   * Focus
   *
   */
  class FocusMoveByT : public IndiDeviceActionT<IndiFocuserT, FocusMoveByT> {
  public:
    static bool setupAction(IndiFocuserT * inDevice, const po::variables_map & cmdLineMap) {
      AT_ASSERT(IndiPlugin, cmdLineMap.count("timeout") > 0, "Expecting timeout option being set.");
      AT_ASSERT(IndiPlugin, cmdLineMap.count("device_port") > 0, "Expecting device_port option being set.");
      string devicePort = cmdLineMap["device_port"].as<string>();
      const float timeoutMs = 1000.0 * cmdLineMap["timeout"].as<float>(); // Conversion to ms
      LOG(info) << "Setting focuser device_port to: '" << devicePort << "'..." << endl;
      inDevice->setDevicePort(devicePort, timeoutMs);
      return true; // Needs reconnect
    }
    static void performAction(IndiFocuserT * inDevice, const po::variables_map & cmdLineMap) {
      AT_ASSERT(IndiPlugin, cmdLineMap.count("steps") > 0, "Expecting steps option being set.");
      
      int steps = cmdLineMap["steps"].as<int>();

      LOG(info) << "FocusMoveByT - steps: " << steps << endl;

      FocusDirectionT::TypeE direction = (steps < 0 ? FocusDirectionT::INWARDS : FocusDirectionT::OUTWARDS);
      inDevice->moveFocusBy(abs(steps), direction, 0);
      unsigned int estimatedDelay = IndiFocuserT::estimateMaxFocuserMovementDelayMs(abs(steps));

      WAIT_MAX_FOR_PRINT(!inDevice->isMovementInProgess(), estimatedDelay, CommonAstroToolsAppT::isInQuietMode(),
			 "[Focuser pos: " << inDevice->getAbsPos() << "]...", "Hit timeout while waiting for focuser.");
    }
  };

  class FocusMoveToT : public IndiDeviceActionT<IndiFocuserT, FocusMoveToT> {
  public:
    static bool setupAction(IndiFocuserT * inDevice, const po::variables_map & cmdLineMap) {
      AT_ASSERT(IndiPlugin, cmdLineMap.count("timeout") > 0, "Expecting timeout option being set.");
      AT_ASSERT(IndiPlugin, cmdLineMap.count("device_port") > 0, "Expecting device_port option being set.");
      string devicePort = cmdLineMap["device_port"].as<string>();
      const float timeoutMs = 1000.0 * cmdLineMap["timeout"].as<float>(); // Conversion to ms
      LOG(info) << "Setting focuser device_port to: '" << devicePort << "'..." << endl;
      inDevice->setDevicePort(devicePort, timeoutMs);
      return true; // Needs reconnect
    }
    static void performAction(IndiFocuserT * inDevice, const po::variables_map & cmdLineMap) {
      AT_ASSERT(IndiPlugin, cmdLineMap.count("position") > 0, "Expecting position option being set.");
      
      unsigned int position = cmdLineMap["position"].as<unsigned int>();

      LOG(info) << "FocusMoveToT - position: " << position << endl;

      inDevice->setAbsPos(position, 0);

      unsigned int estimatedDelay = IndiFocuserT::estimateMaxFocuserMovementDelayMs(abs(position - inDevice->getAbsPos()));

      WAIT_MAX_FOR_PRINT(!inDevice->isMovementInProgess(), estimatedDelay, CommonAstroToolsAppT::isInQuietMode(),
			 "[Focuser pos: " << inDevice->getAbsPos() << "]...", "Hit timeout while waiting for focuser.");
    }
  };

  class FocusResetT : public IndiDeviceActionT<IndiFocuserT, FocusResetT> {
  public:
    static bool setupAction(IndiFocuserT * inDevice, const po::variables_map & cmdLineMap) {
      AT_ASSERT(IndiPlugin, cmdLineMap.count("timeout") > 0, "Expecting timeout option being set.");
      AT_ASSERT(IndiPlugin, cmdLineMap.count("device_port") > 0, "Expecting device_port option being set.");
      string devicePort = cmdLineMap["device_port"].as<string>();
      const float timeoutMs = 1000.0 * cmdLineMap["timeout"].as<float>(); // Conversion to ms
      LOG(info) << "Setting focuser device_port to: '" << devicePort << "'..." << endl;
      inDevice->setDevicePort(devicePort, timeoutMs);
      return true; // Needs reconnect
    }
    static void performAction(IndiFocuserT * inDevice, const po::variables_map & cmdLineMap) {
      AT_ASSERT(IndiPlugin, cmdLineMap.count("timeout") > 0, "Expecting timeout option being set.");
      const float timeoutMs = 1000.0 * cmdLineMap["timeout"].as<float>(); // Conversion to ms
      inDevice->resetPosition(timeoutMs);
    }
  };

  class FocusInfoT : public IndiDeviceActionT<IndiFocuserT, FocusInfoT> {
  public:
    static bool setupAction(IndiFocuserT * inDevice, const po::variables_map & cmdLineMap) {
      AT_ASSERT(IndiPlugin, cmdLineMap.count("timeout") > 0, "Expecting timeout option being set.");
      AT_ASSERT(IndiPlugin, cmdLineMap.count("device_port") > 0, "Expecting device_port option being set.");
      string devicePort = cmdLineMap["device_port"].as<string>();
      const float timeoutMs = 1000.0 * cmdLineMap["timeout"].as<float>(); // Conversion to ms
      LOG(info) << "Setting focuser device_port to: '" << devicePort << "'..." << endl;
      inDevice->setDevicePort(devicePort, timeoutMs);
      return true; // Needs reconnect
    }
    static void performAction(IndiFocuserT * inDevice, const po::variables_map & cmdLineMap) {
      // TODO: Get abs, temperature and port only if they exist... otherwise print "n.a." --> try catch...
      cout << "Position: " << inDevice->getAbsPos()
	   << ", temperature: " << inDevice->getTemperature() << sDegreeCelsius
	   << ", state: " << (inDevice->isMovementInProgess() ? "busy" : "ready")
	   << ", device port: " << inDevice->getDevicePort()
	   << endl;
    }
  };



  IndiPluginT::IndiPluginT() : PluginT("IndiPlugin") {
    LOG(debug) << "Constructor of IndiPlugin..." << endl;
 
    DEFINE_OPTION(optIndiServer, "indi_server", po::value<HostnameAndPortT>()->default_value(HostnameAndPortT(IndiClientT::sDefaultIndiHostname, IndiClientT::sDefaultIndiPort)), "INDI server name and port.");
    DEFINE_OPTION(optDeviceName, "device_name", po::value<string>()->required(), "INDI device name.");
    DEFINE_OPTION(optExposureTime, "exposure_time", po::value<float>()->required(), "Camera exposure time in seconds.");
    DEFINE_OPTION(optFrameType, "frame_type", po::value<FrameTypeT::TypeE>()->default_value(FrameTypeT::LIGHT), "Frame type (light|dark|flat|bias).");
    DEFINE_OPTION(optFrameSize, "frame_size", po::value<FrameT>()->default_value(FrameT(0, 0, 0, 0)), "Frame size (X x Y x W x H).");
    DEFINE_OPTION(optBinning, "binning", po::value<BinningT>()->default_value(BinningT(1, 1)), "Camera binning (X x Y).");
    DEFINE_OPTION(optOutput, "output", po::value<string>()->default_value("a.FTS"), "Output file.");
    DEFINE_OPTION(optTimeout, "timeout", po::value<float>()->default_value(-1), "Seconds until command times out (default: no timeout).");
    DEFINE_OPTION(optFilter, "filter", po::value<string>()->required(), "Filter to be selected (position number or name).");
    DEFINE_OPTION(optSteps, "steps", po::value<int>()->required(), "Steps to move focus (<0: inwards, >0 outwards).");
    DEFINE_OPTION(optPosition, "position", po::value<unsigned int>()->required(), "Set absolute focus position (0..MAX).");
    DEFINE_OPTION(optDevicePort, "device_port", po::value<string>()->default_value("/dev/ttyUSB0"), "Set device port.");

    /**
     * take_picture command.
     */
    po::options_description takePictureDescr("take_picture command options");
    takePictureDescr.add(optIndiServer);
    takePictureDescr.add(optDeviceName);
    takePictureDescr.add(optExposureTime);
    takePictureDescr.add(optBinning);
    takePictureDescr.add(optFrameType);
    takePictureDescr.add(optFrameSize);
    takePictureDescr.add(optOutput);    
    takePictureDescr.add_options()("display_picture", "Display picture.");
    REGISTER_CONSOLE_CMD_LINE_COMMAND("take_picture", takePictureDescr, (& IndiDeviceActionT<IndiCameraT, CameraTakePictureT>::performAction));

    /**
     * cooler_enable command.
     */
    DEFINE_OPTION(optCoolerEnable, "temperature", po::value<float>()->required(), "Chip temperature °C.");

    po::options_description coolerEnableDescr("cooler_enable command options");
    coolerEnableDescr.add(optIndiServer);
    coolerEnableDescr.add(optDeviceName);
    coolerEnableDescr.add(optTimeout);
    coolerEnableDescr.add(optCoolerEnable);
    REGISTER_CONSOLE_CMD_LINE_COMMAND("cooler_enable", coolerEnableDescr, (& IndiDeviceActionT<IndiCameraT, CameraCoolerEnableT>::performAction));

    /**
     * cooler_disable command.
     */
    po::options_description coolerDisableDescr("cooler_disable command options");
    coolerDisableDescr.add(optIndiServer);
    coolerDisableDescr.add(optDeviceName);
    coolerDisableDescr.add(optTimeout);
    REGISTER_CONSOLE_CMD_LINE_COMMAND("cooler_disable", coolerDisableDescr, (& IndiDeviceActionT<IndiCameraT, CameraCoolerDisableT>::performAction));

    /**
     * cooler_info command.
     */
    po::options_description coolerInfoDescr("cooler_info command options");
    coolerInfoDescr.add(optIndiServer);
    coolerInfoDescr.add(optDeviceName);
    coolerInfoDescr.add(optTimeout);
    REGISTER_CONSOLE_CMD_LINE_COMMAND("cooler_info", coolerInfoDescr, (& IndiDeviceActionT<IndiCameraT, CameraCoolerInfoT>::performAction));

    /**
     * filter_select command.
     */
    po::options_description filterSelectDescr("filter_select command options");
    filterSelectDescr.add(optIndiServer);
    filterSelectDescr.add(optDeviceName);
    filterSelectDescr.add(optFilter);
    filterSelectDescr.add(optTimeout);
    REGISTER_CONSOLE_CMD_LINE_COMMAND("filter_select", filterSelectDescr, (& IndiDeviceActionT<IndiFilterWheelT, FilterWheelFilterSelectT>::performAction));

    /**
     * filter_info command.
     * Print selected filter (pos + name ('unknown' is default)),
     * num of filter positions.
     */
    po::options_description filterInfoDescr("filter_info command options");
    filterInfoDescr.add(optIndiServer);
    filterInfoDescr.add(optDeviceName);
    filterInfoDescr.add(optTimeout);
    REGISTER_CONSOLE_CMD_LINE_COMMAND("filter_info", filterInfoDescr, (& IndiDeviceActionT<IndiFilterWheelT, FilterWheelFilterInfoT>::performAction));

    /**
     * focus_move_by command  --steps -> num of pos/neg steps.
     */
    po::options_description focusMoveByDescr("focus_move_by command options");
    focusMoveByDescr.add(optIndiServer);
    focusMoveByDescr.add(optDeviceName);
    focusMoveByDescr.add(optSteps);
    focusMoveByDescr.add(optTimeout);
    focusMoveByDescr.add(optDevicePort);
    REGISTER_CONSOLE_CMD_LINE_COMMAND("focus_move_by", focusMoveByDescr, (& IndiDeviceActionT<IndiFocuserT, FocusMoveByT>::performAction));

    /**
     * focus_move_to command  --position -> absolute position (0..N).
     */
    po::options_description focusMoveToDescr("focus_move_to command options");
    focusMoveToDescr.add(optIndiServer);
    focusMoveToDescr.add(optDeviceName);
    focusMoveToDescr.add(optPosition);
    focusMoveToDescr.add(optTimeout);
    focusMoveToDescr.add(optDevicePort);
    REGISTER_CONSOLE_CMD_LINE_COMMAND("focus_move_to", focusMoveToDescr, (& IndiDeviceActionT<IndiFocuserT, FocusMoveToT>::performAction));

    /**
     * focus_reset command.
     */
    po::options_description focusResetDescr("focus_reset_to command options");
    focusResetDescr.add(optIndiServer);
    focusResetDescr.add(optDeviceName);
    focusResetDescr.add(optTimeout);
    focusResetDescr.add(optDevicePort);
    REGISTER_CONSOLE_CMD_LINE_COMMAND("focus_reset", focusResetDescr, (& IndiDeviceActionT<IndiFocuserT, FocusResetT>::performAction));

    /**
     * focus_info command
     * Current focus position, focus temperature (if any), last movement time(?),
     * TODO: auto-move coefficients (if any?)
     */
    po::options_description focusInfoDescr("focus_info command options");
    focusInfoDescr.add(optIndiServer);
    focusInfoDescr.add(optDeviceName);
    focusInfoDescr.add(optTimeout);
    focusInfoDescr.add(optDevicePort);
    REGISTER_CONSOLE_CMD_LINE_COMMAND("focus_info", focusInfoDescr, (& IndiDeviceActionT<IndiFocuserT, FocusInfoT>::performAction));


    // TODO: focus_abort...
  }
};
