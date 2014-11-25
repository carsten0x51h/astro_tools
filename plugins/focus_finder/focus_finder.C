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

#include <boost/bind.hpp>
#include <boost/function.hpp>

#include "astro_tools_app.hpp"
#include "at_logging.hpp"
#include "at_validator.hpp"

#include "focus_finder.hpp"

#include "centroid.hpp"
#include "focus_finder_linear_interpolation_impl.hpp"

namespace AT {
  DEF_Exception(FocusFinderPlugin);
  DEF_Exception(UnknownFocusFinderImpl);
  DEF_Exception(RequireOption);
  DEF_Exception(ImageDimension);
  DEF_Exception(WindowOutOfBounds);

  class FocusFinderActionT {
  public:
    // TODO: Save new pointers in container and finally delete all?!
    static FocusFinderT * createFocusFinder(const string & inStrategyName) {
      FocusFinderT * focusFinder = 0;
      if (! strcmp(inStrategyName.c_str(), "linear_interpolation")) focusFinder = new FocusFinderLinearInterpolationImplT();
      /*TODO: Add further cases...*/
      else {
	const string exStr = "'" + inStrategyName + "' is not a known strategy.";
	throw UnknownFocusFinderImplExceptionT(exStr.c_str());
      }
      return focusFinder;
    }

    static void destroyFocusFinder(FocusFinderT * inFocusFinder) {
      AT_ASSERT(FocusFinderPlugin, inFocusFinder, "Expecting valid pointer.");
      delete inFocusFinder;
    }

    static void performAction(void) {
      const po::variables_map & cmdLineMap = CommonAstroToolsAppT::getCmdLineOptionsMap();
      
      // Check cmdline args - depends
      AT_ASSERT(FocusFinderPlugin, cmdLineMap.count("indi_server") > 0, "Expecting indi_server option being set.");
      AT_ASSERT(FocusFinderPlugin, cmdLineMap.count("strategy") > 0, "Expecting strategy option being set.");
      
      const HostnameAndPortT & hostnameAndPort = cmdLineMap["indi_server"].as<HostnameAndPortT>();
      const string focusFinderStrategy = cmdLineMap["strategy"].as<string>();
      
      LOG(info) << "Indi server: " << hostnameAndPort << ", Strategy: " << focusFinderStrategy << endl;

      FocusFinderT * focusFinder = FocusFinderActionT::createFocusFinder(focusFinderStrategy); // NOTE: throws if unknown strategy
      focusFinder->findFocus();
      FocusFinderActionT::destroyFocusFinder(focusFinder);
    }
  };



  template <typename ActionT, unsigned int windowSize>
  class CalcStarActionT {
  public:
    static unsigned int getWindowSize() { return windowSize; } 
    static void performAction(void) {
      const po::variables_map & cmdLineMap = CommonAstroToolsAppT::getCmdLineOptionsMap();
      AT_ASSERT(FocusFinderPlugin, cmdLineMap.count("input") > 0, "Expecting input option being set.");
      const string & inputFilename = cmdLineMap["input"].as<string>();
      
      // Note: Window size is fixed by impl... - e.g. 31 x 31
      //const unsigned int windowSize = 31; // TODO: Define 31 or whatever globally! But where? FocusFinderT? Or Centroid?Fwhm?HFD?
      const unsigned int halfWindowSize = windowSize / 2;  // TODO: use >> 1 operator instead?! What if windowSize is even?
      
      CImg<float> image(inputFilename.c_str()); // TODO: What happens if file does not exist?!
      PositionT selectionCenter;
      
      if (image.width() == windowSize && image.height() == windowSize) {
	selectionCenter = PositionT(halfWindowSize, halfWindowSize);
	
	LOG(debug) << dec << "Image size (w h)=(" << image.width() << " " << image.height()
		   << ") exactly fits analysis window (w w)=" << windowSize << " " << windowSize
		   << "). Position equals center (x y)=" << selectionCenter << endl;
	
      } else if (image.width() >= windowSize && image.height() >= windowSize) {
	// Require position argument sinze image is bigger than analysis window.
	if (! cmdLineMap.count("position"))
	  throw RequireOptionExceptionT("Require --position argument.");
	
	selectionCenter = cmdLineMap["position"].as<PositionT>();
	
	LOG(debug) << dec << "Image size (w h)=(" << image.width() << " " << image.height()
		   << ") is bigger than analysis window (w w)=(" << windowSize << " " << windowSize
		   << "). Position specified by user (x y)=" << selectionCenter << endl;
	
	// Evaluate position (check if in bounds...)
	if (! isWindowInBounds(DimensionT(image.width(), image.height()), selectionCenter, windowSize)) {
	  stringstream ss;
	  ss << dec << "Selected area intersects image boundary. Image size (w h)=(" << image.width() << " " << image.height()
	     << "), center (x y)=" << selectionCenter << ", windowSize: " << windowSize << "px" << endl;
	  throw WindowOutOfBoundsExceptionT(ss.str().c_str());
	}
	
	LOG(info) << dec << "Position (x y)=" << selectionCenter << ", window size (w w)=(" << windowSize << " " << windowSize << endl;
      } else {
	stringstream ss;
	ss << dec << "Image too small for analysis. Image size (w h)=(" << image.width() << " " << image.height() << "),"
	   << " but size required is at least (w w)=(" << windowSize << " " << windowSize << ")" << endl;
	throw ImageDimensionExceptionT(ss.str().c_str());
      }

      ActionT::performAction(image, selectionCenter, cmdLineMap);
    }
  };

  // TODO: Where to put this?!
  static const unsigned int sWindowSize = 31;

  class CalcStarCentroidActionT : public CalcStarActionT<CalcStarCentroidActionT, sWindowSize> {
  public:
    static void performAction(const CImg<float> & inImage, const PositionT & inSelectionCenter, const po::variables_map & inCmdLineMap) {
      // At this place, 'position' is valid and marks the center of the analysis window
      // and all pixels wihin this window exist.
      // TODO: pass sWindowSize as template argument?!
      PositionT centroid = CentroidCalcT::starCentroid(inImage, inSelectionCenter, sWindowSize, CoordTypeT::ABSOLUTE);
      cout << dec << "Centroid: " << centroid << endl;
    }
  };

  class CalcStarParmsActionT : public CalcStarActionT<CalcStarParmsActionT, sWindowSize> {
  public:
    static void performAction(const CImg<float> & inImage, const PositionT & inSelectionCenter, const po::variables_map & inCmdLineMap) {
      PositionT centroid = CentroidCalcT::starCentroid(inImage, inSelectionCenter, sWindowSize, CoordTypeT::ABSOLUTE);

      // Check if there is enough image around the centroid
      if (! isWindowInBounds(DimensionT(inImage.width(), inImage.height()), centroid, sWindowSize)) {
	stringstream ss;
	ss << dec << "Selected area intersects image boundary. Image size (w h)=(" << inImage.width() << " " << inImage.height()
	   << "), center (x y)=" << centroid << ", sWindowSize: " << sWindowSize << "px" << endl;
	throw WindowOutOfBoundsExceptionT(ss.str().c_str());
      }
      
      // Check for additional parameter consistency
      bool haveFocalDistance = (inCmdLineMap.count("focal_distance") > 0);
      bool havePixelSize = (inCmdLineMap.count("pixel_size") > 0);

      if (haveFocalDistance != havePixelSize) {
	stringstream ss;
	ss << "Require '" << (havePixelSize ? "--focal_distance" : "--pixel_size")
	   << "' argument when using '" << (havePixelSize ? "--pixel_size" : "--focal_distance")
	   << "' argument." << endl;
	throw RequireOptionExceptionT(ss.str().c_str());
      }

      // TODO: Where to put constants? Use sWindowSize / 2?
      const size_t sHfdOuterRadiusPx = 15;

      HfdT hfd(inImage, sHfdOuterRadiusPx, centroid.get<0>(), centroid.get<1>(), sWindowSize);
      FwhmT fwhmHorz(inImage, FwhmT::DirectionT::HORZ, centroid.get<0>(), centroid.get<1>(), sWindowSize);
      FwhmT fwhmVert(inImage, FwhmT::DirectionT::VERT, centroid.get<0>(), centroid.get<1>(), sWindowSize);

      double hfdArcSec = 0, fwhmHorzArcSec = 0, fwhmVertArcSec = 0;

      if (haveFocalDistance && havePixelSize) {
	AT_ASSERT(FocusFinderPlugin, inCmdLineMap.count("binning") > 0, "Expecting binning option being set.");

	unsigned int focalDistance = inCmdLineMap["focal_distance"].as<unsigned int>();
	const DimensionT & pixelSize = inCmdLineMap["pixel_size"].as<DimensionT>();
	const BinningT & binning = inCmdLineMap["binning"].as<BinningT>();

	hfdArcSec = FwhmT::pxToArcsec(hfd.getValue(), focalDistance, pixelSize, binning);
	fwhmHorzArcSec = FwhmT::pxToArcsec(fwhmHorz.getValue(), focalDistance, pixelSize, binning);
	fwhmVertArcSec = FwhmT::pxToArcsec(fwhmVert.getValue(), focalDistance, pixelSize, binning);
      }

      stringstream hfdArcSecSs;
      hfdArcSecSs << "=" << hfdArcSec << "\"";
      cout << "Hfd=" << hfd.getValue() << "px"
	   << (hfdArcSec ? hfdArcSecSs.str() : "") << endl;

      stringstream fwhmHorzArcSecSs;
      fwhmHorzArcSecSs << "=" << fwhmHorzArcSec << "\"";
      cout << "Fwhm(" << FwhmT::DirectionT::asStr(FwhmT::DirectionT::HORZ) << ")=" << fwhmHorz.getValue() << "px"
	   << (fwhmHorzArcSec ? fwhmHorzArcSecSs.str() : "") << endl;

      stringstream fwhmVertArcSecSs;
      fwhmVertArcSecSs << "=" << fwhmVertArcSec << "\"";
      cout << "Fwhm(" << FwhmT::DirectionT::asStr(FwhmT::DirectionT::VERT) << ")=" << fwhmVert.getValue() << "px"
	   << (fwhmVertArcSec ? fwhmVertArcSecSs.str() : "") << endl;
    }
  };


  FocusFinderPluginT::FocusFinderPluginT() : PluginT("FocusFinderPlugin") {
    LOG(debug) << "Constructor of FocusFinderPluginT..." << endl;

    // Common
    DEFINE_OPTION(optIndiServer, "indi_server", po::value<HostnameAndPortT>()->default_value(HostnameAndPortT(IndiClientT::sDefaultIndiHostname, IndiClientT::sDefaultIndiPort)), "INDI server name and port.");
    DEFINE_OPTION(optTimeout, "timeout", po::value<float>()->default_value(-1), "Seconds until command times out (default: no timeout).");
    DEFINE_OPTION(optInput, "input", po::value<string>(), "Input file.");
    DEFINE_OPTION(optSelectionCenterPos, "position", po::value<PositionT>(), "Selection center (X x Y) px.");
    DEFINE_OPTION(optFocalDistance, "focal_distance", po::value<unsigned int>(), "Telescope focal distance in mm."); 
    DEFINE_OPTION(optPixelSize, "pixel_size", po::value<DimensionT>(), "Pixel size (W x H) um."); 

    // Camera
    DEFINE_OPTION(optCameraDeviceName, "camera_device", po::value<string>()->required(), "INDI camera device name.");
    DEFINE_OPTION(optExposureTime, "exposure_time", po::value<float>()->required(), "Camera exposure time in seconds.");
    DEFINE_OPTION(optBinning, "binning", po::value<BinningT>()->default_value(BinningT(1, 1)), "Camera binning (X x Y).");
    DEFINE_OPTION(optStarSelect, "star_select", po::value<string>()->default_value("display"), "Star select [auto|display|(x,y,w,h)]");

    // Focuser
    DEFINE_OPTION(optFocuserDeviceName, "focuser_device", po::value<string>()->required(), "INDI focuser device name.");
    DEFINE_OPTION(optFocuserDevicePort, "focuser_device_port", po::value<string>()->default_value("/dev/ttyUSB0"), "Set focuser device port.");

    // Filter wheel
    DEFINE_OPTION(optFilterDeviceName, "filter_device", po::value<string>(), "INDI filter device name.");
    DEFINE_OPTION(optFilter, "filter", po::value<string>(), "Filter to be selected for focusing (position number or name).");

    // Focus finder options
    DEFINE_OPTION(optStrategy, "strategy", po::value<string>()->default_value("linear_interpolation"), "Focus finder strategy [linear_interpolation]."); // TODO: enum?! each strategy registers? vector?


    /**
     * focus_find command.
     */
    po::options_description focusFindDescr("focus_find command options");
    focusFindDescr.add(optIndiServer);
    focusFindDescr.add(optCameraDeviceName);
    focusFindDescr.add(optExposureTime);
    focusFindDescr.add(optBinning);
    focusFindDescr.add(optStarSelect);
    focusFindDescr.add(optFocuserDeviceName);
    focusFindDescr.add(optFocuserDevicePort);
    focusFindDescr.add(optFilterDeviceName);
    focusFindDescr.add(optFilter);
    focusFindDescr.add(optStrategy);
    focusFindDescr.add(optTimeout);
    //focusFindDescr.add_options()("display_picture", "Display picture.");
    REGISTER_CONSOLE_CMD_LINE_COMMAND("focus_find", focusFindDescr, (& FocusFinderActionT::performAction));


    /**
     * Calc centroid for given image....
     *
     * In: Path to image, optionally enable display of image with centroid (?) -> requires Qt?!
     * Out:  Absolute position (x,y) of centroid of passed image.
     */
    po::options_description calcStarCentroidDescr("calc_star_centroid command options");
    calcStarCentroidDescr.add(optInput);
    calcStarCentroidDescr.add(optSelectionCenterPos);
    REGISTER_CONSOLE_CMD_LINE_COMMAND("calc_star_centroid", calcStarCentroidDescr, (& CalcStarActionT<CalcStarCentroidActionT, sWindowSize>::performAction));

    /**
     * Calc FWHM and HFD for given star.
     *
     * In:  Path to image, optionally enable graphical display of FWHM (?) -> requires Qt?!.
     * Out: FWHM & HFD values of passed image.
     */
    po::options_description calcStarParmsDescr("calc_star_parms command options");
    calcStarParmsDescr.add(optInput);
    calcStarParmsDescr.add(optSelectionCenterPos);
    calcStarParmsDescr.add(optFocalDistance);
    calcStarParmsDescr.add(optPixelSize);
    calcStarParmsDescr.add(optBinning);
    REGISTER_CONSOLE_CMD_LINE_COMMAND("calc_star_parms", calcStarParmsDescr, (& CalcStarActionT<CalcStarParmsActionT, sWindowSize>::performAction));


  }
};
