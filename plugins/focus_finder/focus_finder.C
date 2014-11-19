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
#include "focus_finder.hpp"

#include "centroid.hpp"

#include "at_validator.hpp"
#include "indi/indi_validator.hpp"

namespace AT {
  DEF_Exception(FocusFinderPlugin);
  DEF_Exception(UnknownFocusFinderImpl);
  DEF_Exception(RequireOption);
  DEF_Exception(ImageDimension);
  DEF_Exception(WindowOutOfBounds);

  /**
   * Focus finder interface.
   */
  class FocusFinderT {
  public:
    virtual void findFocus() = 0;
  };

  /**
   * Average focus finder.
   */
  class FocusFinderAverageT : public FocusFinderT {
  public:
    virtual void findFocus() {
      cerr << "findFocus() - Average focus finder..." << endl;
    }
  };



  class FocusFinderActionT {
  public:
    static FocusFinderT * createFocusFinder(const string & inStrategyName) {
      if (! strcmp(inStrategyName.c_str(), "average")) return new FocusFinderAverageT();
      /*TODO: Add further cases...*/

      const string exStr = "'" + inStrategyName + "' is not a known strategy.";
      throw UnknownFocusFinderImplExceptionT(exStr.c_str());
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
    DEFINE_OPTION(optStrategy, "strategy", po::value<string>()->default_value("average"), "Focus finder strategy [average].");


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

// const size_t FocusFinderT::sWindowWidthPx = 15;
// const size_t FocusFinderT::sWindowHeightPx = 15;
// const size_t FocusFinderT::sHfdOuterRadiusPx = 5;


// const size_t FocusFinderT::sHfdOuterRadiusPx = 15;

// FocusFinderT::FocusFinderT(IndiClientTmplT * inIndiClient, const string & inCameraDeviceName, const string & inFocuserDeviceName) : mIndiClient(inIndiClient), mCameraDeviceName(inCameraDeviceName), mFocuserDeviceName(inFocuserDeviceName), mStop(false) {

//   if (! mIndiClient)
//     throw FocusFinderExceptionT("No indi client set.");
  
//   // Number change listener
//   mIndiClient->registerNumberListener(boost::bind(& FocusFinderT::numberChangeHandler, this, _1));
// }

// FocusFinderT::~FocusFinderT() {
//   cerr << "FocusFinderT::~FocusFinderT()... mIndiClient: " << mIndiClient << endl;
//   if (mIndiClient)
//     mIndiClient->unregisterNumberListener(boost::bind(& FocusFinderT::numberChangeHandler, this, _1));
// }

// void FocusFinderT::numberChangeHandler(INumberVectorProperty * inVecNumber) {
//   if (! inVecNumber)
//     return;

//   if (! strcmp(inVecNumber->name, IndiFocuserClientT::VecPropNameT::asStr(IndiFocuserClientT::VecPropNameT::ABS_FOCUS_POSITION))) {
//     double absFocuserValue = 0;
//     mIndiClient->getNumberChecked(mFocuserDeviceName, IndiFocuserClientT::VecPropNameT::asStr(IndiFocuserClientT::VecPropNameT::ABS_FOCUS_POSITION), IndiFocuserClientT::PropNameT::asStr(IndiFocuserClientT::PropNameT::RELATIVE_ABSOLUTE_POSITION), & absFocuserValue);

//     FocusFinderDataT ffd(absFocuserValue, FwhmT(), FwhmT(), HfdT(), mVCurve); // TODO: Ok?! Not so nice?!
//     mFocusFinderUpdateListeners(& ffd);
//   }

//   return;
// }

// // TODO: Required?!
// // void FocusFinderT::recenter(ostream & os, IndiCameraClientT & inCameraClient, IndiFocuserClientT & inFocuserClient, CImg<float> * outImg, int * outStartX, int * outStartY, double inExposureTime, unsigned int inOuterHfdRadius, unsigned int numStepsToDetermineDirection) {
// //   cerr << "FocusFinderT::recenter..." << endl;

// //   float centerX, centerY;
// //   try {
// //     CentroidCalcT::starCentroid(*outImg, 0 /*x*/, 0 /*y*/, FocusFinderT::sWindowWidthPx, FocusFinderT::sWindowHeightPx, & centerX, & centerY, CoordTypeT::ABSOLUTE, false /* no debug msgs */);
// //   } catch(CentroidExceptionT & exc) {
// //     cerr << "Unable to determine centroid. Leaving it as is." << endl;
// //     return;
// //   }

// //   const int BACK = ceil(FocusFinderT::sWindowWidthPx / 2.0) - 1;
// //   int deltaX = (int) centerX - BACK;
// //   int deltaY = (int) centerY - BACK;
    
// //   if (deltaX || deltaY) {
// //     os << ">>>> RE-CENTERING by (dx, dy)=(" << deltaX << ", " << deltaY << ") -> startX: " << *outStartX << ", startY: " << *outStartY << ", w: " << FocusFinderT::sWindowWidthPx << ", h: " << FocusFinderT::sWindowHeightPx << ", taking new image..." << endl;
// //     *outStartX = *outStartX - deltaX;
// //     *outStartY = *outStartY - deltaY;
// //     inCameraClient.takePicture(outImg, 1 /*horz bin*/, 1 /*vert bin*/, *outStartX /*x*/, *outStartY /*y*/, FocusFinderT::sWindowWidthPx /*w*/, FocusFinderT::sWindowHeightPx /*h*/, FrameTypeT::LIGHT, inExposureTime /*s*/, false /*no compr.*/);
// //   }
// // }

// /**
//  * -Determine initilal dL (initial direction)
//  * -Take picture
//  * -Save FWHM & HFD
//  * -Moving focus up N steps
//  * -Take picture
//  * -Compare new FWHM & HFD against saved values
//  * -If focus improved, "up" is the right direction
//  * -If focus gets worse, "down" is the right direction
//  * -Move foucs "down" N steps
//  */
// bool
// FocusFinderT::determineInitialDirection(ostream & os, IndiCameraClientT & inCameraClient, IndiFocuserClientT & inFocuserClient, CImg<float> * outImg, int inStartX, int inStartY, double inExposureTime, unsigned int inOuterHfdRadius, unsigned int numStepsToDetermineDirection, IndiFocuserClientT::FocusDirectionT::TypeE * outDirectionToImproveFocus)
// {
//   //LOG("Correlating focus direction with focus quality...");

//   IndiFocuserClientT::FocusDirectionT::TypeE directionToImproveFocus;

//   inCameraClient.takePicture(outImg, 1 /*horz bin*/, 1 /*vert bin*/, inStartX /*x*/, inStartY /*y*/, FocusFinderT::sWindowWidthPx /*w*/, FocusFinderT::sWindowHeightPx /*h*/, FrameTypeT::LIGHT, inExposureTime /*s*/, false /*no compr.*/);
    
//   FwhmT fwhmHorz1(*outImg, FwhmT::DirectionT::HORZ);
//   FwhmT fwhmVert1(*outImg, FwhmT::DirectionT::VERT);
//   HfdT hfd1(*outImg, inOuterHfdRadius /*radius px*/);

//   // Send update...
//   FocusFinderDataT ffd(inFocuserClient.getAbsPos(), fwhmHorz1, fwhmVert1, hfd1, mVCurve);
//   mFocusFinderUpdateListeners(& ffd);

//   inFocuserClient.moveFocusBy(numStepsToDetermineDirection, IndiFocuserClientT::FocusDirectionT::INWARDS);

//   inCameraClient.takePicture(outImg, 1 /*horz bin*/, 1 /*vert bin*/, inStartX /*x*/, inStartY /*y*/, FocusFinderT::sWindowWidthPx /*w*/, FocusFinderT::sWindowHeightPx /*h*/, FrameTypeT::LIGHT, inExposureTime /*s*/, false /*no compr.*/);

//   //recenter(os, inCameraClient, inFocuserClient, outImg, & inStartX, & inStartY, inExposureTime, inOuterHfdRadius, numStepsToDetermineDirection);


//   FwhmT fwhmHorz2(*outImg, FwhmT::DirectionT::HORZ);
//   FwhmT fwhmVert2(*outImg, FwhmT::DirectionT::VERT);
//   HfdT hfd2(*outImg, inOuterHfdRadius /*radius px*/);

//   // Send update...
//   FocusFinderDataT ffd2(inFocuserClient.getAbsPos(), fwhmHorz2, fwhmVert2, hfd2, mVCurve);
//   mFocusFinderUpdateListeners(& ffd2);
    
//   //directionToImproveFocus = (hfd2.getValue() < hfd1.getValue() ? IndiFocuserClientT::FocusDirectionT::INWARDS : IndiFocuserClientT::FocusDirectionT::OUTWARDS);
//   directionToImproveFocus = (fwhmHorz2.getValue() < fwhmHorz1.getValue() && fwhmVert2.getValue() < fwhmVert1.getValue() ? IndiFocuserClientT::FocusDirectionT::INWARDS : IndiFocuserClientT::FocusDirectionT::OUTWARDS);

//   inFocuserClient.moveFocusBy(numStepsToDetermineDirection, IndiFocuserClientT::FocusDirectionT::OUTWARDS);

//   os << "DONE. Direction which improves focus is " << IndiFocuserClientT::FocusDirectionT::asStr(directionToImproveFocus)
//      << ", FWHM1=(horz:" << fwhmHorz1.getValue() << ", vert:" << fwhmVert1.getValue() << "), HFD1=" << hfd1.getValue()
//      << ", FWHM2=(horz:" << fwhmHorz2.getValue() << ", vert:" << fwhmVert2.getValue() << "), HFD2=" << hfd2.getValue() << endl;

//   if (outDirectionToImproveFocus)
//     *outDirectionToImproveFocus = directionToImproveFocus;

//   return true;
// }



// /**
//  * -Find rough focus (in: FWMH & HFD at least to be reached, out: success/failure, FWMH & HFD obtained)
//  * -1. Take picture
//  * -2. Save FWHM & HFD
//  * -3. Move M steps into prev. direction
//  * -4. Take picture
//  * -5. Repeat at 1. until new FWHM & HFD are worse than saved
//  * -6. Move focus in oppsosite direction by M steps
//  */
// bool FocusFinderT::findRoughFocus(ostream & os, IndiCameraClientT & inCameraClient, IndiFocuserClientT & inFocuserClient, CImg<float> * outImg, int inStartX, int inStartY, double inExposureTime, unsigned int inOuterHfdRadius, unsigned int numStepsToDetermineDirection, IndiFocuserClientT::FocusDirectionT::TypeE inDirectionToImproveFocus, bool * outUserStop, unsigned int inStepsToReachRoughFocus) {

//   //LOG("Determinig rough focus...");
  
//   inCameraClient.takePicture(outImg, 1 /*horz bin*/, 1 /*vert bin*/, inStartX /*x*/, inStartY /*y*/, FocusFinderT::sWindowWidthPx /*w*/, FocusFinderT::sWindowHeightPx /*h*/, FrameTypeT::LIGHT, inExposureTime /*s*/, false /*no compr.*/);
  
//   size_t iterCounter = 0;
//   const unsigned int maxIterCnt = 100; // TODO: configure? What is a good value? Depends on inStepsToReachRoughFocus...?
  
//   do {
//     FOCUS_FINDER_STOP();

//     if (iterCounter > maxIterCnt)
//       throw FocusFinderExceptionT("Max. number of iterations reached. Something is probably wrong.");

//     FwhmT fwhmHorzPrev(*outImg, FwhmT::DirectionT::HORZ);
//     FwhmT fwhmVertPrev(*outImg, FwhmT::DirectionT::VERT);
//     HfdT hfdPrev(*outImg, inOuterHfdRadius /*radius px*/);

//     // Send update...
//     FocusFinderDataT ffd3(inFocuserClient.getAbsPos(), fwhmHorzPrev, fwhmVertPrev, hfdPrev, mVCurve);
//     mFocusFinderUpdateListeners(& ffd3);

//     inFocuserClient.moveFocusBy(inStepsToReachRoughFocus, inDirectionToImproveFocus);
//     inCameraClient.takePicture(outImg, 1 /*horz bin*/, 1 /*vert bin*/, inStartX /*x*/, inStartY /*y*/, FocusFinderT::sWindowWidthPx /*w*/, FocusFinderT::sWindowHeightPx /*h*/, FrameTypeT::LIGHT, inExposureTime /*s*/, false /*no compr.*/);

//     //recenter(os, inCameraClient, inFocuserClient, outImg, & inStartX, & inStartY, inExposureTime, inOuterHfdRadius, numStepsToDetermineDirection);

//     FwhmT fwhmHorzNew(*outImg, FwhmT::DirectionT::HORZ);
//     FwhmT fwhmVertNew(*outImg, FwhmT::DirectionT::VERT);
//     HfdT hfdNew(*outImg, inOuterHfdRadius /*radius px*/);

//     //mVCurve[inFocuserClient.getAbsPos()] = fwhmVertNew.getValue() + fwhmHorzNew.getValue(); // TODO: ok?! TODO: maybe pass mVCurve as param as well?!
//     mVCurve[inFocuserClient.getAbsPos()] = hfdNew.getValue(); // TODO: ok?!

//     // Send update...
//     FocusFinderDataT ffd4(inFocuserClient.getAbsPos(), fwhmHorzNew, fwhmVertNew, hfdNew, mVCurve);
//     mFocusFinderUpdateListeners(& ffd4);

//     os << "iterCounter: " << iterCounter << ", hfdPrev.getValue(): " << hfdPrev.getValue() << ", hfdNew.getValue(): " << hfdNew.getValue() << endl;

//     // TODO: We may continue 2 times in a row if the value became worse... otherwise we may stop too early.
//     // TODO: Or we take two pictures in a row and create the average...
//     if ((fwhmVertPrev.getValue() + fwhmHorzPrev.getValue()) < (fwhmVertNew.getValue() + fwhmHorzNew.getValue())) {
//       inFocuserClient.moveFocusBy(inStepsToReachRoughFocus, (inDirectionToImproveFocus == IndiFocuserClientT::FocusDirectionT::INWARDS ? IndiFocuserClientT::FocusDirectionT::OUTWARDS : IndiFocuserClientT::FocusDirectionT::INWARDS) );

//       inCameraClient.takePicture(outImg, 1 /*horz bin*/, 1 /*vert bin*/, inStartX /*x*/, inStartY /*y*/, FocusFinderT::sWindowWidthPx /*w*/, FocusFinderT::sWindowHeightPx /*h*/, FrameTypeT::LIGHT, inExposureTime /*s*/, false /*no compr.*/);

//       //recenter(os, inCameraClient, inFocuserClient, outImg, & inStartX, & inStartY, inExposureTime, inOuterHfdRadius, numStepsToDetermineDirection);

//       FwhmT fwhmHorzRough(*outImg, FwhmT::DirectionT::HORZ);
//       FwhmT fwhmVertRough(*outImg, FwhmT::DirectionT::VERT);
//       HfdT hfdRough(*outImg, inOuterHfdRadius /*radius px*/);

//       FocusFinderDataT ffd5(inFocuserClient.getAbsPos(), fwhmHorzRough, fwhmVertRough, hfdRough, mVCurve);
//       mFocusFinderUpdateListeners(& ffd5);

//       os << "Reached rough focus position after " << iterCounter << " iterations (step-size=" << inStepsToReachRoughFocus << "), measured: "
//     	 << "FWHM=(horz:" << fwhmHorzPrev.getValue() << ", vert:" << fwhmVertPrev.getValue() << "), HFD=" << hfdPrev.getValue() << endl;
//       break;
//     }
      
//     ++iterCounter;

//   } while (true);

//   //os << "DONE. " << endl;

//   return true;
// }


// /**
//  * -Determine Fmin
//  *  -1. Take picture
//  *  -2. Determine FWHM & HFD
//  *  -3. _Reduce_ focus pos. by K steps, accumulate K
//  *  -4. Take picture
//  *  -5. Repeat 1. as long as new FWHM & HFD are < FWHM/HFD BOUNDARY
//  *  -6. Save "Fmin" focus position (and corresponding HWHM/HFD)
//  *  -7. Move focus back to "center" position (increase focus by Sum(K))
//  * 
//  * -Determine Fmax
//  *  -1. Take picture
//  *  -2. Determine FWHM & HFD
//  *  -3. _Increase_focus pos. by K steps, accumulate K
//  *  -4. Take picture
//  *  -5. Repeat 1. as long as new FWHM & HFD are < FWHM/HFD BOUNDARY
//  *  -6. Save "Fmax" focus position (and corresponding HWHM/HFD)
//  *  -7. Move focus back to "center" position (decrease focus by Sum(K))
//  */
// // TODO: try-catch error handling?! return false?!
// bool FocusFinderT::findExtrema(ostream & os, IndiCameraClientT & inCameraClient, IndiFocuserClientT & inFocuserClient, CImg<float> * outImg, int inStartX, int inStartY, double inExposureTime, unsigned int inOuterHfdRadius, unsigned int numStepsToDetermineDirection, IndiFocuserClientT::FocusDirectionT::TypeE inDirectionToImproveFocus, bool * outUserStop, unsigned int inStepsToReachRoughFocus, MinMaxFocusPosT::TypeE inMinMaxFocusPos, int * outFExtremaPos, const QualityMeasureStrategyT * inQualityMeasureStrategy) {

//   //LOG("Find " << MinMaxFocusPosT::asStr(inMinMaxFocusPos) << "..." << endl);

//   const unsigned int maxFwhmValue = 10; // TODO / FIXME - Experimental: FWHM 14 is boundary... we may pass this as parameter..
  
//   int extremaPos = -1;
//   int startFocusPos = inFocuserClient.getAbsPos();
//   os << "startFocusPos: " << startFocusPos << endl;

//   do {
//     FOCUS_FINDER_STOP();

//     inCameraClient.takePicture(outImg, 1 /*horz bin*/, 1 /*vert bin*/, inStartX /*x*/, inStartY /*y*/, FocusFinderT::sWindowWidthPx /*w*/, FocusFinderT::sWindowHeightPx /*h*/, FrameTypeT::LIGHT, inExposureTime /*s*/, false /*no compr.*/);

//     FwhmT fwhmHorz1(*outImg, FwhmT::DirectionT::HORZ);
//     FwhmT fwhmVert1(*outImg, FwhmT::DirectionT::VERT);
//     HfdT hfd1(*outImg, inOuterHfdRadius /*radius px*/);

//     //mVCurve[inFocuserClient.getAbsPos()] = fwhmVert1.getValue() + fwhmHorz1.getValue(); // TODO: ok?!
//     //mVCurve[inFocuserClient.getAbsPos()] = hfd1.getValue(); // TODO: ok?!
//     mVCurve[inFocuserClient.getAbsPos()] = inQualityMeasureStrategy->calculate(& fwhmHorz1, & fwhmVert1, & hfd1);


//     FocusFinderDataT ffd1(inFocuserClient.getAbsPos(), fwhmHorz1, fwhmVert1, hfd1, mVCurve);
//     mFocusFinderUpdateListeners(& ffd1);

//     if (fwhmHorz1.getValue() > maxFwhmValue || fwhmVert1.getValue() > maxFwhmValue) {
//       os << "Ok, found " << MinMaxFocusPosT::asStr(inMinMaxFocusPos) << ": " << inFocuserClient.getAbsPos() << ", moving back to start position..." << endl;
//       extremaPos = inFocuserClient.getAbsPos();

//       // TODO: Precheck that first value if fwhm is not already bigger than maxFwhmValue... if so all this does not make sense...
//       double delta = (inMinMaxFocusPos == MinMaxFocusPosT::MIN_FOCUS_POS ? startFocusPos - inFocuserClient.getAbsPos() : inFocuserClient.getAbsPos() - startFocusPos);
//       IndiFocuserClientT::FocusDirectionT::TypeE direction = (inMinMaxFocusPos == MinMaxFocusPosT::MIN_FOCUS_POS ? IndiFocuserClientT::FocusDirectionT::OUTWARDS : IndiFocuserClientT::FocusDirectionT::INWARDS);
//       inFocuserClient.moveFocusBy(delta, direction);

//       os << "Moved back to start pos: " << inFocuserClient.getAbsPos() << endl;

//       inCameraClient.takePicture(outImg, 1 /*horz bin*/, 1 /*vert bin*/, inStartX /*x*/, inStartY /*y*/, FocusFinderT::sWindowWidthPx /*w*/, FocusFinderT::sWindowHeightPx /*h*/, FrameTypeT::LIGHT, inExposureTime /*s*/, false /*no compr.*/);

//       //recenter(os, inCameraClient, inFocuserClient, outImg, & inStartX, & inStartY, inExposureTime, inOuterHfdRadius, numStepsToDetermineDirection);

//       FwhmT fwhmHorz33(*outImg, FwhmT::DirectionT::HORZ);
//       FwhmT fwhmVert33(*outImg, FwhmT::DirectionT::VERT);
//       HfdT hfd33(*outImg, inOuterHfdRadius /*radius px*/);

//       FocusFinderDataT ffd33(inFocuserClient.getAbsPos(), fwhmHorz33, fwhmVert33, hfd33, mVCurve);
//       int * posPtr = (inMinMaxFocusPos == MinMaxFocusPosT::MIN_FOCUS_POS ? & ffd33.mAbsMinFocusPos : & ffd33.mAbsMaxFocusPos);
//       *posPtr = extremaPos;
//       mFocusFinderUpdateListeners(& ffd33);

//       if (outFExtremaPos)
// 	*outFExtremaPos = extremaPos;

//       break;
//     }

//     IndiFocuserClientT::FocusDirectionT::TypeE backDir = (inMinMaxFocusPos == MinMaxFocusPosT::MIN_FOCUS_POS ? IndiFocuserClientT::FocusDirectionT::INWARDS : IndiFocuserClientT::FocusDirectionT::OUTWARDS);
//     inFocuserClient.moveFocusBy(inStepsToReachRoughFocus, backDir);

//     FocusFinderDataT ffd44(inFocuserClient.getAbsPos(), FwhmT(), FwhmT(), HfdT(), mVCurve);
//     mFocusFinderUpdateListeners(& ffd44);

//   } while(true);

//   return true;
// }

// /**
//  * -Record V-Curve (in: Fmin, Fmax, J? (TODO: Or determine dynamically? J=f(HFD and/or FWHM)? ))
//  *  -1. Move focus to Fmin (decrease by Fmin)
//  *  -2. Take picture
//  *  -3. Save FWHM & HFD and current focus position (and sub image?)
//  *  -4. If Fmax reached, continue at 7.
//  *  -5. Else:Increase focus by J steps
//  *  -6. Continue at 2.
//  *  -7. Bring focus back to start pos. (Decrease focus by f_rightFmax)
//  */
// bool FocusFinderT::recordVCurve(ostream & os, IndiCameraClientT & inCameraClient, IndiFocuserClientT & inFocuserClient, CImg<float> * outImg, int inStartX, int inStartY, double inExposureTime, unsigned int inOuterHfdRadius, unsigned int numStepsToDetermineDirection, IndiFocuserClientT::FocusDirectionT::TypeE inDirectionToImproveFocus, bool * outUserStop, unsigned int inStepsToReachRoughFocus, int inFmin, int inFmax, const QualityMeasureStrategyT * inQualityMeasureStrategy) {
//   os << "Recording V-Curve..." << endl;
//   os << "Moving to Fmin..." << endl;
//   inFocuserClient.moveFocusBy(inFocuserClient.getAbsPos() - inFmin, IndiFocuserClientT::FocusDirectionT::INWARDS);
  
//   int curPos = inFocuserClient.getAbsPos();
//   int delta = 500; // TODO: Pass / Calculate

//   do {
//     FOCUS_FINDER_STOP();

//     inCameraClient.takePicture(outImg, 1 /*horz bin*/, 1 /*vert bin*/, inStartX /*x*/, inStartY /*y*/, FocusFinderT::sWindowWidthPx /*w*/, FocusFinderT::sWindowHeightPx /*h*/, FrameTypeT::LIGHT, inExposureTime /*s*/, false /*no compr.*/);

//     //recenter(os, inCameraClient, inFocuserClient, outImg, & inStartX, & inStartY, inExposureTime, inOuterHfdRadius, numStepsToDetermineDirection);

//     FwhmT fwhmHorz1(*outImg, FwhmT::DirectionT::HORZ);
//     FwhmT fwhmVert1(*outImg, FwhmT::DirectionT::VERT);
//     HfdT hfd1(*outImg, inOuterHfdRadius /*radius px*/);

//     //mVCurve[inFocuserClient.getAbsPos()] = fwhmVert1.getValue() + fwhmHorz1.getValue(); // TODO: ok?!
//     //mVCurve[inFocuserClient.getAbsPos()] = hfd1.getValue(); // TODO: ok?!
//     mVCurve[inFocuserClient.getAbsPos()] = inQualityMeasureStrategy->calculate(& fwhmHorz1, & fwhmVert1, & hfd1);

//     FocusFinderDataT ffd1(inFocuserClient.getAbsPos(), fwhmHorz1, fwhmVert1, hfd1, mVCurve);
//     mFocusFinderUpdateListeners(& ffd1);


//     inFocuserClient.moveFocusBy(delta, IndiFocuserClientT::FocusDirectionT::OUTWARDS);
//     curPos += delta;

//     if (curPos >= inFmax) {
//       os << "Reached Fmax - finished VCurve recording." << endl;
//       break;
//     }
  
//   } while(true);

//   os << "Finished VCurve recording." << endl;

//   return true;
// }



// bool FocusFinderT::findFocus(int inCenterX, unsigned int inCenterY, int inSize, double inExposureTime, unsigned int inOuterHfdRadius, unsigned int numStepsToDetermineDirection, unsigned int inStepsToReachRoughFocus, bool * outUserStop, ostream & os, const QualityMeasureStrategyT * inQualityMeasureStrategy) {
//   if (! mIndiClient)
//     throw FocusFinderExceptionT("No indi client set.");
  
//   if (mCameraDeviceName.empty())
//     throw FocusFinderExceptionT("No camera device name set.");
  
//   if (mFocuserDeviceName.empty())
//     throw FocusFinderExceptionT("No focuser device name set.");
  
//   if (outUserStop)
//     *outUserStop = false;
  
//   // Camera & focuser access helper
//   IndiCameraClientT cameraClient(mIndiClient, mCameraDeviceName);
//   IndiFocuserClientT focuserClient(mIndiClient, mFocuserDeviceName);
  
//   const int BACK = ceil(inSize / 2.0) - 1;
//   int startX = (int) inCenterX - BACK;
//   int startY = (int) inCenterY - BACK;

//   os << ">>>> FRAME IS!!! X: " << startX << ", Y: " << startY << ", w: " << FocusFinderT::sWindowWidthPx << ", h: " << FocusFinderT::sWindowHeightPx << endl;

//   CImg<float> img;
//   IndiFocuserClientT::FocusDirectionT::TypeE directionToImproveFocus;

//   // TODO: Many parameters may be set as class members!
//   if (! determineInitialDirection(os, cameraClient, focuserClient, & img, startX, startY, inExposureTime, inOuterHfdRadius, numStepsToDetermineDirection, & directionToImproveFocus))
//     return false;
  
//   if (! findRoughFocus(os, cameraClient, focuserClient, & img, startX, startY, inExposureTime, inOuterHfdRadius, numStepsToDetermineDirection, directionToImproveFocus, outUserStop, inStepsToReachRoughFocus))
//     return false;

//   // -Determine rough relative focus boundaries (in: FWHM/HFD BOUNDARY, out; dFmin [steps], dFmax [steps])
//   os << "Determine rough relative focus boundaries (in: FWHM/HFD BOUNDARY, out; dFmin [steps], dFmax [steps]..." << endl;
//   int Fmin = 0, Fmax = 0;

//   mVCurve.clear();

//   if (! findExtrema(os, cameraClient, focuserClient, & img, startX, startY, inExposureTime, inOuterHfdRadius, numStepsToDetermineDirection, directionToImproveFocus, outUserStop, inStepsToReachRoughFocus, MinMaxFocusPosT::MIN_FOCUS_POS, & Fmin, inQualityMeasureStrategy))
//       return false;

//   if (! findExtrema(os, cameraClient, focuserClient, & img, startX, startY, inExposureTime, inOuterHfdRadius, numStepsToDetermineDirection, directionToImproveFocus, outUserStop, inStepsToReachRoughFocus, MinMaxFocusPosT::MAX_FOCUS_POS, & Fmax, inQualityMeasureStrategy))
//       return false;

//   os << "Ok, extrema determined - FminPos: " << Fmin << ", FmaxPos: " << Fmax << endl;


//   mVCurve.clear();

//   if (! recordVCurve(os, cameraClient, focuserClient, & img, startX, startY, inExposureTime, inOuterHfdRadius, numStepsToDetermineDirection, directionToImproveFocus, outUserStop, inStepsToReachRoughFocus, Fmin, Fmax, inQualityMeasureStrategy))
//     return false;


  

//   // Move to opt. focus pos - TODO: sep. function...
//   int xpos;
//   double ymin = mVCurve.getMinY(& xpos);
//   os << "Min FWHM: " << ymin << ", opt focus pos: " << xpos << endl;

//   int curFocusPos = focuserClient.getAbsPos();
//   int delta;
//   IndiFocuserClientT::FocusDirectionT::TypeE direction;

//   if(curFocusPos - xpos > 0) {    
//     delta = curFocusPos - xpos; // Move "left" (INWARDS)
//     direction = IndiFocuserClientT::FocusDirectionT::INWARDS;
//   } else {
//     delta = xpos - curFocusPos; // Move "right" (OUTWARDS)
//     direction = IndiFocuserClientT::FocusDirectionT::OUTWARDS;
//   }
//   focuserClient.moveFocusBy(delta, direction);

//   os << "Moved to optimal focus pos: " << focuserClient.getAbsPos() << " (should be " << xpos << ")..." << endl;

//   cameraClient.takePicture(& img, 1 /*horz bin*/, 1 /*vert bin*/, startX /*x*/, startY /*y*/, FocusFinderT::sWindowWidthPx /*w*/, FocusFinderT::sWindowHeightPx /*h*/, FrameTypeT::LIGHT, inExposureTime /*s*/, false /*no compr.*/);

//   FwhmT fwhmHorz1(img, FwhmT::DirectionT::HORZ);
//   FwhmT fwhmVert1(img, FwhmT::DirectionT::VERT);
//   HfdT hfd1(img, inOuterHfdRadius /*radius px*/);
  
//   FocusFinderDataT ffd1(focuserClient.getAbsPos(), fwhmHorz1, fwhmVert1, hfd1, mVCurve);
//   mFocusFinderUpdateListeners(& ffd1);
  
  

  
//   // -Determine optimal focus position from V-Curve(s) (requires valid V-Curve(s)) (in: V-Curves (a V-Curve is a list of relative focusPos & FWHM/HFD pairs))
//   //    -1. Build one average V-Curve (all V-Curves must be within same relative Fmin & Fmax) (Note: Then for each focusPos there is exactly one HFD & FWHM value per V-Curve)
//   //       -For idx = Fmin to Fmax
//   //          -For all V-Curves
//   //             -Sum the idx value of all V-Curves
//   //          -Divide accumulated idx value by the number of V-Curves

//   //    -2. Separate V-Curve in decreasing data-points and increasing data-points, minimum is to be included in both sets (TODO: Really?))
//   //       -TODO...

//   //    -3. Determine optimal focus position 
//   //       NOTE: Important condition: f_decreasing_fwhm&hfd(pos) = f_increasing_fwhm&hfd(pos) == FWHM == HFD == 0!! --> pos is optimal focus position
//   //           --> Find a_1, b_1 & a_2, b_2 of both functions so that (MSE1 + MSE2) is minimized
//   //           --> Maybe hard to formulate in gsl?! --> We may implement this manually....



//   // -Move to optimal focus position (in: optimal focus position)
//   //    -TODO: Does this involve any corrections (feedback?)

//   return true; // Focus found!
// }
