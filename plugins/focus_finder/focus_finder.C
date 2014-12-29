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
    static void focusFinderStatusUpdates(const FocusFinderDataT * inFfUpdData) {
      // TODO: Improve output ... 001%,... always same number of , places... or use \t...
      cout << "\r" << setw(5) << (int) (100.0f * inFfUpdData->getProgress()) << "%" << std::fixed << setw(40)
	   << inFfUpdData->getUpdMsg() << setw(10)
	   << "POS=" << inFfUpdData->getAbsPos() << setw(14)
	   << "FWHM_horz=" << std::setprecision(5) << inFfUpdData->getStarData().getFwhmHorz().getValue() << "\"" << setw(14)
	   << "FWHM_vert=" << std::setprecision(5) << inFfUpdData->getStarData().getFwhmVert().getValue() << "\"" << setw(10)
	   << "HFD=" << std::setprecision(5) << inFfUpdData->getStarData().getHfd().getValue() << "px" << setw(14)
	   << "Fitness=" << std::setprecision(5) << inFfUpdData->getStarData().getFitness() << flush;
    }

    static void performAction(void) {
      const po::variables_map & cmdLineMap = CommonAstroToolsAppT::getCmdLineOptionsMap();
      
      // Check cmdline args - depends
      AT_ASSERT(FocusFinderPlugin, cmdLineMap.count("indi_server") > 0, "Expecting indi_server option being set.");
      AT_ASSERT(FocusFinderPlugin, cmdLineMap.count("camera_device") > 0, "Expecting camera_device option being set.");
      AT_ASSERT(FocusFinderPlugin, cmdLineMap.count("focuser_device") > 0, "Expecting focuser_device option being set.");
      AT_ASSERT(FocusFinderPlugin, cmdLineMap.count("focuser_device_port") > 0, "Expecting focuser_device_port option being set.");
      AT_ASSERT(FocusFinderPlugin, cmdLineMap.count("binning") > 0, "Expecting binning option being set.");
      AT_ASSERT(FocusFinderPlugin, cmdLineMap.count("star_select") > 0, "Expecting star_select option being set.");

      // Additional focus finder options
      AT_ASSERT(FocusFinderPlugin, cmdLineMap.count("window_size") > 0, "Expecting option being set.");
      AT_ASSERT(FocusFinderPlugin, cmdLineMap.count("num_steps_to_determine_direction") > 0, "Expecting num_steps_to_determine_direction option being set.");
      AT_ASSERT(FocusFinderPlugin, cmdLineMap.count("steps_to_reach_focus") > 0, "Expecting steps_to_reach_focus option being set.");
      AT_ASSERT(FocusFinderPlugin, cmdLineMap.count("extrema_fitness_boundary") > 0, "Expecting extrema_fitness_boundary option being set.");
      AT_ASSERT(FocusFinderPlugin, cmdLineMap.count("outer_hfd_radius_px") > 0, "Expecting outer_hfd_radius_px option being set.");
      AT_ASSERT(FocusFinderPlugin, cmdLineMap.count("rough_focus_max_iter_cnt") > 0, "Expecting rough_focus_max_iter_cnt option being set.");
      AT_ASSERT(FocusFinderPlugin, cmdLineMap.count("take_picture_fit_gauss_curve_max_retry_cnt") > 0, "Expecting take_picture_fit_gauss_curve_max_retry_cnt option being set.");
      //AT_ASSERT(FocusFinderPlugin, cmdLineMap.count("debug_show_take_picture_image") > 0, "Expecting debug_show_take_picture_image option being set.");
      AT_ASSERT(FocusFinderPlugin, cmdLineMap.count("rough_focus_search_range_perc") > 0, "Expecting rough_focus_search_range_perc option being set.");
      AT_ASSERT(FocusFinderPlugin, cmdLineMap.count("rough_focus_record_num_curves") > 0, "Expecting rough_focus_record_num_curves option being set.");
      AT_ASSERT(FocusFinderPlugin, cmdLineMap.count("rough_focus_granularity_steps") > 0, "Expecting rough_focus_granularity_steps option being set.");
      AT_ASSERT(FocusFinderPlugin, cmdLineMap.count("fine_focus_record_num_curves") > 0, "Expecting fine_focus_record_num_curves option being set.");
      AT_ASSERT(FocusFinderPlugin, cmdLineMap.count("fine_focus_granularity_steps") > 0, "Expecting fine_focus_granularity_steps option being set.");
      AT_ASSERT(FocusFinderPlugin, cmdLineMap.count("fine_search_range_steps") > 0, "Expecting fine_search_range_steps option being set.");
      AT_ASSERT(FocusFinderPlugin, cmdLineMap.count("vcurve_fit_eps_abs") > 0, "Expecting vcurve_fit_eps_abs option being set.");
      AT_ASSERT(FocusFinderPlugin, cmdLineMap.count("vcurve_fit_eps_rel") > 0, "Expecting vcurve_fit_eps_rel option being set.");

      const HostnameAndPortT & hostnameAndPort = cmdLineMap["indi_server"].as<HostnameAndPortT>();
      const string & cameraDeviceName = cmdLineMap["camera_device"].as<string>();
      const string & focuserDeviceName = cmdLineMap["focuser_device"].as<string>();
      const string & focuserDevicePort = cmdLineMap["focuser_device_port"].as<string>();
      float exposureTimeSec = cmdLineMap["exposure_time"].as<float>();
      const BinningT & binning = cmdLineMap["binning"].as<BinningT>();
      const string & starSelect = cmdLineMap["star_select"].as<string>();

      const unsigned int windowSize = cmdLineMap["window_size"].as<unsigned int>();
      const unsigned int numStepsToDetermineDirection = cmdLineMap["num_steps_to_determine_direction"].as<unsigned int>();
      const unsigned int stepsToReachFocus = cmdLineMap["steps_to_reach_focus"].as<unsigned int>();
      const unsigned int extremaFitnessBoundary = cmdLineMap["extrema_fitness_boundary"].as<unsigned int>();
      const unsigned int outerHfdRadiusPx = cmdLineMap["outer_hfd_radius_px"].as<unsigned int>();
      const unsigned int roughFocusMaxIterCnt = cmdLineMap["rough_focus_max_iter_cnt"].as<unsigned int>();
      const unsigned int takePictureFitGaussCurveMaxRetryCnt = cmdLineMap["take_picture_fit_gauss_curve_max_retry_cnt"].as<unsigned int>();
      //const bool debugShowTakePictureImage = cmdLineMap["debug_show_take_picture_image"].as<unsigned int>();
      const unsigned int roughFocusSearchRangePerc = cmdLineMap["rough_focus_search_range_perc"].as<unsigned int>();
      const unsigned int roughFocusRecordNumCurves = cmdLineMap["rough_focus_record_num_curves"].as<unsigned int>();
      const unsigned int roughFocusGranularitySteps = cmdLineMap["rough_focus_granularity_steps"].as<unsigned int>();
      const unsigned int fineFocusRecordNumCurves = cmdLineMap["fine_focus_record_num_curves"].as<unsigned int>();
      const unsigned int fineFocusGranularitySteps = cmdLineMap["fine_focus_granularity_steps"].as<unsigned int>();
      const unsigned int fineSearchRangeSteps = cmdLineMap["fine_search_range_steps"].as<unsigned int>();
      const unsigned int vcurveFitEpsAbs = cmdLineMap["vcurve_fit_eps_abs"].as<unsigned int>();
      const unsigned int vcurveFitEpsRel = cmdLineMap["vcurve_fit_eps_rel"].as<unsigned int>();

      LOG(info) << "Indi server: " << hostnameAndPort
		<< ", cameraDeviceName: " << cameraDeviceName << ", focuserDeviceName: " << focuserDeviceName
		<< ", focuserDevicePort: " << focuserDevicePort << ", exposureTimeSec: " << exposureTimeSec
		<< ", binning: " << binning << ", starSelect: " << starSelect
		<< ", windowSize: " << windowSize << ", numStepsToDetermineDirection: " << numStepsToDetermineDirection
		<< ", stepsToReachFocus: " << stepsToReachFocus << ", extremaFitnessBoundary: " << extremaFitnessBoundary
		<< ", outerHfdRadiusPx: " << outerHfdRadiusPx << ", roughFocusMaxIterCnt: " << roughFocusMaxIterCnt
		<< ", takePictureFitGaussCurveMaxRetryCnt: " << takePictureFitGaussCurveMaxRetryCnt
		<< ", roughFocusSearchRangePerc: " << roughFocusSearchRangePerc << ", roughFocusRecordNumCurves: " << roughFocusRecordNumCurves
		<< ", roughFocusGranularitySteps: " << fineFocusRecordNumCurves << ", fineFocusGranularitySteps: " << fineFocusGranularitySteps
		<< ", fineSearchRangeSteps: " << fineSearchRangeSteps << ", vcurveFitEpsAbs: " << vcurveFitEpsAbs
		<< ", vcurveFitEpsRel: " << vcurveFitEpsRel << endl;

      IndiClientT indiClient(hostnameAndPort.getHostname(), hostnameAndPort.getPort(), true /* autoconnect*/);

      if (indiClient.isConnected()) {

	// Get camera device
	IndiCameraT * cameraDevice = indiClient.getCamera(cameraDeviceName);
	
	if (! cameraDevice) {
	  stringstream ss;
	  ss << "Invalid device handle for device '" << cameraDeviceName << "' returned.";
	  throw FocusFinderPluginExceptionT(ss.str().c_str());
	}

	// Get focuser device
	IndiFocuserT * focuserDevice = indiClient.getFocuser(focuserDeviceName);

	if (! focuserDevice) {
	  stringstream ss;
	  ss << "Invalid device handle for device '" << focuserDeviceName << "' returned.";
	  throw FocusFinderPluginExceptionT(ss.str().c_str());
	}

	focuserDevice->setDevicePort(focuserDevicePort);

	// Connect devices, throws if problem occurs during connect
	cameraDevice->connect();
	focuserDevice->connect();

	// Crosscheck if really connected (not expected)
	AT_ASSERT(FocusFinderPlugin, cameraDevice->isConnected() && focuserDevice->isConnected(), "Expected camera and focuser to be connected.");


	// Star selection
	PositionT starCenterPos;

	if (! strcmp(starSelect.c_str(), "auto")) {
	  // TODO: Implement automatic star selection
	  AT_ASSERT(FocusFinderPlugin, false, "Mode 'auto' not yet implemented");
	} else if (! strcmp(starSelect.c_str(), "display")) {
	  // TODO: Implement star selection by displaying window
	  AT_ASSERT(FocusFinderPlugin, false, "Mode 'display' not yet implemented");
	} else {
	  // Check if valid position
	  boost::any v;
	  vector<string> values;
	  values.push_back(starSelect);
	  validate(v, values, & starCenterPos, 0);
	  starCenterPos = any_cast<PositionT>(v); // throws boost::bad_any_cast
	}

	LOG(info) << "Star center pos: " << starCenterPos << endl;

	FocusFinderLinearInterpolationImplT ffli(cameraDevice, focuserDevice, starCenterPos, exposureTimeSec, binning);

	// TODO: Introduce typedef for signals2::connection!, do we need x?!
	signals2::connection focusFinderUpdateHandle = ffli.registerFocusFinderUpdateListener(boost::bind(& FocusFinderActionT::focusFinderStatusUpdates, _1));

	// Set further configurations
	// TODO: Question is - do we pass this as optional cmd line parms? or do we provide an additional cfg file with focus finder settings?
	//       If so, whee do we store the file?
	ffli.setWindowSize(windowSize);
	ffli.setNumStepsToDetermineDirection(numStepsToDetermineDirection);
	ffli.setStepsToReachFocus(stepsToReachFocus);
	ffli.setExtremaFitnessBoundary(extremaFitnessBoundary);
	ffli.setOuterHfdRadiusPx(outerHfdRadiusPx);
	ffli.setRoughFocusMaxIterCnt(roughFocusMaxIterCnt);
	ffli.setTakePictureFitGaussCurveMaxRetryCnt(takePictureFitGaussCurveMaxRetryCnt);
	//ffli.setDebugShowTakePictureImage(debugShowTakePictureImage);
	ffli.setRoughFocusSearchRangePerc(roughFocusSearchRangePerc);
	ffli.setRoughFocusRecordNumCurves(roughFocusRecordNumCurves);
	ffli.setRoughFocusGranularitySteps(roughFocusGranularitySteps);
	ffli.setFineFocusRecordNumCurves(fineFocusRecordNumCurves);
	ffli.setFineFocusGranularitySteps(fineFocusGranularitySteps);
	ffli.setFineSearchRangeSteps(fineSearchRangeSteps);
	ffli.setVCurveFitEpsAbs(vcurveFitEpsAbs);
	ffli.setVCurveFitEpsRel(vcurveFitEpsRel);
	
	// Find focus - TODO: Catch anything here?!
	ffli.findFocus();

	ffli.unregisterFocusFinderUpdateListener(focusFinderUpdateHandle);

      } else {
	stringstream ss;
	ss << "Could not connect to INDI client: '" << indiClient << "'." << endl;
	throw FocusFinderPluginExceptionT(ss.str().c_str());
      }
    }
  };


  // TODO: unsigned int windowSize as tmpl. argument no longer required, we pass it as cmd. argument because we don't know it at compile time!
  template <typename ActionT, unsigned int windowSize>
  class CalcStarActionT {
  public:
    static unsigned int getWindowSize() { return windowSize; } 
    static void performAction(void) {
      const po::variables_map & cmdLineMap = CommonAstroToolsAppT::getCmdLineOptionsMap();
      AT_ASSERT(FocusFinderPlugin, cmdLineMap.count("input") > 0, "Expecting input option being set.");
      const string & inputFilename = cmdLineMap["input"].as<string>();
      
      const unsigned int halfWindowSize = windowSize / 2;
      
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
  static const unsigned int sWindowSize = 31; // TODO: Pass as argument instead?!

  class CalcStarCentroidActionT : public CalcStarActionT<CalcStarCentroidActionT, sWindowSize> {
  public:
    static void performAction(const CImg<float> & inImage, const PositionT & inSelectionCenter, const po::variables_map & inCmdLineMap) {
      // At this place, 'position' is valid and marks the center of the analysis window
      // and all pixels wihin this window exist.
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
      const size_t sHfdOuterRadiusPx = 15; // TODO: Pass as argunment?!

      HfdT hfd(inImage, sHfdOuterRadiusPx, centroid.get<0>(), centroid.get<1>(), sWindowSize);
      FwhmT fwhmHorz(extractLine(inImage, DirectionT::HORZ, centroid, sWindowSize));
      FwhmT fwhmVert(extractLine(inImage, DirectionT::VERT, centroid, sWindowSize));

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
      cout << "Fwhm(" << DirectionT::asStr(DirectionT::HORZ) << ")=" << fwhmHorz.getValue() << "px"
	   << (fwhmHorzArcSec ? fwhmHorzArcSecSs.str() : "") << endl;

      stringstream fwhmVertArcSecSs;
      fwhmVertArcSecSs << "=" << fwhmVertArcSec << "\"";
      cout << "Fwhm(" << DirectionT::asStr(DirectionT::VERT) << ")=" << fwhmVert.getValue() << "px"
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
    DEFINE_OPTION(optStarSelect, "star_select", po::value<string>()->default_value("display"), "Star select [auto|display|(x,y)]");

    // Focuser
    DEFINE_OPTION(optFocuserDeviceName, "focuser_device", po::value<string>()->required(), "INDI focuser device name.");
    DEFINE_OPTION(optFocuserDevicePort, "focuser_device_port", po::value<string>()->default_value("/dev/ttyUSB0"), "Set focuser device port.");

    // Filter wheel
    DEFINE_OPTION(optFilterDeviceName, "filter_device", po::value<string>(), "INDI filter device name.");
    DEFINE_OPTION(optFilter, "filter", po::value<string>(), "Filter to be selected for focusing (position number or name).");

    // Focus finder
    DEFINE_OPTION(optWindowSize, "window_size", po::value<unsigned int>()->default_value(31), "Star evaluation window size in px.");
    DEFINE_OPTION(optNumStepsToDetermineDirection, "num_steps_to_determine_direction", po::value<unsigned int>()->default_value(3000), "Star evaluation window size in px.");
    DEFINE_OPTION(optStepsToReachFocus, "steps_to_reach_focus", po::value<unsigned int>()->default_value(3000), "Steps to reach focus.");
    DEFINE_OPTION(optExtremaFitnessBoundary, "extrema_fitness_boundary", po::value<unsigned int>()->default_value(25), "Extrema fitness boundary.");
    DEFINE_OPTION(optOuterHfdRadiusPx, "outer_hfd_radius_px", po::value<unsigned int>()->default_value(5), "Outer HFD radius px.");
    DEFINE_OPTION(optRoughFocusMaxIterCnt, "rough_focus_max_iter_cnt", po::value<unsigned int>()->default_value(20), "Rough focus max iter cnt.");
    DEFINE_OPTION(optTakePictureFitGaussCurveMaxRetryCnt, "take_picture_fit_gauss_curve_max_retry_cnt", po::value<unsigned int>()->default_value(5), "Take picture fit gauss curve max retry cnt.");
    //DEFINE_OPTION(optDebugShowTakePictureImage, "debug_show_take_picture_image", po::value<bool>()->default_value(false), "Show each taken picture (debug).");
    DEFINE_OPTION(optRoughFocusSearchRangePerc, "rough_focus_search_range_perc", po::value<unsigned int>()->default_value(70), "Rough focus search range percentage.");
    DEFINE_OPTION(optRoughFocusRecordNumCurves, "rough_focus_record_num_curves", po::value<unsigned int>()->default_value(2), "Rough focus record num curves.");
    DEFINE_OPTION(optRoughFocusGranularitySteps, "rough_focus_granularity_steps", po::value<unsigned int>()->default_value(500), "Rough focus granularity steps.");
    DEFINE_OPTION(optFineFocusRecordNumCurves, "fine_focus_record_num_curves", po::value<unsigned int>()->default_value(3), "Fine focus record num curves.");
    DEFINE_OPTION(optFineFocusGranularitySteps, "fine_focus_granularity_steps", po::value<unsigned int>()->default_value(100), "Fine focus granularity steps.");
    DEFINE_OPTION(optFineSearchRangeSteps, "fine_search_range_steps", po::value<unsigned int>()->default_value(3000), "Fine search range steps.");
    DEFINE_OPTION(optVCurveFitEpsAbs, "vcurve_fit_eps_abs", po::value<unsigned int>()->default_value(1), "VCurve fit eps abs.");
    DEFINE_OPTION(optVCurveFitEpsRel, "vcurve_fit_eps_rel", po::value<unsigned int>()->default_value(1), "VCurve fit eps rel.");


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
    focusFindDescr.add(optTimeout);

    // Additional focus finder parameter
    focusFindDescr.add(optWindowSize);
    focusFindDescr.add(optNumStepsToDetermineDirection);
    focusFindDescr.add(optStepsToReachFocus);
    focusFindDescr.add(optExtremaFitnessBoundary);
    focusFindDescr.add(optOuterHfdRadiusPx);
    focusFindDescr.add(optRoughFocusMaxIterCnt);
    focusFindDescr.add(optTakePictureFitGaussCurveMaxRetryCnt);
    //focusFindDescr.add(optDebugShowTakePictureImage);
    focusFindDescr.add(optRoughFocusSearchRangePerc);
    focusFindDescr.add(optRoughFocusRecordNumCurves);
    focusFindDescr.add(optRoughFocusGranularitySteps);
    focusFindDescr.add(optFineFocusRecordNumCurves);
    focusFindDescr.add(optFineFocusGranularitySteps);
    focusFindDescr.add(optFineSearchRangeSteps);
    focusFindDescr.add(optVCurveFitEpsAbs);
    focusFindDescr.add(optVCurveFitEpsRel);

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
