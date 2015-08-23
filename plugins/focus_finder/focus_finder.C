//TODO: FwhMT class extension, Noise reduction, hot pixel filter(?), 

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

#include <ncurses.h>

#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <limits>

#include <unistd.h>
#include <stdlib.h>

#include "astro_tools_app.hpp"
#include "at_logging.hpp"
#include "io_util.hpp"
#include "at_validator.hpp"

#include "focus_finder.hpp"

#include "threshold.hpp"
#include "normalize.hpp"
#include "cluster.hpp"
#include "centroid.hpp"
#include "star_frame_selector.hpp"
#include "focus_finder_linear_interpolation_impl.hpp"

namespace AT {

  DEF_Exception(FocusFinderPlugin);
  DEF_Exception(UnknownFocusFinderImpl);
  DEF_Exception(RequireOption);
  DEF_Exception(ImageDimension);
  DEF_Exception(WindowOutOfBounds);

  static void
  manualConsoleFocusCntl(IndiCameraT * inCameraDevice, IndiFocuserT * inFocuserDevice,
			 FrameT<unsigned int> & inSelectionFrame, float inExposureTimeSec,
			 BinningT inBinning)
  {
    CImgDisplay currentImageDisp;
    CImg<float> image;
    int key;

    int stepSize = 10;
    int stepFactor = 10;
    int focusPos = inFocuserDevice->getAbsPos();
    float hfdValue = -1;
  
    initscr();
    crmode();
    keypad(stdscr, TRUE);
    noecho();
    clear();
    mvprintw(5,5, "Focus finder. Press 'q' to quit");
    move(7,5);
    refresh();
    key = getch();

    while(key != 'q') {
      move(7,5);
      clrtoeol();
    
      if ((key >= 'A' && key <= 'Z') || (key >= 'a' && key <= 'z')) {
	//printw("Key was %c", (char) key);
      } else {
	switch(key){
	case KEY_LEFT:
	  if (stepSize > 1) {
	    stepSize =  stepSize / stepFactor;
	  }
	  move(5,5);
	  clrtoeol();
	  mvprintw(5, 5, "Step size: %d", stepSize);
	  break;
	
	case KEY_RIGHT:
	  if (stepSize < 10000) {
	    stepSize =  stepSize * stepFactor;
	  }
	  move(5, 5);
	  clrtoeol();
	  mvprintw(5, 5, "Step size: %d", stepSize);
	  break;

	case KEY_UP:
	  move(4, 5);
	  clrtoeol();
	  focusPos += stepSize;
	  mvprintw(4, 5, "Focus pos (dest): %d", focusPos);
	  inFocuserDevice->setAbsPos(focusPos, 0 /* non-blocking */);
	  break;
	
	case KEY_DOWN:
	  move(4, 5);
	  clrtoeol();
	  focusPos -= stepSize;
	  mvprintw(4, 5, "Focus pos (dest): %d", focusPos);
	  inFocuserDevice->setAbsPos(focusPos, 0 /* non-blocking */);
	  break;
	
	case 27 /*LOCAL_ESCAPE_KEY*/:
	  printw("%s", "Stopping focus motion.");
	  inFocuserDevice->abortMotion(0 /*non blocking*/);
	  break;
	
	  // case KEY_END:
	  // 	printw("%s", "END key");
	  // 	break;
	  
	default:
	  //printw("Unmatched - %d", key);
	  break;
	} /* switch */
      } /*else*/


      // Update measurement data
      string focuserStatus = (inFocuserDevice->isMovementInProgess() ? "busy" : "ready"); 
      stringstream ssCameraStatus;
      if (inCameraDevice->isExposureInProgress()) {
	ssCameraStatus << "exposing..." << inCameraDevice->getExposureTime() << "s";
      } else {
	ssCameraStatus << "ready";
      }

      // Calculate status
      const size_t infoColumn = 50;
      mvprintw(2, infoColumn, "Focus status: %s", focuserStatus.c_str());
      mvprintw(3, infoColumn, "Focus pos (is): %d", inFocuserDevice->getAbsPos());
      mvprintw(4, infoColumn, "Camera status: %s", ssCameraStatus.str().c_str());
      if (hfdValue > 0) {
	mvprintw(5, infoColumn, "HFD: %f\"", hfdValue);
      } else {
	mvprintw(5, infoColumn, "HFD: n.a.");
      }
      mvprintw(6, infoColumn, "FWHM(horz): %f\"", 2.5);
      mvprintw(7, infoColumn, "FWHM(vert): %f\"", 3.5);

    
      // Keep exposure running...
      if (! inCameraDevice->isExposureInProgress()) {	
	LOG(trace) << "setBinning(" << inBinning << ")..." << endl;
	inCameraDevice->setBinning(inBinning);
	LOG(trace) << "setFrame(" << inSelectionFrame << ")..." << endl;
	inCameraDevice->setFrame(inSelectionFrame);
	LOG(trace) << "setFrameType(" << FrameTypeT::asStr(FrameTypeT::LIGHT) << ")..." << endl;
	inCameraDevice->setFrameType(FrameTypeT::LIGHT);
	LOG(trace) << "setCompressed(false)..." << endl;
	inCameraDevice->setCompressed(false); // No compression when moving image to CImg

	// Get previous image (if any)
	inCameraDevice->getImage(& image);

	// HACK!
	CImg<float> loadedImg("test33.fits");
	image = loadedImg.get_crop(inSelectionFrame.get<0>(), inSelectionFrame.get<1>(), inSelectionFrame.get<0>() + inSelectionFrame.get<2>() - 1, inSelectionFrame.get<1>() + inSelectionFrame.get<3>() - 1);
	// HACK!

	LOG(trace) << "Received image [subframe: " << inSelectionFrame << "] from camera - size: " << image.width() << " x " << image.height() << endl;

	// Recalculate picture data...
	LOG(trace) << "image.width(): " << image.width() << ", image.height(): " << image.height() << endl;
	LOG(trace) << "inSelectionFrame.get<2>(): " << inSelectionFrame.get<2>() << ", inSelectionFrame.get<3>(): " << inSelectionFrame.get<3>() << endl;
	
	if (image.width() == (inSelectionFrame.get<2>() / inBinning.get<0>()) && image.height() == (inSelectionFrame.get<3>() / inBinning.get<0>())) {

	  // TODO: Calculate value below instead of hard-coding it...
	  float maxPossiblePixelValue = 65535.0;
	  CImg<unsigned char> normalizedImage(normalize(image, maxPossiblePixelValue, 5.0 /*5%*/));

	  currentImageDisp.display(normalizedImage);

	  // Try to calculate star data
	  // TODO: try catch?
	  HfdT hfd(image);
	  hfdValue = hfd.getValue();

	  // TODO: Also calc FWHMs, max pixel value, ...
	  
	}
	
	LOG(trace) << "startExposure(" << inExposureTimeSec << "s)..." << endl;
	inCameraDevice->startExposure(inExposureTimeSec); // Non-blocking call...!!
      }
    
      refresh();
      timeout(10);
      key = getch();
    } /* while */
    endwin();
  }


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
      const string & starSelectMethod = cmdLineMap["star_select"].as<string>();
      typename StarFrameSelectorT::StarSelectionTypeT::TypeE starRecognitionMethod = cmdLineMap["star_recognition"].as<typename StarFrameSelectorT::StarSelectionTypeT::TypeE>();

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
      const string & focusMode = cmdLineMap["focus_mode"].as<string>();

      LOG(info) << "Indi server: " << hostnameAndPort
		<< ", cameraDeviceName: " << cameraDeviceName << ", focuserDeviceName: " << focuserDeviceName
		<< ", focuserDevicePort: " << focuserDevicePort << ", exposureTimeSec: " << exposureTimeSec
		<< ", binning: " << binning << ", starSelect: " << starSelectMethod
		<< ", starRecognitionMethod: " << starRecognitionMethod
		<< ", windowSize: " << windowSize << ", numStepsToDetermineDirection: " << numStepsToDetermineDirection
		<< ", stepsToReachFocus: " << stepsToReachFocus << ", extremaFitnessBoundary: " << extremaFitnessBoundary
		<< ", outerHfdRadiusPx: " << outerHfdRadiusPx << ", roughFocusMaxIterCnt: " << roughFocusMaxIterCnt
		<< ", takePictureFitGaussCurveMaxRetryCnt: " << takePictureFitGaussCurveMaxRetryCnt
		<< ", roughFocusSearchRangePerc: " << roughFocusSearchRangePerc << ", roughFocusRecordNumCurves: " << roughFocusRecordNumCurves
		<< ", roughFocusGranularitySteps: " << fineFocusRecordNumCurves << ", fineFocusGranularitySteps: " << fineFocusGranularitySteps
		<< ", fineSearchRangeSteps: " << fineSearchRangeSteps << ", vcurveFitEpsAbs: " << vcurveFitEpsAbs
		<< ", vcurveFitEpsRel: " << vcurveFitEpsRel
		<< ", focusMode: " << focusMode << endl;

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
	
	// Get max. resolution
	long bitPix = cameraDevice->getBitsPerPixel();
	DimensionT<int> maxRes(cameraDevice->getMaxResolution().get<0>(), cameraDevice->getMaxResolution().get<1>());
	LOG(info) << "Max camera resoultion: " << maxRes << ", bitsPerPix: " << bitPix << endl;
	
	// NOTE: Calculation seems to be done inside camera driver..
	FrameT<float> fullFrame(0, 0, maxRes.get<0>(), maxRes.get<1>());
	
	// Take an image
	// NOTE: To select a star a higher binning makes sense...?!!!!
	LOG(trace) << "setBinning(" << binning << ")..." << endl;
	cameraDevice->setBinning(binning);
	//cameraDevice->setBinning(BinningT(4,4));
	LOG(trace) << "setFrame(" << fullFrame << ")..." << endl;
	cameraDevice->setFrame(fullFrame);
	LOG(trace) << "setFrameType(" << FrameTypeT::asStr(FrameTypeT::LIGHT) << ")..." << endl;
	cameraDevice->setFrameType(FrameTypeT::LIGHT);
	LOG(trace) << "setCompressed(false)..." << endl;
	cameraDevice->setCompressed(false); // No compression when moving image to CImg
	LOG(trace) << "startExposure(" << exposureTimeSec << "s)..." << endl;
	cameraDevice->startExposure(exposureTimeSec); // Non-blocking call...!!
	// We have to wait for the first image...
	unsigned int estimatedTime = 1000 * exposureTimeSec + 15000 /* 15 sec. to transfer 1x1 binned image */;
	WAIT_MAX_FOR_PRINT(! cameraDevice->isExposureInProgress(), estimatedTime,
			   CommonAstroToolsAppT::isInQuietMode(), "[Exposure left " << cameraDevice->getExposureTime() << "s]...",
			   "Hit timeout while waiting for camera.");
	
	CImg<float> image;
	cameraDevice->getImage(& image);

	// DEBUG START
	//image.save("TEST.fits");
	// DEBUG END

	// HACK!!!!
	image.load("test33.fits");
	//image = image.get_crop(0, 0, image.width() / 2, image.height() / 2);
	//image.load("test2.jpeg");

	
	
	// Star selection
	const string & starSelectMethod = cmdLineMap["star_select"].as<string>();
	typename StarFrameSelectorT::StarSelectionTypeT::TypeE starRecognitionMethod = cmdLineMap["star_recognition"].as<typename StarFrameSelectorT::StarSelectionTypeT::TypeE>();
	typename CentroidT::CentroidTypeT::TypeE centroidMethod = cmdLineMap["centroid_method"].as<typename CentroidT::CentroidTypeT::TypeE>();

	FrameT<unsigned int> selectedFrame = StarFrameSelectorT::calc(image, bitPix, starSelectMethod, starRecognitionMethod, centroidMethod);

	if (! strcmp(focusMode.c_str(), "manual")) { // Manual focusing
	  manualConsoleFocusCntl(cameraDevice, focuserDevice, selectedFrame, exposureTimeSec, binning);
	} else { // Automatic focusing
	  FocusFinderLinearInterpolationImplT ffli(cameraDevice, focuserDevice, selectedFrame, exposureTimeSec, binning);

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
	}
      } else {
	stringstream ss;
	ss << "Could not connect to INDI client: '" << indiClient << "'." << endl;
	throw FocusFinderPluginExceptionT(ss.str().c_str());
      }
    }
  };


  template <typename ActionT>
  class CalcStarActionT {
  public:
    static void performAction(void) {
      const po::variables_map & cmdLineMap = CommonAstroToolsAppT::getCmdLineOptionsMap();
      AT_ASSERT(FocusFinderPlugin, cmdLineMap.count("input") > 0, "Expecting input option being set.");
      const string & inputFilename = cmdLineMap["input"].as<string>();
      long bitPix = 0;
 
      CImg<float> image;

      // Read file to CImg
      // NOTE: We need to use the readFile function based on CCfits because we need the image depth (bits)
      try {
	LOG(info) << "Opening file " << inputFilename.c_str() << endl;
	readFile(image, inputFilename.c_str(), & bitPix);
      } catch (FitsException &) {
	throw FileNotFoundExceptionT("Read FITS failed.");
      }

      // Star selection
      const string & starSelectMethod = cmdLineMap["star_select"].as<string>();
      typename StarFrameSelectorT::StarSelectionTypeT::TypeE starRecognitionMethod = cmdLineMap["star_recognition"].as<typename StarFrameSelectorT::StarSelectionTypeT::TypeE>();
      typename CentroidT::CentroidTypeT::TypeE centroidMethod = cmdLineMap["centroid_method"].as<typename CentroidT::CentroidTypeT::TypeE>();
      FrameT<unsigned int> selectedFrame = StarFrameSelectorT::calc(image, bitPix, starSelectMethod, starRecognitionMethod, centroidMethod);

      // Rectify selection frame
      FrameT<float> squareFrame = rectify(selectedFrame);
      LOG(debug)  << "Selected frame: " << dec << selectedFrame << " -> rectified: " << squareFrame << endl;
      
      ActionT::performAction(image, squareFrame, cmdLineMap);
    }
  };

  class CalcStarCentroidActionT : public CalcStarActionT<CalcStarCentroidActionT> {
  public:
    static void performAction(const CImg<float> & inImage, const FrameT<float> & inSquareFrame, const po::variables_map & inCmdLineMap) {
      // Determine the centroid
      typename CentroidT::CentroidTypeT::TypeE centroidMethod = inCmdLineMap["centroid_method"].as<typename CentroidT::CentroidTypeT::TypeE>();
      PointT<float> centroid;
      CentroidT::calc(inImage, inSquareFrame, & centroid, 0 /*centeredImg not required*/, CoordTypeT::ABSOLUTE, centroidMethod);
      cout << dec << "Centroid (method=" << CentroidT::CentroidTypeT::asStr(centroidMethod) << "): " << centroid << endl;
    }
  };

  class CalcStarParmsActionT : public CalcStarActionT<CalcStarParmsActionT> {
  public:
    static void performAction(const CImg<float> & inImage, const FrameT<float> & inSquareFrame, const po::variables_map & inCmdLineMap) {     
      // TODO: Add additional arguments: zoom factor
      LOG(trace) << "Entering CalcStarParmsActionT::performAction()..." << endl;
      LOG(trace) << "Input image size: " << inImage.width() << " x " << inImage.height() << endl;

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
      
      // Determine the centroid
      typename CentroidT::CentroidTypeT::TypeE centroidMethod = inCmdLineMap["centroid_method"].as<typename CentroidT::CentroidTypeT::TypeE>();
      PointT<float> centroid;
      CentroidT::calc(inImage, inSquareFrame, & centroid, 0 /*centeredImg not required*/, CoordTypeT::ABSOLUTE, centroidMethod);
      LOG(debug) << "Centroid: " << centroid << endl;
      
      HfdT hfd(inImage, centroid);

      // FIXMEFIXME
      FwhmT fwhmHorz(extractLine(inImage, DirectionT::HORZ, centroid, inSquareFrame.get<2>() /*w*/));
      FwhmT fwhmVert(extractLine(inImage, DirectionT::VERT, centroid, inSquareFrame.get<3>() /*h*/));

      double hfdArcSec = 0, fwhmHorzArcSec = 0, fwhmVertArcSec = 0;

      if (haveFocalDistance && havePixelSize) {
	AT_ASSERT(FocusFinderPlugin, inCmdLineMap.count("binning") > 0, "Expecting binning option being set.");

	unsigned int focalDistance = inCmdLineMap["focal_distance"].as<unsigned int>();
	const DimensionT<int> & pixelSize = inCmdLineMap["pixel_size"].as<DimensionT<int> >();
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

      // Export images
      if (inCmdLineMap.count("export") > 0) {
	const string & exportFolder = inCmdLineMap["export"].as<string>();
	CImg<unsigned char> viewImg(hfd.genView());

	// TODO: Check if exportFolder exists?! Create?
	string hfdPath = exportFolder + "/hfd_view.jpg";
	viewImg.save(hfdPath.c_str());
      }
    }
  };


  FocusFinderPluginT::FocusFinderPluginT() : PluginT("FocusFinderPlugin") {
    LOG(debug) << "Constructor of FocusFinderPluginT..." << endl;

    // Common
    DEFINE_OPTION(optIndiServer, "indi_server", po::value<HostnameAndPortT>()->default_value(HostnameAndPortT(IndiClientT::sDefaultIndiHostname, IndiClientT::sDefaultIndiPort)), "INDI server name and port.");
    DEFINE_OPTION(optTimeout, "timeout", po::value<float>()->default_value(-1), "Seconds until command times out (default: no timeout).");
    DEFINE_OPTION(optInput, "input", po::value<string>(), "Input file.");
    DEFINE_OPTION(optStarSelect, "star_select", po::value<string>()->default_value("display"), "Star select [auto|display|(x,y)]");
    DEFINE_OPTION(optStarRecognition, "star_recognition", po::value<typename StarFrameSelectorT::StarSelectionTypeT::TypeE>()->default_value(StarFrameSelectorT::StarSelectionTypeT::PROXIMITY), "Star select [proximity|clustering]");

    DEFINE_OPTION(optFocalDistance, "focal_distance", po::value<unsigned int>(), "Telescope focal distance in mm."); 
    DEFINE_OPTION(optPixelSize, "pixel_size", po::value<DimensionT<int> >(), "Pixel size (W x H) um.");
    DEFINE_OPTION(optCentroidMethod, "centroid_method", po::value<typename CentroidT::CentroidTypeT::TypeE>()->default_value(CentroidT::CentroidTypeT::IWC), "Centroid method (iwc|iwc_sub|moment2).");
    DEFINE_OPTION(optExportFolder, "export", po::value<string>(), "Export folder.");

    
    // Camera
    DEFINE_OPTION(optCameraDeviceName, "camera_device", po::value<string>()->required(), "INDI camera device name.");
    DEFINE_OPTION(optExposureTime, "exposure_time", po::value<float>()->required(), "Camera exposure time in seconds.");
    DEFINE_OPTION(optBinning, "binning", po::value<BinningT>()->default_value(BinningT(1, 1)), "Camera binning (X x Y).");

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

    DEFINE_OPTION(optFocusMode, "focus_mode", po::value<string>()->default_value("manual"), "Focus mode [manual|auto].");

    

    /**
     * focus_find command.
     */
    po::options_description focusFindDescr("focus_find command options");
    focusFindDescr.add(optIndiServer);
    focusFindDescr.add(optCameraDeviceName);
    focusFindDescr.add(optExposureTime);
    focusFindDescr.add(optBinning);
    focusFindDescr.add(optStarSelect);
    focusFindDescr.add(optStarRecognition);
    focusFindDescr.add(optCentroidMethod);
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
    focusFindDescr.add(optFocusMode);
    
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
    calcStarCentroidDescr.add(optStarSelect);
    focusFindDescr.add(optStarRecognition);
    calcStarCentroidDescr.add(optCentroidMethod);
    REGISTER_CONSOLE_CMD_LINE_COMMAND("calc_star_centroid", calcStarCentroidDescr, (& CalcStarActionT<CalcStarCentroidActionT>::performAction));

    /**
     * Calc FWHM and HFD for given star.
     *
     * In:  Path to image, optionally enable graphical display of FWHM (?) -> requires Qt?!.
     * Out: FWHM & HFD values of passed image.
     */
    po::options_description calcStarParmsDescr("calc_star_parms command options");
    calcStarParmsDescr.add(optInput);
    calcStarParmsDescr.add(optStarSelect);
    focusFindDescr.add(optStarRecognition);
    calcStarParmsDescr.add(optFocalDistance);
    calcStarParmsDescr.add(optPixelSize);
    calcStarParmsDescr.add(optBinning);
    calcStarParmsDescr.add(optCentroidMethod);
    calcStarParmsDescr.add(optExportFolder);    
    REGISTER_CONSOLE_CMD_LINE_COMMAND("calc_star_parms", calcStarParmsDescr, (& CalcStarActionT<CalcStarParmsActionT>::performAction));
  }
};
