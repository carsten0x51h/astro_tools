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
  
  template <typename... Args> void
  mv_print(int x, int y, const char * c, Args&&... args) {
    move(y, x);
    clrtoeol();
    mvprintw(y, x, c, std::forward<Args>(args)...);
  }

  // TODO / IDEA: Better console menu: Move to different entries with up/down, change their values by left/right or +/-...
  //              Just for Abort & Quit (and maybe some other operations) add specific keys - like ESC for abort, q for quit...
  static void
  manualConsoleFocusCntl(IndiCameraT * inCameraDevice, IndiFocuserT * inFocuserDevice,
			 const FrameT<unsigned int> & inSelectionFrame, float inExposureTimeSec,
			 BinningT inBinning, bool inFollowStar = true)
  {
    const unsigned int statusPosX = 5;
    const unsigned int statusPosY = 27;
    const size_t infoColumn = 5;

    // TODO FIXME: getSelectionFrame() needs to take care of border!! Use borderFactor below...
    const float borderFactor = 3.0; // TODO: Pass as argument?!
    PointT<float> centerPos = frameToCenterPos(inSelectionFrame);
    FrameT<float> imageFrame = centerPosToFrame(centerPos, borderFactor * inSelectionFrame.get<2>(), borderFactor * inSelectionFrame.get<3>());
    
    LOG(trace) << "Initial selectionFrame(by click): " << inSelectionFrame
	       << ", centerPos: " << centerPos << ", initial imageFrame: " << imageFrame << endl;
    
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
    move(7,5);
    refresh();

    timeout(10);
    key = getch();
    
    while(key != 'q') {
    
      if ((key >= 'A' && key <= 'Z') || (key >= 'a' && key <= 'z')) {
	//printw("Key was %c", (char) key);
      } else {
	switch(key){
	case KEY_LEFT:
	  if (stepSize > 1) {
	    stepSize =  stepSize / stepFactor;
	  }
	  break;
	
	case KEY_RIGHT:
	  if (stepSize < 10000) {
	    stepSize =  stepSize * stepFactor;
	  }
	  break;

	case KEY_UP:
	  focusPos += stepSize;
	  inFocuserDevice->setAbsPos(focusPos, 0 /* non-blocking */);
	  break;
	
	case KEY_DOWN:
	  focusPos -= stepSize;
	  inFocuserDevice->setAbsPos(focusPos, 0 /* non-blocking */);
	  break;
	
	case 27 /*LOCAL_ESCAPE_KEY*/:
	  mv_print(statusPosX, statusPosY, "Stopping focus motion.");
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

      mv_print(5, 25, "Focus finder. Press 'q' to quit");
      mv_print(infoColumn, 1, "Focus status: %s", focuserStatus.c_str());
      mv_print(infoColumn, 2, "Step size: %d", stepSize);
      mv_print(infoColumn, 3, "Focus pos (is): %d", inFocuserDevice->getAbsPos());
      mv_print(infoColumn, 4, "Focus pos (dest): %d", focusPos);
      
      mv_print(infoColumn, 9, "Camera status: %s", ssCameraStatus.str().c_str());
      mv_print(infoColumn, 10, "Cooler status: ");
      mv_print(infoColumn, 11, "Temperature: ");

      
      if (hfdValue > 0) {
	mv_print(infoColumn, 15, "HFD: %f\"", hfdValue);
      } else {
	mv_print(infoColumn, 15, "HFD: n.a.");
      }
      mv_print(infoColumn, 16, "FWHM(horz): %f\"", 2.5);
      mv_print(infoColumn, 17, "FWHM(vert): %f\"", 3.5);
    
      // Keep exposure running...
      if (! inCameraDevice->isExposureInProgress()) {
	// Get previous image (if any)
	inCameraDevice->getImage(& image);

	LOG(trace) << "Received image [subframe: " << imageFrame << "] from camera - size: " << image.width() << " x " << image.height() << endl;
	LOG(trace) << "image.width(): " << image.width() << ", image.height(): " << image.height() << endl;
	LOG(trace) << "inBinning.get<0>(): " << inBinning.get<0>() << ", inBinning.get<1>(): " << inBinning.get<1>() << endl;

	if (image.width() == imageFrame.get<2>() && image.height() == imageFrame.get<3>()) {
	  // Perform re-centroiding i.e. re-set selectionFrame in order to follow the star
	  // ------------------------------------------------------------------------------------------------------------ //
	  FrameT<unsigned int> newFrame;

	  // IDEA: Pass current image, re-center using given method
	  PointT<float> assumedCenter((float) image.width() / 2.0, (float) image.height() / 2.0); // We assume that the star did not move too far from the image center

	  LOG(trace) << "image.width(): " << image.width() << ", image.height(): " << image.height()
		     << ", --> assumed center: " << assumedCenter << endl;

	  // TODO: Do not pass StarFrameSelectorT::StarRecognitionTypeT::PROXIMITY,CentroidT::CentroidTypeT::IWC...
	  //       Instead pass what has been selected by parameters..
	  bool insideBounds = StarFrameSelectorT::calc(image, 0 /*bitPix - TODO / HACK: not needed */,
						       assumedCenter, & newFrame,
						       StarFrameSelectorT::StarRecognitionTypeT::PROXIMITY,
						       CentroidT::CentroidTypeT::IWC, inSelectionFrame.get<2>() /*frameSize*/);

	  AT_ASSERT(StarFrameSelector, insideBounds, "Expected frame to be inside bounds.");
 
	  // Calculate new imageFrame
	  PointT<float> newLocalCenterPos = frameToCenterPos(newFrame); // In "imageFrame" coordinates
	  float deltaX = ((float) imageFrame.get<2>() / 2.0) - newLocalCenterPos.get<0>();
	  float deltaY = ((float) imageFrame.get<3>() / 2.0) - newLocalCenterPos.get<1>();
	  PointT<float> oldCenterPos = frameToCenterPos(imageFrame); // In "full frame" coordinates
	  PointT<float> newCenterPos(oldCenterPos.get<0>() - deltaX, oldCenterPos.get<1>() - deltaY); // In "full frame" coordinates
	  FrameT<unsigned int> newImageFrame = centerPosToFrame(newCenterPos, imageFrame.get<2>(), imageFrame.get<3>());

	  LOG(debug) << "Old imageFrame: " << imageFrame << ", oldCenterPos: " << oldCenterPos << endl;
	  LOG(trace) << "frameSize used: " << inSelectionFrame.get<2>() << ", assumedCenter: " << assumedCenter
		     << ", newFrame: " << newFrame << " --> newLocalCenterPos: " << newLocalCenterPos
		     << ", --> deltaX: " << deltaX << ", deltaY: " << deltaY << endl;
	  LOG(debug) << "New imageFrame: " << newImageFrame << ", newCenterPos: " << newCenterPos << endl;

	  // TODO / FIXME: Check if it hits boundaries!!??
	  
	  // Get sub-frame for analysis
	  // ------------------------------------------------------------------------------------------------------------ //
	  // Convert to imageFrame coordinates
	  FrameT<float> selectionFrame(newFrame);

	  CImg<float> subImg = image.get_crop(selectionFrame.get<0>() /*x0*/,
	   				      selectionFrame.get<1>() /*y0*/,
	   				      selectionFrame.get<0>() + selectionFrame.get<2>() - 1/*x1=x0+w-1*/,
	   				      selectionFrame.get<1>() + selectionFrame.get<3>() - 1/*y1=y0+h-1*/);

	  // Calculate star data
	  // ------------------------------------------------------------------------------------------------------------ //
	  // TODO: try catch?
	  HfdT hfd(subImg); // NOTE: HfdT takes image center as centroid, it does not matter if image is bigger
	  hfdValue = hfd.getValue();
	  // TODO: Also calc FWHMs, max pixel value, ...
	  // TODO: Or can FWHM just do the job?!?!?!!?

	  // DEBUG START
	  // CImg<unsigned char> tmpNormalizedImage(normalize(subImg, 65535.0, 5.0));
	  // CImgDisplay thDsp(tmpNormalizedImage, "SUB-IMG...");
	  // while (! thDsp.is_closed()) {
	  //   thDsp.wait();
	  // }
	  // CImg<unsigned char> hfdView = hfd.genView();
	  // CImgDisplay thDsp2(hfdView, "HFD-VIEW...");
	  // while (! thDsp2.is_closed()) {
	  //   thDsp2.wait();
	  // }
	  // DEBUG END
	  
	  // Finally, scale and display the image
	  // ------------------------------------------------------------------------------------------------------------ //
	  // TODO: Calculate value below instead of hard-coding it...
	  float maxPossiblePixelValue = 65535.0;
	  CImg<unsigned char> normalizedImage(normalize(image, maxPossiblePixelValue, 5.0 /*TODO: HACK FIXME! Should not be hardcoded*/));

	  // Zoom image so that the star can be seen better...
	  // NOTE: RGB because we may add additional things in color to indicate for example the centroid / frame...
	  const float scaleFactor = 3.0;
	  
	  CImg<unsigned char> rgbImg(normalizedImage.width(), normalizedImage.height(), 1 /*depth*/, 3 /*3 channels - RGB*/);    
	  cimg_forXY(normalizedImage, x, y) {
	    int value = normalizedImage(x,y);
	    rgbImg(x, y, 0 /*red*/) = value;
	    rgbImg(x, y, 1 /*green*/) = value;
	    rgbImg(x, y, 2 /*blue*/) = value;
	  }

	  rgbImg.resize(scaleFactor * normalizedImage.width(), scaleFactor * normalizedImage.height(),
			-100 /*size_z*/, -100 /*size_c*/, 1 /*interpolation_type*/);

	  // Draw selected region
	  const unsigned char red[3] = { 255, 0, 0 }, green[3] = { 0, 255, 0 }, blue[3] = { 0, 0, 255 };
	  rgbImg.draw_rectangle(floor(scaleFactor * selectionFrame.get<0>() + 0.5), floor(scaleFactor * selectionFrame.get<1>() + 0.5),
	  			floor(scaleFactor * (selectionFrame.get<0>() + selectionFrame.get<2>()) + 0.5),
	  			floor(scaleFactor * (selectionFrame.get<1>() + selectionFrame.get<3>()) + 0.5),
	  			green, 1 /*opacity*/, ~0 /*pattern*/);

	  // Draw center
	  const size_t cCrossSize = 3;

	  // TODO: Write "drawCross" function...
	  // PointT<float> localNewCenterPos(newCenterPos.get<0>() - imageFrame.get<0>() - selectionFrame.get<0>(),
	  // 				  newCenterPos.get<1>() - imageFrame.get<1>() - selectionFrame.get<1>());
	  PointT<float> localOldCenterPos(oldCenterPos.get<0>() - imageFrame.get<0>(),
	   				  oldCenterPos.get<1>() - imageFrame.get<1>());
	  // TODO:  +1 ?!!
	  // rgbImg.draw_line(floor(scaleFactor * (localOldCenterPos.get<0>() - cCrossSize + 1) + 0.5), floor(scaleFactor * (localOldCenterPos.get<1>() + 1) + 0.5),
	  // 		   floor(scaleFactor * (localOldCenterPos.get<0>() + cCrossSize + 1) + 0.5), floor(scaleFactor * (localOldCenterPos.get<1>() + 1) + 0.5),
	  // 		   blue, 1 /*opacity*/);
	  
	  // rgbImg.draw_line(floor(scaleFactor * (localOldCenterPos.get<0>() + 1) + 0.5), floor(scaleFactor * (localOldCenterPos.get<1>() - cCrossSize + 1) + 0.5),
	  // 		   floor(scaleFactor * (localOldCenterPos.get<0>() + 1) + 0.5), floor(scaleFactor * (localOldCenterPos.get<1>() + cCrossSize + 1) + 0.5),
	  // 		   blue, 1 /*opacity*/);


	  
	  // PointT<float> localNewCenterPos(newCenterPos.get<0>() - imageFrame.get<0>(),
	  //  				  newCenterPos.get<1>() - imageFrame.get<1>());

	  // rgbImg.draw_line(floor(scaleFactor * (localNewCenterPos.get<0>() - cCrossSize + 1) + 0.5), floor(scaleFactor * (localNewCenterPos.get<1>() + 1) + 0.5),
	  // 		   floor(scaleFactor * (localNewCenterPos.get<0>() + cCrossSize + 1) + 0.5), floor(scaleFactor * (localNewCenterPos.get<1>() + 1) + 0.5),
	  // 		   red, 1 /*opacity*/);
	  
	  // rgbImg.draw_line(floor(scaleFactor * (localNewCenterPos.get<0>() + 1) + 0.5), floor(scaleFactor * (localNewCenterPos.get<1>() - cCrossSize + 1) + 0.5),
	  // 		   floor(scaleFactor * (localNewCenterPos.get<0>() + 1) + 0.5), floor(scaleFactor * (localNewCenterPos.get<1>() + cCrossSize + 1) + 0.5),
	  // 		   red, 1 /*opacity*/);

	  rgbImg.draw_line(floor(scaleFactor * (localOldCenterPos.get<0>() - cCrossSize) + 0.5), floor(scaleFactor * localOldCenterPos.get<1>() + 0.5),
			   floor(scaleFactor * (localOldCenterPos.get<0>() + cCrossSize) + 0.5), floor(scaleFactor * localOldCenterPos.get<1>() + 0.5),
			   blue, 1 /*opacity*/);
	  
	  rgbImg.draw_line(floor(scaleFactor * localOldCenterPos.get<0>() + 0.5), floor(scaleFactor * (localOldCenterPos.get<1>() - cCrossSize) + 0.5),
			   floor(scaleFactor * localOldCenterPos.get<0>() + 0.5), floor(scaleFactor * (localOldCenterPos.get<1>() + cCrossSize) + 0.5),
			   blue, 1 /*opacity*/);

	  
	  PointT<float> localNewCenterPos(newCenterPos.get<0>() - imageFrame.get<0>(),
	   				  newCenterPos.get<1>() - imageFrame.get<1>());

	  rgbImg.draw_line(floor(scaleFactor * (localNewCenterPos.get<0>() - cCrossSize) + 0.5), floor(scaleFactor * localNewCenterPos.get<1>() + 0.5),
			   floor(scaleFactor * (localNewCenterPos.get<0>() + cCrossSize) + 0.5), floor(scaleFactor * localNewCenterPos.get<1>() + 0.5), red, 1 /*opacity*/);
	  rgbImg.draw_line(floor(scaleFactor * localNewCenterPos.get<0>() + 0.5), floor(scaleFactor * (localNewCenterPos.get<1>() - cCrossSize) + 0.5),
			   floor(scaleFactor * localNewCenterPos.get<0>() + 0.5), floor(scaleFactor * (localNewCenterPos.get<1>() + cCrossSize) + 0.5), red, 1 /*opacity*/);

	  
	  currentImageDisp.display(rgbImg);

	  
	  if (inFollowStar) {
	    // If star following is enabled, the camera sub-frame is re-centered after each image
	    imageFrame = newImageFrame;
	  }
	} else {
	  LOG(debug) << "Received something unexpected! Expected image size: " << imageFrame.get<2>() << " x " << imageFrame.get<3>()
		     << " but received " << image.width() << " x " << image.height() << " -> ignoring." << endl;
	}
	
	// Start new exposure
	// ------------------------------------------------------------------------------------------------------------ //
	LOG(trace) << "setBinning(" << inBinning << ")..." << endl;
	inCameraDevice->setBinning(inBinning);
	LOG(trace) << "setBinnedFrame(" << imageFrame << ", " << inBinning << ")..." << endl;
	inCameraDevice->setBinnedFrame(imageFrame, inBinning);
	LOG(trace) << "setFrameType(" << FrameTypeT::asStr(FrameTypeT::LIGHT) << ")..." << endl;
	inCameraDevice->setFrameType(FrameTypeT::LIGHT);
	LOG(trace) << "setCompressed(false)..." << endl;
	inCameraDevice->setCompressed(false); // No compression when moving image to CImg

	LOG(trace) << "startExposure(" << inExposureTimeSec << "s)..." << endl;
	inCameraDevice->startExposure(inExposureTimeSec); // Non-blocking call...!!
      }
    
      refresh();
      timeout(10);
      key = getch();
    } /* while */
    endwin();
  }

  static FrameT<unsigned int>
  getStarFrame(const CImg<float> & inImg, long bitPix, const string & starSelectMethod,
	       typename StarFrameSelectorT::StarRecognitionTypeT::TypeE inStarRecognitionMethod,
	       typename CentroidT::CentroidTypeT::TypeE inCentroidMethod, unsigned int inFrameSize = 31) {

    DimensionT<int> imageDimension(inImg.width(), inImg.height());
    PointT<float> selectionCenter;
    FrameT<unsigned int> selectedFrame;
    
    if (! strcmp(starSelectMethod.c_str(), "auto")) {
      //StarFrameSelectorT::calc();
      // Create a binary image to prepare clustering
      float th = ThresholdT::calc(inImg, bitPix, ThresholdT::ThresholdTypeT::OTSU);
      LOG(debug) << "Calculated threshold: " << th << endl;
	  
      CImg<float> thresholdImg(inImg); // Create a copy
      thresholdImg.threshold(th);
	  
      // // DEBUG START
      // CImgDisplay thDsp(thresholdImg, "TH IMG...");
      // while (! thDsp.is_closed()) {
      // 	thDsp.wait();
      // }
      // // DEBUG END
	  
      // Perform star clustering to determine interesting regions
      // TODO: Enabled clustering should not exit program!! Comment in...
      list<FrameT<int> > selectionList;
      ClusterT::calc(thresholdImg, & selectionList);
      
      AT_ASSERT(StarFrameSelector, selectionList.size(), "Automatic star selection needs at least one star.");
      LOG(debug) << "Automatically select 1 star out of " << selectionList.size() << "..." << endl;

      // TODO: Implement automatic star selection - For now we pick just the first one......
      // IDEA: Pick first (best? -> closest to center, brightness at max/2?) star from cluster list... / throw if more than one star?
      selectedFrame = *selectionList.begin();
      
    } else if (! strcmp(starSelectMethod.c_str(), "display")) {
      // Normalize image
      // TODO / FIXME: 5%???
      CImg<unsigned char> normalizedImage(normalize(inImg, pow(2.0, bitPix), 10.0 /*5%*/));

      // NOTE: See http://cimg.eu/reference/structcimg__library_1_1CImgDisplay.html
      // TODO: We will use our own graphical star-selector later - with zoom, value preview etc.
      CImgDisplay dispStarSelect(normalizedImage, "Select a star (click left)...");

      while (! dispStarSelect.is_closed()) {
	dispStarSelect.wait();

	if (dispStarSelect.button() & 1) { // Left button clicked.
	  
	  // NOTE:  If the image is bigger than the screen / window it is stretched and the mouse position
	  //        returned has to be computed accordingly.
	  int x = (float) inImg.width() / (float) dispStarSelect.width() * (float)dispStarSelect.mouse_x();
	  int y = (float) inImg.height() / (float) dispStarSelect.height() * (float)dispStarSelect.mouse_y();
	  selectionCenter = PointT<float>(x, y);
	  LOG(debug) << "Left click - position (x,y)=" << selectionCenter << endl;

	  if (StarFrameSelectorT::calc(inImg, bitPix, selectionCenter, & selectedFrame, inStarRecognitionMethod, inCentroidMethod, inFrameSize)) {
	    //DEBUG START - Show, what has been selected!!
	    // CImg<float> cropImg = normalizedImage.get_crop(selectedFrame.get<0>(), selectedFrame.get<1>(),
	    // 						 selectedFrame.get<0>() + selectedFrame.get<2>() - 1,
	    // 						 selectedFrame.get<1>() + selectedFrame.get<3>() - 1);
	    // CImgDisplay cropDsp(cropImg, "DEBUG - CROP IMG...");
	    // while (! cropDsp.is_closed()) {
	    //   cropDsp.wait();
	    // }
	    //DEBUG END

	    // We have a valid selecction - exit selection loop
	    break;
	  } else {
	    LOG(warning) << "Selected frame hits image boundary." << endl;
	  }
	}
      } // end while
    } else {
      // Check if valid position
      boost::any v;
      vector<string> values;
      values.push_back(starSelectMethod);
      validate(v, values, & selectionCenter, 0);
      selectionCenter = any_cast<PointT<float> >(v); // throws boost::bad_any_cast
      
      // Just create a frame with the given window size at exactly the given position...
      selectedFrame = centerPosToFrame(selectionCenter, inFrameSize);
    }

    // Finally check again, if selected frame is not going to cut any image boundaries
    if (! insideBounds(imageDimension, selectedFrame)) {
      stringstream ss;
      ss << "Selected frame hits image boundary - image bounds: " << imageDimension
	 << ", starSearchFrame: " << selectedFrame << endl;
      
      LOG(error) << ss.str().c_str() << endl;
      throw FocusFinderPluginExceptionT(ss.str().c_str());
    }
    return selectedFrame;
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
      typename StarFrameSelectorT::StarRecognitionTypeT::TypeE starRecognitionMethod = cmdLineMap["star_recognition"].as<typename StarFrameSelectorT::StarRecognitionTypeT::TypeE>();

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
      const bool followStar = cmdLineMap["follow_star"].as<bool>();
  
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
		<< ", focusMode: " << focusMode
		<< ", followStar: " << followStar << endl;

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
	//cameraDevice->setBinning(BinningT(4,4));
	LOG(trace) << "setBinning(" << binning << ")..." << endl;
	cameraDevice->setBinning(binning);
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

	// Star selection
	const string & starSelectMethod = cmdLineMap["star_select"].as<string>();
	typename StarFrameSelectorT::StarRecognitionTypeT::TypeE starRecognitionMethod = cmdLineMap["star_recognition"].as<typename StarFrameSelectorT::StarRecognitionTypeT::TypeE>();
	typename CentroidT::CentroidTypeT::TypeE centroidMethod = cmdLineMap["centroid_method"].as<typename CentroidT::CentroidTypeT::TypeE>();
	FrameT<unsigned int> selectedFrame = getStarFrame(image, bitPix, starSelectMethod, starRecognitionMethod, centroidMethod);

	LOG(debug) << "Selected frame by click: " << selectedFrame << endl;
	
	if (! strcmp(focusMode.c_str(), "manual")) { // Manual focusing
	  manualConsoleFocusCntl(cameraDevice, focuserDevice, selectedFrame, exposureTimeSec, binning, followStar);
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
      typename StarFrameSelectorT::StarRecognitionTypeT::TypeE starRecognitionMethod = cmdLineMap["star_recognition"].as<typename StarFrameSelectorT::StarRecognitionTypeT::TypeE>();
      typename CentroidT::CentroidTypeT::TypeE centroidMethod = cmdLineMap["centroid_method"].as<typename CentroidT::CentroidTypeT::TypeE>();
      FrameT<unsigned int> selectedFrame = getStarFrame(image, bitPix, starSelectMethod, starRecognitionMethod, centroidMethod);

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

      // FIXMEFIXME - TODO: ???
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
    DEFINE_OPTION(optStarRecognition, "star_recognition", po::value<typename StarFrameSelectorT::StarRecognitionTypeT::TypeE>()->default_value(StarFrameSelectorT::StarRecognitionTypeT::PROXIMITY), "Star select [proximity|clustering|none]");

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
    DEFINE_OPTION(optFollowStar, "follow_star", po::value<bool>()->default_value(true), "Follow a selected star by adjusting the selected camera sub-frame.");

    

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
    focusFindDescr.add(optFollowStar);
    
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
