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

#include <boost/thread.hpp>

#include "focuser_console_ui.hpp"
#include "star_frame_selector.hpp"
#include "focus_finder_parabel_fit_impl.hpp"
#include "centroid.hpp"
#include "fwhm.hpp"
#include "hfd.hpp"
#include "at_console_display.hpp"
#include "at_console_menu.hpp"

using namespace std;
using namespace boost;

namespace AT {

  struct TaskStateT {
    enum TypeE {
      READY,
      RUNNING,
      ABORTED,
      FINISHED,
      _Count
    };

    static const char * asStr(const TypeE & inType) {
      switch (inType) {
      case READY: return "READY";
      case RUNNING: return "RUNNING";
      case ABORTED: return "ABORTED";
      case FINISHED: return "FINISHED";
      default: return "<?>";
      }
    }
    MAC_AS_TYPE(Type, E, _Count);
  };

  void
  drawFrame(CImg<unsigned char> * inoutImg, const FrameT<float> & inFrame, const unsigned char inColor[3], float inScaleFactor = 1.0) {
    int x1 = floor(inScaleFactor * inFrame.get<0>() + 0.5);
    int y1 = floor(inScaleFactor * inFrame.get<1>() + 0.5);
    int x2 = floor(inScaleFactor * (inFrame.get<0>() + inFrame.get<2>()) + 0.5);
    int y2 = floor(inScaleFactor * (inFrame.get<1>() + inFrame.get<3>()) + 0.5);
    inoutImg->draw_rectangle(x1, y1, x2, y2, inColor, 1 /*opacity*/, ~0 /*pattern*/);
  }

  void
  drawCross(CImg<unsigned char> * inoutImg, const PointT<float> & inPos, const unsigned char inColor[3], size_t inCrossSize = 3, float inScaleFactor = 1.0) {
    inoutImg->draw_line(floor(inScaleFactor * (inPos.get<0>() - inCrossSize + 1) + 0.5), floor(inScaleFactor * (inPos.get<1>() + 1) + 0.5),
			floor(inScaleFactor * (inPos.get<0>() + inCrossSize + 1) + 0.5), floor(inScaleFactor * (inPos.get<1>() + 1) + 0.5),
			inColor, 1 /*opacity*/);
    
    inoutImg->draw_line(floor(inScaleFactor * (inPos.get<0>() + 1) + 0.5), floor(inScaleFactor * (inPos.get<1>() - inCrossSize + 1) + 0.5),
			floor(inScaleFactor * (inPos.get<0>() + 1) + 0.5), floor(inScaleFactor * (inPos.get<1>() + inCrossSize + 1) + 0.5),
			inColor, 1 /*opacity*/);
  }

  void
  genSelectionView(const CImg<float> & inCurrSubImg, const FrameT<unsigned int> & inFrame, CImg<unsigned char> * outRgbImg, float inScaleFactor = 3.0) {
    const unsigned char red[3] = { 255, 0, 0 }, green[3] = { 0, 255, 0 }, blue[3] = { 0, 0, 255 };
    const size_t cCrossSize = 3;

    // Finally, generate the image for the user...
    float maxPossiblePixelValue = 65535.0; // TODO...
    CImg<unsigned char> normalizedImage(normalize(inCurrSubImg, maxPossiblePixelValue, 5.0 /*TODO: HACK FIXME! Should not be hardcoded*/));

    // NOTE: RGB because we may add additional things in color to indicate for example the centroid / frame...
    CImg<unsigned char> & rgbImgRef = *outRgbImg;
    rgbImgRef.resize(normalizedImage.width(), normalizedImage.height(), 1 /*size_z*/, 3 /*3 channels - RGB*/, 1 /*interpolation_type*/);
    
    cimg_forXY(normalizedImage, x, y) {
      int value = normalizedImage(x,y);
      rgbImgRef(x, y, 0 /*red*/) = value;
      rgbImgRef(x, y, 1 /*green*/) = value;
      rgbImgRef(x, y, 2 /*blue*/) = value;
    }
	    
    rgbImgRef.resize(inScaleFactor * normalizedImage.width(), inScaleFactor * normalizedImage.height(),
		     -100 /*size_z*/, -100 /*size_c*/, 1 /*interpolation_type*/);

    // Draw the "new" frame - this is the frame from which the next "imageFrame" will be calculated (using
    // the "border" - in this case factor 3. This new "imageFrame" will be the boundary for the next image
    // taken from the camera. From the drawm selection the HFD, FWHM etc. are calculated.
    drawFrame(& rgbImgRef, inFrame, red, inScaleFactor);
    drawCross(& rgbImgRef, frameToCenterPos(inFrame), red, cCrossSize, inScaleFactor);
  }


  
  struct StartAbortT {
    enum TypeE {
      START,
      ABORT,
      _Count
    };

    static const char * asStr(const TypeE & inType) {
      switch (inType) {
      case START: return "START";
      case ABORT: return "ABORT";
      default: return "<?>";
      }
    }
    MAC_AS_TYPE(Type, E, _Count);
  };


  struct ExposureModeT {
    enum TypeE {
      SINGLE,
      LOOP,
      _Count
    };

    static const char * asStr(const TypeE & inType) {
      switch (inType) {
      case SINGLE: return "SINGLE";
      case LOOP: return "LOOP";
      default: return "<?>";
      }
    }
    MAC_AS_TYPE(Type, E, _Count);
  };


  /******************************************************************************
   * EXPOSURE TASK
   ******************************************************************************/
  struct ExposureCntlDataT {
    float exposureTime;
    FrameT<float> imageFrame;
    BinningT binning;  
  };

  struct ExposureStatusDataT {
    TaskStateT::TypeE taskState;
    int progress;
    int secondsLeft;
    ExposureStatusDataT() : taskState(TaskStateT::READY), progress(0), secondsLeft(0) {}
  };


  // NOTE: We may move this to a class if states are required 
  void exposeTask(IndiCameraT * inCameraDevice, atomic<ExposureCntlDataT> * cntlData, atomic<ExposureStatusDataT> * statusData, CImg<float> * imageData) {
    ExposureStatusDataT status;
    status.taskState = TaskStateT::RUNNING;
    statusData->store(status);  // updates the variable safely

    ExposureCntlDataT cntl = cntlData->load();
  
    LOG(trace) << "setBinning(" << cntl.binning << ")..." << endl;
    inCameraDevice->setBinning(cntl.binning);

    //TODO: setBinning after first selection of imageFrame causes CentroidException (Frame hit simage bounds...)
    //      -> need to recalc imageFrame(?) - based on old binning and given new binning!
    LOG(trace) << "setBinnedFrame(" << cntl.imageFrame << ", " << cntl.binning << ")..." << endl;
    inCameraDevice->setBinnedFrame(cntl.imageFrame, cntl.binning);
	
    LOG(trace) << "setFrameType(" << FrameTypeT::asStr(FrameTypeT::LIGHT) << ")..." << endl;
    inCameraDevice->setFrameType(FrameTypeT::LIGHT);

    LOG(trace) << "setCompressed(false)..." << endl;
    inCameraDevice->setCompressed(false); // No compression when moving image to CImg
  
    inCameraDevice->startExposure(cntl.exposureTime); // Non-blocking call
    // NOTE: All up to here could be done before statrting the thread!

  
    while (inCameraDevice->isExposureInProgress()) {
      // Update status...
      status.secondsLeft = inCameraDevice->getExposureTime();
      status.progress = 100.0 * (1.0 - ((float) status.secondsLeft / (float)cntl.exposureTime));
      statusData->store(status);  // updates the variable safely

      try {
	chrono::milliseconds dura(200);
	this_thread::sleep_for(dura); // Sleeps for a bit - this is a predefined interruption point
      } catch(boost::thread_interrupted const& ) {
	inCameraDevice->abortExposure();
	status.taskState = TaskStateT::ABORTED;
	statusData->store(status);  // updates the variable safely
	return;
      }
    }

    // Get the image from the camera
    inCameraDevice->getImage(imageData);

    // Update status
    status.taskState = TaskStateT::FINISHED;
    status.progress = 100;
    status.secondsLeft = 0;
    statusData->store(status);  // updates the variable safely
  
    return;
  }


  /******************************************************************************
   * FIND FOCUS TASK
   ******************************************************************************/
  struct FocusFindStatusDataT {
    bool isRunning;
    int progress;
    float fwhmHorz;
    float fwhmVert;
    float hfd;
    FocusFindStatusDataT() : isRunning(false), progress(0), fwhmHorz(0), fwhmVert(0), hfd(0) {}
  };

  void focusFindTask(atomic<FocusFindStatusDataT> * statusData) {
    FocusFindStatusDataT status;
    status.isRunning = true;
    statusData->store(status);  // updates the variable safely
  
    for(int i = 0; i < 100; i++) {
      status.progress = i; // TODO
      status.fwhmHorz = 123.5; // TODO
      status.fwhmVert = 456.5; // TODO
      status.hfd = 111.5; // TODO
    
      statusData->store(status);  // updates the variable safely

      try {
	chrono::milliseconds dura(200);
	this_thread::sleep_for(dura); // Sleeps for a bit
	boost::this_thread::interruption_point();
      } catch(boost::thread_interrupted const& ) {
	status.isRunning = false;
	statusData->store(status);  // updates the variable safely
	return;
      }
    }
  
    status.isRunning = false;
    statusData->store(status);  // updates the variable safely
  
    return;
  }


  void
  manualConsoleFocusCntl(const po::variables_map & inCmdLineMap, IndiCameraT * inCameraDevice, IndiFocuserT * inFocuserDevice, IndiFilterWheelT * inFilterWheelDevice, const FrameT<unsigned int> & inSelectionFrameFF, float inExposureTimeSec, BinningT inBinning, bool inFollowStar)  {

    static ConsoleDisplayT consoleDisplay;
    static std::vector<MenuEntryT *> menuEntries;
    static ConsoleMenuT consoleMenu(& consoleDisplay, menuEntries);
    static MenuSeparatorT sep;


    // "Tasks"
    thread exposureThread;
    atomic<ExposureStatusDataT> exposureStatus;
    atomic<ExposureCntlDataT> exposureCntlData;
    CImg<float> currSubImage;
    
    thread focusFindThread;
    atomic<FocusFindStatusDataT> focusFindStatus;

    // TODO: ControlCameraTemperatureT * controlCameraTemperature = 0;

    // Windows - TODO: Not always available?
    CImgDisplay currImageDisp, currentHfdDisp, currentFwhmHorzDisp, currentFwhmVertDisp;
  
    // Build the menu
    int expTimeVal = inExposureTimeSec;

    MenuFieldT<int> expTimeMenuField(& expTimeVal, "Exposure time: ", 1 /* steps */, StepModeT::LINEAR,
				     1 /* min */, 10000 /*max*/ /*TODO: query min/max from INDI*/,
				     [](int * inValPtr) { return std::to_string(*inValPtr) + "s"; });
    menuEntries.push_back(& expTimeMenuField);


    BinningT maxBinning = inCameraDevice->getMaxBinning();
    BinningT minBinning = inCameraDevice->getMinBinning();
    int binValXY = 1;
    MenuFieldT<int> binningMenuField(& binValXY, "Binning: ", 1 /* steps */, StepModeT::LINEAR,
				     minBinning.get<0>() /* min - NOTE: assuming that x=y */, maxBinning.get<0>() /*max - NOTE: assuming that x=y*/,
				     [](int * inValPtr) { return std::to_string(*inValPtr) +  "x" + std::to_string(*inValPtr); });
    menuEntries.push_back(& binningMenuField);


    int filterSelect = inFilterWheelDevice->getPos();
    MenuFieldT<int> filterSelectMenuField(& filterSelect, "Filter: ", 1 /* steps */, StepModeT::LINEAR,
					  inFilterWheelDevice->getMinPos() /* min */, inFilterWheelDevice->getMaxPos() /*max*/,
					  [](int * inValPtr) { return std::to_string(*inValPtr) +  " (TODO - query...)"; },
					  [&](int * inValPtr) {
					    inFilterWheelDevice->setPos(*inValPtr, 0 /* do not block */);
					  },
					  [&]() {
					    /*ABORT handler*/
					    // TODO: abort not yet implemented - supported by INDI API?!
					    //inFilterWheelDevice->abort();
					  });

    // Check if there is actually a filter ...
    if (inFilterWheelDevice) {
      menuEntries.push_back(& filterSelectMenuField);
    } else {
      menuEntries.push_back(& sep);
    }

  
    ExposureModeT::TypeE expMode = ExposureModeT::SINGLE;
    MenuSelectT<ExposureModeT> expModeMenuSelect(& expMode, "Exposure mode: ",
						 [](ExposureModeT::TypeE * inValPtr) { return ExposureModeT::asStr(*inValPtr); });
    menuEntries.push_back(& expModeMenuSelect);


    StartAbortT::TypeE expStartAbort = StartAbortT::START;
    MenuSelectT<StartAbortT> expStartAbortMenuSelect(& expStartAbort, "Exposure: ",
						     [](StartAbortT::TypeE * inValPtr) { return StartAbortT::asStr(*inValPtr); },
						     [&](StartAbortT::TypeE * inValPtr) {
						       // XXX
						     },
						     [&exposureThread, &exposureStatus]() {
						       /*ABORT handler*/
						       TaskStateT::TypeE taskState = exposureStatus.load().taskState;
						       if (taskState == TaskStateT::RUNNING) {
							 exposureThread.interrupt();
							 // TODO: Set
							 // filterSelect = INDI current value...
						       }
						     });
    menuEntries.push_back(& expStartAbortMenuSelect);

    menuEntries.push_back(& sep);


    int focusStepSize = 100;
    int absFocusPosDest = inFocuserDevice->getAbsPos();
    int focusDelta = 0;
    MenuFieldT<int> absFocusPosDestMenuField(& absFocusPosDest, "Abs focus pos: ", focusStepSize /* steps */, StepModeT::LINEAR,
					     inFocuserDevice->getMinPos() /* min */, inFocuserDevice->getMaxPos() /*max*/,
					     [](int * inValPtr) { return std::to_string(*inValPtr) + " steps"; },
					     [&](int * inValPtr) {
					       focusDelta = fabs(inFocuserDevice->getAbsPos() - *inValPtr);
					       inFocuserDevice->setAbsPos(*inValPtr, 0); // 0 = do not block
					     },
					     [&]() {
					       /*ABORT handler*/
					       inFocuserDevice->abortMotion();
					     });

    MenuFieldT<int> focusStepSizeMenuField(& focusStepSize, "Focus step size: ", 10 /* steps */, StepModeT::FACTOR,
					   1 /* min */, inFocuserDevice->getMaxPos() / 10 /* max/10 */,
					   [](int * inValPtr) { return std::to_string(*inValPtr) + " steps"; },
					   [&absFocusPosDestMenuField, &focusStepSize](int * inValPtr) {
					     // CHANGE HANDLER
					     absFocusPosDestMenuField.setSteps(focusStepSize);
					   });
    menuEntries.push_back(& focusStepSizeMenuField);
    menuEntries.push_back(& absFocusPosDestMenuField);

    menuEntries.push_back(& sep);

    StartAbortT::TypeE focusFindStartAbort = StartAbortT::START;
    MenuSelectT<StartAbortT> focusFindStartAbortMenuSelect(& focusFindStartAbort, "Focus find: ",
							   [](StartAbortT::TypeE * inValPtr) { return StartAbortT::asStr(*inValPtr); },
							   [&](StartAbortT::TypeE * inValPtr) {
							     bool running = focusFindStatus.load().isRunning;
							   
							     if (*inValPtr == StartAbortT::START) {
							       // Start thread if not yet active.
							       if (! running) {
								 focusFindThread = thread(focusFindTask, & focusFindStatus);
							       }
							     } else {
							       // Abort - only interrupt if still running
							       if (running) {
								 focusFindThread.interrupt();
							       }
							     }
							   },
							   [&]() {
							     /*ABORT handler*/
							     bool running = focusFindStatus.load().isRunning;
							     if (running) {
							       focusFindThread.interrupt();
							       // TODO: Set
							       // filterSelect = INDI current value...
							     }
							   });
    menuEntries.push_back(& focusFindStartAbortMenuSelect);
  

    // Set the initial image frame
    // NOTE:
    //   FF = Full Frame coordinates
    //   IF = Image Frame coordinates
    const float borderFactor = 3.0; // TODO: Pass as argument?!
    FrameT<float> currSelectionFrameFF = inSelectionFrameFF;
    PointT<float> currCenterPosFF = frameToCenterPos(currSelectionFrameFF);
    FrameT<float> currImageFrameFF = centerPosToFrame(currCenterPosFF, borderFactor * currSelectionFrameFF.get<2>(), borderFactor * currSelectionFrameFF.get<3>());    


    while (! consoleMenu.wantExit()) {

      // Handle menu
      consoleMenu.update();

      ////////////////////////////
      // Handle filter selction //
      ////////////////////////////
      if (inFilterWheelDevice) {
	stringstream ssFilterWheelStatus;
	if (inFilterWheelDevice->isMovementInProgess()) {
	  ssFilterWheelStatus << "moving... is: " << inFilterWheelDevice->getPos();
	} else {
	  ssFilterWheelStatus << "ready (pos=" << inFilterWheelDevice->getPos() << ").";
	}
	consoleDisplay.print(ConsoleMenuT::cLeftMenuBorder, 16, "Filter status: %s", ssFilterWheelStatus.str().c_str());
      }


      ////////////////////////////
      // Handle focus           //
      ////////////////////////////
      if (inFocuserDevice->isMovementInProgess()) {
	int progress = 100 * (1.0 - ((float)((int) inFocuserDevice->getAbsPos() - (int) absFocusPosDest) / focusDelta));
	consoleDisplay.print(ConsoleMenuT::cLeftMenuBorder, 15, "Moving focus... pos: %d/%d - progress: %d%\n",
			     inFocuserDevice->getAbsPos(), absFocusPosDest, progress);
      } else {
	consoleDisplay.print(ConsoleMenuT::cLeftMenuBorder, 15, "Focus ready - pos: %d\n", inFocuserDevice->getAbsPos());
      }
      
      
      ////////////////////////////
      // Handle exposure...     //
      ////////////////////////////
      ExposureStatusDataT exposureStatusData = exposureStatus.load();
      
      switch(exposureStatusData.taskState) {
      case TaskStateT::READY:
	if (expStartAbort == StartAbortT::ABORT) {
	  ExposureCntlDataT cntlData;
	  cntlData.exposureTime = expTimeVal;
	  cntlData.imageFrame = currImageFrameFF;
	  cntlData.binning = inBinning;
	  exposureCntlData.store(cntlData);

	  // Thread ONLY gets image - post processing does not take place in thread!
      	  exposureThread = thread(exposeTask, inCameraDevice, & exposureCntlData, & exposureStatus, & currSubImage);
	}
	break;
      
      case TaskStateT::RUNNING:
	// Abort - only interrupt if still running
	if (expStartAbort == StartAbortT::START) {
	  exposureThread.interrupt();
	} else {
	  consoleDisplay.print(ConsoleMenuT::cLeftMenuBorder, 13, "Exposure progress: %d%, seconds left: %d\n", exposureStatusData.progress, exposureStatusData.secondsLeft);
	}
	break;

      case TaskStateT::FINISHED:
      case TaskStateT::ABORTED:
	if (exposureStatusData.taskState == TaskStateT::FINISHED) {
	  // Post process image... we assume that the star did not move too far from the image center
	  // NOTE: Boundaries of currSubImage are based on currImageFrameFF.
	  PointT<float> assumedCenter((float) currSubImage.width() / 2.0, (float) currSubImage.height() / 2.0);
	  FrameT<unsigned int> newSelectionFrameIF;
      	  bool insideBounds = StarFrameSelectorT::calc(currSubImage, 0 /*bitPix - TODO / HACK: not needed */,
      						       assumedCenter, & newSelectionFrameIF,
      						       StarFrameSelectorT::StarRecognitionTypeT::PROXIMITY,
      						       CentroidT::CentroidTypeT::IWC, inSelectionFrameFF.get<2>() /*frameSize*/);
	  
      	  AT_ASSERT(StarFrameSelector, insideBounds, "Expected frame to be inside bounds.");	  

	  FrameT<float> newSelectionFrameFF(currImageFrameFF.get<0>() /*x*/ + newSelectionFrameIF.get<0>() /*x*/,
					    currImageFrameFF.get<1>() /*y*/ + newSelectionFrameIF.get<1>() /*y*/,
					    newSelectionFrameIF.get<2>() /*w*/, newSelectionFrameIF.get<3>() /*h*/);
	  PointT<float> newCenterPosFF = frameToCenterPos(newSelectionFrameFF);
	  FrameT<float> newImageFrameFF = centerPosToFrame(newCenterPosFF, borderFactor * newSelectionFrameFF.get<2>(), borderFactor * newSelectionFrameFF.get<3>());

	  // Finally, generate and display the image
	  CImg<unsigned char> rgbImg;
	  genSelectionView(currSubImage, newSelectionFrameIF, & rgbImg, 3.0 /*zoom*/);
	  currImageDisp.display(rgbImg); // TODO - disable if no Wnds...


      	  // Calculate star data
      	  // ------------------------------------------------------------------------------------------------------------ //
	  CImg<float> subImg = currSubImage.get_crop(newSelectionFrameIF.get<0>() /*x0*/,
						     newSelectionFrameIF.get<1>() /*y0*/,
						     newSelectionFrameIF.get<0>() + newSelectionFrameIF.get<2>() - 1/*x1=x0+w-1*/,
						     newSelectionFrameIF.get<1>() + newSelectionFrameIF.get<3>() - 1/*y1=y0+h-1*/);
      	  try {
	    // TODO: HFD value INCREASES if coming to focus using the simulator... !!! Maybe a simulator problem?! --> need real test!!!
      	    HfdT hfd(subImg); // NOTE: HfdT takes image center as centroid, it does not matter if image is bigger
	    consoleDisplay.print(ConsoleMenuT::cLeftMenuBorder, 19, "HFD: %f\n", hfd.getValue());
	    currentHfdDisp.display(hfd.genView());
      	  } catch(std::exception & exc) {
      	    LOG(warning) << "HFD calculation failed!"  << endl;
      	  }
	  
      	  // Subtract median image
      	  double med = subImg.median();
      	  CImg<float> imageSubMed(subImg.width(), subImg.height());
      	  cimg_forXY(subImg, x, y) {
      	    imageSubMed(x, y) = (subImg(x, y) > med ? subImg(x, y) - med : 0);
      	  }
	  
      	  try {
      	    FwhmT fwhmHorz(extractLine<DirectionT::HORZ>(imageSubMed));
	    consoleDisplay.print(ConsoleMenuT::cLeftMenuBorder, 20, "FWHM(horz): %f\n", fwhmHorz.getValue());
      	    currentFwhmHorzDisp.display(fwhmHorz.genView());
      	  } catch(std::exception & exc) {
      	    LOG(warning) << "FWHM(horz) calculation failed!"  << endl;
      	  }

	  try {
      	    FwhmT fwhmVert(extractLine<DirectionT::VERT>(imageSubMed));
	    consoleDisplay.print(ConsoleMenuT::cLeftMenuBorder, 21, "FWHM(vert): %f\n", fwhmVert.getValue());
      	    currentFwhmVertDisp.display(fwhmVert.genView());
      	  } catch(std::exception & exc) {
      	    LOG(warning) << "FWHM(vert) calculation failed!"  << endl;
      	  }
	  
	  
	  // Set newly calculated subImg frame for next camera shot...
	  currSelectionFrameFF = newSelectionFrameFF;
	  currCenterPosFF = newCenterPosFF;
	  currImageFrameFF = newImageFrameFF;
	}

	if (expMode == ExposureModeT::SINGLE) {
	  expStartAbort = StartAbortT::START;
	}
	exposureStatusData.taskState = TaskStateT::READY;
	exposureStatus.store(exposureStatusData);
	break;

      default:
	break;
      }

      ////////////////////////////
      // Handle focus finder    //
      ////////////////////////////
      FocusFindStatusDataT focusFindStatusData = focusFindStatus.load();	
      if (focusFindStatusData.isRunning) {
	consoleDisplay.print(ConsoleMenuT::cLeftMenuBorder, 14, "Focus finder progress: %d, hfd: %f\n", focusFindStatusData.progress, focusFindStatusData.hfd);
      } else {
	focusFindStartAbort = StartAbortT::START;
      }

      
      
      // Wait a moment to keep processor down...
      chrono::milliseconds dura(10);
      this_thread::sleep_for(dura); // Sleeps for a bit
    } // end loop
  }
  




  








  
  //void
  //manualConsoleFocusCntl(const po::variables_map & inCmdLineMap, IndiCameraT * inCameraDevice, IndiFocuserT * inFocuserDevice, IndiFilterWheelT * inFilterWheelDevice, const FrameT<unsigned int> & inSelectionFrame, float inExposureTimeSec, BinningT inBinning, bool inFollowStar) // {
  //   const unsigned int statusPosX = 5;
  //   const unsigned int statusPosY = 27;
  //   const size_t infoColumn = 50;

  //   // TODO FIXME: getSelectionFrame() needs to take care of border!! Use borderFactor below...
  //   const float borderFactor = 3.0; // TODO: Pass as argument?!
  //   PointT<float> centerPos = frameToCenterPos(inSelectionFrame);
  //   FrameT<float> imageFrame = centerPosToFrame(centerPos, borderFactor * inSelectionFrame.get<2>(), borderFactor * inSelectionFrame.get<3>());

  //   bool haveFocalDistance = (inCmdLineMap.count("focal_distance") > 0);
  //   bool havePixelSize = (inCmdLineMap.count("pixel_size") > 0);
  //   unsigned int focalDistance = inCmdLineMap["focal_distance"].as<unsigned int>();
  //   const DimensionT<float> & pixelSize = inCmdLineMap["pixel_size"].as<DimensionT<float> >();
    
  //   LOG(trace) << "Initial selectionFrame(by click): " << inSelectionFrame
  // 	       << ", centerPos: " << centerPos << ", initial imageFrame: " << imageFrame << endl;
    
  //   CImgDisplay currentImageDisp, currentFwhmHorzDisp, currentFwhmVertDisp, currentHfdDisp, hfdHistoryDisp, fwhmHorzHistoryDisp, fwhmVertHistoryDisp;
  //   CImg<float> image;
  //   FocusFinderParabelFitImplT findFocusTask(inCameraDevice, inFocuserDevice); // TODO: Use pointer to TaskT<FocusFinderInfoT, MtTaskPolicyT> instead, later... so we can switch strategy at runtime!
    
  //   int key;

  //   int focusPos = inFocuserDevice->getAbsPos();
  //   float hfdValue = -1;
  //   float fwhmHorzValue = -1;
  //   float fwhmVertValue = -1;
  //   float maxPixelValue = -1;
    
  //   initscr();
  //   crmode();
  //   keypad(stdscr, TRUE);
  //   noecho();
  //   clear();
  //   move(7 /*y*/,5 /*x*/);
  //   refresh();

  //   timeout(10);
  //   key = getch();

  //   // Build the menu
  //   vector<MenuEntryT *> menuActions;
    
  //   SetFocusStepSizeT * focusStepSize = new SetFocusStepSizeT();
  //   ControlExposureT * controlExposure = new ControlExposureT(inCameraDevice, inExposureTimeSec);
  //   ControlBinningT * controlBinning = new ControlBinningT(inCameraDevice, inBinning);
  //   AutoFindFocusT * autoFindFocus = new AutoFindFocusT(inCameraDevice, inFocuserDevice, controlExposure, & findFocusTask);
  //   FilterSelectT * filterSelect = 0;
  //   ControlCameraTemperatureT * controlCameraTemperature = 0;
    
  //   addMenuItem(& menuActions, focusStepSize);
  //   addMenuItem(& menuActions, new MoveFocusT(inFocuserDevice, focusStepSize));
  //   addMenuItem(& menuActions, new SeparatorT());
  //   addMenuItem(& menuActions, new SeparatorT());

  //   addMenuItem(& menuActions, controlExposure);
  //   addMenuItem(& menuActions, controlBinning);

  //   if (inCameraDevice->hasCooler()) {
  //     controlCameraTemperature = new ControlCameraTemperatureT(inCameraDevice, inCameraDevice->getTemperature());
  //     addMenuItem(& menuActions, controlCameraTemperature);
  //   }

  //   if (inFilterWheelDevice) {
  //     filterSelect = new FilterSelectT(inFilterWheelDevice, inFilterWheelDevice->getPos());
  //     addMenuItem(& menuActions, new SeparatorT());
  //     addMenuItem(& menuActions, new SeparatorT());
  //     addMenuItem(& menuActions, filterSelect);
  //   }
    
  //   addMenuItem(& menuActions, new SeparatorT());
  //   addMenuItem(& menuActions, new SeparatorT());
  //   addMenuItem(& menuActions, autoFindFocus);

    
  //   int position = 0;
  //   int numEntries = menuActions.size();

  //   LimitedQueueT<float> limitedQueueHfd(50);
  //   LimitedQueueT<float> limitedQueueFwhmHorz(50);
  //   LimitedQueueT<float> limitedQueueFwhmVert(50);
  
  //   //bool exposureInProgress = false;
  //   //bool newImage = false;
    
  //   while(key != 'q') {

  //     // if (inCameraDevice->isExposureInProgress()) {
  //     // 	exposureInProgress = true;
  //     // 	newImage = false;
  //     // } else {
  //     // 	// Get previous image (if any)
  //     // 	inCameraDevice->getImage(& image);

  //     // 	LOG(debug) << "exposureInProgress: " << exposureInProgress << ", newImage: " << newImage << endl;
	
  //     // 	if (exposureInProgress) {
  //     // 	  exposureInProgress = false;
  //     // 	  LOG(debug) << "SET newImage to true!" << endl;
  //     // 	  newImage = true;
  //     // 	}

  //     // 	LOG(trace) << "Received image [subframe: " << imageFrame << "] from camera - size: " << image.width() << " x " << image.height() << endl;
  //     // 	LOG(trace) << "image.width(): " << image.width() << ", image.height(): " << image.height() << endl;
  //     // 	LOG(trace) << "controlBinning->getBinning().get<0>(): " << controlBinning->getBinning().get<0>()
  //     // 		   << ", controlBinning->getBinning().get<1>(): " << controlBinning->getBinning().get<1>() << endl;

  //     // 	if (image.width() == imageFrame.get<2>() && image.height() == imageFrame.get<3>()) {
  //     // 	  // Perform re-centroiding i.e. re-set selectionFrame in order to follow the star
  //     // 	  // ------------------------------------------------------------------------------------------------------------ //
  //     // 	  FrameT<unsigned int> newFrame;

  //     // 	  // IDEA: Pass current image, re-center using given method
  //     // 	  PointT<float> assumedCenter((float) image.width() / 2.0, (float) image.height() / 2.0); // We assume that the star did not move too far from the image center

  //     // 	  LOG(trace) << "image.width(): " << image.width() << ", image.height(): " << image.height()
  //     // 		     << ", --> assumed center: " << assumedCenter << endl;

  //     // 	  // TODO: Do not pass StarFrameSelectorT::StarRecognitionTypeT::PROXIMITY,CentroidT::CentroidTypeT::IWC...
  //     // 	  //       Instead pass what has been selected by parameters..
  //     // 	  bool insideBounds = StarFrameSelectorT::calc(image, 0 /*bitPix - TODO / HACK: not needed */,
  //     // 						       assumedCenter, & newFrame,
  //     // 						       StarFrameSelectorT::StarRecognitionTypeT::PROXIMITY,
  //     // 						       CentroidT::CentroidTypeT::IWC, inSelectionFrame.get<2>() /*frameSize*/);

  //     // 	  AT_ASSERT(StarFrameSelector, insideBounds, "Expected frame to be inside bounds.");
 
  //     // 	  // Calculate new imageFrame
  //     // 	  PointT<float> newLocalCenterPos = frameToCenterPos(newFrame); // In "imageFrame" coordinates
  //     // 	  float deltaX = ((float) imageFrame.get<2>() / 2.0) - newLocalCenterPos.get<0>();
  //     // 	  float deltaY = ((float) imageFrame.get<3>() / 2.0) - newLocalCenterPos.get<1>();
  //     // 	  PointT<float> oldCenterPos = frameToCenterPos(imageFrame); // In "full frame" coordinates
  //     // 	  PointT<float> newCenterPos(oldCenterPos.get<0>() - deltaX, oldCenterPos.get<1>() - deltaY); // In "full frame" coordinates
  //     // 	  FrameT<unsigned int> newImageFrame = centerPosToFrame(newCenterPos, imageFrame.get<2>(), imageFrame.get<3>());

  //     // 	  LOG(debug) << "Old imageFrame: " << imageFrame << ", oldCenterPos: " << oldCenterPos << endl;
  //     // 	  LOG(trace) << "frameSize used: " << inSelectionFrame.get<2>() << ", assumedCenter: " << assumedCenter
  //     // 		     << ", newFrame: " << newFrame << " --> newLocalCenterPos: " << newLocalCenterPos
  //     // 		     << ", --> deltaX: " << deltaX << ", deltaY: " << deltaY << endl;
  //     // 	  LOG(debug) << "New imageFrame: " << newImageFrame << ", newCenterPos: " << newCenterPos << endl;

  //     // 	  // TODO / FIXME: Check if it hits boundaries!!??
	  
  //     // 	  // Get sub-frame for analysis
  //     // 	  // ------------------------------------------------------------------------------------------------------------ //
  //     // 	  // Convert to imageFrame coordinates
  //     // 	  FrameT<float> selectionFrame(newFrame);

  //     // 	  CImg<float> subImg = image.get_crop(selectionFrame.get<0>() /*x0*/,
  //     // 	   				      selectionFrame.get<1>() /*y0*/,
  //     // 	   				      selectionFrame.get<0>() + selectionFrame.get<2>() - 1/*x1=x0+w-1*/,
  //     // 	   				      selectionFrame.get<1>() + selectionFrame.get<3>() - 1/*y1=y0+h-1*/);

  //     // 	  // Calculate star data
  //     // 	  // ------------------------------------------------------------------------------------------------------------ //
  //     // 	  // TODO: try catch?
  //     // 	  try {
  //     // 	    // TODO: HFD value INCREASES if coming to focus using the simulator... !!! Maybe a simulator problem?! --> need real test!!!
  //     // 	    HfdT hfd(subImg); // NOTE: HfdT takes image center as centroid, it does not matter if image is bigger
  //     // 	    hfdValue = hfd.getValue();
  //     // 	    currentHfdDisp.display(hfd.genView());
  //     // 	  } catch(std::exception & exc) {
  //     // 	    LOG(warning) << "HFD calculation failed!"  << endl;
  //     // 	  }
	  
  //     // 	  // Subtract median image
  //     // 	  double med = subImg.median();
  //     // 	  CImg<float> imageSubMed(subImg.width(), subImg.height());
  //     // 	  cimg_forXY(subImg, x, y) {
  //     // 	    imageSubMed(x, y) = (subImg(x, y) > med ? subImg(x, y) - med : 0);
  //     // 	  }
	  
  //     // 	  try {
  //     // 	    FwhmT fwhmHorz(extractLine<DirectionT::HORZ>(imageSubMed));
  //     // 	    fwhmHorzValue = fwhmHorz.getValue();
  //     // 	    currentFwhmHorzDisp.display(fwhmHorz.genView());
  //     // 	  } catch(std::exception & exc) {
  //     // 	    LOG(warning) << "FWHM(horz) calculation failed!"  << endl;
  //     // 	  }
	  
  //     // 	  try {
  //     // 	    FwhmT fwhmVert(extractLine<DirectionT::VERT>(imageSubMed));
  //     // 	    fwhmVertValue = fwhmVert.getValue();
  //     // 	    currentFwhmVertDisp.display(fwhmVert.genView());
  //     // 	  } catch(std::exception & exc) {
  //     // 	    LOG(warning) << "FWHM(horz) calculation failed!"  << endl;
  //     // 	  }
	  
  //     // 	  maxPixelValue = subImg.max();

	  
  //     // 	  // DEBUG START
  //     // 	  // CImg<unsigned char> tmpNormalizedImage(normalize(subImg, 65535.0, 5.0));
  //     // 	  // CImgDisplay thDsp(tmpNormalizedImage, "SUB-IMG...");
  //     // 	  // while (! thDsp.is_closed()) {
  //     // 	  //   thDsp.wait();
  //     // 	  // }
  //     // 	  // CImg<unsigned char> hfdView = hfd.genView();
  //     // 	  // CImgDisplay thDsp2(hfdView, "HFD-VIEW...");
  //     // 	  // while (! thDsp2.is_closed()) {
  //     // 	  //   thDsp2.wait();
  //     // 	  // }
  //     // 	  // DEBUG END
	  
	  
  //     // 	  if (inFollowStar && controlExposure->getExposureMode() != ControlExposureT::ExposureModeT::SINGLE) {
  //     // 	    // If star following is enabled, the camera sub-frame is re-centered after each image
  //     // 	    imageFrame = newImageFrame;
  //     // 	  }
  //     // 	} else {
  //     // 	  LOG(debug) << "Received something unexpected! Expected image size: " << imageFrame.get<2>() << " x " << imageFrame.get<3>()
  //     // 		     << " but received " << image.width() << " x " << image.height() << " -> ignoring." << endl;
  //     // 	}
	
  //     // 	// TODO: Should this be moved to the menu items?!
  //     // 	LOG(trace) << "setBinning(" << controlBinning->getBinning() << ")..." << endl;
  //     // 	inCameraDevice->setBinning(controlBinning->getBinning());

  //     // 	//TODO: setBinning after first selection of imageFrame causes CentroidException (Frame hit simage bounds...) -> need to recalc imageFrame(?) - based on old binning and given new binning!
  //     // 	LOG(trace) << "setBinnedFrame(" << imageFrame << ", " << controlBinning->getBinning() << ")..." << endl;
  //     // 	inCameraDevice->setBinnedFrame(imageFrame, controlBinning->getBinning());
	
  //     // 	LOG(trace) << "setFrameType(" << FrameTypeT::asStr(FrameTypeT::LIGHT) << ")..." << endl;
  //     // 	inCameraDevice->setFrameType(FrameTypeT::LIGHT);
  //     // 	LOG(trace) << "setCompressed(false)..." << endl;
  //     // 	inCameraDevice->setCompressed(false); // No compression when moving image to CImg

  //     // 	// TODO: getExposureDone?? TODO: or restart exposure if auto finder is enabled
  //     // 	if (! controlExposure->getExposureDone() || controlExposure->getExposureMode() == ControlExposureT::ExposureModeT::LOOP) {
  //     // 	  LOG(trace) << "startExposure(" << inExposureTimeSec << "s)..." << endl;
  //     // 	  inCameraDevice->startExposure(controlExposure->getExposureTime()); // Non-blocking call...!!
  //     // 	  controlExposure->setExposureDone(true);
  //     // 	}
  //     // } // end if - ! exposure in progress

      
  //     // if (newImage) {
  //     // 	newImage = false;
	
  //     // 	if (hfdValue > 0) {
  //     // 	  stringstream hfdArcSecSs;
  //     // 	  if (haveFocalDistance && havePixelSize) {
  //     // 	    hfdArcSecSs << " = " << FwhmT::pxToArcsec(hfdValue, focalDistance, pixelSize, controlBinning->getBinning()) << "\"";
  //     // 	  }
  //     // 	  mv_print(infoColumn, 15, "HFD: %fpx%s", hfdValue, hfdArcSecSs.str().c_str());
	
  //     // 	  // Add values to queue
  //     // 	  limitedQueueHfd.push(hfdValue);
  //     // 	} else {
  //     // 	  mv_print(infoColumn, 15, "HFD: n.a.");
  //     // 	}
  //     // 	hfdHistoryDisp.display(limitedQueueHfd.genView(400, 100));
	
  //     // 	if (fwhmHorzValue > 0) {
  //     // 	  stringstream fwhmHorzArcSecSs;

  //     // 	  if (haveFocalDistance && havePixelSize) {
  //     // 	    fwhmHorzArcSecSs << " = " << FwhmT::pxToArcsec(fwhmHorzValue, focalDistance, pixelSize, controlBinning->getBinning()) << "\"";
  //     // 	  }
  //     // 	  // Add values to queue
  //     // 	  limitedQueueFwhmHorz.push(fwhmHorzValue);

  //     // 	  mv_print(infoColumn, 16, "FWHM(horz): %fpx%s", fwhmHorzValue, fwhmHorzArcSecSs.str().c_str());
  //     // 	} else {
  //     // 	  mv_print(infoColumn, 16, "FWHM(horz): n.a.");
  //     // 	}
  //     // 	fwhmHorzHistoryDisp.display(limitedQueueFwhmHorz.genView(400, 100));

	
  //     // 	if (fwhmVertValue > 0) {
  //     // 	  stringstream fwhmVertArcSecSs;

  //     // 	  if (haveFocalDistance && havePixelSize) {
  //     // 	    fwhmVertArcSecSs << " = " << FwhmT::pxToArcsec(fwhmVertValue, focalDistance, pixelSize, controlBinning->getBinning()) << "\"";
  //     // 	  }
  //     // 	  // Add values to queue
  //     // 	  limitedQueueFwhmVert.push(fwhmVertValue);

  //     // 	  mv_print(infoColumn, 17, "FWHM(vert): %fpx%s", fwhmVertValue, fwhmVertArcSecSs.str().c_str());
  //     // 	} else {
  //     // 	  mv_print(infoColumn, 17, "FWHM(vert): n.a.");
  //     // 	}
  //     // 	fwhmVertHistoryDisp.display(limitedQueueFwhmVert.genView(400, 100));

  //     // 	if (maxPixelValue > 0) {
  //     // 	  mv_print(infoColumn, 18, "MAX: %f", maxPixelValue);
  //     // 	} else {
  //     // 	  mv_print(infoColumn, 18, "MAX: n.a.");
  //     // 	}
  //     // } // end if - ! inCameraDevice->isExposureInProgress() && newImage

      
  //     mv_print(5, 25, "Focus finder. Press 'q' to quit");

  //     switch(key) {
  //     case KEY_UP:
  // 	while (position > 0) {
  // 	  position--;
  // 	  bool isSeparator =  ! strcmp(menuActions[position]->getName(), "");
  // 	  if (! isSeparator) { break; }
  // 	}
  // 	break;
	
  //     case KEY_DOWN:
  // 	while (position < numEntries - 1) {
  // 	  position++;
  // 	  bool isSeparator =  ! strcmp(menuActions[position]->getName(), "");
  // 	  if (! isSeparator) { break; }
  // 	}
  // 	break;
	
  //     case 9: /* TAB */
  // 	bool isSeparator;
  // 	do {
  // 	  position++;
  // 	  position = position % numEntries;
	  
  // 	  isSeparator =  ! strcmp(menuActions[position]->getName(), "");
  // 	} while (isSeparator);
  // 	break;

  //     default: {
  // 	mv_print(10, 21, "%s", menuActions[position]->getHelpText());
  // 	menuActions[position]->execute(key);
  // 	break;
  //     }
  //     } /* switch */

  //     const int cTopMenuBorder = 1;
  //     const int cLeftMenuBorder = 10;
  //     const int cLeftMenuWidth = 20;

  //     // Highlight selected menu entry
  //     for (vector<MenuEntryT *>::iterator it = menuActions.begin(); it != menuActions.end(); ++it) {
  // 	const MenuEntryT * menuEntry = *it;
  // 	int pos = (*it)->getMenuPos();

  // 	if (pos == position) { attron(A_STANDOUT); }
  // 	mv_print(cLeftMenuBorder, cTopMenuBorder + pos, "%s", menuEntry->getName());
  // 	if (pos == position) { attroff(A_STANDOUT); }

  // 	mv_print(cLeftMenuBorder + cLeftMenuWidth, cTopMenuBorder + pos, "%s", menuEntry->getValueAsStr().c_str());
  //     }



  //     // Update focus data
  //     stringstream ssCameraStatus;
  //     if (inCameraDevice->isExposureInProgress()) {
  // 	ssCameraStatus << "exposing..." << inCameraDevice->getExposureTime() << "s";
  //     } else {
  // 	ssCameraStatus << "ready";
  //     }

  //     mv_print(infoColumn, cTopMenuBorder + controlExposure->getMenuPos(), "Camera status: %s", ssCameraStatus.str().c_str());

  //     if (inCameraDevice->hasCooler()) {
  // 	size_t coolerPos = cTopMenuBorder + controlCameraTemperature->getMenuPos();
  // 	mv_print(infoColumn, coolerPos, "Cooler status: ");
  // 	mv_print(infoColumn, coolerPos + 1, "Temperature: ");
  //     }

  //     if (inFilterWheelDevice) {
  // 	stringstream ssFilterWheelStatus;
  // 	if (inFilterWheelDevice->isMovementInProgess()) {
  // 	  ssFilterWheelStatus << "moving... is: " << inFilterWheelDevice->getPos();
  // 	} else {
  // 	  ssFilterWheelStatus << "ready (pos=" << inFilterWheelDevice->getPos() << ").";
  // 	}
  // 	mv_print(infoColumn, cTopMenuBorder + filterSelect->getMenuPos(), "Filter status: %s", ssFilterWheelStatus.str().c_str());
  //     }

  //     string focuserStatus = (inFocuserDevice->isMovementInProgess() ? "busy" : "ready"); 
  //     size_t focusStepSizePos = cTopMenuBorder + focusStepSize->getMenuPos();

  //     mv_print(infoColumn, focusStepSizePos, "Focus status: %s", focuserStatus.c_str());
  //     mv_print(infoColumn, focusStepSizePos + 1, "Focus pos (is): %d", inFocuserDevice->getAbsPos());
  //     if (inFocuserDevice->supportsTemperature()) {
  // 	mv_print(infoColumn, focusStepSizePos + 2, "Focus temp: %d\0xf8 C", inFocuserDevice->getTemperature());
  //     }

  //     // Update auto finder status
  //     size_t autoFindFocusPos = cTopMenuBorder + autoFindFocus->getMenuPos();
  //     mv_print(infoColumn, autoFindFocusPos, "Status: %s", FocusFinderInfoT::StateT::asStr(findFocusTask.getStatus().mState));
  //     mv_print(infoColumn, autoFindFocusPos + 1, "Progress: %d", findFocusTask.getStatus().mProgress);

  //     refresh();
  //     timeout(10);
  //     key = getch();
  //   } /* while */

  //   // Cleanup menu
  //   for (vector<MenuEntryT *>::iterator it = menuActions.begin(); it != menuActions.end(); ++it) {
  //     delete *it;
  //   }
  //   endwin();    
  // }
}; // end AT
