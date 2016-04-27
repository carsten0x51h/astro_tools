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

#include "focuser_console_ui.hpp"
#include "centroid.hpp"
#include "fwhm.hpp"
#include "hfd.hpp"
#include "util.hpp"
#include "focus_finder_common.hpp"
#include "focus_finder_impl.hpp"

using namespace std;
using namespace boost;

namespace AT {


  void
  genSelectionView(const CImg<float> & inCurrSubImg, int dx, int dy,
		   CImg<unsigned char> * outRgbImg, float inScaleFactor = 3.0) {
    const unsigned char red[3] = { 255, 0, 0 }, green[3] = { 0, 255, 0 }, blue[3] = { 0, 0, 255 };
    const size_t cCrossSize = 3;

    CImg<unsigned char> normalizedImage(inCurrSubImg);
    if (inCurrSubImg.max() > 255) {
      normalizedImage.get_log10().normalize(0, 255);
    } else {
      normalizedImage.normalize(0, 255); // We do a log10() to stetch the image... maybe we need a better way...
    }

    
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
    PointT<float> starCenter(inCurrSubImg.width() / 2 + dx, inCurrSubImg.height() / 2 + dy);

    drawCross(& rgbImgRef, starCenter, red, cCrossSize, inScaleFactor);
    drawFrame(& rgbImgRef, getSelectionFrame(starCenter), red, inScaleFactor);
  }




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
    LOG(trace) << "setBinnedFrame(" << getImageFrame(cntl.centerPosFF) << ", " << cntl.binning << ")..." << endl;
    inCameraDevice->setBinnedFrame(getImageFrame(cntl.centerPosFF), cntl.binning);
	
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


  void
  FocusFinderConsoleCntlT::focusFinderStartHandler(const FocusFindCntlDataT * inFocusFinderCntlData) {
    lock_guard<mutex> guard(mFocusFinderMtx);
    mFwhmHorzFocusCurve->clear();
    mFwhmVertFocusCurve->clear();
  }
  
  //TODO: Move static variables if possible into class - same for FocusFinder event handlers!  
  // TODO: We may write this as a lambda function...  
  void
  FocusFinderConsoleCntlT::focusFinderAbortHandler(bool inManualAbort, string inCause) {
    if (! inManualAbort) {
      // Print error msg.
      lock_guard<mutex> guard(mFocusFinderMtx);
      mLastErrorStr = inCause;
    }
  }


  void
  FocusFinderConsoleCntlT::focusFinderStatusUpdHandler(const FocusFindStatusDataT * inFocusFindStatus) {
    lock_guard<mutex> guard(mFocusFinderMtx);
    mFocusFindStatus = *inFocusFindStatus; // Make a copy
    
    // Update UI
    if (! mFocusFindStatus.currImage.is_empty()) {
      CImg<unsigned char> rgbImg;
      genSelectionView(mFocusFindStatus.currImage, mFocusFindStatus.dx, mFocusFindStatus.dy, & rgbImg, 3.0);
      currImageDisp.display(rgbImg); // TODO - disable if no Wnds...
    }
  }

  void
  FocusFinderConsoleCntlT::focusFinderNewSampleHandler(const FocusCurveT * inFocusCurve, float inFocusPos, const CImg<float> & inImgFrame) {
    lock_guard<mutex> guard(mFocusFinderMtx);

    HfdT hfd(inImgFrame);
    if (hfd.valid()) {
      currentHfdDisp.display(hfd.genView());      
    }

    CImg<float> subMedImg = subMedianImg<float>(inImgFrame);
    
    FwhmT fwhmHorz(extractLine<DirectionT::HORZ>(subMedImg));
    if (fwhmHorz.valid()) {
      currentFwhmHorzDisp.display(fwhmHorz.genView());
    }

    FwhmT fwhmVert(extractLine<DirectionT::VERT>(subMedImg));
    if (fwhmVert.valid()) {
      currentFwhmVertDisp.display(fwhmVert.genView());
    }

    // Draw current dots (focus curve so far)
    currFocusCurveDisp.display(inFocusCurve->genView(600 /*width*/, 600 /*height*/, false /*do not draw interpolation lines*/));


    
    // Draw more curves
    mFwhmHorzFocusCurve->add(inFocusPos, inImgFrame);
    currFwhmHorzFocusCurveDisp.display(mFwhmHorzFocusCurve->genView(600 /*width*/, 600 /*height*/, false /*do not draw interpolation lines*/));

    mFwhmVertFocusCurve->add(inFocusPos, inImgFrame);
    currFwhmVertFocusCurveDisp.display(mFwhmVertFocusCurve->genView(600 /*width*/, 600 /*height*/, false /*do not draw interpolation lines*/));
  }

  void
  FocusFinderConsoleCntlT::focusFinderNewFocusCurveHandler(const FocusCurveT * inFocusCurve, const PosToImgMapT * inPosToImgMap, const PointT<float> * inSp, const LineT<float> * inLine1, const LineT<float> * inLine2) {
    // TODO:... lines and point should not be passed sep. here... either part of FocusCurve OR calculated in a sep "Interpolation" class...
    lock_guard<mutex> guard(mFocusFinderMtx);
    currFocusCurveDisp.display(inFocusCurve->genView(600 /*width*/, 600 /*height*/, true, inLine1, inLine2));

    // Clear others curves... TODO...
    mFwhmHorzFocusCurve->clear();
    mFwhmVertFocusCurve->clear();
  }

  
  FocusFinderConsoleCntlT::FocusFinderConsoleCntlT(const po::variables_map & inCmdLineMap, IndiCameraT * inCameraDevice, IndiFocuserT * inFocuserDevice, IndiFilterWheelT * inFilterWheelDevice, const FrameT<unsigned int>  & inSelectionFrameFF /*TODO: Will probably be removed...*/) : mCmdLineMap(inCmdLineMap), mCameraDevice(inCameraDevice), mFocuserDevice(inFocuserDevice), mFilterWheelDevice(inFilterWheelDevice), mFocalDistance(0), mPixelSizeUm(DimensionT<float>(0,0)) {

    mFocusFinderImpl =
      new FocusFinderImplT(mCameraDevice, inFocuserDevice, inFilterWheelDevice,
			   FocusCurveT::sHfdStrategy,
			   FocusFinderImplT::sHfdLimitStrategy);

    // Addirional curves... TODO: ok?
    mFwhmHorzFocusCurve = new FocusCurveT(FocusCurveT::sFwhmHorzStrategy);
    mFwhmVertFocusCurve = new FocusCurveT(FocusCurveT::sFwhmVertStrategy);
    
    
    // Register focus finder listeners
    mFocusFinderStartHandlerConn = mFocusFinderImpl->registerFocusFinderStartListener(boost::bind(& FocusFinderConsoleCntlT::focusFinderStartHandler, this, _1));
    mFocusFinderStatusUpdHandlerConn = mFocusFinderImpl->registerStatusUpdListener(boost::bind(& FocusFinderConsoleCntlT::focusFinderStatusUpdHandler, this, _1));
    mFocusFinderNewSampleHandlerConn = mFocusFinderImpl->registerNewSampleListener(boost::bind(& FocusFinderConsoleCntlT::focusFinderNewSampleHandler, this, _1, _2, _3));
    mFocusFinderNewFocusCurveHandlerConn = mFocusFinderImpl->registerNewFocusCurveListener(boost::bind(& FocusFinderConsoleCntlT::focusFinderNewFocusCurveHandler, this, _1,_2,_3,_4,_5));
    mFocusFinderAbortHandlerConn = mFocusFinderImpl->registerFocusFinderAbortListener(boost::bind(& FocusFinderConsoleCntlT::focusFinderAbortHandler, this, _1,_2));

    mFocusFinderImpl->setRecordBaseDir(inCmdLineMap["seq_record_dir"].as<string>());
    mFocalDistance = (inCmdLineMap.count("focal_distance") ? inCmdLineMap["focal_distance"].as<unsigned int>() : 0);
    mPixelSizeUm = (inCmdLineMap.count("pixel_size") ? inCmdLineMap["pixel_size"].as<DimensionT<float> >() : 0);

    bool imgFrameRecenter = inCmdLineMap["img_frame_recenter"].as<bool>();

    // Set the initial image frame
    // NOTE:
    //   FF = Full Frame coordinates
    //   IF = Image Frame coordinates
    mCurrCenterPosFF = frameToCenterPos(inSelectionFrameFF);

    
    // Build the menu
    mExpTimeVal = mCmdLineMap["exposure_time"].as<float>();
    mMenuEntries.push_back(new MenuFieldT<float>(& mExpTimeVal, "Exposure time: ", 1 /* steps */, StepModeT::LINEAR,
						 mCameraDevice->getMinExposureTime() /* min */, mCameraDevice->getMaxExposureTime() /*max*/,
						 [](float * inValPtr) {
						   // Seconds to minutes and seconds
						   int sec = ((int)(*inValPtr)) % 60;
						   int min = (int) ((float) (*inValPtr) / 60.0);
						   stringstream ss;
						   ss << *inValPtr << "s = (" << setfill('0') << setw(2) << min << "m"
						      << setfill('0') << setw(2) << sec << "s)";
						   return ss.str();
						 })
			   );


    BinningT maxBinning = mCameraDevice->getMaxBinning();
    BinningT minBinning = mCameraDevice->getMinBinning();

    BinningT curBinning = inCmdLineMap["binning"].as<BinningT>();
    mBinValXY = curBinning.get<0>(); // NOTE: Assume x=y
    mMenuEntries.push_back(new MenuFieldT<int>(& mBinValXY, "Binning: ", 1 /* steps */, StepModeT::LINEAR,
					       minBinning.get<0>() /* min - NOTE: assuming that x=y */, maxBinning.get<0>() /*NOTE: assuming that x=y*/,
					       [](int * inValPtr) { return std::to_string(*inValPtr) +  "x" + std::to_string(*inValPtr); })
			   );


    // Check if there is actually a filter ...
    if (mFilterWheelDevice) {
    mFilterSelect = mFilterWheelDevice->getPos();
    mMenuEntries.push_back(new MenuFieldT<int>(& mFilterSelect, "Filter: ", 1 /* steps */, StepModeT::LINEAR,
					       mFilterWheelDevice->getMinPos() /* min */, mFilterWheelDevice->getMaxPos() /*max*/,
					       [](int * inValPtr) { return std::to_string(*inValPtr) +  " (TODO - query...)"; },
					       [&](int * inValPtr) {
						 mFilterWheelDevice->setPos(*inValPtr, 0 /* do not block */);
					       },
					       [&]() {
						 /*ABORT handler*/
						 // No abort supported by INDI Atik Wheel driver.
					       })
			   );
    } else {
      mMenuEntries.push_back(new MenuSeparatorT());
    }


    // Camera cooler
    mCameraCoolerState = (mCameraDevice->hasCooler() && mCameraDevice->isCoolerEnabled() ? CoolerStateT::ENABLED : CoolerStateT::DISABLED);
    mCameraTemperature = (mCameraDevice->hasCooler() ? mCameraDevice->getTemperature() : 0);

    
    if (mCameraDevice->hasCooler()) {
      mMenuEntries.push_back(new MenuSelectT<CoolerStateT>(& mCameraCoolerState, "Cooler state: ",
							   [](CoolerStateT::TypeE * inValPtr) { return CoolerStateT::asStr(*inValPtr); },
							   [&](CoolerStateT::TypeE * inValPtr) {
							     if (*inValPtr == CoolerStateT::ENABLED) {
							       // Enable / disable cooler - TODO: Handle busy state?
							       // NOTE: setTemperature directly enables the cooler
							       mCameraDevice->setTemperature(mCameraTemperature, 0); // 0 = do not block
							       mCameraDevice->setCoolerEnabled(true);
							     } else {
							       mCameraDevice->setCoolerEnabled(false);
							     }
							   },
							   [&]() {
							     /*ABORT handler*/
							     mCameraDevice->setCoolerEnabled(false);
							   })
			     );
			     
    
    mMenuEntries.push_back(new MenuFieldT<int>(& mCameraTemperature, "Temperature: ", 1 /* steps */, StepModeT::LINEAR,
					       mCameraDevice->getMinTemperature() /* min */, mCameraDevice->getMaxTemperature() /*max*/,
					       [](int * inValPtr) {
						 stringstream ssTemp;
						 ssTemp << *inValPtr << " degree celsius";
						 return ssTemp.str();
					       },
					       [&](int * inValPtr) {
						 // UPDATE handler
						 if (mCameraCoolerState == CoolerStateT::ENABLED) {
						   mCameraDevice->setTemperature(mCameraTemperature, 0); // 0 = do not block
						 }
					       },
					       [&]() {
						 // ABORT handler - ESC
						 // ...
					       })
			   );
    }

  
    mExpMode = ExposureModeT::SINGLE;
    mMenuEntries.push_back(new MenuSelectT<ExposureModeT>(& mExpMode, "Exposure mode: ",
							  [](ExposureModeT::TypeE * inValPtr) {
							    return ExposureModeT::asStr(*inValPtr);
							  })
			   );

    mExpStartAbort = StartAbortT::START;
    mMenuEntries.push_back(new MenuSelectT<StartAbortT>(& mExpStartAbort, "Exposure: ",
							[](StartAbortT::TypeE * inValPtr) { return StartAbortT::asStr(*inValPtr); },
							[&](StartAbortT::TypeE * inValPtr) {
							  // XXX
							},
							[&]() {
							  /*ABORT handler*/
							  TaskStateT::TypeE taskState = mExposureStatus.load().taskState;
							  if (taskState == TaskStateT::RUNNING) {
							    mExposureThread.interrupt();
							    // TODO: Set
							    // filterSelect = INDI current value...
							  }
							})
			   );

    mMenuEntries.push_back(new MenuSeparatorT());


    mFocusStepSize = 100;
    mAbsFocusPosDest = mFocuserDevice->getAbsPos();
    mFocusDelta = 0;
    
    mAbsFocusPosDestMenuField =
      new MenuFieldT<int>(& mAbsFocusPosDest, "Abs focus pos: ", mFocusStepSize /* steps */, StepModeT::LINEAR,
			  mFocuserDevice->getMinPos() /* min */, mFocuserDevice->getMaxPos() /*max*/,
			  [](int * inValPtr) { return std::to_string(*inValPtr) + " steps"; },
			  [&](int * inValPtr) {
			    mFocusDelta = fabs(mFocuserDevice->getAbsPos() - *inValPtr);
			    mFocuserDevice->setAbsPos(*inValPtr, 0); // 0 = do not block
			  },
			  [&]() {
			    /*ABORT handler*/
			    mFocuserDevice->abortMotion();
			  });
    
    mMenuEntries.push_back(new MenuFieldT<int>(& mFocusStepSize, "Focus step size: ", 10 /* steps */, StepModeT::FACTOR,
					       mFocuserDevice->getMinPos() /* min */, mFocuserDevice->getMaxPos() / 10 /* max/10 */,
					       [](int * inValPtr) { return std::to_string(*inValPtr) + " steps"; },
					       [&](int * inValPtr) {
						 // CHANGE HANDLER
						 mAbsFocusPosDestMenuField->setSteps(mFocusStepSize);
					       })
			   );
    
    mMenuEntries.push_back(mAbsFocusPosDestMenuField);

    mMenuEntries.push_back(new MenuSeparatorT());

    
    mFocusFindStartAbort = StartAbortT::START;
    mMenuEntries.push_back(new MenuSelectT<StartAbortT>(& mFocusFindStartAbort, "Focus find: ",
							[](StartAbortT::TypeE * inValPtr) { return StartAbortT::asStr(*inValPtr); },
							[&](StartAbortT::TypeE * inValPtr) {
							  //bool running = mFocusFindStatus.load().isRunning;
							  lock_guard<mutex> guard(mFocusFinderMtx);
							  bool running = mFocusFindStatus.isRunning;
							  
							  if (*inValPtr == StartAbortT::ABORT) {
							    // Start thread if not yet active.
							    if (! running) {
							      // TODO: Pass those to the class before starting.... constructor / set?
							      FocusFindCntlDataT focusFindCntlData;
							      focusFindCntlData.binning = BinningT(mBinValXY, mBinValXY);
							      focusFindCntlData.exposureTime = mExpTimeVal;
							      focusFindCntlData.centerPosFF = mCurrCenterPosFF;
							      focusFindCntlData.imgFrameRecenter = imgFrameRecenter;
							      
							      mFocusFinderImpl->setCntlData(focusFindCntlData);
							      
							      // TODO: Still required? & focusFindCntl, & focusFindStatus...??
							      mFocusFindThread = thread(boost::bind(& FocusFinderImplT::run, boost::ref(*mFocusFinderImpl)));
							      // Just make sure that status is updated asap...
							      // might not be required later when we have a class...
							      mFocusFindStatus.isRunning = true;
							    }
							    
							  } else {
							    // Abort - only interrupt if still running
							    if (running) {
							      mFocusFindThread.interrupt();
							    }
							  }
							},
							[&]() {
							  /*ABORT handler*/
							  lock_guard<mutex> guard(mFocusFinderMtx);
							  if (mFocusFindStatus.isRunning) {
							    mFocusFindThread.interrupt();
							  }
							})
			   );
    
    
    // Finally, create the menu
    mConsoleMenu = ConsoleMenuT(& mConsoleDisplay, & mMenuEntries);
    LOG(info) << "MENU CREATED..." << endl;
  }

  FocusFinderConsoleCntlT::~FocusFinderConsoleCntlT() {
    // Delete menu entries
    for (std::vector<MenuEntryT *>::iterator it = mMenuEntries.begin(); it != mMenuEntries.end(); ++it) {
      delete *it;
    }
    
    // Unregistering console UI listeners
    LOG(debug) << "Unregistering focus finder handler..." << endl;
    lock_guard<mutex> guard(mFocusFinderMtx);
    mFocusFinderImpl->unregisterFocusFinderStartListener(mFocusFinderStartHandlerConn);
    mFocusFinderImpl->unregisterStatusUpdListener(mFocusFinderStatusUpdHandlerConn);
    mFocusFinderImpl->unregisterNewSampleListener(mFocusFinderNewSampleHandlerConn);
    mFocusFinderImpl->unregisterNewFocusCurveListener(mFocusFinderNewFocusCurveHandlerConn);
    mFocusFinderImpl->unregisterFocusFinderAbortListener(mFocusFinderAbortHandlerConn);

    delete mFocusFinderImpl;

    delete mFwhmHorzFocusCurve;
    delete mFwhmVertFocusCurve;
  }

  
  void
  FocusFinderConsoleCntlT::show() {

    while (! mConsoleMenu.wantExit()) {

      // Handle menu
      mConsoleMenu.update();

      ////////////////////////////
      // Handle filter selction //
      ////////////////////////////
      if (mFilterWheelDevice) {
	stringstream ssFilterWheelStatus;
	if (mFilterWheelDevice->isMovementInProgess()) {
	  ssFilterWheelStatus << "moving... is: " << mFilterWheelDevice->getPos();
	} else {
	  ssFilterWheelStatus << "ready (pos=" << mFilterWheelDevice->getPos() << ").";
	}
	mConsoleDisplay.print(ConsoleMenuT::cLeftMenuBorder, 17, "Filter status: %s", ssFilterWheelStatus.str().c_str());
      }


      ////////////////////////////
      // Handle focus           //
      ////////////////////////////
      if (mFocuserDevice->isMovementInProgess()) {
	int progress = 100 * (1.0 - ((float)((int) mFocuserDevice->getAbsPos() - (int) mAbsFocusPosDest) / mFocusDelta));
	mConsoleDisplay.print(ConsoleMenuT::cLeftMenuBorder, 15, "Moving focus... pos: %d/%d - progress: %d%\n", mFocuserDevice->getAbsPos(), mAbsFocusPosDest, progress);
      } else {
	mConsoleDisplay.print(ConsoleMenuT::cLeftMenuBorder, 15, "Focus ready - pos: %d\n", mFocuserDevice->getAbsPos());
      }

      if (mFocuserDevice->supportsTemperature()) {
  	mConsoleDisplay.print(ConsoleMenuT::cLeftMenuBorder, 16, "Focus temp: %d\0xf8 C", mFocuserDevice->getTemperature());
      }
      
            
      //////////////////////////////////////
      // Handle camera temperature...     //
      //////////////////////////////////////
      if (mCameraDevice->hasCooler()) {
	// TODO: (mCameraDevice->isTemperatureReached() ? "yes" : "no") caused segfault?!
      	mConsoleDisplay.print(ConsoleMenuT::cLeftMenuBorder, 18, "Camera cooler state: %s - Dest temp: %d C, Is temp: %f C\n",
			      CoolerStateT::asStr(mCameraCoolerState), mCameraTemperature, (float) mCameraDevice->getTemperature());
      }
      
      
      ////////////////////////////
      // Handle exposure...     //
      ////////////////////////////
      ExposureStatusDataT exposureStatusData = mExposureStatus.load();
      
      switch(exposureStatusData.taskState) {
      case TaskStateT::READY:
	if (mExpStartAbort == StartAbortT::ABORT) {
	  ExposureCntlDataT cntlData;
	  cntlData.exposureTime = mExpTimeVal;
	  cntlData.centerPosFF = mCurrCenterPosFF;
	  cntlData.binning = BinningT(mBinValXY, mBinValXY);
	  mExposureCntl.store(cntlData);

	  // Thread ONLY gets image - post processing does not take place in thread!
      	  mExposureThread = thread(exposeTask, mCameraDevice, & mExposureCntl, & mExposureStatus, & mCurrSubImage);
	}
	break;
      
      case TaskStateT::RUNNING:
	// Abort - only interrupt if still running
	if (mExpStartAbort == StartAbortT::START) {
	  mExposureThread.interrupt();
	} else {
	  mConsoleDisplay.print(ConsoleMenuT::cLeftMenuBorder, 13, "Exposure progress: %d%, seconds left: %d\n", exposureStatusData.progress, exposureStatusData.secondsLeft);
	}
	break;

      case TaskStateT::FINISHED:
      case TaskStateT::ABORTED:
	if (exposureStatusData.taskState == TaskStateT::FINISHED) {

	  int dx = 0, dy = 0;
	  HfdT hfd;
	  FwhmT fwhmHorz, fwhmVert;
	  int maxPixValue = 0;
	  
	  try {
	    calcStarValues(mCurrSubImage, & dx, & dy, & hfd, & fwhmHorz, & fwhmVert, & maxPixValue);
	    bool imgFrameRecenter = mCmdLineMap["img_frame_recenter"].as<bool>();

	    if (imgFrameRecenter) {
	      mCurrCenterPosFF.get<0>() += dx;
	      mCurrCenterPosFF.get<1>() += dy;
	    }
	    
	    mConsoleDisplay.print(ConsoleMenuT::cLeftMenuBorder, 19, "HFD: %f\n", hfd.getValue());
	    currentHfdDisp.display(hfd.genView());

	    // FWHM horz
	    if (fwhmHorz.valid()) {
	      stringstream fwhmHorzArcSecSs;
	      if (mFocalDistance && mPixelSizeUm != DimensionT<float>(0,0)) {
		fwhmHorzArcSecSs << " = " << FwhmT::pxToArcsec(fwhmHorz.getValue(), mFocalDistance, mPixelSizeUm, BinningT(mBinValXY, mBinValXY)) << "\"";
	      }
	      mConsoleDisplay.print(ConsoleMenuT::cLeftMenuBorder, 20, "FWHM(horz): %fpx%s\n", fwhmHorz.getValue(), fwhmHorzArcSecSs.str().c_str());
	      currentFwhmHorzDisp.display(fwhmHorz.genView());
	    } else {
	      mConsoleDisplay.print(ConsoleMenuT::cLeftMenuBorder, 20, "FWHM(horz): n.a.");
	    }
	    
	    // FWHM vert
	    if (fwhmVert.valid()) {
	      stringstream fwhmVertArcSecSs;
	      if (mFocalDistance && mPixelSizeUm != DimensionT<float>(0,0)) {
		fwhmVertArcSecSs << " = " << FwhmT::pxToArcsec(fwhmVert.getValue(), mFocalDistance, mPixelSizeUm, BinningT(mBinValXY, mBinValXY)) << "\"";
	      }
	      mConsoleDisplay.print(ConsoleMenuT::cLeftMenuBorder, 21, "FWHM(vert): %fpx%s\n", fwhmVert.getValue(), fwhmVertArcSecSs.str().c_str());
	      currentFwhmVertDisp.display(fwhmVert.genView());
	    } else {
	      mConsoleDisplay.print(ConsoleMenuT::cLeftMenuBorder, 21, "FWHM(vert): n.a.");
	    }

	    // TODO: Also calc saturation?
	    mConsoleDisplay.print(ConsoleMenuT::cLeftMenuBorder, 22, "Max pix val: %d\n", maxPixValue);

	  } catch(CentroidExceptionT & exc) {
	    // Unable to calculate star values - probably no star...
	    LOG(info) << "Unable to calculate star data... probably no valid star selected" << endl; 
	  }
	  
	  // Finally, generate and display the image
	  CImg<unsigned char> rgbImg;
	  genSelectionView(mCurrSubImage, dx, dy, & rgbImg, 3.0 /*zoom*/);
	  currImageDisp.display(rgbImg);
	}

	if (mExpMode == ExposureModeT::SINGLE) {
	  mExpStartAbort = StartAbortT::START;
	}
	exposureStatusData.taskState = TaskStateT::READY;
	mExposureStatus.store(exposureStatusData);
	break;

      default:
	break;
      }

      ////////////////////////////
      // Handle focus finder    //
      ////////////////////////////
      if (mFocusFindStatus.isRunning) {
      	mConsoleDisplay.print(ConsoleMenuT::cLeftMenuBorder, 14, "Focus finder progress: %d\n",
      			     mFocusFindStatus.progress);
      } else {
      	mFocusFindStartAbort = StartAbortT::START;
      }

      // Print error in focus finder, if any
      {
	lock_guard<mutex> guard(mFocusFinderMtx);
	if (! mLastErrorStr.empty()) {
	  mConsoleDisplay.print(ConsoleMenuT::cLeftMenuBorder, 23, "Last error: %s", mLastErrorStr.c_str());
	}
      }
      
      // Wait a moment to keep processor down...
      chrono::milliseconds dura(10);
      this_thread::sleep_for(dura);
    } // end loop

  }
	  

}; // end AT
