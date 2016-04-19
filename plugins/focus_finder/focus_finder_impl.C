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

#include "focus_finder_impl.hpp"

// TODO: REMOVE
#include <iostream>
// TODO: REMOVE
using namespace std;

using namespace boost;



namespace AT {

  // template<typename T>
  // class PointLstAccessorT {
  // public:
  //   typedef PointLstT<T> TypeT;
  //   static DataPointT getDataPoint(size_t inIdx, typename TypeT::const_iterator inIt) {
  //     DataPointT dp(inIt->get<0>(), inIt->get<1>());
  //     return dp;
  //   }
  // };

  
  DEF_Exception(FocusFinderImpl);

  // TODO: Maybe as template... then supply e.g. HfdT as T...
  // Then: calcStarValues -> calcStarValue as template as well...
  void
  FocusFinderImplT::recordSequence(FocusCurveT * outHfdFocusCurve, int stepSize, float hfdLimit) {
    LOG(info) << "recordSequence - stepSize: " << stepSize << ", hfdLimit: " << hfdLimit << endl;

    mFocusFindStatusData.isRunning = true;
    callStatusUpdListener(& mFocusFindStatusData);

    PointT<float> currCenterPosFF = mCntlData.centerPosFF;
    int focusStartPos = mFocuserDevice->getAbsPos();
    
    // Take a picture and determine current HFD...
    CImg<float> currSubImage;
    mCameraDevice->takePicture(& currSubImage, mCntlData.exposureTime, getImageFrame(currCenterPosFF),
			       FrameTypeT::LIGHT, mCntlData.binning, false /*compressed*/);
    HfdT hfd;
    int dx = 0, dy = 0;
    calcStarValues(currSubImage, & dx, & dy, & hfd);

    if (mCntlData.wantRecenter) {
      currCenterPosFF.get<0>() += dx;
      currCenterPosFF.get<1>() += dy;
    }

    // Notify listeners of new sample
    callNewSampleListener(outHfdFocusCurve);

    
    // TODO: Save image data - status......
    // Update status...
    mFocusFindStatusData.currAbsFocusPos = mFocuserDevice->getAbsPos();
    mFocusFindStatusData.currCenterPosFF = currCenterPosFF;
    mFocusFindStatusData.dx = dx;
    mFocusFindStatusData.dy = dy;
    mFocusFindStatusData.currImage = currSubImage; // Copy
    mFocusFindStatusData.progress = 0;
    mFocusFindStatusData.hfd = hfd; // Copy
    
    callStatusUpdListener(& mFocusFindStatusData);
    
    // Do "numSteps" steps in both directions
    int currAbsFocusDestPos = focusStartPos;
    int sign[2] = { -1, 1 };

    for (size_t i = 0; i < 2; ++i) {
      FocusCurveT::SegmentT & curveSegment = outHfdFocusCurve->getSegment(i);
	
      for (size_t step = 0; step < mNumStepsRough; ++step) {
  	// Move focus
  	currAbsFocusDestPos += sign[i] * stepSize;

	LOG(info) << "recordSequence - Setting new dest position: " << currAbsFocusDestPos << endl;

	//TODO: For some reason WAIT_MAX_FOR(isMovementInProgess(), inTimeoutMs, "Hit timeout while waiting for focuser reaching position.");
	//in setAbsPos() does not get TRUE... If I write a loop here which directly uses 	isMovementInProgess() it works!...


	mFocuserDevice->setAbsPos(currAbsFocusDestPos); // Wait for focus to be moved - TODO!! Check if the function blocks by default!!!
	
  	// TODO: CHECK if setAbsPos is BLOCKING!! Does not seem to be the case!!!!! -> WAIT in setAbsPos is commented out...
				 
  	// Take picture etc.
  	mCameraDevice->takePicture(& currSubImage, mCntlData.exposureTime, getImageFrame(currCenterPosFF),
  				    FrameTypeT::LIGHT, mCntlData.binning, false /*compressed*/);
  	calcStarValues(currSubImage, & dx, & dy, & hfd);

  	if (mCntlData.wantRecenter) {
  	  currCenterPosFF.get<0>() += dx;
  	  currCenterPosFF.get<1>() += dy;
  	}
	
	// Notify listeners of new sample
	callNewSampleListener(outHfdFocusCurve);

	
  	// Update status... - TODO: We may put lines below into a function or a macro...
  	mFocusFindStatusData.currAbsFocusPos = mFocuserDevice->getAbsPos();
  	mFocusFindStatusData.currCenterPosFF = currCenterPosFF;
	mFocusFindStatusData.currImage = currSubImage; // Copy
	mFocusFindStatusData.dx = dx;
	mFocusFindStatusData.dy = dy;
	mFocusFindStatusData.progress = 100.0 * ((float) step / (mNumStepsRough - 1)); // TODO: Need total progress as well...
  	mFocusFindStatusData.hfd = hfd;
	callStatusUpdListener(& mFocusFindStatusData);

  	curveSegment[mFocuserDevice->getAbsPos()] = hfd.getValue();

	
	boost::this_thread::interruption_point();

  	// Break move if HFD limit is reached (only if HFD limit is defined)
  	if (hfdLimit && hfd.getValue() >= hfdLimit) {
  	  LOG(info) << "HFD limit ("<< hfdLimit << ") reached: " << hfd.getValue() << ". Stop." << endl;
  	  break;
  	}	
      } // inner for
      
      // Back to start pos for second curve part...
      currAbsFocusDestPos = focusStartPos;
      mFocuserDevice->setAbsPos(focusStartPos);
    }

    // Finally go back to startPos - recordSequence should return
    // focuser to previous position...
    mFocuserDevice->setAbsPos(focusStartPos);
  }


  
  void
  FocusFinderImplT::run() {
    AT_ASSERT(FocusFinderImpl, mCntlData.valid(), "No valid cntl data!");

    // Set running...
    mFocusFindStatusData.isRunning = true;
    callStatusUpdListener(& mFocusFindStatusData);

    try {
      PointT<float> currCenterPosFF = mCntlData.centerPosFF;
      int stepSize = mCntlData.stepSize;
    
      // If no stepSize is configured, determine it
      // NOTE: This step is also REQUIRED to roughly center the focus pos!
      //       So we will porbably have to do it all the time... or if a "centerFocus" option is set...
      if (! stepSize) {
    	// Determine initial boundaries - use current position (rough focus and min-max values of focuser)
    	int focusStartPos = mFocuserDevice->getAbsPos();
    	int deltaL = fabs(mFocuserDevice->getMinPos() - focusStartPos);
    	int deltaR = fabs(mFocuserDevice->getMaxPos() - focusStartPos);

    	stepSize = std::min(deltaL, deltaR) / mNumStepsRough;

    	LOG(debug) << "focusStartPos: " << focusStartPos << endl;
    	LOG(debug) << "MaxFocusPos: " << mFocuserDevice->getMaxPos() << endl;
    	LOG(debug) << "MinFocusPos: " << mFocuserDevice->getMinPos() << endl;
    	LOG(debug) << "deltaL: " << deltaL << ", deltaR: " << deltaR << endl;
    	LOG(info) << "stepSize: " << stepSize << endl;

    	// Take one shot to determine hfd
    	CImg<float> currSubImage;
    	mCameraDevice->takePicture(& currSubImage, mCntlData.exposureTime, getImageFrame(currCenterPosFF), FrameTypeT::LIGHT, mCntlData.binning, false /*compressed*/);

    	HfdT hfd;
    	int dx = 0, dy = 0;
    	calcStarValues(currSubImage, & dx, & dy, & hfd);
      
    	// Determine HFD limit = N * initial HFD (~in focus) 
    	float hfdLimit = mHfdLimitFactor * hfd.getValue();
    	LOG(info) << "HFD limit = " << mHfdLimitFactor << " * " << hfd.getValue() << " = " << hfdLimit << endl;
      
    	FocusCurveT recordedFocusCurve;
    	LineT<float> line1, line2;
    	recordSequence(& recordedFocusCurve, stepSize, hfdLimit);
	
    	PointT<float> sp = recordedFocusCurve.calcOptFocusPos(LineFitTypeT::OLS /*LineFitTypeT::BISQUARE - sometimes does not converge!?*/, & line1, & line2);
    	LOG(info) << "Result of calcOptFocusPos: " << sp << endl;

	callFocusDeterminedListener(& recordedFocusCurve, & sp, & line1, & line2);
	
	
    	// TODO: Move display stuff out of here!!
    	// Draw data points with lines
    	//genCurve(recordedSequence, & hfdCurveRgbImg, & line1, & line2, green, cCrossSize);
    	//currHfdCurveDisp.display(hfdCurveRgbImg);

	// Notify listeners of status update (TODO: Required here?)
	mFocusFindStatusData.progress = 100; // TODO: CALC!! Update furthervallues?
	callStatusUpdListener(& mFocusFindStatusData);
	
    	// Now calculate the (new) step-size - we again want 10 steps in each direction.
    	// -> 1. Calculate, L and R  -> interpolate using determined slope
    	float leftLimitPos = line1.f_inv(hfdLimit);
    	float rightLimitPos = line2.f_inv(hfdLimit);
    	float leftLength = fabs(leftLimitPos - sp.get<0>());
    	float rightLength = fabs(rightLimitPos - sp.get<0>());
    	float meanLength = (leftLength + rightLength) / 2.0;
      
    	LOG(debug) << "leftLimitPos: " << leftLimitPos << ", rightLimitPos: " << rightLimitPos << ", SPx: " << sp.get<0>() << endl;
    	LOG(debug) << "-> leftLength: " << leftLength << ", rightLength: " << rightLength << " ---> meanLength: " << meanLength << endl;
      
    	stepSize = meanLength / mNumStepsFine;
      
    	LOG(info) << "DETERMINED STEP SIZE FOR FINE CURVE RECORDING: " << stepSize << " (" << mNumStepsFine << " steps per direction)." << endl;

    	// Move focus to roughly determine start position
    	mFocuserDevice->setAbsPos(sp.get<0>());
    	LOG(info) << "INITIALLY CENTERED FOCUS POS AT: " << sp.get<0>() << endl;
      
    	// TODO: ODER BESSER?? Einfach die Position mit dem kleinsten Messewert nehmen?? ->Besser als interpolation bzw. Schnittpunkt???
      }



      // // Now, record M "fine" curves...
      // LOG(info) << "Now, recording " << mNumCurvesToRecord << " curves. stepSize = " << stepSize << " (" << mNumStepsFine << " steps per direction)." << endl;

      // FocusCurveT recordedSequences[mNumCurvesToRecord];
      // PointT<float> sps[mNumCurvesToRecord];
    
      // for (size_t m = 0; m < mNumCurvesToRecord; ++m) {
      // 	LOG(info) << "Recording fine sequence " << (m + 1) << "/" << mNumCurvesToRecord  << "..." << endl;

      // 	LineT<float> line1, line2;
      // 	// TODO: ENABLE!
      // 	//recordSequence(cntl, status, & recordedSequences[m], stepSize, 0 /* this time no hfd limit */);
      // 	//sps[m] = calcOptFocusPos(recordedSequences[m], LineFitTypeT::OLS /*LineFitTypeT::BISQUARE - sometimes does not converge!?*/, & line1, & line2);

      // 	//genCurve(recordedSequences[m], & hfdCurveRgbImg, & line1, & line2, green, cCrossSize);
      // 	//currHfdCurveDisp.display(hfdCurveRgbImg);
      
      // 	//LOG(info) << "SP[" << m << "] = " << sps[m] << endl;

      // 	// TODO: Send a status update (call handlers) - new sequence (with line...)  -> different / additional handler/listener?
      // 	//   mFocusFindStatusData.progress = 100.0 * ((float) i / 10.0);
      // 	//   callStatusUpdListener(& mFocusFindStatusData);
      // }

      // // Calculate mean position
      // // TODO: Here are multiple ways to do so...
      // float sumX = 0;
      // for (size_t m = 0; m < mNumCurvesToRecord; ++m) {
      // 	sumX += sps[m].get<0>();
      // }
      // float meanOptFocusPos = sumX / mNumCurvesToRecord;
      // LOG(info) << "MEAN OPT FOCUS POS: " << meanOptFocusPos << endl;



      
    
    } catch(boost::thread_interrupted const&) {
      // Interrupted
      // NOTE: Eventually some cleanup is necessary - e.g. reset focus pos...
      LOG(info) << "Focus finder was interrupted!" << endl;
    }

    mFocusFindStatusData.isRunning = false;
    callStatusUpdListener(& mFocusFindStatusData);
  }
};

  


  
 
