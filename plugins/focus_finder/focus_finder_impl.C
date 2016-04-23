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
#include <sys/stat.h>       // mkdir
 
#include "focus_finder_impl.hpp"
#include "io_util.hpp"

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
  DEF_Exception(FocusFinderImplRecording);

  void
  FocusFinderImplT::writeRecordFile(const string & inCurrRecordDir, const CImg<float> & inImg, int inSegmentNo, int inStepNo, int inFocusPos) {
    // TODO: Better way to handle directories / filenames? Boost?
    stringstream ssRecFilename;
    ssRecFilename << inCurrRecordDir << "/seg" << inSegmentNo << "_step" << inStepNo << "_pos" << inFocusPos << ".fits";
    LOG(info) << "Recording image to '" << ssRecFilename.str() << "'..." << endl;
    
    long naxis = 2;
    long naxes[2] = { inImg.width(), inImg.height() };
    std::auto_ptr<FITS> pFits(0);
    pFits.reset(new FITS(string("!") + ssRecFilename.str().c_str() , USHORT_IMG , naxis , naxes) );
  
    // NOTE: At this point we assume that there is only 1 layer.
    long nelements = std::accumulate(& naxes[0], & naxes[naxis], 1, std::multiplies<long>());
    std::valarray<int> array(nelements);
    cimg_forXY(inImg, x, y) { array[inImg.offset(x, y)] = inImg(x, inImg.height() - y -1); }
  
    long fpixel(1);
    pFits->pHDU().write(fpixel, nelements, array);

    pFits->pHDU().addKey("SEGMENT_NO", inSegmentNo, "Recorded in segment no");
    pFits->pHDU().addKey("STEP_NO", inStepNo, "Recorded at step");
    pFits->pHDU().addKey("FOCUS_POS", inFocusPos, "Recorded at focus position");  
}

  
  string
  FocusFinderImplT::prepareRecordDir() {
    // TODO: Better way to handle directories / filenames? Boost?
    stringstream ssNewRecordDir;
    ssNewRecordDir << mRecordBaseDir << "/seq_" << currDateTimeStr();

    LOG(debug) << "Creating recording directory " << ssNewRecordDir.str() << endl;

    const int dir_err = mkdir(ssNewRecordDir.str().c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    if (-1 == dir_err) {
      throw FocusFinderImplRecordingExceptionT("Error creating directory!");
    }
    return ssNewRecordDir.str();
  }

  
  // TODO: Maybe as template... then supply e.g. HfdT as T...
  // Then: calcStarValues -> calcStarValue as template as well...
  void
  FocusFinderImplT::recordSequence(FocusCurveT * outHfdFocusCurve, int stepSize, int inNumSteps, float inHfdLimit) {
    vector<int> stepSizes; // all equal

    for (size_t i=0; i < inNumSteps; ++i) {
      stepSizes.push_back(stepSize);
    }

    // HACK!! NEED BETER SOLUTION!!
    if (! inNumSteps) { /*"INIFNITE NUMBER OF STEPS..."*/
      for (size_t i=0; i < 100; ++i) {
	stepSizes.push_back(stepSize);
      }
    }
    
    recordSequence(outHfdFocusCurve, stepSizes, inHfdLimit);
  }
  
  void
  FocusFinderImplT::recordSequence(FocusCurveT * outHfdFocusCurve, const vector<int> & inStepSizes, float inHfdLimit) {
    if (! inStepSizes.size()) {
      LOG(warning) << "Step-size container empty! No sequence recorded." << endl;
      return;
    }

    LOG(info) << "recordSequence - hfdLimit: " << inHfdLimit << endl;

    mFocusFindStatusData.isRunning = true;
    callStatusUpdListener(& mFocusFindStatusData);

    // Prepare image recording directory if enabled
    string currRecordDir = (! mRecordBaseDir.empty() ? prepareRecordDir() : string(""));
    
    PointT<float> currCenterPosFF = mCntlData.centerPosFF;
    int focusStartPos = mFocuserDevice->getAbsPos();
    
    // Take one picture to make sure that star is (will be) centered. This shot will not be recorded!
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


    
    // Do "numSteps" steps in both directions
    int currAbsFocusDestPos = focusStartPos;
    int sign[2] = { -1, 1 };
    string dir[2] = { "left", "right" };
    for (size_t i = 0; i < 2; ++i) {
      FocusCurveT::SegmentT & curveSegment = outHfdFocusCurve->getSegment(i);
	
      for (size_t step = 0; step < inStepSizes.size(); ++step) {
  	// Move focus
  	currAbsFocusDestPos += sign[i] * inStepSizes.at(step);

	LOG(info) << "recordSequence - Setting new dest position: " << currAbsFocusDestPos << endl;
	
	mFocuserDevice->setAbsPos(currAbsFocusDestPos);
	
  	// Take picture etc.
  	mCameraDevice->takePicture(& currSubImage, mCntlData.exposureTime, getImageFrame(currCenterPosFF),
  				    FrameTypeT::LIGHT, mCntlData.binning, false /*compressed*/);

	try {
	  calcStarValues(currSubImage, & dx, & dy, & hfd);
	} catch (CentroidExceptionT & exc) {
	  // Unable to determine star data - probably went far out of focus - stop in current direction...
	  LOG(warning) << "Unable to determine star data while recording sequence. Stop going into that direction." << endl;
	  break;
	}

  	if (mCntlData.wantRecenter) {
  	  currCenterPosFF.get<0>() += dx;
  	  currCenterPosFF.get<1>() += dy;
  	}

	// Record image if enabled
	if (! currRecordDir.empty()) {
	  writeRecordFile(currRecordDir, currSubImage, i, step, mFocuserDevice->getAbsPos());
	}
	
	// Notify listeners of new sample
	callNewSampleListener(outHfdFocusCurve);

	
  	// Update status... - TODO: We may put lines below into a function or a macro...
  	mFocusFindStatusData.currAbsFocusPos = mFocuserDevice->getAbsPos();
  	mFocusFindStatusData.currCenterPosFF = currCenterPosFF;
	mFocusFindStatusData.currImage = currSubImage; // Copy
	mFocusFindStatusData.dx = dx;
	mFocusFindStatusData.dy = dy;
	mFocusFindStatusData.progress = 100.0 * ((float) step / (inStepSizes.size() - 1)); // TODO: Need total progress as well...
  	mFocusFindStatusData.hfd = hfd;
	callStatusUpdListener(& mFocusFindStatusData);

  	curveSegment[mFocuserDevice->getAbsPos()] = hfd.getValue();

	
	boost::this_thread::interruption_point();

  	// Break move if HFD limit is reached (only if HFD limit is defined)
  	if (inHfdLimit && hfd.getValue() > inHfdLimit) {
  	  LOG(info) << "HFD limit ("<< inHfdLimit << ") reached (" << dir[i] << "): " << hfd.getValue() << ". Stop." << endl;
  	  break;
  	}	
      } // inner for
      
      // Back to start pos for second curve part...
      currAbsFocusDestPos = focusStartPos;
      mFocuserDevice->setAbsPos(focusStartPos);
    }
  }


  
  void
  FocusFinderImplT::run() {
    AT_ASSERT(FocusFinderImpl, mCntlData.valid(), "No valid cntl data!");
    
    // Set running...
    mFocusFindStatusData.isRunning = true;
    callStatusUpdListener(& mFocusFindStatusData);

    // TODO: Call start handler... (eactly here or before status update?)

    
    try {
      PointT<float> currCenterPosFF = mCntlData.centerPosFF;
      int stepSize = mCntlData.stepSize;

      // TODO: Replace left/right positions by a "range" which is relative to the focusStartPos??!!! --> see meanLength in step 1.
      int leftLimitPos = -1;  // TODO: Init from mCntlData / config?
      int rightLimitPos = -1;  // TODO: Init from mCntlData / config?

      
      // 0. Take one shot to determine current hfd (assuming that focus is "ok") - this will
      //    be the reference value to calculate the limiting HFD value for the boundaries (important)
      //    TODO: Could also be supplied as parameter / configuration - then determination is not required.
      CImg<float> currSubImage;
      mCameraDevice->takePicture(& currSubImage, mCntlData.exposureTime, getImageFrame(currCenterPosFF), FrameTypeT::LIGHT, mCntlData.binning, false /*compressed*/);
      
      HfdT hfd;
      int dx = 0, dy = 0;
      calcStarValues(currSubImage, & dx, & dy, & hfd);
      
      // Determine HFD limit = N * initial HFD (star is roughly in focus). The value is limited by 80% of the max HFD value. 
      // The limitation is important because otherwise - if star is not in focus the base HFD will be higher - this base HFD
      // times the factor can lead to a huge HFD which can never be reached. Then the focus stop condition will never be
      // fulfilled.
      float hfdLimit = mHfdLimitFactor * hfd.getValue();
      float maxHfdLimit = hfd.getMaxHfdLimit();
      
      if (hfdLimit > maxHfdLimit) {
	hfdLimit = maxHfdLimit;
      }

      const float minHfdDistFactor = 0.8; /*80% - TODO: Improve name? Improve value? */
      
      if (hfd.getValue() > minHfdDistFactor * maxHfdLimit) {
	// Star too weak? Bad seeing? Bad initial focus?
	LOG(error) << "Initial HFD " << hfd.getValue() << " too close to max HFD " << maxHfdLimit << ". (limit is 50% of max HFD)." << endl;
	throw FocusFinderExceptionT("Initial HFD too close to max HFD.");
      }
      
      LOG(info) << "HFD limit = " << mHfdLimitFactor << " * " << hfd.getValue() << ", limited by max. HFD: " << maxHfdLimit << " --> " << hfdLimit << endl;
      
      
      // 1. If no stepSize is configured, determine it
      // NOTE / TODO: This step is also REQUIRED to roughly center the focus pos!
      //              So we will porbably have to do it all the time... or if a "centerFocus" option is set...
      if (! stepSize) {
	// TODO: Update FocusFinder PHASE...

    	// Determine initial boundaries - use current position (rough focus and min-max values of focuser)
	// To protect hardware, we subtract another 10% to not reach the absolute focuser boundary 
	// "Outer limits"
    	int focusStartPos = mFocuserDevice->getAbsPos();
    	int deltaL = fabs((1.1 * mFocuserDevice->getMinPos()) - focusStartPos);
    	int deltaR = fabs((0.9 * mFocuserDevice->getMaxPos()) - focusStartPos);
	int minLR = std::min(deltaL, deltaR);

	int absLimitL = focusStartPos - minLR;
	
	int maxStepSize = 0.05 * minLR; // 1/50 of minLR is maxStepSize
	
    	LOG(debug) << "focusStartPos: " << focusStartPos << endl;
    	LOG(debug) << "MaxFocusPos: " << mFocuserDevice->getMaxPos() << endl;
    	LOG(debug) << "MinFocusPos: " << mFocuserDevice->getMinPos() << endl;
    	LOG(debug) << "absLimitL: " << absLimitL << endl;

	// Calculate non-linear step-sizes
	// NOTE: We also may have chosen right... - symmetry...
	// TODO: Better naming... improve code..
	vector<int> stepSizesLeft;

	int currAbsPosLeft = focusStartPos;
	int currStepSizeLeft = 1;
	int numSteps = 0;
	
	LOG(info) << "Calculating non-linear step sizes..." << endl;

	// NOTE: This is only require for one direction - the step-size list is symmetric...
	while(currAbsPosLeft > absLimitL) {
	  currAbsPosLeft -= currStepSizeLeft;

	  stepSizesLeft.push_back(currStepSizeLeft);

	  LOG(info) << "Step-size " << (++numSteps) << ": " << currStepSizeLeft << endl;
	  
	  currStepSizeLeft *= 2;
	  currStepSizeLeft = (currStepSizeLeft > maxStepSize ? maxStepSize : currStepSizeLeft);
	}
	LOG(info) << "Calculating non-linear step sizes..." << endl;

    	LOG(info) << "Abs max focus movement in sum per direction: " << currAbsPosLeft << endl;

	
    	FocusCurveT recordedFocusCurve;
    	//LineT<float> line1, line2;
    	recordSequence(& recordedFocusCurve, stepSizesLeft, hfdLimit);

    	//PointT<float> sp = recordedFocusCurve.calcOptFocusPos(LineFitTypeT::OLS /*LineFitTypeT::BISQUARE - sometimes does not converge!?*/, & line1, & line2);
	// HACK!
    	// TODO: ODER BESSER?? Einfach die Position mit dem kleinsten Messewert nehmen?? ->Besser als interpolation bzw. Schnittpunkt???
	// TODO TODO TODO!!!!!!
	// Idee: Geraden müssen durch HFD_min punkt gehen! Rest wird approximiert .. d.h. SP ist HFD_min...
	PointT<float> sp = recordedFocusCurve.getMinValPoint(); // Minimum found during scan is a good start position
    	LOG(info) << "Start position for curve recording: " << sp << endl;

	//callFocusDeterminedListener(& recordedFocusCurve, & sp, & line1, & line2);
	
	// Notify listeners of status update (TODO: Required here?)
	mFocusFindStatusData.progress = 100; // TODO: CALC!! Update further values?
	callStatusUpdListener(& mFocusFindStatusData);

	PointT<float> min, max;
	recordedFocusCurve.getBounds(& min, & max);
	float meanLength = fabs(max.get<0>() - min.get<0>()) / 2;
	
    	stepSize = meanLength / mNumStepsFine;
      
    	LOG(info) << "STEP1 FINISHED: DETERMINED STEP SIZE FOR CURVE RECORDING: " << stepSize << " (" << mNumStepsFine << " steps per direction)." << endl;

    	// Move focus to roughly determine start position
    	mFocuserDevice->setAbsPos(sp.get<0>());
    	LOG(info) << "INITIAL FOCUS POS FOR STEP 2: " << sp.get<0>() << endl;	
      }


      
      // 3. Now, record M "fine" curves...
      LOG(info) << "Now, recording " << mNumCurvesToRecord << " curves. stepSize = " << stepSize << " (" << mNumStepsFine << " steps per direction)." << endl;
      FocusCurveT recordedFocusCurves[mNumCurvesToRecord];
      PointT<float> sps[mNumCurvesToRecord];
    
      for (size_t m = 0; m < mNumCurvesToRecord; ++m) {
      	LOG(info) << "Recording fine sequence " << (m + 1) << "/" << mNumCurvesToRecord  << "..." << endl;
	
      	LineT<float> line1, line2;
      	recordSequence(& recordedFocusCurves[m], stepSize, 0 /* INFINITE - mNumStepsFine - maybe just pass absolute limits...or directly put check into recordSequence asking max boundaries -10% or something...... */, hfdLimit);
	
      	sps[m] = recordedFocusCurves[m].calcOptFocusPos(/*LineFitTypeT::OLS*/ LineFitTypeT::BISQUARE/* - sometimes does not converge!?*/, & line1, & line2);
      	LOG(info) << "SP of fine curve " << (m+1) << ": " << sps[m] << endl;

      	callFocusDeterminedListener(& recordedFocusCurves[m], & sps[m], & line1, & line2);

      	//TODO: Send a status update (call handlers) - new sequence (with line...)  -> different / additional handler/listener?
      	mFocusFindStatusData.progress = 100.0 * ((float) m / 10.0);
      	callStatusUpdListener(& mFocusFindStatusData);
      }

      // Calculate mean position
      // TODO: Here are multiple ways to do so...
      float sumX = 0;
      for (size_t m = 0; m < mNumCurvesToRecord; ++m) {
      	sumX += sps[m].get<0>();
      }
      float meanOptFocusPos = sumX / mNumCurvesToRecord;

      // TODO: Finally, move focus to this position! (or should we return to start-pos?? and just return optimal pos? -
      //       so user can decide if he wants to go there...?) At least in GUI we can ask if user wants to go there..
      mFocuserDevice->setAbsPos(meanOptFocusPos);

      
      // Finally take another picture with "opt" focus
      // TODO: Probably it makes senste to move code below (incl. status update) into a private member function
      //CImg<float> currSubImage;
      mCameraDevice->takePicture(& currSubImage, mCntlData.exposureTime, getImageFrame(currCenterPosFF),
      				    FrameTypeT::LIGHT, mCntlData.binning, false /*compressed*/);
      //HfdT hfd;
      dx = 0;
      dy = 0;
      calcStarValues(currSubImage, & dx, & dy, & hfd);

      LOG(info) << "FINALLY - MEAN OPT FOCUS POS REACHED: " << meanOptFocusPos << ", hfd: " << hfd.getValue() << endl;
      
      // Update status... - TODO: We may put lines below into a function or a macro...
      mFocusFindStatusData.currAbsFocusPos = mFocuserDevice->getAbsPos();
      mFocusFindStatusData.currCenterPosFF = currCenterPosFF;
      mFocusFindStatusData.currImage = currSubImage; // Copy
      mFocusFindStatusData.dx = dx;
      mFocusFindStatusData.dy = dy;
      mFocusFindStatusData.progress = 100.0;
      mFocusFindStatusData.hfd = hfd;
      callStatusUpdListener(& mFocusFindStatusData);
      

	
      // TODO: Call finish handler...
      
    } catch(boost::thread_interrupted const&) {
      // Interrupted
      // NOTE: Eventually some cleanup is necessary - e.g. reset focus pos...
      LOG(info) << "Focus finder was interrupted!" << endl;
    } catch(FocusFinderExceptionT & exc) {
      LOG(error) << "Initial HFD too close to max HFD. Star too weak? Bad seeing? Try to increase exposure time. Or improve initial focus position." << endl;
      // TODO: Communicate this problem to the UI somehow.. maybe not catch here? Or via status update -> errorCode/text field?
    }

    mFocusFindStatusData.isRunning = false;
    callStatusUpdListener(& mFocusFindStatusData);
  }
};

  


  
 
