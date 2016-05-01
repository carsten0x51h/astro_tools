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
  
  /**
   * Determine HFD limit = N * initial HFD (star is roughly in focus). The value is
   * limited by 80% of the max HFD value. The limitation is important because
   * otherwise - if star is not in focus the base HFD will be higher - this base HFD
   * times the factor can lead to a huge HFD which can never be reached. Then the
   * focus stop condition will never be fulfilled.
   */
  FocusFinderImplT::CalcLimitFuncT FocusFinderImplT::sHfdLimitStrategy = [](float inFocusMeasure, bool * outInitialMeasureAcceptable) {
      const float hfdLimitFactor = 2.0;   // 200%
			     
      float hfdLimit = hfdLimitFactor * inFocusMeasure;
      float maxHfdLimit = HfdT::getMaxHfdLimit(HfdT::outerHfdDiameter);
      
      LOG(debug) << "Measured HFD: " << inFocusMeasure << ", hfdLimitFactor: " << hfdLimitFactor << " -> proposed HFD limit=" << hfdLimit << endl;

      if (hfdLimit > maxHfdLimit) {
	LOG(info) << "HFD limit limited by maxHfdLimit." << endl;
	hfdLimit = maxHfdLimit;
      }

      LOG(info) << "Final HFD limit limited=" << hfdLimit << endl;

      // Star too weak? Bad seeing? Bad initial focus?
      if (inFocusMeasure > 0.8 /*80%*/ * maxHfdLimit) {
	*outInitialMeasureAcceptable = false;
			       
	LOG(error) << "Initial measure " << inFocusMeasure << " too close to max HFD " << maxHfdLimit
	           << ". (limit is 80% of max HFD)." << endl;
      }
      return hfdLimit;
    };




  
    CImg<float>
    FocusFinderImplT::extractImgFrame(const CImg<float> & inFrameImage, int * outDx, int * outDy) const {
    // Post process image... we assume that the star did not move too far from the image center
    // NOTE: Boundaries of currSubImage are based on currImageFrameFF.
    // TODO: Should IWC etc. be parametrized? / Be configurable?
    PointT<float> assumedCenter((float) inFrameImage.width() / 2.0, (float) inFrameImage.height() / 2.0);
    FrameT<unsigned int> newSelectionFrameIF;
    bool insideBounds = StarFrameSelectorT::calc(inFrameImage, 0 /*bitPix - TODO / HACK: not needed */,
						 assumedCenter, & newSelectionFrameIF,
						 StarFrameSelectorT::StarRecognitionTypeT::PROXIMITY,
						 CentroidT::CentroidTypeT::IWC, cSelectionFrameSize /*frameSize*/);
	  
    AT_ASSERT(StarFrameSelector, insideBounds, "Expected frame to be inside bounds.");	  

    PointT<float> newCenterPosIF = frameToCenterPos(newSelectionFrameIF);
    
    if (outDx) {
      *outDx = newCenterPosIF.get<0>() - assumedCenter.get<0>();
    }
    if (outDy) {
      *outDy = newCenterPosIF.get<1>() - assumedCenter.get<1>();
    }
    
    return inFrameImage.get_crop(newSelectionFrameIF.get<0>() /*x0*/,
				 newSelectionFrameIF.get<1>() /*y0*/,
				 newSelectionFrameIF.get<0>() + newSelectionFrameIF.get<2>() - 1/*x1=x0+w-1*/,
				 newSelectionFrameIF.get<1>() + newSelectionFrameIF.get<3>() - 1/*y1=y0+h-1*/);
  }
  
  
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
      stringstream ss;
      ss << "Error creating directory '" << ssNewRecordDir.str() << "'!" << endl;
      throw FocusFinderImplRecordingExceptionT(ss.str());
    }
    return ssNewRecordDir.str();
  }
  
  void
  FocusFinderImplT::recordSequence(PosToImgMapT * outPosToImgMap, int stepSize, int inNumSteps, size_t inNumSeqIterations, size_t inSeqNo) {
    vector<int> stepSizes; // all equal

    for (size_t i=0; i < inNumSteps; ++i) {
      stepSizes.push_back(stepSize);
    }
    recordSequence(outPosToImgMap, stepSizes, inNumSeqIterations, inSeqNo);
  }
  
  void
  FocusFinderImplT::recordSequence(PosToImgMapT * outPosToImgMap, const vector<int> & inStepSizes, size_t inNumSeqIterations, size_t inSeqNo) {
    if (! inStepSizes.size()) {
      LOG(warning) << "Step-size container empty! No sequence recorded." << endl;
      return;
    }

    LOG(info) << "recordSequence - limit: " << mLimit << endl;

    mFocusFindStatusData.isRunning = true;
    mFocusFindStatusData.sequenceProgress = 0.0;
    mFocusFindStatusData.phase = mPhase;
    callStatusUpdListener(& mFocusFindStatusData);

    // Prepare image recording directory if enabled
    string currRecordDir = (! mRecordBaseDir.empty() ? prepareRecordDir() : string(""));
    int focusStartPos = mFocuserDevice->getAbsPos();
    
    // Take one picture to make sure that star is (will be) centered. This shot will not be recorded!
    CImg<float> currSubImage;
    mCameraDevice->takePicture(& currSubImage, mCntlData.exposureTime, getImageFrame(mCurrCenterPosFF),
			       FrameTypeT::LIGHT, mCntlData.binning, false /*compressed*/);
    int dx = 0, dy = 0;
    CImg<float> imgFrame = extractImgFrame(currSubImage, & dx, & dy);
    
    if (mCntlData.imgFrameRecenter) {
      LOG(info) << "Frame recentering enabled - dx: " << dx << ", dy: " << dy << endl;
      mCurrCenterPosFF.get<0>() += dx;
      mCurrCenterPosFF.get<1>() += dy;
    }


    
    // Do "numSteps" steps in both directions
    int currAbsFocusDestPos = focusStartPos;
    int sign[2] = { -1, 1 };
    for (size_t i = 0; i < 2; ++i) {
      FocusDirectionT::TypeE focusDirection = static_cast<FocusDirectionT::TypeE>(i);

      for (size_t step = 0; step < inStepSizes.size(); ++step) {	
  	// Move focus
  	currAbsFocusDestPos += sign[i] * inStepSizes.at(step);

	LOG(info) << "recordSequence - Setting new dest position: " << currAbsFocusDestPos << endl;
	
	mFocuserDevice->setAbsPos(currAbsFocusDestPos);
	
  	// Take picture etc.
  	mCameraDevice->takePicture(& currSubImage, mCntlData.exposureTime, getImageFrame(mCurrCenterPosFF),
  				    FrameTypeT::LIGHT, mCntlData.binning, false /*compressed*/);

	try {
	  imgFrame = extractImgFrame(currSubImage, & dx, & dy);
	} catch (CentroidExceptionT & exc) {
	  // Unable to determine star data - probably went far out of focus - stop in current direction...
	  LOG(warning) << "Unable to determine star data while recording sequence. Stop going into that direction." << endl;
	  break;
	}

	if (mCntlData.imgFrameRecenter) {
	  LOG(info) << "Frame recentering enabled - dx: " << dx << ", dy: " << dy << endl;
  	  mCurrCenterPosFF.get<0>() += dx;
  	  mCurrCenterPosFF.get<1>() += dy;
  	}

	// Record image if enabled
	if (! currRecordDir.empty()) {
	  writeRecordFile(currRecordDir, imgFrame, i, step, mFocuserDevice->getAbsPos());
	}
	
	// Notify listeners of new sample
	// TODO
	// TODO
	// TODO
	//TODO: We may pass ptr to FocusCurve directly and store new values directly! Then recordedPosToImgMap might no longer be required!
	// TODO: Only feature we loose is returning multiple FocusCurves at once...
	// TODO
	// TODO
	// TODO
	FocusCurveT recordedFocusCurve(*outPosToImgMap, mFocusMeasureFunc /*TODO: Add interpolation method...?*/);
	callNewSampleListener(& recordedFocusCurve, mFocuserDevice->getAbsPos(), imgFrame, mLimit);


	float focusMeasure = mFocusMeasureFunc(imgFrame);
	bool limitReached = (mLimit && focusMeasure >= mLimit);
	
  	// Update status... - TODO: We may put lines below into a function or a macro...
  	mFocusFindStatusData.currAbsFocusPos = mFocuserDevice->getAbsPos();
  	mFocusFindStatusData.currCenterPosFF = mCurrCenterPosFF;
	mFocusFindStatusData.currImage = currSubImage; // Copy
	mFocusFindStatusData.currBinning = mCntlData.binning;
	mFocusFindStatusData.dx = dx;
	mFocusFindStatusData.dy = dy;
	mFocusFindStatusData.sequenceProgress = (! limitReached ? 50.0 * i + 50.0 * ((float) step / (inStepSizes.size() - 1)) : 50 * (i + 1));
	mFocusFindStatusData.phaseProgress = 100.0 * inSeqNo / (float) inNumSeqIterations + (mFocusFindStatusData.sequenceProgress / inNumSeqIterations);
	mFocusFindStatusData.phase = mPhase;
	callStatusUpdListener(& mFocusFindStatusData);
	
  	// Break move if focusMeasure limit is reached (only if limit is defined)
	if (limitReached) {
	  LOG(info) << "Limit ("<< mLimit << ") reached (" << FocusDirectionT::asStr(focusDirection)
		    << "): " << focusMeasure << ". Stop." << endl;
  	  break;
	} else {
	  // Associate the current focus position with the current star image
	  (*outPosToImgMap)[mFocuserDevice->getAbsPos()] = imgFrame;
	}

	boost::this_thread::interruption_point();

      } // inner for
      
      // Back to start pos for second curve part...
      currAbsFocusDestPos = focusStartPos;
      mFocuserDevice->setAbsPos(focusStartPos);
    }
  }


  float
  FocusFinderImplT::determineLimit() {
    mPhase = PhaseT::DET_LIMIT;

    CImg<float> currSubImage, imgFrame;
    mCameraDevice->takePicture(& currSubImage, mCntlData.exposureTime, getImageFrame(mCurrCenterPosFF),
			       FrameTypeT::LIGHT, mCntlData.binning, false /*compressed*/);

    int dx = 0, dy = 0;
    imgFrame = extractImgFrame(currSubImage, & dx, & dy);

    // Recenter the image frame for the next shot (follow the star)
    if (mCntlData.imgFrameRecenter) {
      LOG(info) << "Frame recentering enabled - dx: " << dx << ", dy: " << dy << endl;
      mCurrCenterPosFF.get<0>() += dx;
      mCurrCenterPosFF.get<1>() += dy;
    }

    // Call the injected behaviour to calculate the focus measure (e.g. HFD)
    float focusMeasure = mFocusMeasureFunc(imgFrame);
      
    // Call the injected behaviour to initially calculate the limiting value (e.g. HFD based)
    bool initialMeasureAcceptable = true;
    float limit = mCalcLimitFunc(focusMeasure, & initialMeasureAcceptable);

    if (! initialMeasureAcceptable) {
      throw FocusFinderImplExceptionT("Initial measure too close to limit.");
    }

    return limit;
  }

  void
  FocusFinderImplT::determineStepSizeAndStartPos(int * outStepSize, float * outStartPos) {
    AT_ASSERT(FocusFinderImpl, outStepSize, "outStepSize not set!");
    AT_ASSERT(FocusFinderImpl, outStartPos, "outStartPos not set!");

    // Update FocusFinder PHASE...
    mPhase = PhaseT::DET_STEP_SIZE;
    
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
	
    // NOTE: This is only require for one direction - the step-size list is symmetric...
    LOG(info) << "Calculating non-linear step sizes..." << endl;

    while(currAbsPosLeft > absLimitL) {
      currAbsPosLeft -= currStepSizeLeft;

      stepSizesLeft.push_back(currStepSizeLeft);

      LOG(info) << "Step-size " << (++numSteps) << ": " << currStepSizeLeft << endl;
	  
      currStepSizeLeft *= 2;
      currStepSizeLeft = (currStepSizeLeft > maxStepSize ? maxStepSize : currStepSizeLeft);
    }
    LOG(info) << "Abs max focus movement in sum per direction: " << currAbsPosLeft << endl;

    // Recording the sequence to determine best step size
    PosToImgMapT recordedPosToImgMap;
    recordSequence(& recordedPosToImgMap, stepSizesLeft, 1 /*numSeqIterations*/, 0 /*seq no*/);

    FocusCurveT recordedFocusCurve(recordedPosToImgMap, mFocusMeasureFunc /*TODO: Add interpolation method...?*/);
	
    PointT<float> sp = recordedFocusCurve.getMinValPoint(); // Minimum found during scan is a good start position
    LOG(info) << "Start position for curve recording: " << sp << endl;
	
    // Notify listeners of status update (TODO: Required here?)
    mFocusFindStatusData.sequenceProgress = 100;
    mFocusFindStatusData.phaseProgress = 100;
    mFocusFindStatusData.phase = mPhase;
    callStatusUpdListener(& mFocusFindStatusData);

    PointT<float> min, max;
    recordedFocusCurve.getBounds(& min, & max);

    float meanLength = fabs(max.get<0>() - min.get<0>()) / 2;

    *outStepSize = meanLength / mNumStepsFine;
    *outStartPos = sp.get<0>();
      
    LOG(info) << "STEP1 FINISHED: DETERMINED STEP SIZE FOR CURVE RECORDING: " << *outStepSize << " ("
	      << mNumStepsFine << " steps per direction) and start pos: " << *outStartPos << endl;
  }

  
  void
  FocusFinderImplT::run() {
    AT_ASSERT(FocusFinderImpl, mCntlData.valid(), "No valid cntl data!");

    // Set running...
    mPhase = PhaseT::INITIALIZING;
    
    mFocusFindStatusData.isRunning = true;
    mFocusFindStatusData.phase = mPhase;
    mFocusFindStatusData.sequenceProgress = 0;
    mFocusFindStatusData.phaseProgress = 0;
    callStatusUpdListener(& mFocusFindStatusData);

    // Call start handler...
    callFocusFinderStartListener(& mCntlData);
    
    try {
      int stepSize = mCntlData.stepSize;
      mCurrCenterPosFF = mCntlData.centerPosFF;
      
      /**
       * 1. If no stepSize is configured, determine it - NOTE: This function also roughly centers the focus!
       */
      if (! stepSize) {
	// Initially determine limits
	mLimit = determineLimit();

	float focusStartPos;
	determineStepSizeAndStartPos(& stepSize, & focusStartPos);
	
	// Move focus to roughly determine start position
	LOG(info) << "SETTING INITIAL FOCUS POS FOR STEP 2: " << focusStartPos << endl;
	mFocuserDevice->setAbsPos(focusStartPos);
      }

      // HACK!!! TODO!!! FIXME!!! Overwrite for test
      //stepSize = 10;
	
      /**
       * 2. Again, take one shot to determine current limit (e.g. HFD) (assuming that focus is "ok") - this will
       *    be the reference value to calculate the limiting value for the boundaries.
       */
      mLimit = determineLimit();

      
      /**
       * 3. Now, record M "fine" curves...
       */
      mPhase = PhaseT::FOCUS_CURVES_RECORDING;

      LOG(info) << "Now, recording " << mNumCurvesToRecord << " curves. stepSize = " << stepSize << " (" << mNumStepsFine << " steps per direction)." << endl;
      PosToImgMapT recordedPosToImgMap[mNumCurvesToRecord]; // TODO: Maybe as member?? Or maybe PosToFocusMeasure as member?? So results can be requested by client, later!
      PointT<float> sps[mNumCurvesToRecord];

      
      for (size_t m = 0; m < mNumCurvesToRecord; ++m) {
      	LOG(info) << "Recording fine sequence " << (m + 1) << "/" << mNumCurvesToRecord  << "..." << endl;
	
      	recordSequence(& recordedPosToImgMap[m], stepSize, 2 * mNumStepsFine /* max. number of steps - want to hit 'limit' */,
		       mNumCurvesToRecord /*numSeqIterations*/, m /*seq no*/);

	FocusCurveT recordedCurve(recordedPosToImgMap[m], mFocusMeasureFunc /*TODO: Add interpolation method...?*/);

      	LineT<float> line1, line2;
      	sps[m] = recordedCurve.calcOptFocusPos(LineFitTypeT::OLS /*LineFitTypeT::BISQUARE - sometimes does not converge!?*/, & line1, & line2);
      	LOG(info) << "SP of fine curve " << (m+1) << ": " << sps[m] << endl;
				  
      	// Tell clients that there is a new focus curve available
      	callNewFocusCurveListener(& recordedCurve, & recordedPosToImgMap[m], & sps[m], & line1, & line2, mLimit);
	
      	//TODO: Send a status update (call handlers) - new sequence (with line...)  -> different / additional handler/listener?

	mFocusFindStatusData.sequenceProgress = 100; // Sequence recorded
	mFocusFindStatusData.phaseProgress = 100.0 * ((float) (m + 1) / (float) mNumCurvesToRecord); // Depends on number of curves to record

	mFocusFindStatusData.phase = mPhase;
      	callStatusUpdListener(& mFocusFindStatusData);
      }

      
      /**
       * 4. Calculate opt. focus position and go for it... 
       *    TODO: Here are multiple ways to do so...
       */
      mPhase = PhaseT::SET_OPT_POS;

      float sumX = 0;
      for (size_t m = 0; m < mNumCurvesToRecord; ++m) {
      	sumX += sps[m].get<0>();
      }
      float meanOptFocusPos = sumX / mNumCurvesToRecord;

      // Move focus to opt. position. (or should we return to start-pos?? and just return optimal pos? -
      //       so user can decide if he wants to go there...?) Atl east in GUI we can ask if user wants to go there..
      mFocuserDevice->setAbsPos(meanOptFocusPos);

      
      /**
       * 5. Finally take another picture with "opt" focus.
       */
      CImg<float> currSubImage, imgFrame;
      mCameraDevice->takePicture(& currSubImage, mCntlData.exposureTime, getImageFrame(mCurrCenterPosFF),
				 FrameTypeT::LIGHT, mCntlData.binning, false /*compressed*/);

      imgFrame = extractImgFrame(currSubImage);
      float focusMeasure = mFocusMeasureFunc(imgFrame);
      
      LOG(info) << "FINALLY - MEAN OPT FOCUS POS REACHED: " << meanOptFocusPos << ", focus-measure: " << focusMeasure << endl;

      // Tell clients that opt focus has been determined
      callFocusDeterminedListener(meanOptFocusPos, imgFrame, focusMeasure);
      
      // Update status... - TODO: We may put lines below into a function or a macro...
      mFocusFindStatusData.currAbsFocusPos = mFocuserDevice->getAbsPos();
      mFocusFindStatusData.currCenterPosFF = mCurrCenterPosFF;
      mFocusFindStatusData.currImage = currSubImage; // Copy
      mFocusFindStatusData.currBinning = mCntlData.binning;
      mFocusFindStatusData.dx = 0;
      mFocusFindStatusData.dy = 0;
      mFocusFindStatusData.sequenceProgress = 100.0;
      mFocusFindStatusData.phaseProgress = 100.0;
      mFocusFindStatusData.phase = mPhase;
      callStatusUpdListener(& mFocusFindStatusData);

      
      // Tell clients that focus was found...
      callFocusFinderFinishedListener(meanOptFocusPos);
    } catch(boost::thread_interrupted const&) {
      // TODO: Go back to startPos!!
      // Interrupted
      // NOTE: Eventually some cleanup is necessary - e.g. reset focus pos...
      LOG(info) << "Focus finder was interrupted!" << endl;
      callFocusFinderAbortListener(true /*manual abort*/, "Manually aborted.");
    } catch(const FocusFinderImplExceptionT & exc) {
      // TODO: Go back to startPos!!
      stringstream ss;
      ss << "Initial focusMeasure too close to theoretical maximum. Star too weak? Bad seeing? Try to increase "
	 << "exposure time. Or improve initial focus position. Details: "
	 << exc.what() << endl;
      LOG(error) << ss.str() << endl;
      callFocusFinderAbortListener(false /*no manual abort*/, ss.str());
    } catch(const FocusFinderImplRecordingExceptionT & exc) {
      // TODO: Go back to startPos!!
      stringstream ss;
      ss << "Problem recording focus finder sequence. Details: " << exc.what() << endl;
      LOG(error) << ss.str() << endl;
      callFocusFinderAbortListener(false /*no manual abort*/, ss.str());
    } catch(const std::exception & exc) {
      // TODO: Go back to startPos!!
      stringstream ss;
      ss << "Unknown problem occured while executing focus finder. Details: " << exc.what() << endl;
      LOG(error) << ss.str() << endl;
      callFocusFinderAbortListener(false /*no manual abort*/, ss.str());
    }

    mPhase = PhaseT::READY;
    mFocusFindStatusData.isRunning = false;
    mFocusFindStatusData.phase = mPhase;
    mFocusFindStatusData.sequenceProgress = 0;
    mFocusFindStatusData.phaseProgress = 0;
    callStatusUpdListener(& mFocusFindStatusData);
  }
};

  


  
 
