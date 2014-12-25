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

#include "focus_finder_linear_interpolation_impl.hpp"

#define FOCUS_FINDER_STOP(inStopFlag)					\
  if (inStopFlag) {							\
    LOG(warning) << "Focus finder cancelled!" << endl;			\
    throw FocusFinderLinearInterpolationExceptionT("Focus finder cancelled."); \
  }									\


namespace AT {
  // TODO: We may finally calculate the variance of the collected curve data with respect to the calculated parameter curve
  void FocusFinderLinearInterpolationImplT::findFocus() {
    LOG(trace) << dec << "FocusFinderLinearInterpolationImplT::findFocus() - Entering..." << endl;
    
    FocusDirectionT::TypeE direction = this->determineInitialDirectionOfImprovement();
    LOG(debug) << dec << "Direction of improvement: " << FocusDirectionT::asStr(direction) << "..." << endl;

    this->findRoughFocus(direction);
    LOG(info) << dec << "Found rough focus at abs pos: " << mFocuserDevice->getAbsPos() << "..." << endl;


    int Fmin = this->findExtrema(direction, MinMaxFocusPosT::MIN_FOCUS_POS);
    int Fmax = this->findExtrema(direction, MinMaxFocusPosT::MAX_FOCUS_POS);
    LOG(info) << dec << "Determined focus boundaries [min, max]=[" << Fmin << ", " << Fmax << "]..." << endl;

    // Crosscheck that Fmax > Fmin
    if (Fmax <= Fmin) {
      stringstream ssExc;
      ssExc << "Fmax=" << Fmax << " must be greater than Fmin=" << Fmin << "." << endl;
      const string tmpStr = ssExc.str();
      throw FocusFinderLinearInterpolationExceptionT(tmpStr.c_str());
    }

    // Rough focus search - from Fmin and Fmax calculate start and end position.
    float centerPos = Fmin + fabs(Fmax - Fmin) / 2;
    float halfLength = fabs(Fmax - centerPos);
    float percLength = halfLength * 0.01 * (float) mRoughFocusSearchRangePerc;
    
    int roughAbsStartPos = centerPos - percLength;
    int roughAbsEndPos = centerPos + percLength;

    LOG(debug) << dec << "centerPos: " << centerPos << ", halfLength: " << halfLength << ", mRoughFocusSearchRangePerc: "
	       << mRoughFocusSearchRangePerc << "%, percLength: " << percLength << endl;

    double roughOptAbsFocusPos = this->findOptimalFocusInRange(roughAbsStartPos, roughAbsEndPos, mRoughFocusRecordNumCurves, mRoughFocusGranularitySteps);

    StarDataT roughStarData;
    takePictureCalcStarData(& roughStarData);

    LOG(info) << dec << "Took picture - resulting star data: " << roughStarData << ", pos: " << mFocuserDevice->getAbsPos() << endl;
    LOG(debug) << dec << "Rough focus determined: " << roughOptAbsFocusPos << ", starData: " << roughStarData << endl;


    // Fine focus search - from roughOptAbsFocusPos calc start and end pos.
    int fineAbsStartPos = roughOptAbsFocusPos - mFineSearchRangeSteps;
    int fineAbsEndPos = roughOptAbsFocusPos + mFineSearchRangeSteps;

    LOG(debug) << dec << "roughOptAbsFocusPos: " << roughOptAbsFocusPos << ", mFineSearchRangeSteps: " << mFineSearchRangeSteps
	       << ", fine search range=[" << fineAbsStartPos << ", " << fineAbsEndPos << "]..." << endl;

    double fineOptAbsFocusPos = this->findOptimalFocusInRange(fineAbsStartPos, fineAbsEndPos, mFineFocusRecordNumCurves, mFineFocusGranularitySteps);

    // Determine final seeing (average)
    const size_t _numSingleFrames = 5;
    float fineStarDataFitnessSum = 0;
    StarDataT fineStarData;

    for (size_t i = 0; i < _numSingleFrames; ++i) {
      // Take picture, calc star values and send update
      takePictureCalcStarData(& fineStarData);
      LOG(info) << dec << "Took picture - resulting star data: " << fineStarData << ", pos: " << mFocuserDevice->getAbsPos() << endl;

      fineStarDataFitnessSum += fineStarData.getFitness();
      
      // Send update to listeners
      FocusFinderDataT focusFinderData(mFocuserDevice->getAbsPos(), fineStarData, (float) i / (float) _numSingleFrames,
				       "Determining final seeing (average)..." /*, TODO: mVCurve*/);

      this->callFocusFinderUpdateListener(& focusFinderData);
    }

    float meanFineStarDataFitness = fineStarDataFitnessSum / (float) _numSingleFrames;

    // Send update to listeners
    ostringstream ossUpdMsg;
    ossUpdMsg << "Done. Final average fitness: " << meanFineStarDataFitness << flush;
    FocusFinderDataT focusFinderData(mFocuserDevice->getAbsPos(), fineStarData, 1.0f, ossUpdMsg.str() /*, TODO: mVCurve*/);
    this->callFocusFinderUpdateListener(& focusFinderData);

    // Logging summary
    LOG(info) << dec << "Rough focus found at: " << roughOptAbsFocusPos << ", starData: " << roughStarData << endl;
    LOG(info) << dec << "Fine focus found at: " << fineOptAbsFocusPos << ", mean fine star data fitness: " << meanFineStarDataFitness << endl;
  }

  double
  FocusFinderLinearInterpolationImplT::findOptimalFocusInRange(int inAbsStartPos, int inAbsEndPos, size_t inNumVCurves, size_t inCurveGranularitySteps) {
    LOG(info) << dec << "Calculated start/end positions [start, end]=[" << inAbsStartPos << ", " << inAbsEndPos << "]" << endl;
    
    // Record VCurves - vector contains empty VCurves
    VCurveVecT vcurves(inNumVCurves);
    
    LOG(info) << dec << "Recording " << inNumVCurves << " VCurves with "
	      << (fabs(inAbsEndPos - inAbsStartPos) / inCurveGranularitySteps) << " points each..." << endl;
    
    for (VCurveVecT::iterator it = vcurves.begin(); it != vcurves.end(); ++it) {
      FOCUS_FINDER_STOP(mStopFlag);

      LOG(info) << dec << " > Recording VCurve " << std::distance(vcurves.begin(), it) << "..." << endl;
      
      VCurveT & curVCurve = *it;
      this->recordVCurve(inAbsStartPos, inAbsEndPos, inCurveGranularitySteps, & curVCurve, false /*do not move back to old pos*/);
      
      LOG(info) << dec << "-> VCurve " << std::distance(vcurves.begin(), it) << " recorded: " << curVCurve << endl;
    }

    // Calculate optimal focus position
    double optAbsFocusPos = this->calcOptimalAbsFocusPos(vcurves);
    LOG(info) << "Optimal focus pos: " << optAbsFocusPos << endl;
    
    // Move to optimal focus position
    mFocuserDevice->setAbsPos(optAbsFocusPos);

    LOG(trace) << dec << "Moved focus to pos: " << mFocuserDevice->getAbsPos() << endl;
    
    return optAbsFocusPos;
  }


  double
  FocusFinderLinearInterpolationImplT::calcOptimalAbsFocusPos(const VCurveVecT & inVCurves) {
    typedef CurveFitTmplT<ParabelFitTraitsT> ParabelMatcherT;

    AT_ASSERT(FocusFinderLinearInterpolation, inVCurves.size(), "VCurve vector is empty!");

    VCurveT masterVCurve;
    for (VCurveVecT::const_iterator itCurve = inVCurves.begin(); itCurve != inVCurves.end(); ++itCurve) {
      masterVCurve += *itCurve;
    }
    
    // Calculate average
    masterVCurve /= (double) inVCurves.size();

    LOG(info) << "Master VCurve: " << masterVCurve << endl;
  
    // Do the LM fit, throws CurveFitExceptionT if error boundaries are specified and NOT satisfied
    ParabelMatcherT::CurveParamsT::TypeT parabelParms;
    ParabelMatcherT::fitGslLevenbergMarquart<VCurveAccessorT>(masterVCurve, & parabelParms, mVCurveFitEpsAbs, mVCurveFitEpsRel, false /* no ex */);

    float a = parabelParms[ParabelMatcherT::CurveParamsT::A_IDX];
    float b = parabelParms[ParabelMatcherT::CurveParamsT::B_IDX];
    float c = parabelParms[ParabelMatcherT::CurveParamsT::C_IDX];
      
    LOG(debug) << "Calculated parabel parms - a: " << a << ", b: " << b << ", c: " << c << endl;
    
    // Calculate xmin, ymin
    float xMin = - b / (2.0f * a);
    float yMin = - 0.25f * (b * b) / a + c;

    LOG(debug) << "xMin: " << xMin << ", yMin: " << yMin << endl;

    // TODO: Adapt...
    // vector<float> & fitValues = (*outFitValues);
    // for(size_t i = 0; i < imgValues.size() && i < FwhmT::MAX_PTS; ++i) {
    //   fitValues[i] = ParabelFitTraitsT::fx(i, /*TODO: was - still works? dataPoints[i].x*/, *outGaussParms);
    // }

    return xMin;
  }

  void
  FocusFinderLinearInterpolationImplT::takePictureCalcStarData(StarDataT * outStarData, CImg<float> * outImg) {
    AT_ASSERT(FocusFinderLinearInterpolation, outStarData, "outStarData expected to be set!");
    CImg<float> img;
    
    /** 
     * Calc frameSize from inStarCenterPos and mWindowSize.
     *
     * NOTE: We multiply window size by 3 because we want to recenter the star window within the area.
     *       This is required since the star may will move due to seeing effects. Multiplying by 3
     *       allows moving the window in all directions depending on the new star position always
     *       having enough pixels around.
     */
    FrameT imgFrame = centerPosToFrame(mStarCenterPos, 3.0 * mWindowSize);

    // Calc star values, throws if star could not be determined
    size_t retryCnt = 0;
    while(retryCnt < mTakePictureFitGaussCurveMaxRetryCnt) {
      FOCUS_FINDER_STOP(mStopFlag);

      try {
	// Take a picture, throws if not connected or problem with device
	mCameraDevice->takePicture(& img, mExposureTimeSec, imgFrame, FrameTypeT::LIGHT, mBinning, false /* not compressed */);

	// Determine centroid (-> recenter), throws in case of failure -> no valid star detected
	PositionT centerPos(img.width() / 2, img.height() / 2);

	// Centroid coordinates are determined absolute to img which is 3 * mWindowSize in width and height.
	PositionT centroid = CentroidCalcT::starCentroid(img, centerPos, img.width(), CoordTypeT::ABSOLUTE);	
	FrameT subImgFrame = centerPosToFrame(centroid, mWindowSize);

	LOG(debug) << "Recentering to centroid - (x,y)=(" << centroid.get<0>() << ", " << centroid.get<1>() << ")..." << endl;

	const CImg <float> & subImg = img.get_crop(subImgFrame.get<0>() /*x*/,
						   subImgFrame.get<1>() /*y*/,
						   subImgFrame.get<0>() /*x*/ + subImgFrame.get<2>() /*w*/,
						   subImgFrame.get<1>() /*y*/ + subImgFrame.get<3>() /*h*/);

	// Extract image window
	outStarData->getFwhmHorz().set(extractLine(subImg, DirectionT::HORZ));
	outStarData->getFwhmVert().set(extractLine(subImg, DirectionT::VERT));
	outStarData->getHfd().set(subImg, mOuterHfdRadiusPx);
	break; // success

      } catch (CurveFitExceptionT & exc) {
	// Fitting did not work as expected...
	LOG(warning) << "Fitting did not work as expected...retry " << (retryCnt+1) << " / " << mTakePictureFitGaussCurveMaxRetryCnt << endl;
	++retryCnt;
      }
    }

    if (mTakePictureFitGaussCurveMaxRetryCnt <= retryCnt) {
      throw CurveFitExceptionT("Could not fit curve even after retrying.");
    }

    if (outImg) {
      *outImg = img; // Copy image if desired
    }

    if (mDebugShowTakePictureImage) {
      CImgDisplay disp1(img, "image");
      while(! disp1.is_closed()) { CImgDisplay::wait(disp1); }
    }
  }


  FocusDirectionT::TypeE
  FocusFinderLinearInterpolationImplT::determineInitialDirectionOfImprovement() {
    LOG(trace) << "Determining initial focuser direction...";
	
    // TODO: Check current absolute position to decide which direction we should initially move!

    // Take picture, calc star values and send update
    StarDataT starData;
    takePictureCalcStarData(& starData);
    LOG(trace) << dec << "Took first picture - resulting star data: " << starData << ", pos: " << mFocuserDevice->getAbsPos() << endl;

    // Send update to listeners
    FocusFinderDataT focusFinderData(mFocuserDevice->getAbsPos(), starData, 0.5f, "Determining initial focuser direction..."  /*, TODO: mVCurve*/);
    this->callFocusFinderUpdateListener(& focusFinderData);

    // Move mNumStepsToDetermineDirection inwards
    FOCUS_FINDER_STOP(mStopFlag);
    mFocuserDevice->moveFocusBy(mNumStepsToDetermineDirection, FocusDirectionT::INWARDS);
    LOG(trace) << dec << "Moved focus " << FocusDirectionT::asStr(FocusDirectionT::INWARDS) << " by "
	       << mNumStepsToDetermineDirection << " steps, pos: " << mFocuserDevice->getAbsPos() << endl;

    // Take another picture, calc star values and send update
    StarDataT starDataNew;
    takePictureCalcStarData(& starDataNew);
    LOG(trace) << dec << "Took second picture - resulting star data: " << starDataNew
	       << ", pos: " << mFocuserDevice->getAbsPos() << endl;

    // Send update to listeners
    FocusFinderDataT focusFinderDataNew(mFocuserDevice->getAbsPos(), starDataNew, 0.5f, "Determining initial focuser direction..." /*, TODO: mVCurve*/);
    this->callFocusFinderUpdateListener(& focusFinderDataNew);

    // Move focus back -> mNumStepsToDetermineDirection outwards
    FOCUS_FINDER_STOP(mStopFlag);
    mFocuserDevice->moveFocusBy(mNumStepsToDetermineDirection, FocusDirectionT::OUTWARDS);
    LOG(trace) << dec << "Moved focus back " << FocusDirectionT::asStr(FocusDirectionT::OUTWARDS)
	       << " by " << mNumStepsToDetermineDirection << " steps. "
	       << ", pos: " << mFocuserDevice->getAbsPos() << endl;

    // TODO: fitness means here: the greater the worse... the smaller the better... this is counter intuitive!
    FocusDirectionT::TypeE directionToImproveFocus = (starData.getFitness() > starDataNew.getFitness() ? FocusDirectionT::INWARDS : FocusDirectionT::OUTWARDS);

    LOG(info) << dec << "Direction which improves focus is: " << FocusDirectionT::asStr(directionToImproveFocus) << endl;
    LOG(debug) << dec << "StarData1: " << starData << endl << "StartData2: " << starDataNew  << ", pos: " << mFocuserDevice->getAbsPos() << endl;

    return directionToImproveFocus;
  }


  /**
   * -Find rough focus (in: FWMH & HFD at least to be reached, out: success/failure, FWMH & HFD obtained)
   * -1. Take picture
   * -2. Save FWHM & HFD
   * -3. Move M steps into prev. direction
   * -4. Take picture
   * -5. Repeat at 1. until new FWHM & HFD are worse than saved
   * -6. Move focus in oppsosite direction by M steps
   *
   * TODO: We may continue 2 times in a row if the value became worse... otherwise we may stop too early.
   * TODO: Or we take two pictures in a row and create the average...
   */
  void
  FocusFinderLinearInterpolationImplT::findRoughFocus(FocusDirectionT::TypeE inDirectionToImproveFocus) {
    static const char * updMsg = "Looking for rough focus...";
    LOG(info) << "Looking for rough focus in direction " << FocusDirectionT::asStr(inDirectionToImproveFocus) << "..." << endl;

    // Take picture
    StarDataT starDataNew, starDataPrev;
    takePictureCalcStarData(& starDataNew);
    LOG(trace) << dec << "Took picture - resulting star data: " << starDataNew << ", pos: " << mFocuserDevice->getAbsPos() << endl;
    
    // Send update to listeners
    FocusFinderDataT focusFinderData(mFocuserDevice->getAbsPos(), starDataNew, 0.0f, updMsg /*, TODO: mVCurve*/);
    this->callFocusFinderUpdateListener(& focusFinderData);
    
    size_t iterCounter = 0;
    
    do {
      starDataPrev = starDataNew;
      
      FOCUS_FINDER_STOP(mStopFlag);
	  
      // Move focus
      mFocuserDevice->moveFocusBy(mStepsToReachRoughFocus, inDirectionToImproveFocus);
      LOG(trace) << dec << "Moved focus " << FocusDirectionT::asStr(inDirectionToImproveFocus)
		 << " by " << mStepsToReachRoughFocus << " steps, pos: " << mFocuserDevice->getAbsPos() << endl;
      
      // Take picture
      takePictureCalcStarData(& starDataNew);
      LOG(trace) << dec << "Took picture - resulting star data new: " << starDataNew << ", pos: " << mFocuserDevice->getAbsPos() << endl;
      
      // Send update to listeners
      FocusFinderDataT focusFinderDataNew(mFocuserDevice->getAbsPos(), starDataNew, (float) iterCounter / (float) mRoughFocusMaxIterCnt,
					  updMsg /*, TODO: mVCurve*/);
 
     this->callFocusFinderUpdateListener(& focusFinderDataNew);
      
      // Print results to better compare it....
      LOG(debug) << dec << "Iteration: " << iterCounter << ", pos: " << mFocuserDevice->getAbsPos()
		 << ", HFD_prev=" << starDataPrev.getHfd().getValue() << ", HFD_new=" << starDataNew.getHfd().getValue() << endl
		 << ", Fwhm_horz_prev=" << starDataPrev.getFwhmHorz().getValue() << ", Fwhm_horz_new=" << starDataNew.getFwhmHorz().getValue() << endl
		 << ", Fwhm_vert_prev=" << starDataPrev.getFwhmVert().getValue() << ", Fwhm_vert_new=" << starDataNew.getFwhmVert().getValue() << endl;
      
      ++iterCounter;
      
    } while (iterCounter < mRoughFocusMaxIterCnt && starDataNew.getFitness() < starDataPrev.getFitness()); // end while
    
    // Did we find a rough focus within max amount of iterations?
    if (iterCounter < mRoughFocusMaxIterCnt) {
      // Rough focus found - move focus back
      FocusDirectionT::TypeE backDirection = FocusDirectionT::invert(inDirectionToImproveFocus);
      mFocuserDevice->moveFocusBy(mStepsToReachRoughFocus, backDirection);
      LOG(trace) << dec << "Moved focus back " << FocusDirectionT::asStr(backDirection) << " by "
		 << mStepsToReachRoughFocus << " steps. Pos: " << mFocuserDevice->getAbsPos() << endl;
      
      // Take picture
      StarDataT starDataFinal;
      takePictureCalcStarData(& starDataFinal);
      LOG(trace) << dec << "Took picture - resulting star data (final): " << starDataFinal
		 << ", pos: " << mFocuserDevice->getAbsPos() << endl;
      
      // Send update to listeners
      FocusFinderDataT focusFinderDataFinal(mFocuserDevice->getAbsPos(), starDataFinal, 1.0f, updMsg /*, TODO: mVCurve*/);
      this->callFocusFinderUpdateListener(& focusFinderDataFinal);
      
      // Just logging...
      LOG(info) << dec << "Found rough focus (abs pos: " << mFocuserDevice->getAbsPos() << ") after "
		<< iterCounter << " iterations (step size used=" << mStepsToReachRoughFocus
		<< "), measured - HFD=" << starDataFinal.getHfd().getValue()
		<< ", FWHM_horz=" << starDataFinal.getFwhmHorz().getValue()
		<< ", FWHM_vert=" << starDataFinal.getFwhmVert().getValue()
		<< " -> fitness: " << starDataFinal.getFitness() << endl;
    } else {
      // Rough focus not found
      throw FocusFinderLinearInterpolationExceptionT("Max. number of iterations reached. Unable to find max focus. Something is probably wrong.");
    }
  }
  
  int
  FocusFinderLinearInterpolationImplT::findExtrema(FocusDirectionT::TypeE inDirectionToImproveFocus, MinMaxFocusPosT::TypeE inMinMaxFocusPos) {
    ostringstream ossUpdMsg;
    ossUpdMsg << "Finding " << MinMaxFocusPosT::asStr(inMinMaxFocusPos) << " boundary...";

    FocusDirectionT::TypeE direction = (inMinMaxFocusPos == MinMaxFocusPosT::MIN_FOCUS_POS ? FocusDirectionT::INWARDS : FocusDirectionT::OUTWARDS);
    LOG(info) << "Looking for " << MinMaxFocusPosT::asStr(inMinMaxFocusPos) << " in direction " << FocusDirectionT::asStr(direction) << "..." << endl;

    int startFocusPos = mFocuserDevice->getAbsPos();
    LOG(debug) << "startFocusPos: " << startFocusPos << endl;

    // Take picture
    StarDataT starData;
    takePictureCalcStarData(& starData);
    LOG(trace) << dec << "Took picture - resulting star data: " << starData << endl;

    // Send update to listeners
    FocusFinderDataT focusFinderData(mFocuserDevice->getAbsPos(), starData, 0.0f, ossUpdMsg.str() /*, TODO: mVCurve*/);
    this->callFocusFinderUpdateListener(& focusFinderData);

    // Loop until max. fitness has been reached
    float progress = 0.0f;
    while(mExtremaFitnessBoundary > starData.getFitness()) {
      LOG(debug) << dec << "Not yet reached mExtremaFitnessBoundary " << mExtremaFitnessBoundary
		 << ", measured fitness: " << starData.getFitness()
		 << ", current pos: " << mFocuserDevice->getAbsPos() << endl;

      FOCUS_FINDER_STOP(mStopFlag);

      // Move focuser
      mFocuserDevice->moveFocusBy(mStepsToReachRoughFocus, direction);
      LOG(trace) << dec << "Moved focus " << FocusDirectionT::asStr(direction) << " by " << mStepsToReachRoughFocus << " steps. " << endl;

      // Take picture
      takePictureCalcStarData(& starData);
      LOG(trace) << dec << "Took picture - resulting star data: " << starData << endl;
      
      // Send update to listeners
      float newProgress = ((mExtremaFitnessBoundary > starData.getFitness()) ? starData.getFitness() / mExtremaFitnessBoundary : 1.0f);
      if (progress <= newProgress) {
	progress = newProgress;
      }
      FocusFinderDataT focusFinderData2(mFocuserDevice->getAbsPos(), starData, progress, ossUpdMsg.str() /*, TODO: mVCurve*/);
      this->callFocusFinderUpdateListener(& focusFinderData2);
    }
    
    int extremaPos = mFocuserDevice->getAbsPos();
    double extremaFitness = starData.getFitness();

    LOG(debug) << "Ok, found " << MinMaxFocusPosT::asStr(inMinMaxFocusPos) << ": " << extremaPos << ", moving back to start position..." << endl;

    // Move focuser back to start
    mFocuserDevice->setAbsPos(startFocusPos);
    LOG(trace) << dec << "Moved focus back to start position " << startFocusPos << "." << endl;

    // Take picture
    takePictureCalcStarData(& starData);
    LOG(trace) << dec << "Took picture - resulting star data: " << starData << endl;
    
    // Send update to listeners
    FocusFinderDataT focusFinderData2(mFocuserDevice->getAbsPos(), starData, 1.0f, ossUpdMsg.str() /*, TODO: mVCurve*/);
    this->callFocusFinderUpdateListener(& focusFinderData2);      

    LOG(info) << dec << "Reached extrema (boundary is " << mExtremaFitnessBoundary
	      << "), looking for " << MinMaxFocusPosT::asStr(inMinMaxFocusPos)
	      << " in direction " << FocusDirectionT::asStr(direction)
	      << ", measured fitness: " << extremaFitness
	      << ", current pos: " << extremaPos << endl;
    
    return extremaPos; // focuser pos
  }

  void FocusFinderLinearInterpolationImplT::recordVCurve(int inAbsStartPos, int inAbsEndPos, size_t inGranularitySteps, VCurveT * outVCurve, bool inMoveBackToOldPos) {
    static const char * updMsg = "Recording V-Curve...";

    if (! outVCurve) {
      throw FocusFinderLinearInterpolationExceptionT("No vcurve passed.");
    }

    LOG(info) << "Start recording V-Curve..." << endl;
    LOG(debug) << "Moving to start position " << inAbsStartPos << "..." << endl;

    // Remember old focus position
    int oldFocusPos = mFocuserDevice->getAbsPos();

    // Move focuser to start position
    mFocuserDevice->setAbsPos(inAbsStartPos);
    LOG(trace) << dec << "Moved focus to start position " << inAbsStartPos << "." << endl;
    
    int curPos = mFocuserDevice->getAbsPos();
    StarDataT starData;

    while (curPos < inAbsEndPos) {
      FOCUS_FINDER_STOP(mStopFlag);
      
      // Take picture
      takePictureCalcStarData(& starData);
      LOG(trace) << dec << "Took picture - resulting star data: " << starData << endl;
      
      // Send update to listeners
      FocusFinderDataT focusFinderData(mFocuserDevice->getAbsPos(), starData, 1.0f - (fabs(inAbsEndPos - curPos) / fabs(inAbsEndPos - inAbsStartPos)),
				       updMsg /*, TODO: mVCurve*/);

      this->callFocusFinderUpdateListener(& focusFinderData);
      
      // Add position + fitness to VCurve
      outVCurve->insert(make_pair(curPos, starData.getFitness()));

      // Move focuser
      curPos += inGranularitySteps;
      mFocuserDevice->setAbsPos(curPos);
      LOG(trace) << dec << "Moved focus to abs pos " << curPos << endl;
    }
    
    if (inMoveBackToOldPos) {
      mFocuserDevice->setAbsPos(oldFocusPos);
      LOG(trace) << dec << "Moved focus back to start pos " << oldFocusPos << endl;

      // Send update to listeners
      FocusFinderDataT focusFinderData(mFocuserDevice->getAbsPos(), starData, 1.0f, updMsg /*, TODO: mVCurve*/);
      this->callFocusFinderUpdateListener(& focusFinderData);      
    }
  }
}; // end AT namespace
