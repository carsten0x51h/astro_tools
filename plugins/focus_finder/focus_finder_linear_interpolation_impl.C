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

namespace AT {  
  // TODO: Maybe rename those FocusFinder classes to ...Minimizer - because FocusFinder is too generic - it is the whole thing...
  // but this class only tries to find the minimum HFD and/or Fwhm. In addition we could pass a minimizer policy... which tells the 
  // minimizer which star values should be taken into account.
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

    // Rough focus search
    // From Fmin and Fmax calculate start and end position
    float centerPos = Fmin + fabs(Fmax - Fmin) / 2;
    float halfLength = fabs(Fmax - centerPos);
    float percLength = halfLength * 0.01 * (float) mRoughFocusSearchRangePerc;
    
    int roughAbsStartPos = centerPos - percLength;
    int roughAbsEndPos = centerPos + percLength;

    LOG(debug) << dec << "centerPos: " << centerPos << ", halfLength: " << halfLength << ", mRoughFocusSearchRangePerc: " << mRoughFocusSearchRangePerc << "%, percLength: " << percLength << endl;

    StarDataT roughStarData;
    double roughOptAbsFocusPos = this->findOptimalFocusInRange(roughAbsStartPos, roughAbsEndPos, mRoughFocusRecordNumCurves, mRoughFocusGranularitySteps, & roughStarData);
    LOG(debug) << dec << "Rough focus determined: " << roughOptAbsFocusPos << ", starData: " << roughStarData << endl;


    // Fine focus search
    // From roughOptAbsFocusPos calc start and end pos.
    int fineAbsStartPos = roughOptAbsFocusPos - mFineSearchRangeSteps;
    int fineAbsEndPos = roughOptAbsFocusPos + mFineSearchRangeSteps;

    LOG(debug) << dec << "roughOptAbsFocusPos: " << roughOptAbsFocusPos << ", mFineSearchRangeSteps: " << mFineSearchRangeSteps
	       << ", fine search range=[" << fineAbsStartPos << ", " << fineAbsEndPos << "]..." << endl;

    StarDataT fineStarData;
    double fineOptAbsFocusPos = this->findOptimalFocusInRange(fineAbsStartPos, fineAbsEndPos, mFineFocusRecordNumCurves, mFineFocusGranularitySteps, & fineStarData);
    LOG(debug) << dec << "Fine focus determined: " << roughOptAbsFocusPos << ", starData: " << fineStarData << endl;


    // Logging summary
    LOG(info) << dec << "Rough focus found at: " << roughOptAbsFocusPos << ", starData: " << roughStarData << endl;
    LOG(info) << dec << "Fine focus found at: " << fineOptAbsFocusPos << ", starData: " << fineStarData << endl;
  }

  double
  FocusFinderLinearInterpolationImplT::findOptimalFocusInRange(int inAbsStartPos, int inAbsEndPos, size_t inNumVCurves, size_t inCurveGranularitySteps, StarDataT * outStarData) {
    LOG(info) << dec << "Calculated start/end positions [start, end]=[" << inAbsStartPos << ", " << inAbsEndPos << "]" << endl;
    
    // Record VCurves
    VCurveVecT vcurves(inNumVCurves); // Contains empty VCurves
    
    LOG(info) << dec << "Recording " << inNumVCurves << " VCurves with "
	      << (fabs(inAbsEndPos - inAbsStartPos) / inCurveGranularitySteps) << " points each..." << endl;
    
    for (VCurveVecT::iterator it = vcurves.begin(); it != vcurves.end(); ++it) {
      LOG(info) << dec << " > Recording VCurve " << std::distance(vcurves.begin(), it) << "..." << endl;
      
      VCurveT & curVCurve = *it;
      this->recordVCurve(inAbsStartPos, inAbsEndPos, inCurveGranularitySteps, & curVCurve, false /*do not move back to old pos*/);
      
      LOG(info) << dec << "-> VCurve " << std::distance(vcurves.begin(), it) << " recorded: " << curVCurve << endl;
    }

    // Calculate optimal focus position
    double optAbsFocusPos = this->calcOptimalAbsFocusPos(vcurves);
    LOG(info) << "Optimal focus pos: " << optAbsFocusPos << endl;
    
    // Move to optimal focus position
    float delta = mFocuserDevice->getAbsPos() - optAbsFocusPos;
    size_t steps = (size_t) fabs(delta);
    FocusDirectionT::TypeE finalDirection = (delta < 0 ? FocusDirectionT::OUTWARDS : FocusDirectionT::INWARDS);
    
    mFocuserDevice->moveFocusBy(steps, finalDirection); // TODO: We may use setAbsPos instead...
    
    LOG(trace) << dec << "Moved focus " << FocusDirectionT::asStr(finalDirection) << " by "
	       << steps << " steps, pos: " << mFocuserDevice->getAbsPos() << endl;
    
    // Take picture, calc star values and send update
    StarDataT starData;
    takePictureCalcStarData(& starData);
    LOG(info) << dec << "Took picture - resulting star data: " << starData << ", pos: " << mFocuserDevice->getAbsPos() << endl;
    
    if (outStarData) {
      *outStarData = starData; // copy
    }

    // Send update to listeners
    FocusFinderDataT focusFinderData(mFocuserDevice->getAbsPos(), starData /*, TODO: mVCurve*/);
    this->callFocusFinderUpdateListener(& focusFinderData);
    
    return optAbsFocusPos;
  }


  double
  FocusFinderLinearInterpolationImplT::calcOptimalAbsFocusPos(const VCurveVecT & inVCurves) {
    // TODO: Implement retry... if fitting does not work - but how?!
    //-> take another picture?!?! or just increase possible error by *2 ?!
    //-> Maybe there is a better solution by just still using the results?? ...
    //   Because the results will be the same if data does not change!! Maybe add "noExIfErrorCondNotFulfilled" ??!
    typedef FunctionFitTmplT<ParabelFitTraitsT> ParabelMatcherT;

    AT_ASSERT(FocusFinderLinearInterpolation, inVCurves.size(), "VCurve vector is empty!");

    VCurveT masterVCurve;
    for (VCurveVecT::const_iterator itCurve = inVCurves.begin(); itCurve != inVCurves.end(); ++itCurve) {
      masterVCurve = masterVCurve + *itCurve; // TODO: Use +=...
    }
    
    // Calculate average
    masterVCurve = masterVCurve / (double) inVCurves.size(); // TODO: Use /=

    LOG(info) << "Master VCurve: " << masterVCurve << endl;

    ParabelMatcherT::ParamsT parabelParms;

    // Allocate temporary data buffer
    // TODO: Should be handled internally
    fitgsl_data * dat = ParabelMatcherT::fitgsl_alloc_data(masterVCurve.size());
  
    // Fill data
    size_t idx = 0;
    for (VCurveT::const_iterator it = masterVCurve.begin(); it != masterVCurve.end(); ++it, ++idx) {
      dat->pt[idx].x = it->first;
      dat->pt[idx].y = it->second;
    }

    // Do the LM fit - TODO: this function should throw if it does not succeed?!
    int err = ParabelMatcherT::fitgsl_lm(dat, & parabelParms, mVCurveFitEpsAbs, mVCurveFitEpsRel);
  
    if (err) {
      // Free allocated data
      // TODO: Should be handled internally
      //ParabelMatcherT::fitgsl_free_data(dat);

      // TODO: Try to fin dout what is the rel and abs error compared to the required ones! --> print
      stringstream ss;
      ss << "FocusFinderLinearInterpolationImplT::calcOptimalAbsFocusPos - fitgsl_lm() returned non-zero status: " << err
	 << ", a: " << parabelParms[ParabelMatcherT::IdxT::A_IDX]
	 << ", b: " << parabelParms[ParabelMatcherT::IdxT::B_IDX]
	 << ", c: " << parabelParms[ParabelMatcherT::IdxT::C_IDX]
	 << ", however, for now we do not throw! TODO!" << endl;

      //throw CurveFittingExceptionT(ss.str().c_str()); Remember to comment in free above!
    }

    float a = parabelParms[ParabelMatcherT::IdxT::A_IDX];
    float b = parabelParms[ParabelMatcherT::IdxT::B_IDX];
    float c = parabelParms[ParabelMatcherT::IdxT::C_IDX];
    
    // TODO: Logging...
    cerr << "Calculated parabel parms - a: " << a << ", b: " << b << ", c: " << c << endl;

    // Calculate xmin, ymin
    float xMin = - b / (2.0f * a);
    float yMin = - 0.25f * (b * b) / a + c;

    // TODO: Logging...
    cerr << "xMin: " << xMin << ", yMin: " << yMin << endl;
  
    // Free allocated data
    // TODO: Should be handled internally
    ParabelMatcherT::fitgsl_free_data(dat);

    return xMin;
  }

  void
  FocusFinderLinearInterpolationImplT::takePictureCalcStarData(StarDataT * outStarData, CImg<float> * outImg) {
    AT_ASSERT(FocusFinderLinearInterpolation, outStarData, "outStarData expected to be set!");
    CImg<float> img;
    
    // Calc frameSize from inStarCenterPos
    FrameT imgFrame = centerPosToFrame(mStarCenterPos, mWindowSize);

    // Calc star values, throws if star could not be determined
    size_t retryCnt = 0;
    while(retryCnt < mTakePictureFitGaussCurveMaxRetryCnt) {
      try {
	// Take a picture, throws if not connected or problem with device
	mCameraDevice->takePicture(& img, mExposureTimeSec, imgFrame, FrameTypeT::LIGHT, mBinning, false /* not compressed */);

	// Calc PSNR to decide if valid star was selected
	if (! StarDataT::isValidStar(img, 70 /* TODO: as parm?! */)) {
	  // Too much noise - no star detected / star too weak....
	  throw FocusFinderLinearInterpolationExceptionT("No valid star selected.");
	}

	outStarData->getFwhmHorz().set(img, FwhmT::DirectionT::HORZ);
	outStarData->getFwhmVert().set(img, FwhmT::DirectionT::VERT);
	outStarData->getHfd().set(img, mOuterHfdRadiusPx);
	break; // success

      } catch (CurveFittingExceptionT & exc) {
	// Fitting did not work as expected...
	LOG(warning) << "Fitting did not work as expected...retry " << (retryCnt+1) << " / " << mTakePictureFitGaussCurveMaxRetryCnt << endl;
	++retryCnt;
      }
    }

    if (mTakePictureFitGaussCurveMaxRetryCnt <= retryCnt) {
      throw CurveFittingExceptionT("Could not fit curve even after retrying.");
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
    FocusFinderDataT focusFinderData(mFocuserDevice->getAbsPos(), starData /*, TODO: mVCurve*/);
    this->callFocusFinderUpdateListener(& focusFinderData);

    // Move mNumStepsToDetermineDirection inwards
    mFocuserDevice->moveFocusBy(mNumStepsToDetermineDirection, FocusDirectionT::INWARDS);
    LOG(trace) << dec << "Moved focus " << FocusDirectionT::asStr(FocusDirectionT::INWARDS) << " by "
	       << mNumStepsToDetermineDirection << " steps, pos: " << mFocuserDevice->getAbsPos() << endl;

    // Take another picture, calc star values and send update
    StarDataT starDataNew;
    takePictureCalcStarData(& starDataNew);
    LOG(trace) << dec << "Took second picture - resulting star data: " << starDataNew
	       << ", pos: " << mFocuserDevice->getAbsPos() << endl;

    // Send update to listeners
    FocusFinderDataT focusFinderDataNew(mFocuserDevice->getAbsPos(), starDataNew /*, TODO: mVCurve*/);
    this->callFocusFinderUpdateListener(& focusFinderDataNew);

    // Move focus back -> mNumStepsToDetermineDirection outwards
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

    LOG(info) << "Looking for rough focus in direction " << FocusDirectionT::asStr(inDirectionToImproveFocus) << "..." << endl;
    
    // Take picture
    StarDataT starDataNew, starDataPrev;
    takePictureCalcStarData(& starDataNew);
    LOG(trace) << dec << "Took picture - resulting star data: " << starDataNew << ", pos: " << mFocuserDevice->getAbsPos() << endl;
    
    // Send update to listeners
    FocusFinderDataT focusFinderData(mFocuserDevice->getAbsPos(), starDataNew /*, TODO: mVCurve*/);
    this->callFocusFinderUpdateListener(& focusFinderData);
    
    size_t iterCounter = 0;
    
    do {
      starDataPrev = starDataNew;
      
      //FOCUS_FINDER_STOP();
      
      // Move focus
      mFocuserDevice->moveFocusBy(mStepsToReachRoughFocus, inDirectionToImproveFocus);
      LOG(trace) << dec << "Moved focus " << FocusDirectionT::asStr(inDirectionToImproveFocus)
		 << " by " << mStepsToReachRoughFocus << " steps, pos: " << mFocuserDevice->getAbsPos() << endl;
      
      // Take picture
      takePictureCalcStarData(& starDataNew);
      LOG(trace) << dec << "Took picture - resulting star data new: " << starDataNew << ", pos: " << mFocuserDevice->getAbsPos() << endl;
      
      // Send update to listeners
      FocusFinderDataT focusFinderDataNew(mFocuserDevice->getAbsPos(), starDataNew /*, TODO: mVCurve*/);
      this->callFocusFinderUpdateListener(& focusFinderDataNew);
      
      // Print results to better compare it....
      LOG(debug) << dec << "Iteration: " << iterCounter << ", pos: " << mFocuserDevice->getAbsPos()
		 << ", HFD_prev=" << starDataPrev.getHfd().getValue() << ", HFD_new=" << starDataNew.getHfd().getValue() << endl
		 << ", Fwhm_horz_prev=" << starDataPrev.getFwhmHorz().getValue() << ", Fwhm_horz_new=" << starDataNew.getFwhmHorz().getValue() << endl
		 << ", Fwhm_vert_prev=" << starDataPrev.getFwhmVert().getValue() << ", Fwhm_vert_new=" << starDataNew.getFwhmVert().getValue() << endl;
      
      // Record VCurve..?!
      //mVCurve[inFocuserClient.getAbsPos()] = fwhmVertNew.getValue() + fwhmHorzNew.getValue(); // TODO: ok?! TODO: maybe pass mVCurve as param as well?!
      //mVCurve[inFocuserClient.getAbsPos()] = hfdNew.getValue(); // TODO: ok?!
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
      FocusFinderDataT focusFinderDataFinal(mFocuserDevice->getAbsPos(), starDataFinal /*, TODO: mVCurve*/);
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

    FocusDirectionT::TypeE direction = (inMinMaxFocusPos == MinMaxFocusPosT::MIN_FOCUS_POS ? FocusDirectionT::INWARDS : FocusDirectionT::OUTWARDS);
    LOG(info) << "Looking for " << MinMaxFocusPosT::asStr(inMinMaxFocusPos) << " in direction " << FocusDirectionT::asStr(direction) << "..." << endl;

    int startFocusPos = mFocuserDevice->getAbsPos();
    LOG(debug) << "startFocusPos: " << startFocusPos << endl;

    // Take picture
    StarDataT starData;
    takePictureCalcStarData(& starData);
    LOG(trace) << dec << "Took picture - resulting star data: " << starData << endl;

    // Send update to listeners
    FocusFinderDataT focusFinderData(mFocuserDevice->getAbsPos(), starData /*, TODO: mVCurve*/);
    this->callFocusFinderUpdateListener(& focusFinderData);


    // Loop until max. fitness has been reached
    while(mExtremaFitnessBoundary > starData.getFitness()) {
      LOG(debug) << dec << "Not yet reached mExtremaFitnessBoundary " << mExtremaFitnessBoundary
		 << ", measured fitness: " << starData.getFitness()
		 << ", current pos: " << mFocuserDevice->getAbsPos() << endl;

      //FOCUS_FINDER_STOP();

      // Move focuser
      mFocuserDevice->moveFocusBy(mStepsToReachRoughFocus /* TODO: Is this step size ok?! */, direction);
      LOG(trace) << dec << "Moved focus " << FocusDirectionT::asStr(direction) << " by " << mStepsToReachRoughFocus << " steps. " << endl;

      // Take picture
      takePictureCalcStarData(& starData);
      LOG(trace) << dec << "Took picture - resulting star data: " << starData << endl;
      
      // Send update to listeners
      FocusFinderDataT focusFinderData2(mFocuserDevice->getAbsPos(), starData /*, TODO: mVCurve*/);
      this->callFocusFinderUpdateListener(& focusFinderData2);
      
      //mVCurve[inFocuserClient.getAbsPos()] = inQualityMeasureStrategy->calculate(& fwhmHorz1, & fwhmVert1, & hfd1);
    }
    
    int extremaPos = mFocuserDevice->getAbsPos();
    double extremaFitness = starData.getFitness();

    LOG(debug) << "Ok, found " << MinMaxFocusPosT::asStr(inMinMaxFocusPos) << ": " << extremaPos << ", moving back to start position..." << endl;

    double delta = fabs(startFocusPos - extremaPos);

    // Move focuser back to start
    FocusDirectionT::TypeE backDirection = FocusDirectionT::invert(direction);
    mFocuserDevice->moveFocusBy(delta, backDirection);
    LOG(trace) << dec << "Moved focus back " << FocusDirectionT::asStr(backDirection) << " to start by " << delta << " steps." << endl;

    // Take picture
    takePictureCalcStarData(& starData);
    LOG(trace) << dec << "Took picture - resulting star data: " << starData << endl;
    
    // Send update to listeners
    FocusFinderDataT focusFinderData2(mFocuserDevice->getAbsPos(), starData /*, TODO: mVCurve*/);
    this->callFocusFinderUpdateListener(& focusFinderData2);      

    //mVCurve[inFocuserClient.getAbsPos()] = inQualityMeasureStrategy->calculate(& fwhmHorz1, & fwhmVert1, & hfd1);

    LOG(info) << dec << "Reached extrema (boundary is " << mExtremaFitnessBoundary
	      << "), looking for " << MinMaxFocusPosT::asStr(inMinMaxFocusPos)
	      << " in direction " << FocusDirectionT::asStr(direction)
	      << ", measured fitness: " << extremaFitness
	      << ", current pos: " << extremaPos << endl;
    
    return extremaPos; // focuser pos
  }

  void FocusFinderLinearInterpolationImplT::recordVCurve(int inAbsStartPos, int inAbsEndPos, size_t inGranularitySteps, VCurveT * outVCurve, bool inMoveBackToOldPos) {

    if (! outVCurve) {
      throw FocusFinderLinearInterpolationExceptionT("No vcurve passed.");
    }

    LOG(info) << "Start recording V-Curve..." << endl;
    LOG(debug) << "Moving to start position " << inAbsStartPos << "..." << endl;

    // Remember old focus position
    int oldFocusPos = mFocuserDevice->getAbsPos();

    // Move focuser to start position
    int relMinDelta = mFocuserDevice->getAbsPos() - inAbsStartPos;
    mFocuserDevice->moveFocusBy(relMinDelta, FocusDirectionT::INWARDS);
    LOG(trace) << dec << "Moved focus " << FocusDirectionT::asStr(FocusDirectionT::INWARDS) << " by " << relMinDelta << " steps. " << endl;
    
    int curPos = mFocuserDevice->getAbsPos();
    int delta = inGranularitySteps;
    
    StarDataT starData;

    // TODO: STOP-CONDITION?!... --> Use bool member which can be set from outside...asynchronously.......
    while (curPos < inAbsEndPos) {
      //   FOCUS_FINDER_STOP();
      
      // Take picture
      takePictureCalcStarData(& starData);
      LOG(trace) << dec << "Took picture - resulting star data: " << starData << endl;
      
      // Send update to listeners
      FocusFinderDataT focusFinderData(mFocuserDevice->getAbsPos(), starData /*, TODO: mVCurve*/);
      this->callFocusFinderUpdateListener(& focusFinderData);      
      
      // Add position + fitness to VCurve
      outVCurve->insert(make_pair(curPos, starData.getFitness()));

      // Move focuser
      mFocuserDevice->moveFocusBy(delta, FocusDirectionT::OUTWARDS);
      LOG(trace) << dec << "Moved focus " << FocusDirectionT::asStr(FocusDirectionT::OUTWARDS) << " by " << delta << " steps. " << endl;
      
      curPos += delta;
    }
    
    if (inMoveBackToOldPos) {
      size_t deltaBack = fabs(curPos - oldFocusPos);
      mFocuserDevice->moveFocusBy(deltaBack, FocusDirectionT::INWARDS);
      LOG(trace) << dec << "Moved focus back " << FocusDirectionT::asStr(FocusDirectionT::OUTWARDS)
		 << " to old position " << oldFocusPos << " by " << deltaBack << " steps. " << endl;
    }
  }
}; // end AT namespace
