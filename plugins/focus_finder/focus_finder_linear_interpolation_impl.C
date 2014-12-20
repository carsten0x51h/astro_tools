// TODO
// Test in libreoffice
// (x, y) = (22850, 19.5919)
// (x, y) = (23350, 18.7533)
// (x, y) = (23850, 19.2931)
// (x, y) = (24350, 18.407)
// (x, y) = (24850, 18.9501)
// (x, y) = (25350, 17.0735)
// (x, y) = (25850, 18.5735)
// (x, y) = (26350, 16.614)
// (x, y) = (26850, 17.0103)
// (x, y) = (27350, 16.3023)
// (x, y) = (27850, 15.402)
// (x, y) = (28350, 17.3895)
// (x, y) = (28850, 15.8039)
// (x, y) = (29350, 15.8986)
// (x, y) = (29850, 15.3658)
// (x, y) = (30350, 13.7889)
// (x, y) = (30850, 13.4612)
// (x, y) = (31350, 13.1746)
// (x, y) = (31850, 15.0217)
// (x, y) = (32350, 12.9477)
// (x, y) = (32850, 13.1476)
// (x, y) = (33350, 12.2441)
// (x, y) = (33850, 14.4243)
// (x, y) = (34350, 11.6641)
// (x, y) = (34850, 12.0535)
// (x, y) = (35350, 12.258)
// (x, y) = (35850, 10.8592)
// (x, y) = (36350, 12.6032)
// (x, y) = (36850, 13.3219)
// (x, y) = (37350, 12.9228)
// (x, y) = (37850, 13.7499)
// (x, y) = (38350, 12.9885)
// (x, y) = (38850, 13.0212)
// (x, y) = (39350, 12.268)
// (x, y) = (39850, 14.2114)
// (x, y) = (40350, 14.9318)
// (x, y) = (40850, 15.5126)
// (x, y) = (41350, 13.1647)
// (x, y) = (41850, 14.1658)
// (x, y) = (42350, 13.6438)
// (x, y) = (42850, 15.907)
// (x, y) = (43350, 14.5974)
// (x, y) = (43850, 14.7542)
// (x, y) = (44350, 14.0117)
// (x, y) = (44850, 15.3519)
// (x, y) = (45350, 15.3365)
// (x, y) = (45850, 17.5144)
// (x, y) = (46350, 16.8715)
// (x, y) = (46850, 16.9312)
// (x, y) = (47350, 17.7513)
// (x, y) = (47850, 17.2565)
// (x, y) = (48350, 17.2912)
// (x, y) = (48850, 18.6475)
// (x, y) = (49350, 19.1396)
// (x, y) = (49850, 19.1723)


// xMin: 35850, yMin: 10.8592
// Guessing a=1, b=-71700, c=1.28522e+09
// [2014-Dec-19 22:14:10.145044]: Calculated parabel parms - a: 3.80037e-08, b: -0.00278398, c: 63.9065


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

    double roughOptAbsFocusPos = this->findOptimalFocusInRange(roughAbsStartPos, roughAbsEndPos, mRoughFocusRecordNumCurves, mRoughFocusGranularitySteps);

    StarDataT roughStarData;
    takePictureCalcStarData(& roughStarData);
    LOG(info) << dec << "Took picture - resulting star data: " << roughStarData << ", pos: " << mFocuserDevice->getAbsPos() << endl;

    LOG(debug) << dec << "Rough focus determined: " << roughOptAbsFocusPos << ", starData: " << roughStarData << endl;


    // Fine focus search
    // From roughOptAbsFocusPos calc start and end pos.
    int fineAbsStartPos = roughOptAbsFocusPos - mFineSearchRangeSteps;
    int fineAbsEndPos = roughOptAbsFocusPos + mFineSearchRangeSteps;

    LOG(debug) << dec << "roughOptAbsFocusPos: " << roughOptAbsFocusPos << ", mFineSearchRangeSteps: " << mFineSearchRangeSteps
	       << ", fine search range=[" << fineAbsStartPos << ", " << fineAbsEndPos << "]..." << endl;

    double fineOptAbsFocusPos = this->findOptimalFocusInRange(fineAbsStartPos, fineAbsEndPos, mFineFocusRecordNumCurves, mFineFocusGranularitySteps);

    // Determine final seeing (average)
    const size_t _numSingleFrames = 5;
    float fineStarDataFitnessSum = 0;

    for (size_t i = 0; i < _numSingleFrames; ++i) {
      // Take picture, calc star values and send update
      StarDataT fineStarData;
      takePictureCalcStarData(& fineStarData);
      LOG(info) << dec << "Took picture - resulting star data: " << fineStarData << ", pos: " << mFocuserDevice->getAbsPos() << endl;

      fineStarDataFitnessSum += fineStarData.getFitness();
      
      // Send update to listeners
      FocusFinderDataT focusFinderData(mFocuserDevice->getAbsPos(), fineStarData /*, TODO: mVCurve*/);
      this->callFocusFinderUpdateListener(& focusFinderData);
    }

    float meanFineStarDataFitness = fineStarDataFitnessSum / (float) _numSingleFrames;

    // Logging summary
    LOG(info) << dec << "Rough focus found at: " << roughOptAbsFocusPos << ", starData: " << roughStarData << endl;
    LOG(info) << dec << "Fine focus found at: " << fineOptAbsFocusPos << ", mean fine star data fitness: " << meanFineStarDataFitness << endl;
  }

  double
  FocusFinderLinearInterpolationImplT::findOptimalFocusInRange(int inAbsStartPos, int inAbsEndPos, size_t inNumVCurves, size_t inCurveGranularitySteps) {
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
    
    return optAbsFocusPos;
  }


  double
  FocusFinderLinearInterpolationImplT::calcOptimalAbsFocusPos(const VCurveVecT & inVCurves) {
    // TODO: Implement retry... if fitting does not work - but how?!
    //-> take another picture?!?! or just increase possible error by *2 ?!
    //-> Maybe there is a better solution by just still using the results?? ...
    //   Because the results will be the same if data does not change!! Maybe add "noExIfErrorCondNotFulfilled" ??!
    typedef CurveFitTmplT<ParabelFitTraitsT> ParabelMatcherT;

    AT_ASSERT(FocusFinderLinearInterpolation, inVCurves.size(), "VCurve vector is empty!");

    VCurveT masterVCurve;
    for (VCurveVecT::const_iterator itCurve = inVCurves.begin(); itCurve != inVCurves.end(); ++itCurve) {
      masterVCurve += *itCurve;
    }
    
    // Calculate average
    masterVCurve /= (double) inVCurves.size();

    LOG(info) << "Master VCurve: " << masterVCurve << endl;
  
    // Do the LM fit - TODO: this function should throw if it does not succeed?!
    ParabelMatcherT::ParamsT parabelParms;
    int err = ParabelMatcherT::fitGslLevenbergMarquart(VCurveAccessorT(masterVCurve), & parabelParms, mVCurveFitEpsAbs, mVCurveFitEpsRel);

    float a = parabelParms[ParabelMatcherT::IdxT::A_IDX];
    float b = parabelParms[ParabelMatcherT::IdxT::B_IDX];
    float c = parabelParms[ParabelMatcherT::IdxT::C_IDX];
  
    if (err) {
      stringstream ss;
      ss << "FocusFinderLinearInterpolationImplT::calcOptimalAbsFocusPos - fitgsl_lm() returned non-zero status: " << err
	 << ", a: " << a << ", b: " << b << ", c: " << c << endl;
      throw CurveFitExceptionT(ss.str().c_str());
    }
    
    LOG(debug) << "Calculated parabel parms - a: " << a << ", b: " << b << ", c: " << c << endl;
    
    // Calculate xmin, ymin
    float xMin = - b / (2.0f * a);
    float yMin = - 0.25f * (b * b) / a + c;

    LOG(debug) << "xMin: " << xMin << ", yMin: " << yMin << endl;

    // TODO: Adapt...
    // vector<float> & fitValues = (*outFitValues);
    // for(size_t i = 0; i < imgValues.size() && i < FwhmT::MAX_PTS; ++i) {
    //   fitValues[i] = GaussianFitTraitsT::fx(i, /*TODO: was - still works? dataPoints[i].x*/, *outGaussParms);
    // }

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
