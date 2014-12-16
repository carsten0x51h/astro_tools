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
  const size_t FocusFinderLinearInterpolationImplT::sWindowSize = 31; // px
  
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

    // TODO: Put to another function?!
    // From Fmin and Fmax calculate start and end position
    double centerPos = Fmin + fabs(Fmax - Fmin) / 2;
    double halfLength = fabs(Fmax - centerPos);
    const size_t perc = 70; // TODO: Make configurable?!
    double percLength = halfLength * 0.01 * (double) perc;

    int absStartPos = centerPos - percLength;
    int absEndPos = centerPos + percLength;

    LOG(debug) << dec << "centerPos: " << centerPos << ", halfLength: " << halfLength << ", perc: " << perc << "%, percLength: " << percLength << endl;
    LOG(info) << dec << "Calculated start/end positions [start, end]=[" << absStartPos << ", " << absEndPos << "]" << endl;

    // Record VCurves
    size_t numVCurves = 2; // TODO: Make as member? Make configurable?!
    size_t vCurveGranularitySteps = 500; // TODO: Pass / Calculate?!

    VCurveVecT vcurves(numVCurves); // Contains empty VCurves
    
    LOG(info) << dec << "Recording " << numVCurves << " VCurves with "
	      << (fabs(absEndPos - absStartPos) / vCurveGranularitySteps) << " points each..." << endl;

    for (VCurveVecT::iterator it = vcurves.begin(); it != vcurves.end(); ++it) {
      LOG(info) << dec << " > Recording VCurve " << std::distance(vcurves.begin(), it) << "..." << endl;

      VCurveT & curVCurve = *it;
      this->recordVCurve(absStartPos, absEndPos, vCurveGranularitySteps, & curVCurve, false /*do not move back to old pos*/);

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
    LOG(info) << dec << "Took FINAL picture - resulting star data: " << starData << ", pos: " << mFocuserDevice->getAbsPos() << endl;

    // Send update to listeners
    FocusFinderDataT focusFinderData(mFocuserDevice->getAbsPos(), starData /*, TODO: mVCurve*/);
    this->callFocusFinderUpdateListener(& focusFinderData);
  }

  double
  FocusFinderLinearInterpolationImplT::calcOptimalAbsFocusPos(const VCurveVecT & inVCurves) {
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
    double epsAbs = 5e-2; // TODO: As parameter?!
    double epsRel = 5e-2; // TODO: As parameter?!;

    // Allocate temporary data buffer
    // TODO: Should be handled internally
    fitgsl_data * dat = ParabelMatcherT::fitgsl_alloc_data(masterVCurve.size());
  
    // Fill data
    size_t idx=0;
    for (VCurveT::const_iterator it = masterVCurve.begin(); it != masterVCurve.end(); ++it, ++idx) {
      dat->pt[idx].x = it->first;
      dat->pt[idx].y = it->second;
    }

    // Do the LM fit - TODO: this function should throw if it does not succeed?!
    int err = ParabelMatcherT::fitgsl_lm(dat, & parabelParms, epsAbs, epsRel);
  
    if (err) {
      // Free allocated data
      // TODO: Should be handled internally
      ParabelMatcherT::fitgsl_free_data(dat);
      
      stringstream ss;
      ss << "FocusFinderLinearInterpolationImplT::calcOptimalAbsFocusPos - fitgsl_lm() returned non-zero status: " << err << endl; 
      throw CurveFittingExceptionT(ss.str().c_str());
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

    // Transfer over the data to the global storage. Plot the true points onto the graph as well.
    // TODO: Required?! -> use VCurve instead!?
    // struct PointT {
    //   float x, y;
    //   PointT(float inX = 0, float inY = 0) : x(inX), y(inY) {}
    // };
    // vector<PointT> fitValues(masterVCurve.size());
    // cerr << "Parabel fit values:" << endl;
    // size_t i;
    // for(i = 0; i < values.size(); ++i) {
    //   fitValues[i].x = dat->pt[i].x;
    //   fitValues[i].y = ParabelFitTraitsT::fx(dat->pt[i].x, parabelParms);
    //   cerr << "x=" << fitValues[i].x << ", y=" << fitValues[i].y << endl;
    // }
  
    // Free allocated data
    // TODO: Should be handled internally
    ParabelMatcherT::fitgsl_free_data(dat);

    return xMin;
  }

  void
  FocusFinderLinearInterpolationImplT::takePictureCalcStarData(StarDataT * outStarData, CImg<float> * outImg) {
    AT_ASSERT(FocusFinderLinearInterpolation, outStarData, "outStarData expected to be set!");

    const size_t sHfdOuterRadiusPx = 5; // TODO: Where to put?! --> Should not be passed by default... should either be calculated by HfdT, and / or HfdT should have a good default...
    const size_t maxRetryCnt = 5; // TODO: Pass as parameter?!

    CImg<float> img;
    
    // Calc frameSize from inStarCenterPos
    FrameT imgFrame = centerPosToFrame(mStarCenterPos, FocusFinderLinearInterpolationImplT::sWindowSize);

    // Calc star values, throws if star could not be determined
    size_t retryCnt = 0;
    while(retryCnt < maxRetryCnt) {
      try {
	// Take a picture, throws if not connected or problem with device
	mCameraDevice->takePicture(& img, mExposureTimeSec, imgFrame, FrameTypeT::LIGHT, mBinning, false /* not compressed */);

	outStarData->getFwhmHorz().set(img, FwhmT::DirectionT::HORZ);
	outStarData->getFwhmVert().set(img, FwhmT::DirectionT::VERT);
	outStarData->getHfd().set(img, sHfdOuterRadiusPx);
	break; // success

      } catch (CurveFittingExceptionT & exc) {
	// Fitting did not work as expected...
	LOG(warning) << "Fitting did not work as expected...retry " << (retryCnt+1) << " / " << maxRetryCnt << endl;
	++retryCnt;
      }
    }

    if (maxRetryCnt <= retryCnt) {
      throw CurveFittingExceptionT("Could not fit curve even after retrying.");
    }

    if (outImg)
      *outImg = img; // Copy image if desired

    // TODO: Enable by switch?! Put to another function?
    // DEBUG - DISPLAY IMG
    // CImgDisplay disp1(img, "image");
    // while(! disp1.is_closed()) { CImgDisplay::wait(disp1); }
    // DEBUG END
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
    
    const size_t maxIterCnt = 20; // TODO: configure? What is a good value? Depends on mStepsToReachRoughFocus...?
    
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
      
    } while (iterCounter < maxIterCnt && starDataNew.getFitness() < starDataPrev.getFitness()); // end while
    
    // Did we find a rough focus within max amount of iterations?
    if (iterCounter < maxIterCnt) {
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

    // TODO / FIXME - Experimental: Fitness boundary... we may pass this as parameter..
    // TODO: We may pass maxFitness instead...
    const unsigned int maxFitnessValue = 25;

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
    while(maxFitnessValue > starData.getFitness()) {
      LOG(debug) << dec << "Not yet reached maxFitnessValue " << maxFitnessValue
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

    // double delta = (inMinMaxFocusPos == MinMaxFocusPosT::MIN_FOCUS_POS ? startFocusPos - extremaPos : extremaPos - startFocusPos);
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

    LOG(info) << dec << "Reached maxFitnessValue " << maxFitnessValue
	      << " looking for " << MinMaxFocusPosT::asStr(inMinMaxFocusPos)
	      << " in direction " << FocusDirectionT::asStr(direction)
	      << ", measured fitness: " << extremaFitness
	      << " (boundary is " << maxFitnessValue << ")"
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
  




// bool FocusFinderT::findFocus(int inCenterX, unsigned int inCenterY, int inSize, double inExposureTime, unsigned int inOuterHfdRadius, unsigned int numStepsToDetermineDirection, unsigned int inStepsToReachRoughFocus, bool * outUserStop, ostream & os, const QualityMeasureStrategyT * inQualityMeasureStrategy) {
//   if (! mIndiClient)
//     throw FocusFinderExceptionT("No indi client set.");
  
//   if (mCameraDeviceName.empty())
//     throw FocusFinderExceptionT("No camera device name set.");
  
//   if (mFocuserDeviceName.empty())
//     throw FocusFinderExceptionT("No focuser device name set.");
  
//   if (outUserStop)
//     *outUserStop = false;
  
//   // Camera & focuser access helper
//   IndiCameraClientT cameraClient(mIndiClient, mCameraDeviceName);
//   IndiFocuserClientT focuserClient(mIndiClient, mFocuserDeviceName);
  
//   const int BACK = ceil(inSize / 2.0) - 1;
//   int startX = (int) inCenterX - BACK;
//   int startY = (int) inCenterY - BACK;

//   os << ">>>> FRAME IS!!! X: " << startX << ", Y: " << startY << ", w: " << FocusFinderT::sWindowWidthPx << ", h: " << FocusFinderT::sWindowHeightPx << endl;

//   CImg<float> img;
//   IndiFocuserClientT::FocusDirectionT::TypeE directionToImproveFocus;

//   // TODO: Many parameters may be set as class members!
//   if (! determineInitialDirection(os, cameraClient, focuserClient, & img, startX, startY, inExposureTime, inOuterHfdRadius, numStepsToDetermineDirection, & directionToImproveFocus))
//     return false;
  
//   if (! findRoughFocus(os, cameraClient, focuserClient, & img, startX, startY, inExposureTime, inOuterHfdRadius, numStepsToDetermineDirection, directionToImproveFocus, outUserStop, inStepsToReachRoughFocus))
//     return false;

//   // -Determine rough relative focus boundaries (in: FWHM/HFD BOUNDARY, out; dFmin [steps], dFmax [steps])
//   os << "Determine rough relative focus boundaries (in: FWHM/HFD BOUNDARY, out; dFmin [steps], dFmax [steps]..." << endl;
//   int Fmin = 0, Fmax = 0;

//   mVCurve.clear();

//   if (! findExtrema(os, cameraClient, focuserClient, & img, startX, startY, inExposureTime, inOuterHfdRadius, numStepsToDetermineDirection, directionToImproveFocus, outUserStop, inStepsToReachRoughFocus, MinMaxFocusPosT::MIN_FOCUS_POS, & Fmin, inQualityMeasureStrategy))
//       return false;

//   if (! findExtrema(os, cameraClient, focuserClient, & img, startX, startY, inExposureTime, inOuterHfdRadius, numStepsToDetermineDirection, directionToImproveFocus, outUserStop, inStepsToReachRoughFocus, MinMaxFocusPosT::MAX_FOCUS_POS, & Fmax, inQualityMeasureStrategy))
//       return false;

//   os << "Ok, extrema determined - FminPos: " << Fmin << ", FmaxPos: " << Fmax << endl;


//   mVCurve.clear();

//   if (! recordVCurve(os, cameraClient, focuserClient, & img, startX, startY, inExposureTime, inOuterHfdRadius, numStepsToDetermineDirection, directionToImproveFocus, outUserStop, inStepsToReachRoughFocus, Fmin, Fmax, inQualityMeasureStrategy))
//     return false;


  

//   // Move to opt. focus pos - TODO: sep. function...
//   int xpos;
//   double ymin = mVCurve.getMinY(& xpos);
//   os << "Min FWHM: " << ymin << ", opt focus pos: " << xpos << endl;

//   int curFocusPos = focuserClient.getAbsPos();
//   int delta;
//   IndiFocuserClientT::FocusDirectionT::TypeE direction;

//   if(curFocusPos - xpos > 0) {    
//     delta = curFocusPos - xpos; // Move "left" (INWARDS)
//     direction = IndiFocuserClientT::FocusDirectionT::INWARDS;
//   } else {
//     delta = xpos - curFocusPos; // Move "right" (OUTWARDS)
//     direction = IndiFocuserClientT::FocusDirectionT::OUTWARDS;
//   }
//   focuserClient.moveFocusBy(delta, direction);

//   os << "Moved to optimal focus pos: " << focuserClient.getAbsPos() << " (should be " << xpos << ")..." << endl;

//   cameraClient.takePicture(& img, 1 /*horz bin*/, 1 /*vert bin*/, startX /*x*/, startY /*y*/, FocusFinderT::sWindowWidthPx /*w*/, FocusFinderT::sWindowHeightPx /*h*/, FrameTypeT::LIGHT, inExposureTime /*s*/, false /*no compr.*/);

//   FwhmT fwhmHorz1(img, FwhmT::DirectionT::HORZ);
//   FwhmT fwhmVert1(img, FwhmT::DirectionT::VERT);
//   HfdT hfd1(img, inOuterHfdRadius /*radius px*/);
  
//   FocusFinderDataT ffd1(focuserClient.getAbsPos(), fwhmHorz1, fwhmVert1, hfd1, mVCurve);
//   mFocusFinderUpdateListeners(& ffd1);
  
  

  
//   // -Determine optimal focus position from V-Curve(s) (requires valid V-Curve(s)) (in: V-Curves (a V-Curve is a list of relative focusPos & FWHM/HFD pairs))
//   //    -1. Build one average V-Curve (all V-Curves must be within same relative Fmin & Fmax) (Note: Then for each focusPos there is exactly one HFD & FWHM value per V-Curve)
//   //       -For idx = Fmin to Fmax
//   //          -For all V-Curves
//   //             -Sum the idx value of all V-Curves
//   //          -Divide accumulated idx value by the number of V-Curves

//   //    -2. Separate V-Curve in decreasing data-points and increasing data-points, minimum is to be included in both sets (TODO: Really?))
//   //       -TODO...

//   //    -3. Determine optimal focus position 
//   //       NOTE: Important condition: f_decreasing_fwhm&hfd(pos) = f_increasing_fwhm&hfd(pos) == FWHM == HFD == 0!! --> pos is optimal focus position
//   //           --> Find a_1, b_1 & a_2, b_2 of both functions so that (MSE1 + MSE2) is minimized
//   //           --> Maybe hard to formulate in gsl?! --> We may implement this manually....



//   // -Move to optimal focus position (in: optimal focus position)
//   //    -TODO: Does this involve any corrections (feedback?)

//   return true; // Focus found!
// }

}; // end AT namespace



// const size_t FocusFinderT::sWindowWidthPx = 15;
// const size_t FocusFinderT::sWindowHeightPx = 15;
// const size_t FocusFinderT::sHfdOuterRadiusPx = 5;


// const size_t FocusFinderT::sHfdOuterRadiusPx = 15;

// FocusFinderT::FocusFinderT(IndiClientTmplT * inIndiClient, const string & inCameraDeviceName, const string & inFocuserDeviceName) : mIndiClient(inIndiClient), mCameraDeviceName(inCameraDeviceName), mFocuserDeviceName(inFocuserDeviceName), mStop(false) {

//   if (! mIndiClient)
//     throw FocusFinderExceptionT("No indi client set.");
  
//   // Number change listener
//   mIndiClient->registerNumberListener(boost::bind(& FocusFinderT::numberChangeHandler, this, _1));
// }

// FocusFinderT::~FocusFinderT() {
//   cerr << "FocusFinderT::~FocusFinderT()... mIndiClient: " << mIndiClient << endl;
//   if (mIndiClient)
//     mIndiClient->unregisterNumberListener(boost::bind(& FocusFinderT::numberChangeHandler, this, _1));
// }

// void FocusFinderT::numberChangeHandler(INumberVectorProperty * inVecNumber) {
//   if (! inVecNumber)
//     return;

//   if (! strcmp(inVecNumber->name, IndiFocuserClientT::VecPropNameT::asStr(IndiFocuserClientT::VecPropNameT::ABS_FOCUS_POSITION))) {
//     double absFocuserValue = 0;
//     mIndiClient->getNumberChecked(mFocuserDeviceName, IndiFocuserClientT::VecPropNameT::asStr(IndiFocuserClientT::VecPropNameT::ABS_FOCUS_POSITION), IndiFocuserClientT::PropNameT::asStr(IndiFocuserClientT::PropNameT::RELATIVE_ABSOLUTE_POSITION), & absFocuserValue);

//     FocusFinderDataT ffd(absFocuserValue, FwhmT(), FwhmT(), HfdT(), mVCurve); // TODO: Ok?! Not so nice?!
//     mFocusFinderUpdateListeners(& ffd);
//   }

//   return;
// }

// // TODO: Required?!
// // void FocusFinderT::recenter(ostream & os, IndiCameraClientT & inCameraClient, IndiFocuserClientT & inFocuserClient, CImg<float> * outImg, int * outStartX, int * outStartY, double inExposureTime, unsigned int inOuterHfdRadius, unsigned int numStepsToDetermineDirection) {
// //   cerr << "FocusFinderT::recenter..." << endl;

// //   float centerX, centerY;
// //   try {
// //     CentroidCalcT::starCentroid(*outImg, 0 /*x*/, 0 /*y*/, FocusFinderT::sWindowWidthPx, FocusFinderT::sWindowHeightPx, & centerX, & centerY, CoordTypeT::ABSOLUTE, false /* no debug msgs */);
// //   } catch(CentroidExceptionT & exc) {
// //     cerr << "Unable to determine centroid. Leaving it as is." << endl;
// //     return;
// //   }

// //   const int BACK = ceil(FocusFinderT::sWindowWidthPx / 2.0) - 1;
// //   int deltaX = (int) centerX - BACK;
// //   int deltaY = (int) centerY - BACK;
    
// //   if (deltaX || deltaY) {
// //     os << ">>>> RE-CENTERING by (dx, dy)=(" << deltaX << ", " << deltaY << ") -> startX: " << *outStartX << ", startY: " << *outStartY << ", w: " << FocusFinderT::sWindowWidthPx << ", h: " << FocusFinderT::sWindowHeightPx << ", taking new image..." << endl;
// //     *outStartX = *outStartX - deltaX;
// //     *outStartY = *outStartY - deltaY;
// //     inCameraClient.takePicture(outImg, 1 /*horz bin*/, 1 /*vert bin*/, *outStartX /*x*/, *outStartY /*y*/, FocusFinderT::sWindowWidthPx /*w*/, FocusFinderT::sWindowHeightPx /*h*/, FrameTypeT::LIGHT, inExposureTime /*s*/, false /*no compr.*/);
// //   }
// // }
