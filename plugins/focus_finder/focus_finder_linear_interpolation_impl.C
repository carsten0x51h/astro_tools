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
    
    FocusDirectionT::TypeE direction = this->determineInitialDirection();
    LOG(info) << dec << "Direction of improvement: " << FocusDirectionT::asStr(direction) << "..." << endl;

    this->findRoughFocus(direction);
    LOG(info) << dec << "Found rough focus at abs pos: " << mFocuserDevice->getAbsPos() << "..." << endl;

    int Fmin = this->findExtrema(direction, MinMaxFocusPosT::MIN_FOCUS_POS);
    int Fmax = this->findExtrema(direction, MinMaxFocusPosT::MAX_FOCUS_POS);
    LOG(info) << dec << "Determined focus boundaries [min, max]=[" << Fmin << ", " << Fmax << "]..." << endl;


    LOG(trace) << dec << "FocusFinderLinearInterpolationImplT::findFocus() - Leaving..." << endl;
  }


  void FocusFinderLinearInterpolationImplT::takePictureCalcStarData(StarDataT * outStarData, CImg<float> * outImg) {
    AT_ASSERT(FocusFinderLinearInterpolation, outStarData, "outStarData expected to be set!");
    
    CImg<float> img;
    
    // Calc frameSize from inStarCenterPos
    FrameT imgFrame = centerPosToFrame(mStarCenterPos, FocusFinderLinearInterpolationImplT::sWindowSize);

    // Take a picture, throws if not connected or problem with device
    mCameraDevice->takePicture(& img, mExposureTimeSec, imgFrame, FrameTypeT::LIGHT, mBinning, false /* not compressed */);

    // TODO: Where to put?! --> Should not be passed by default... should either be calculated by HfdT, and / or HfdT should have a good default...
    const size_t sHfdOuterRadiusPx = 10;
    
    // Calc star values, throws if star could not be determined
    outStarData->getFwhmHorz().set(img, FwhmT::DirectionT::HORZ);
    outStarData->getFwhmVert().set(img, FwhmT::DirectionT::VERT);
    outStarData->getHfd().set(img, sHfdOuterRadiusPx);
    
    if (outImg)
      *outImg = img; // Copy image if desired
  }
  

  FocusDirectionT::TypeE
  FocusFinderLinearInterpolationImplT::determineInitialDirection() {
    LOG(trace) << "Determining initial focuser direction...";
	
    // Take picture, calc star values and send update
    StarDataT starData;
    takePictureCalcStarData(& starData);
    LOG(trace) << dec << "Took first picture - resulting star data: " << starData << endl;

    // Send update to listeners
    FocusFinderDataT focusFinderData(mFocuserDevice->getAbsPos(), starData /*, TODO: mVCurve*/);
    this->callFocusFinderUpdateListener(& focusFinderData);

    // Move mNumStepsToDetermineDirection inwards
    mFocuserDevice->moveFocusBy(mNumStepsToDetermineDirection, FocusDirectionT::INWARDS);
    LOG(trace) << dec << "Moved focus " << FocusDirectionT::asStr(FocusDirectionT::INWARDS) << " by " << mNumStepsToDetermineDirection << " steps. " << endl;

    // Take another picture, calc star values and send update
    StarDataT starDataNew;
    takePictureCalcStarData(& starDataNew);
    LOG(trace) << dec << "Took second picture - resulting star data: " << starDataNew << endl;

    // Send update to listeners
    FocusFinderDataT focusFinderDataNew(mFocuserDevice->getAbsPos(), starDataNew /*, TODO: mVCurve*/);
    this->callFocusFinderUpdateListener(& focusFinderDataNew);

    // Move focus back -> mNumStepsToDetermineDirection outwards
    mFocuserDevice->moveFocusBy(mNumStepsToDetermineDirection, FocusDirectionT::OUTWARDS);
    LOG(trace) << dec << "Moved focus back " << FocusDirectionT::asStr(FocusDirectionT::OUTWARDS)
	       << " by " << mNumStepsToDetermineDirection << " steps. " << endl;

    // Calc direction of improvement
    bool fwhmHorzCmp = starData.getFwhmHorz().getValue() < starDataNew.getFwhmHorz().getValue();
    bool fwhmVertCmp = starData.getFwhmVert().getValue() < starDataNew.getFwhmVert().getValue();
    bool hfdCmp = starData.getHfd().getValue() < starDataNew.getHfd().getValue();

    LOG(trace) << dec << "Compare data - fwhmHorzCmp: " << fwhmHorzCmp << ", fwhmVertCmp: " << fwhmVertCmp << ", hfdCmp: " << hfdCmp << endl;


    FocusDirectionT::TypeE directionToImproveFocus;

    if (fwhmHorzCmp && fwhmVertCmp && hfdCmp) {
      directionToImproveFocus = FocusDirectionT::INWARDS;
    } else if (! fwhmHorzCmp && ! fwhmVertCmp && ! hfdCmp) {
      directionToImproveFocus = FocusDirectionT::OUTWARDS;
    } else {
      stringstream ssEx;
      ssEx << "Could not determine initial focuser direction of improvement - "
	   << "star data are inconsistent. Maybe select another star or increase numStepsToDetermineDirection." << endl
	   << "StarData1: " << starData << "StarData2: " << starDataNew << endl;

      throw FocusFinderLinearInterpolationExceptionT(ssEx.str().c_str());
    }

    LOG(info) << dec << "Direction which improves focus is: " << FocusDirectionT::asStr(directionToImproveFocus) << endl;
    LOG(debug) << dec << "StarData1: " << starData << endl << "StartData2: " << starDataNew << endl;
    
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

    LOG(trace) << "Finding rough focus...";
    
    const size_t maxIterCnt = 20; // TODO: configure? What is a good value? Depends on mStepsToReachRoughFocus...?
    
    // Take picture
    StarDataT starDataNew, starDataPrev;
    takePictureCalcStarData(& starDataNew);
    LOG(trace) << dec << "Took picture - resulting star data: " << starDataNew << endl;
    
    // Send update to listeners
    FocusFinderDataT focusFinderData(mFocuserDevice->getAbsPos(), starDataNew /*, TODO: mVCurve*/);
    this->callFocusFinderUpdateListener(& focusFinderData);
    
    size_t iterCounter = 0;
    //StarDataT starDataCur;
    
    do {
      starDataPrev = starDataNew;
      
      //FOCUS_FINDER_STOP();
      
      // Move focus
      mFocuserDevice->moveFocusBy(mStepsToReachRoughFocus, inDirectionToImproveFocus);
      LOG(trace) << dec << "Moved focus " << FocusDirectionT::asStr(inDirectionToImproveFocus)
		 << " by " << mNumStepsToDetermineDirection << " steps. " << endl;
      
      // Take picture
      takePictureCalcStarData(& starDataNew);
      LOG(trace) << dec << "Took picture - resulting star data new: " << starDataNew << endl;
      
      // Send update to listeners
      FocusFinderDataT focusFinderDataNew(mFocuserDevice->getAbsPos(), starDataNew /*, TODO: mVCurve*/);
      this->callFocusFinderUpdateListener(& focusFinderDataNew);
      
      // Print results to better compare it....
      LOG(debug) << dec << "Iteration: " << iterCounter
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
      LOG(debug) << dec << "Moved focus back " << FocusDirectionT::asStr(backDirection) << " by " << mStepsToReachRoughFocus << " steps. " << endl;
      
      // Take picture
      StarDataT starDataFinal;
      takePictureCalcStarData(& starDataFinal);
      LOG(trace) << dec << "Took picture - resulting star data (final): " << starDataFinal << endl;
      
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
    LOG(trace) << "Looking for " << MinMaxFocusPosT::asStr(inMinMaxFocusPos) << " in direction " << FocusDirectionT::asStr(direction) << "..." << endl;

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
      //FOCUS_FINDER_STOP();

      // Move focuser
      mFocuserDevice->moveFocusBy(mStepsToReachRoughFocus /* TODO: Is this step size ok?! */, direction);
      LOG(debug) << dec << "Moved focus " << FocusDirectionT::asStr(direction) << " by " << mStepsToReachRoughFocus << " steps. " << endl;

      // Take picture
      takePictureCalcStarData(& starData);
      LOG(trace) << dec << "Took picture - resulting star data: " << starData << endl;
      
      // Send update to listeners
      FocusFinderDataT focusFinderData2(mFocuserDevice->getAbsPos(), starData /*, TODO: mVCurve*/);
      this->callFocusFinderUpdateListener(& focusFinderData2);
      
      //mVCurve[inFocuserClient.getAbsPos()] = inQualityMeasureStrategy->calculate(& fwhmHorz1, & fwhmVert1, & hfd1);
    }


    int extremaPos = mFocuserDevice->getAbsPos();

    LOG(debug) << "Ok, found " << MinMaxFocusPosT::asStr(inMinMaxFocusPos) << ": " << extremaPos << ", moving back to start position..." << endl;

    // double delta = (inMinMaxFocusPos == MinMaxFocusPosT::MIN_FOCUS_POS ? startFocusPos - extremaPos : extremaPos - startFocusPos);
    double delta = fabs(startFocusPos - extremaPos);

    // Move focuser back to start
    FocusDirectionT::TypeE backDirection = FocusDirectionT::invert(direction);
    mFocuserDevice->moveFocusBy(mStepsToReachRoughFocus, backDirection);
    LOG(debug) << dec << "Moved focus back " << FocusDirectionT::asStr(backDirection) << " to start by " << mStepsToReachRoughFocus << " steps." << endl;

    // Take picture
    takePictureCalcStarData(& starData);
    LOG(debug) << dec << "Took picture - resulting star data: " << starData << endl;
    
    // Send update to listeners
    FocusFinderDataT focusFinderData2(mFocuserDevice->getAbsPos(), starData /*, TODO: mVCurve*/);
    this->callFocusFinderUpdateListener(& focusFinderData2);      

    //mVCurve[inFocuserClient.getAbsPos()] = inQualityMeasureStrategy->calculate(& fwhmHorz1, & fwhmVert1, & hfd1);
    
    return extremaPos; // focuser pos
  }






// /**
//  * -Record V-Curve (in: Fmin, Fmax, J? (TODO: Or determine dynamically? J=f(HFD and/or FWHM)? ))
//  *  -1. Move focus to Fmin (decrease by Fmin)
//  *  -2. Take picture
//  *  -3. Save FWHM & HFD and current focus position (and sub image?)
//  *  -4. If Fmax reached, continue at 7.
//  *  -5. Else:Increase focus by J steps
//  *  -6. Continue at 2.
//  *  -7. Bring focus back to start pos. (Decrease focus by f_rightFmax)
//  */
// bool FocusFinderT::recordVCurve(ostream & os, IndiCameraClientT & inCameraClient, IndiFocuserClientT & inFocuserClient, CImg<float> * outImg, int inStartX, int inStartY, double inExposureTime, unsigned int inOuterHfdRadius, unsigned int numStepsToDetermineDirection, IndiFocuserClientT::FocusDirectionT::TypeE inDirectionToImproveFocus, bool * outUserStop, unsigned int inStepsToReachRoughFocus, int inFmin, int inFmax, const QualityMeasureStrategyT * inQualityMeasureStrategy) {
//   os << "Recording V-Curve..." << endl;
//   os << "Moving to Fmin..." << endl;
//   inFocuserClient.moveFocusBy(inFocuserClient.getAbsPos() - inFmin, IndiFocuserClientT::FocusDirectionT::INWARDS);
  
//   int curPos = inFocuserClient.getAbsPos();
//   int delta = 500; // TODO: Pass / Calculate

//   do {
//     FOCUS_FINDER_STOP();

//     inCameraClient.takePicture(outImg, 1 /*horz bin*/, 1 /*vert bin*/, inStartX /*x*/, inStartY /*y*/, FocusFinderT::sWindowWidthPx /*w*/, FocusFinderT::sWindowHeightPx /*h*/, FrameTypeT::LIGHT, inExposureTime /*s*/, false /*no compr.*/);

//     //recenter(os, inCameraClient, inFocuserClient, outImg, & inStartX, & inStartY, inExposureTime, inOuterHfdRadius, numStepsToDetermineDirection);

//     FwhmT fwhmHorz1(*outImg, FwhmT::DirectionT::HORZ);
//     FwhmT fwhmVert1(*outImg, FwhmT::DirectionT::VERT);
//     HfdT hfd1(*outImg, inOuterHfdRadius /*radius px*/);

//     //mVCurve[inFocuserClient.getAbsPos()] = fwhmVert1.getValue() + fwhmHorz1.getValue(); // TODO: ok?!
//     //mVCurve[inFocuserClient.getAbsPos()] = hfd1.getValue(); // TODO: ok?!
//     mVCurve[inFocuserClient.getAbsPos()] = inQualityMeasureStrategy->calculate(& fwhmHorz1, & fwhmVert1, & hfd1);

//     FocusFinderDataT ffd1(inFocuserClient.getAbsPos(), fwhmHorz1, fwhmVert1, hfd1, mVCurve);
//     mFocusFinderUpdateListeners(& ffd1);


//     inFocuserClient.moveFocusBy(delta, IndiFocuserClientT::FocusDirectionT::OUTWARDS);
//     curPos += delta;

//     if (curPos >= inFmax) {
//       os << "Reached Fmax - finished VCurve recording." << endl;
//       break;
//     }
  
//   } while(true);

//   os << "Finished VCurve recording." << endl;

//   return true;
// }



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
