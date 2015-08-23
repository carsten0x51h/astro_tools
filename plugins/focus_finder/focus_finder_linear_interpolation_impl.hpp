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


#ifndef _FOCUS_FINDER_LINER_INTERPOLATION_IMPL_HPP_
#define _FOCUS_FINDER_LINER_INTERPOLATION_IMPL_HPP_ _FOCUS_FINDER_LINER_INTERPOLATION_IMPL_HPP_

/**
   V-Curve Approximation
-Pre-Conditions
   -Camera connected and working
   -Focuser connected and working
   -Focus filter selected (if any)
   -Focus window selected
   -A "valid" star profile in the selected region (TODO: How to say there is a valid star profile? -> use HFD / FWHM values, other values?! Shape? Flux?)

-Determine initilal dL (initial direction)
   -Take picture
   -Save FWHM & HFD
   -Moving focus up N steps
   -Take picture
   -Compre new FWHM & HFD against saved values
   -If focus improved, "up" is the right direction
   -If focus gets worse, "down" is the right direction
   -Move foucs "down" N steps

-Find rough focus (in: FWMH & HFD at least to be reached, out: success/failure, FWMH & HFD obtained)
   -1. Take picture
   -2. Save FWHM & HFD
   -3. Move M steps into prev. direction
   -4. Take picture
   -5. Repeat at 1. until new FWHM & HFD are worse than saved
   -6. Move focus in oppsosite direction by M steps

-Determine rough relative focus boundaries (in: FWHM/HFD BOUNDARY, out; dFmin [steps], dFmax [steps])
   -Determine Fmin
      -1. Take picture
      -2. Determine FWHM & HFD
      -3. _Reduce_ focus pos. by K steps, accumulate K
      -4. Take picture
      -5. Repeat 1. as long as new FWHM & HFD are < FWHM/HFD BOUNDARY
      -6. Save "Fmin" focus position (and corresponding HWHM/HFD)
      -7. Move focus back to "center" position (increase focus by Sum(K))
   -Determine Fmax
      -1. Take picture
      -2. Determine FWHM & HFD
      -3. _Increase_focus pos. by K steps, accumulate K
      -4. Take picture
      -5. Repeat 1. as long as new FWHM & HFD are < FWHM/HFD BOUNDARY
      -6. Save "Fmax" focus position (and corresponding HWHM/HFD)
      -7. Move focus back to "center" position (decrease focus by Sum(K))

-Record V-Curve (in: Fmin, Fmax, J? (TODO: Or determine dynamically? J=f(HFD and/or FWHM)? ))
   -1. Move focus to Fmin (decrease by Fmin)
   -2. Take picture
   -3. Save FWHM & HFD and current focus position (and sub image?)
   -4. If Fmax reached, continue at 7.
   -5. Else:Increase focus by J steps
   -6. Continue at 2.
   -7. Bring focus back to start pos. (Decrease focus by f_rightFmax)

-Determine optimal focus position from V-Curve(s) (requires valid V-Curve(s)) (in: V-Curves (a V-Curve is a list of relative focusPos & FWHM/HFD pairs))
   -1. Build one average V-Curve (all V-Curves must be within same relative Fmin & Fmax) (Note: Then for each focusPos there is exactly one HFD & FWHM value per V-Curve)
      -For idx = Fmin to Fmax
         -For all V-Curves
            -Sum the idx value of all V-Curves
         -Divide accumulated idx value by the number of V-Curves

   -2. Separate V-Curve in decreasing data-points and increasing data-points, minimum is to be included in both sets (TODO: Really?))
      -TODO...

   -3. Determine optimal focus position 
      NOTE: Important condition: f_decreasing_fwhm&hfd(pos) = f_increasing_fwhm&hfd(pos) == FWHM == HFD == 0!! --> pos is optimal focus position
          --> Find a_1, b_1 & a_2, b_2 of both functions so that (MSE1 + MSE2) is minimized
          --> Maybe hard to formulate in gsl?! --> We may implement this manually....
   -Move to optimal focus position (in: optimal focus position)
   -TODO: Does this involve any corrections (feedback?)
*/

#include <iosfwd>
#include <vector>

#include "focus_finder_common.hpp"

#include "indi/indi_camera.hpp"
#include "indi/indi_focuser.hpp"

namespace AT {

  DEF_Exception(FocusFinderLinearInterpolation);

  typedef VCurveTmplT<int, double> VCurveT;
  typedef vector<VCurveT> VCurveVecT;

  class VCurveAccessorT {
  public:
    typedef VCurveT TypeT;
    static DataPointT getDataPoint(size_t inIdx, TypeT::const_iterator inIt) {
      DataPointT dp(inIt->first, inIt->second);
      return dp;
    }
  };


  /**
   * Linear interpolation focus finder.
   */
  class FocusFinderLinearInterpolationImplT : public FocusFinderT {
  private:
    IndiCameraT * mCameraDevice;
    IndiFocuserT * mFocuserDevice;
    FrameT<unsigned int> mSelectionFrame;
    float mExposureTimeSec;
    BinningT mBinning;

    size_t mWindowSize;
    unsigned int mOuterHfdRadiusPx;
    unsigned int mNumStepsToDetermineDirection;
    unsigned int mStepsToReachRoughFocus;
    double mExtremaFitnessBoundary;
    double mVCurveFitEpsAbs;
    double mVCurveFitEpsRel;
    size_t mRoughFocusMaxIterCnt;
    size_t mTakePictureFitGaussCurveMaxRetryCnt;
    bool mDebugShowTakePictureImage;
    size_t mRoughFocusSearchRangePerc;
    size_t mRoughFocusRecordNumCurves;
    size_t mRoughFocusGranularitySteps;
    size_t mFineFocusRecordNumCurves;
    size_t mFineFocusGranularitySteps;
    size_t mFineSearchRangeSteps;

    bool mStopFlag;

    void takePictureCalcStarData(StarDataT * outStarData, CImg<float> * outImg = 0);

    /**
     * -Determine initilal dL (initial direction)
     * -Take picture
     * -Save FWHM & HFD
     * -Moving focus up N steps
     * -Take picture
     * -Compare new FWHM & HFD against saved values
     * -If focus improved, "up" is the right direction
     * -If focus gets worse, "down" is the right direction
     * -Move foucs "down" N steps
     */
    FocusDirectionT::TypeE determineInitialDirectionOfImprovement();


    /**
     * -Find rough focus (in: FWMH & HFD at least to be reached, out: success/failure, FWMH & HFD obtained)
     * -1. Take picture
     * -2. Save FWHM & HFD
     * -3. Move M steps into prev. direction
     * -4. Take picture
     * -5. Repeat at 1. until new FWHM & HFD are worse than saved
     * -6. Move focus in oppsosite direction by M steps
     */
    void findRoughFocus(FocusDirectionT::TypeE inDirectionToImproveFocus);


    /**
     * -Determine Fmin/(Fmax)
     *  -1. Take picture
     *  -2. Determine FWHM & HFD
     *  -3. _Reduce_/(_Increase_) focus pos. by K steps, accumulate K
     *  -4. Take picture
     *  -5. Repeat 1. as long as new FWHM & HFD are < FWHM/HFD BOUNDARY
     *  -6. Save "Fmin" ("Fmax") focus position (and corresponding HWHM/HFD)
     *  -7. Move focus back to "center" position (increase (decrease) focus by Sum(K))
     */
    int findExtrema(FocusDirectionT::TypeE inDirectionToImproveFocus, MinMaxFocusPosT::TypeE inMinMaxFocusPos);


    /**
     * -Record V-Curve (in: Fmin, Fmax, J? (TODO: Or determine dynamically? J=f(HFD and/or FWHM)? ))
     *  -1. Move focus to Fmin (decrease by Fmin)
     *  -2. Take picture
     *  -3. Save FWHM & HFD and current focus position (and sub image?)
     *  -4. If Fmax reached, continue at 7.
     *  -5. Else:Increase focus by J steps
     *  -6. Continue at 2.
     *  -7. Bring focus back to start pos. (Decrease focus by f_rightFmax)
     */
    void recordVCurve(int inAbsStartPos, int inAbsEndPos, size_t inGranularitySteps, VCurveT * outVCurve, size_t inVCurveNum, size_t inVCurveNumTotal, bool inMoveBackToOldPos = true);

    /**
     * -Find focus in range.
     * -TODO: Add documentation
     */
    double findOptimalFocusInRange(int inAbsStartPos, int inAbsEndPos, size_t inNumVCurves, size_t inCurveGranularitySteps);

    /**
     * -Calculate optimal focus position
     * -TODO: Add documentation
     */
    double calcOptimalAbsFocusPos(const VCurveVecT & inVCurves);

  public:
    FocusFinderLinearInterpolationImplT(IndiCameraT * inCameraDevice,
					IndiFocuserT * inFocuserDevice,
					FrameT<unsigned int> & inSelectionFrame,
					float inExposureTimeSec,
					BinningT inBinning = BinningT(1, 1)) :
      mCameraDevice(inCameraDevice),
      mFocuserDevice(inFocuserDevice),
      mSelectionFrame(inSelectionFrame),
      mExposureTimeSec(inExposureTimeSec),
      mBinning(inBinning),
      mWindowSize(31),
      mOuterHfdRadiusPx(5),
      mNumStepsToDetermineDirection(3000),
      mStepsToReachRoughFocus(3000),
      mExtremaFitnessBoundary(25),
      mVCurveFitEpsAbs(1e-1),
      mVCurveFitEpsRel(1e-1),
      mRoughFocusMaxIterCnt(20),
      mTakePictureFitGaussCurveMaxRetryCnt(5),
      mDebugShowTakePictureImage(false),
      mRoughFocusSearchRangePerc(70),
      mRoughFocusRecordNumCurves(1),
      mRoughFocusGranularitySteps(500),
      mFineFocusRecordNumCurves(3),
      mFineFocusGranularitySteps(50),
      mFineSearchRangeSteps(2000),
      mStopFlag(false) {
    }
    ~FocusFinderLinearInterpolationImplT() {
    }

    virtual void findFocus();
    inline void stop() { mStopFlag = true; }

    /**
     * Configuration options
     */
    inline void setWindowSize(size_t inWindowSize) {
      AT_ASSERT(FocusFinderLinearInterpolation, inWindowSize % 2, "inWindowSize must be odd.");
      mWindowSize = inWindowSize;
    }
    inline size_t getWindowSize() const { return mWindowSize; }

    inline void setNumStepsToDetermineDirection(unsigned int inNumStepsToDetermineDirection) { mNumStepsToDetermineDirection = inNumStepsToDetermineDirection; }
    inline unsigned int getNumStepsToDetermineDirection() const { return mNumStepsToDetermineDirection; }

    inline void setStepsToReachFocus(unsigned int inStepsToReachRoughFocus) { mStepsToReachRoughFocus = inStepsToReachRoughFocus; }
    inline unsigned int getStepsToReachFocus() const { return mStepsToReachRoughFocus; }

    inline void setExtremaFitnessBoundary(double inExtremaFitnessBoundary) { mExtremaFitnessBoundary = inExtremaFitnessBoundary; }
    inline double getExtremaFitnessBoundary() { return mExtremaFitnessBoundary; }

    inline void setOuterHfdRadiusPx(unsigned int inOuterHfdRadiusPx) { mOuterHfdRadiusPx = inOuterHfdRadiusPx; }
    inline unsigned int getOuterHfdRadiusPx() const { return mOuterHfdRadiusPx; }

    inline void setRoughFocusMaxIterCnt(size_t inRoughFocusMaxIterCnt) { mRoughFocusMaxIterCnt = inRoughFocusMaxIterCnt; }
    inline size_t getRoughFocusMaxIterCnt() const { return mRoughFocusMaxIterCnt; }

    inline void setTakePictureFitGaussCurveMaxRetryCnt(size_t inTakePictureFitGaussCurveMaxRetryCnt) { mTakePictureFitGaussCurveMaxRetryCnt = inTakePictureFitGaussCurveMaxRetryCnt; }
    inline size_t getTakePictureFitGaussCurveMaxRetryCnt() const { return mTakePictureFitGaussCurveMaxRetryCnt; }

    inline void setDebugShowTakePictureImage(bool inDebugShowTakePictureImage) { mDebugShowTakePictureImage = inDebugShowTakePictureImage; }
    inline bool getDebugShowTakePictureImage() const { return mDebugShowTakePictureImage; }

    inline void setRoughFocusSearchRangePerc(size_t inRoughFocusSearchRangePerc) { mRoughFocusSearchRangePerc = inRoughFocusSearchRangePerc; }
    inline size_t getRoughFocusSearchRangePerc() const { return mRoughFocusSearchRangePerc; }

    inline void setRoughFocusRecordNumCurves(size_t inRoughFocusRecordNumCurves) { mRoughFocusRecordNumCurves = inRoughFocusRecordNumCurves; }
    inline size_t getRoughFocusRecordNumCurves() const { return mRoughFocusRecordNumCurves; }

    inline void setRoughFocusGranularitySteps(size_t inRoughFocusGranularitySteps) { mRoughFocusGranularitySteps = inRoughFocusGranularitySteps; }
    inline size_t getRoughFocusGranularitySteps() const { return mRoughFocusGranularitySteps; }

    inline void setFineFocusRecordNumCurves(size_t inFineFocusRecordNumCurves) { mFineFocusRecordNumCurves = inFineFocusRecordNumCurves; }
    inline size_t getFineFocusRecordNumCurves() const { return mFineFocusRecordNumCurves; }

    inline void setFineFocusGranularitySteps(size_t inFineFocusGranularitySteps) { mFineFocusGranularitySteps = inFineFocusGranularitySteps; }
    inline size_t getFineFocusGranularitySteps() const { return mFineFocusGranularitySteps; }

    inline void setFineSearchRangeSteps(size_t inFineSearchRangeSteps) { mFineSearchRangeSteps = inFineSearchRangeSteps; }
    inline size_t getFineSearchRangeSteps() const { return mFineSearchRangeSteps; }
    
    inline void setVCurveFitEpsAbs(double inVCurveFitEpsAbs) { mVCurveFitEpsAbs = inVCurveFitEpsAbs; }
    inline double getVCurveFitEpsAbs() { return mVCurveFitEpsAbs; }
    
    inline void setVCurveFitEpsRel(double inVCurveFitEpsRel) { mVCurveFitEpsRel = inVCurveFitEpsRel; }
    inline double getVCurveFitEpsRel() { return mVCurveFitEpsRel; }

    DEFINE_PROP_LISTENER(FocusFinderUpdate, const FocusFinderDataT *);
  };

} // end AT namespace


#endif /* _FOCUS_FINDER_LINER_INTERPOLATION_IMPL_HPP_ */
