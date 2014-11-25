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
#include "focus_finder.hpp"

#include "indi/indi_focuser.hpp"
#include "indi/indi_camera.hpp"

#include "fwhm.hpp"
#include "hfd.hpp"
#include "vcurve.hpp"

// #define FOCUS_FINDER_STOP()			\
//   if (mStop) {					\
//     if (outUserStop)				\
//       *outUserStop = true;			\
//     return false;				\
//   }						\

namespace AT {

  /**
   * Linear interpolation focus finder.
   */
  class FocusFinderLinearInterpolationImplT : public FocusFinderT {
  public:
    virtual void findFocus();

  };







  class QualityMeasureStrategyT {
  public:
    virtual double calculate(const FwhmT * inFwhmHorz, const FwhmT * inFwhmVert, const HfdT * inHfd) const = 0;
  };
  
  class QmsSumAllT :  public QualityMeasureStrategyT {
  public:
    virtual double calculate(const FwhmT * inFwhmHorz, const FwhmT * inFwhmVert, const HfdT * inHfd) const {
      // TODO: Check if all params valid...
      double res = inFwhmHorz->getValue() + inFwhmVert->getValue() + inHfd->getValue();
      cerr << "QmsSumAllT ... calculate... res: " << res << endl;
      return res;
    }
  };



  // struct FocusFinderDataT {
  //   int mAbsFocusPosition;
  //   int mAbsMinFocusPos;
  //   int mAbsMaxFocusPos;
  //   FwhmT mFwhmHorz;
  //   FwhmT mFwhmVert;
  //   HfdT mHfd;
  //   string mStatusText;
  //   VCurveT<int, double> mVCurve;
  //   FocusFinderDataT(int inAbsFocusPosition = 0) : mAbsFocusPosition(inAbsFocusPosition), mAbsMinFocusPos(-1), mAbsMaxFocusPos(-1), mStatusText("") {} // TODO: Remove? HACK?!!
  //   FocusFinderDataT(const string & inStatusText) : mStatusText(inStatusText), mAbsFocusPosition(0) {}  // TODO: Remove? HACK?!!
  //   FocusFinderDataT(int inAbsFocusPosition, const FwhmT & inFwhmHorz, const FwhmT & inFwhmVert, const HfdT & inHfd, const VCurveT<int, double> & inVCurve) : mAbsFocusPosition(inAbsFocusPosition), mFwhmHorz(inFwhmHorz), mFwhmVert(inFwhmVert), mHfd(inHfd), mVCurve(inVCurve), mAbsMinFocusPos(-1), mAbsMaxFocusPos(-1), mStatusText("") { }
  // };
  
  
  // struct MinMaxFocusPosT {
  //   enum TypeE {
  //     MIN_FOCUS_POS,
  //     MAX_FOCUS_POS,
  //     _Count
  //   };
    
  //   static const char * asStr(const TypeE & inType) {
  //     switch (inType) {
  //     case MIN_FOCUS_POS: return "MIN_FOCUS_POS";
  //     case MAX_FOCUS_POS: return "MAX_FOCUS_POS";
  //     default: return "<?>";
  //     }
  //   }
  // }; // end struct
  
  
  // #define LOG(m)				\
  //   os << m << flush;			\
  //   FocusFinderDataT ffdStatus(m);		\
  //   mFocusFinderUpdateListeners(& ffdStatus);	\
  //   return os;					\
  
  
  // class FocusFinderT {
  // private:
    
  //   //void recenter(ostream & os, IndiCameraClientT & inCameraClient, IndiFocuserClientT & inFocuserClient, CImg<float> * outImg, int * outStartX, int * outStartY, double inExposureTime, unsigned int inOuterHfdRadius, unsigned int numStepsToDetermineDirection);
    
  //   bool determineInitialDirection(ostream & os, IndiCameraClientT & inCameraClient, IndiFocuserClientT & inFocuserClient, CImg<float> * outImg, int startX, int startY, double inExposureTime, unsigned int inOuterHfdRadius, unsigned int numStepsToDetermineDirection, IndiFocuserClientT::FocusDirectionT::TypeE * outDirectionToImproveFocus);
    
  //   bool findRoughFocus(ostream & os, IndiCameraClientT & inCameraClient, IndiFocuserClientT & inFocuserClient, CImg<float> * outImg, int inStartX, int inStartY, double inExposureTime, unsigned int inOuterHfdRadius, unsigned int numStepsToDetermineDirection, IndiFocuserClientT::FocusDirectionT::TypeE inDirectionToImproveFocus, bool * outUserStop, unsigned int inStepsToReachRoughFocus);
    
  //   bool findExtrema(ostream & os, IndiCameraClientT & inCameraClient, IndiFocuserClientT & inFocuserClient, CImg<float> * outImg, int inStartX, int inStartY, double inExposureTime, unsigned int inOuterHfdRadius, unsigned int numStepsToDetermineDirection, IndiFocuserClientT::FocusDirectionT::TypeE inDirectionToImproveFocus, bool * outUserStop, unsigned int inStepsToReachRoughFocus, MinMaxFocusPosT::TypeE inMinMaxFocusPos, int * outFExtremaPos, const QualityMeasureStrategyT * inQualityMeasureStrategy);
    
  //   bool recordVCurve(ostream & os, IndiCameraClientT & inCameraClient, IndiFocuserClientT & inFocuserClient, CImg<float> * outImg, int inStartX, int inStartY, double inExposureTime, unsigned int inOuterHfdRadius, unsigned int numStepsToDetermineDirection, IndiFocuserClientT::FocusDirectionT::TypeE inDirectionToImproveFocus, bool * outUserStop, unsigned int inStepsToReachRoughFocus, int inFmin, int inFmax, const QualityMeasureStrategyT * inQualityMeasureStrategy);
    
  // public:
  //   static const size_t sWindowWidthPx;
  //   static const size_t sWindowHeightPx;
  //   static const size_t sHfdOuterRadiusPx;
    
  //   FocusFinderT(IndiClientTmplT * inIndiClient, const string & inCameraDeviceName, const string & inFocuserDeviceName);
  //   ~FocusFinderT();
    
  //   void numberChangeHandler(INumberVectorProperty * inVecNumber);
  //   inline IndiClientTmplT * getIndiClient() const { return mIndiClient; }
    
  //   bool findFocus(int inCenterX, unsigned int inCenterY, int inSize, double inExposureTime, unsigned int inOuterHfdRadius, unsigned int numStepsToDetermineDirection, unsigned int inStepsToReachRoughFocus, bool * outUserStop, ostream & os, const QualityMeasureStrategyT * inQualityMeasureStrategy);
    
  //   inline void stop() { mStop = true; /*TODO: Do we need a mutex here?!!!!!*/ }
    
  //   /*
  //    * Focus Finder Update Listener
  //    */
  //   // TODO: Use macro instead...?!
  //   DEFINE_PROP_LISTENER(FocusFinderUpdate, const FocusFinderDataT *);
  //   //DEFINE_PROP_LISTENER(FocusFinderUpdateStatus, string); // TODO: Maybe later separate...
    
  //   // typedef signals2::signal<void (const FocusFinderDataT *)> FocusFinderUpdateListenerT;
    
  //   // void registerFocusFinderUpdateListener(FocusFinderUpdateListenerT::slot_type inCallBack) {
  //   //   mFocusFinderUpdateListener.connect(inCallBack);
  //   // }
  //   // void unregisterMessageListener(FocusFinderUpdateListenerT::slot_type inCallBack) {
  //   //   mFocusFinderUpdateListener.disconnect(inCallBack);
  //   // }
    
  //   inline const VCurveT<int, double> & getVCurve() const { return mVCurve; }
    
  // private:
  //   IndiClientTmplT * mIndiClient;
  //   string mCameraDeviceName;
  //   string mFocuserDeviceName;
  //   bool mStop;
  //   //FocusFinderUpdateListenerT mFocusFinderUpdateListener;
  //   VCurveT<int, double> mVCurve;
  // };


} // end AT namespace



#endif /* _FOCUS_FINDER_LINER_INTERPOLATION_IMPL_HPP_ */
