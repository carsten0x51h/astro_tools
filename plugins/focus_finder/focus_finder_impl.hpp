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

#include "indi/indi_camera.hpp"
#include "indi/indi_focuser.hpp"
#include "indi/indi_filter_wheel.hpp"

#include "focus_finder_common.hpp"

#include "fwhm.hpp"
#include "hfd.hpp"
#include "util.hpp"

#include "focus_curve.hpp"

#ifndef _FOCUS_FINDER_IMPL_HPP_
#define _FOCUS_FINDER_IMPL_HPP_ _FOCUS_FINDER_IMPL_HPP_


namespace AT {

  struct FocusFindStatusDataT {
    float dx; // Centroid x relative to currImage center
    float dy; // Centroid y relative to currImage center
    CImg<float> currImage;
    PointT<float> currCenterPosFF;
    
    int currAbsFocusPos;
    bool isRunning;
    int progress;
    FwhmT fwhmHorz;
    FwhmT fwhmVert;
    HfdT hfd;
    FocusFindStatusDataT() : dx(0), dy(0), isRunning(false), currAbsFocusPos(0), progress(0) {}
  };

  struct FocusFindCntlDataT {
    float exposureTime;
    PointT<float> centerPosFF;
    BinningT binning;
    bool wantRecenter;
    int stepSize;
    FocusFindCntlDataT() : exposureTime(0), binning(BinningT(1,1)), centerPosFF(PointT<float>(0,0)), wantRecenter(true), stepSize(0) {}
    bool valid() const { return (exposureTime > 0); }
  };


  
  class FocusFinderImplT {
  private:
    FocusFindStatusDataT mFocusFindStatusData;
    IndiCameraT * mCameraDevice;
    IndiFocuserT * mFocuserDevice;
    IndiFilterWheelT * mFilterWheelDevice;

    FocusFindCntlDataT mCntlData;

    string mRecordBaseDir;
    
    const size_t mNumStepsRough = 10;    // TODO: Not const, set function? Is this a good value?
    const size_t mNumStepsFine = 15;     // TODO: Not const, set function? Is this a good value?
    const size_t mNumCurvesToRecord = 3; // TODO: Not const, set function? Is this a good value?
    const float mHfdLimitFactor = 1.8;   // 180% - We may formulate this as a percent value... - TODO: Configurable?! Is this a generic approach?

    void recordSequence(FocusCurveT * outFocusCurve, int stepSize, int inNumSteps, float hfdLimit = 0);
    void recordSequence(FocusCurveT * outHfdFocusCurve, const vector<int> & inStepSizes, float hfdLimit = 0);
    string prepareRecordDir();
    
  public:
    // Events / Handlers
    // -started - called when focuser was started (i.e. if isRunning = true)
    // -newImage/newData/newShot/newSample - called whenever a new image was recorded (and probably processed - centroid, hfd, fwhm, ...)
    // -oneCurveComplete - called when a complete curve was recorded (e.g. 2 lines, or 1 parabel (optional))  - passes calculated SP, curve (e.g. 2 lines, parabel), recorded points, ...)
    // -focusDetermined - called when the focus finder sucessfully determined the final focus (passes: final focus (SP), ...)
    // -aborted - called when the focuser was aborted (either by failure or by user) - passes reason
    // -finished - called whenever run is left (either by abort or because focus was detrmined correctly)
    DEFINE_PROP_LISTENER(NewSample, const FocusCurveT *);
    DEFINE_PROP_LISTENER(FocusDetermined, const FocusCurveT *, const PointT<float> *, const LineT<float> *, const LineT<float> *);
    
    // TODO: Too generic...More specific?
    // -> progressUpdate - called when the progress has changed (passes overall progress, phase progress, current phase)
    DEFINE_PROP_LISTENER(StatusUpd, const FocusFindStatusDataT *);
    // NOTE: Macros ends with private section...

    
  public:
    FocusFinderImplT(IndiCameraT * inCameraDevice, IndiFocuserT * inFocuserDevice, IndiFilterWheelT * inFilterWheelDevice) :
      mCameraDevice(inCameraDevice),
      mFocuserDevice(inFocuserDevice),
      mFilterWheelDevice(inFilterWheelDevice),
      mRecordBaseDir("") {
    }

    inline const FocusFindCntlDataT & getCntlData(const FocusFindCntlDataT & inCntlData) const { return mCntlData; }
    inline void setCntlData(const FocusFindCntlDataT & inCntlData) { mCntlData = inCntlData; }

    const char * getRecordBaseDir() const { return mRecordBaseDir.c_str(); }
    void setRecordBaseDir(const char * inRecordBaseDir) {
      // TODO: Check if directory exists... if not, throw... maybe pass an additional bool which forces creation of the dir...
      mRecordBaseDir = inRecordBaseDir;
    }
    void run();
    
    static void
    writeRecordFile(const string & inCurrRecordDir, const CImg<float> & inImg, int inSegmentNo, int inStepNo, int inFocusPos);
    
    static PointT<float>
    calcOptFocusPos(const FocusCurveT & inFocusCurve, LineFitTypeT::TypeE inLineFitType, LineT<float> * outLine1, LineT<float> * outLine2);
  };
};

#endif /* _FOCUS_FINDER_IMPL_HPP_ */
