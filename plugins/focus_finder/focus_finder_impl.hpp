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

#include <functional>

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

DEF_Exception(FocusFinderImpl);
DEF_Exception(FocusFinderImplRecording);


namespace AT {

  
  class FocusFinderImplT {
  public:
    typedef std::function<float(float, bool *)> CalcLimitFuncT;
    
    struct PhaseT {
      enum TypeE {
	READY,
	INITIALIZING,
	DET_LIMIT,
	DET_STEP_SIZE,
	FOCUS_CURVES_RECORDING,
	SET_OPT_POS,
	_Count
      };
    
      static const char * asStr(const TypeE & inType) {
	switch (inType) {
	case READY: return "READY";
	case INITIALIZING: return "INITIALIZING";
	case DET_LIMIT: return "DET_LIMIT";
	case DET_STEP_SIZE: return "DET_STEP_SIZE";
	case FOCUS_CURVES_RECORDING: return "FOCUS_CURVES_RECORDING";
	case SET_OPT_POS: return "SET_OPT_POS";
	default: return "<?>";
	}
      }
      MAC_AS_TYPE(Type, E, _Count);  
    };
  

    struct FocusFindStatusDataT {
      float dx; // Centroid x relative to currImage center
      float dy; // Centroid y relative to currImage center
      CImg<float> currImage;
      PointT<float> currCenterPosFF;
      BinningT currBinning;
      FocusFinderImplT::PhaseT::TypeE phase;
      int currAbsFocusPos;
      bool isRunning;
      float sequenceProgress;
      float phaseProgress;
      FocusFindStatusDataT() : dx(0), dy(0), isRunning(false), currAbsFocusPos(0), sequenceProgress(0), phaseProgress(0), phase(FocusFinderImplT::PhaseT::READY)  {}
    };

    struct FocusFindCntlDataT {
      float exposureTime;
      PointT<float> centerPosFF;
      BinningT binning;
      bool imgFrameRecenter;
      int stepSize;
      FocusFindCntlDataT() : exposureTime(0), binning(BinningT(1,1)), centerPosFF(PointT<float>(0,0)), imgFrameRecenter(true), stepSize(0) {}
      bool valid() const { return (exposureTime > 0); }
    };



    
  private:
    FocusFindStatusDataT mFocusFindStatusData;
    IndiCameraT * mCameraDevice;
    IndiFocuserT * mFocuserDevice;
    IndiFilterWheelT * mFilterWheelDevice;

    FocusCurveT::FocusMeasureFuncT mFocusMeasureFunc;
    CalcLimitFuncT mCalcLimitFunc;
    
    float mLimit;
    PhaseT::TypeE mPhase;
    
    FocusFindCntlDataT mCntlData;

    string mRecordBaseDir;

    PointT<float> mCurrCenterPosFF;

    const size_t mNumStepsFine = 15;     // TODO: Not const, set function? Is this a good value?
    const size_t mNumCurvesToRecord = 3; // TODO: Not const, set function? Is this a good value?


    
    CImg<float> extractImgFrame(const CImg<float> & inFrameImage, int * outDx = 0, int * outDy = 0) const;
    float determineLimit();
    void determineStepSizeAndStartPos(int * outStepSize, float * outStartPos);
    void recordSequence(PosToImgMapT * outPosToImgMap, int stepSize, int inNumSteps, size_t inNumSeqIterations, size_t inSeqNo);
    void recordSequence(PosToImgMapT * outPosToImgMap, const vector<int> & inStepSizes, size_t inNumSeqIterations, size_t inSeqNo);
    string prepareRecordDir();
    
  public:
    // Events / Handlers
    DEFINE_PROP_LISTENER(FocusFinderStart, const FocusFindCntlDataT *);
    DEFINE_PROP_LISTENER(NewSample, const FocusCurveT *, float, const CImg<float> &, float);
    DEFINE_PROP_LISTENER(NewFocusCurve, const FocusCurveT *, const PosToImgMapT *, const PointT<float> *, const LineT<float> *, const LineT<float> *, float);
    DEFINE_PROP_LISTENER(FocusDetermined, float /*focus*/, const CImg<float> & /*img frame*/, float /*focus measure*/);
    DEFINE_PROP_LISTENER(FocusFinderAbort, bool /*manual abort?*/, string /*reason*/);
    DEFINE_PROP_LISTENER(FocusFinderFinished, float);

    // TODO: Too generic...More specific?
    // -> progressUpdate - called when the progress has changed (passes overall progress, phase progress, current phase)
    DEFINE_PROP_LISTENER(StatusUpd, const FocusFindStatusDataT *);

    // NOTE: Macros ends with private section...

    
  public:
    static CalcLimitFuncT sHfdLimitStrategy;
    
  public:
    FocusFinderImplT(IndiCameraT * inCameraDevice, IndiFocuserT * inFocuserDevice, IndiFilterWheelT * inFilterWheelDevice, FocusCurveT::FocusMeasureFuncT inFocusMeasureFunc, CalcLimitFuncT inCalcLimitFunc = 0) :
      mCameraDevice(inCameraDevice),
      mFocuserDevice(inFocuserDevice),
      mFilterWheelDevice(inFilterWheelDevice),
      mFocusMeasureFunc(inFocusMeasureFunc),
      mCalcLimitFunc(inCalcLimitFunc),
      mRecordBaseDir(""),
      mLimit(0),
      mCurrCenterPosFF(PointT<float>(0,0)),
      mPhase(PhaseT::READY) {
    }
    
    inline const FocusFindCntlDataT & getCntlData(const FocusFindCntlDataT & inCntlData) const { return mCntlData; }
    inline void setCntlData(const FocusFindCntlDataT & inCntlData) { mCntlData = inCntlData; }

    const char * getRecordBaseDir() const { return mRecordBaseDir.c_str(); }
    void setRecordBaseDir(const string & inRecordBaseDir, bool inCreateIfNotExists = true) {
      if (! inRecordBaseDir.empty()) {
	// If directory does not exist but inCreateIfNotExists is true, create it.
	// Othwerwise throw. If creating directory fails, throw.
	// TODO: USE BOOST!!! http://www.boost.org/doc/libs/1_47_0/libs/filesystem/v3/doc/tutorial.html
	struct stat sb;
	
	if (! (stat(inRecordBaseDir.c_str(), & sb) == 0 && S_ISDIR(sb.st_mode))) {
	  /* Directory does not exists. */
	  if (inCreateIfNotExists) {
	    const int dir_err = mkdir(inRecordBaseDir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
	    if (-1 == dir_err) {
	      stringstream ss;
	      ss << "Error creating record base directory '" << inRecordBaseDir << "'!" << endl;
	      throw FocusFinderImplRecordingExceptionT(ss.str());
	    }
	  } else {
	    // Check if directory exists...
	    // TODO: Better error handling...
	    stringstream ss;
	    ss << "Base directory '" << inRecordBaseDir << "' does not exist. Set inCreateIfNotExists=true to create." << endl;
	    throw FocusFinderImplRecordingExceptionT(ss.str());
	  }      
	}
      }
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
