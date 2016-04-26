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

#ifndef _FOCUSER_CONSOLE_UI_HPP_
#define _FOCUSER_CONSOLE_UI_HPP_ _FOCUSER_CONSOLE_UI_HPP_

#include <boost/thread.hpp>
#include <ncurses.h>

#include "at_plugin.hpp"
#include "at_exception.hpp"

#include "indi/indi_camera.hpp"
#include "indi/indi_focuser.hpp"
#include "indi/indi_filter_wheel.hpp"

#include "at_console_display.hpp"
#include "at_console_menu.hpp"
#include "focus_finder_impl.hpp"

using namespace std;

namespace AT {
  
  DEF_Exception(FocusFinderPlugin);
  DEF_Exception(UnknownFocusFinderImpl);
  DEF_Exception(RequireOption);
  DEF_Exception(ImageDimension);
  DEF_Exception(WindowOutOfBounds);
  

  struct StartAbortT {
    enum TypeE {
      START,
      ABORT,
      _Count
    };

    static const char * asStr(const TypeE & inType) {
      switch (inType) {
      case START: return "START";
      case ABORT: return "ABORT";
      default: return "<?>";
      }
    }
    MAC_AS_TYPE(Type, E, _Count);
  };

  
  struct CoolerStateT {
    enum TypeE {
      DISABLED,
      ENABLED,
      _Count
    };

    static const char * asStr(const TypeE & inType) {
      switch (inType) {
      case DISABLED: return "DISABLED";
      case ENABLED: return "ENABLED";
      default: return "<?>";
      }
    }
    MAC_AS_TYPE(Type, E, _Count);
  };


  struct ExposureModeT {
    enum TypeE {
      SINGLE,
      LOOP,
      _Count
    };

    static const char * asStr(const TypeE & inType) {
      switch (inType) {
      case SINGLE: return "SINGLE";
      case LOOP: return "LOOP";
      default: return "<?>";
      }
    }
    MAC_AS_TYPE(Type, E, _Count);
  };



  /***********************************************************************
   * EXPOSURE TASK
   ***********************************************************************/
  struct ExposureCntlDataT {
    float exposureTime;
    PointT<float> centerPosFF;
    BinningT binning;
  };

  struct ExposureStatusDataT {
    TaskStateT::TypeE taskState;
    int progress;
    int secondsLeft;
    ExposureStatusDataT() : taskState(TaskStateT::READY), progress(0), secondsLeft(0) {}
  };



  
  class FocusFinderConsoleCntlT {
  private:
    const po::variables_map & mCmdLineMap;
    IndiCameraT * mCameraDevice;
    IndiFocuserT * mFocuserDevice;
    IndiFilterWheelT * mFilterWheelDevice;
    FocusFinderImplT * mFocusFinderImpl;

    // Focus finder task
    thread mFocusFindThread;
    signals2::connection mFocusFinderStatusUpdHandlerConn;
    signals2::connection mFocusFinderNewSampleHandlerConn;
    signals2::connection mFocusFinderNewFocusCurveHandlerConn;
    signals2::connection mFocusFinderAbortHandlerConn;

    // Exposure task
    thread mExposureThread;
    atomic<ExposureStatusDataT> mExposureStatus;
    atomic<ExposureCntlDataT> mExposureCntl;
    PointT<float> mCurrCenterPosFF; // also used by focus finder task
    CImg<float> mCurrSubImage;

    // Console menu
    ConsoleDisplayT mConsoleDisplay;
    std::vector<MenuEntryT *> mMenuEntries;
    ConsoleMenuT mConsoleMenu;
    
    MenuFieldT<int> * mAbsFocusPosDestMenuField;
    
    // Menu values
    int mFocusStepSize;
    StartAbortT::TypeE mExpStartAbort;
    int mAbsFocusPosDest;
    int mFocusDelta;
    ExposureModeT::TypeE mExpMode;
    float mExpTimeVal;
    int mBinValXY;
    int mFilterSelect;
    CoolerStateT::TypeE mCameraCoolerState;
    int mCameraTemperature;
    StartAbortT::TypeE mFocusFindStartAbort;

    float mFocalDistance;
    DimensionT<float> mPixelSizeUm;

    // Windows - TODO: Not always available? - TODO: Make as class members later..?!
    // TODO: RENAME!
    CImgDisplay currImageDisp, currentHfdDisp, currFocusCurveDisp, currentFwhmHorzDisp, currentFwhmVertDisp;

    mutex mFocusFinderMtx; // TODO: As class member...
    FocusFindStatusDataT mFocusFindStatus; // TODO: This might be a class member, later... no longer static...
    string mLastErrorStr;
    
  public:
    void focusFinderNewFocusCurveHandler(const FocusCurveT * inFocusCurve, const PosToImgMapT * inPosToImgMap, const PointT<float> * inSp, const LineT<float> * inLine1, const LineT<float> * inLine2);
    void focusFinderNewSampleHandler(const FocusCurveT * inFocusCurve, float inFocusPos, const CImg<float> & inImgFrame);
    void focusFinderStatusUpdHandler(const FocusFindStatusDataT * inFocusFindStatus);
    void focusFinderAbortHandler(bool inManualAbort, string inCause);

    
  public:
    FocusFinderConsoleCntlT(const po::variables_map & inCmdLineMap, IndiCameraT * inCameraDevice, IndiFocuserT * inFocuserDevice, IndiFilterWheelT * inFilterWheelDevice, const FrameT<unsigned int>  & inSelectionFrameFF);
    ~FocusFinderConsoleCntlT();
    void show();
  };
  
  
  // TODO: Create ManualConsoleFocusCntl class or similar...??!
  void
  manualConsoleFocusCntl(const po::variables_map & inCmdLineMap, IndiCameraT * inCameraDevice, IndiFocuserT * inFocuserDevice, IndiFilterWheelT * inFilterWheelDevice, const FrameT<unsigned int> & inSelectionFrame, float inExposureTimeSec, BinningT inBinning, bool inFollowStar = true);
  
}; // end AT

#endif /* _FOCUSER_CONSOLE_UI_HPP_ */
