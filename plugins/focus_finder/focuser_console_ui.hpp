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

#include <ncurses.h>

#include "at_plugin.hpp"
#include "at_exception.hpp"
#include "limited_queue.hpp"

#include "indi/indi_camera.hpp"
#include "indi/indi_focuser.hpp"
#include "indi/indi_filter_wheel.hpp"

using namespace std;

namespace AT {

  class MenuEntryT {
  protected:
    string mStatusText;
    
  public:
    MenuEntryT() : mStatusText("") {}
    inline const string & getStatusText() { return mStatusText; };
    virtual void execute(int inKey) = 0;
    virtual const char* getName() const = 0;
    virtual string getValueAsStr() const = 0;
    virtual const char * getHelpText() const = 0;
  };

  class SeparatorT : public MenuEntryT {
    virtual const char * getName() const { return ""; }
    virtual string getValueAsStr() const { return ""; }
    virtual const char * getHelpText() const { return ""; }
    virtual void execute(int inKey) { }
  };

  class SetFocusStepSizeT : public MenuEntryT {
  private:
    int mFocusStepSize;
    const int cStepFactor = 10;
    
  public:
    SetFocusStepSizeT() : mFocusStepSize(10) {}
    virtual const char * getName() const { return "Step size :"; }
    virtual float getStepSize() const { return mFocusStepSize; }
    virtual string getValueAsStr() const { stringstream ss; ss << mFocusStepSize; return ss.str(); }
    virtual const char * getHelpText() const { return ""; }
    virtual void execute(int inKey) {
      switch(inKey) {
      case KEY_LEFT: {
	if (mFocusStepSize > 1) { mFocusStepSize =  mFocusStepSize / cStepFactor; }
	mStatusText = "";
	break;
      }
      case KEY_RIGHT: {
	if (mFocusStepSize < 10000) { mFocusStepSize =  mFocusStepSize * cStepFactor; }
	mStatusText = "";
	break;
      }
      }
    }
  };
  
  class MoveFocusT : public MenuEntryT {
  private:
    IndiFocuserT * mFocuserDevice;
    SetFocusStepSizeT * mFocusStepSize;
    int mFocusPos;
    
  public:
    MoveFocusT(IndiFocuserT * inFocuserDevice, SetFocusStepSizeT * inFocusStepSize) : mFocuserDevice(inFocuserDevice), mFocusStepSize(inFocusStepSize) {
      mFocusPos = mFocuserDevice->getAbsPos();
    }
    virtual const char * getName() const { return "Focus pos (dest) :"; }
    virtual float getFocusPos() const { return mFocusPos; }
    virtual string getValueAsStr() const { stringstream ss; ss << mFocusPos; return ss.str(); }
    virtual const char * getHelpText() const { return "ESC: Abort motion"; }
    virtual void execute(int inKey) {
      switch(inKey) {
      case KEY_LEFT:
	mFocusPos -= mFocusStepSize->getStepSize();
	mFocuserDevice->setAbsPos(mFocusPos, 0 /* non-blocking */);
	mStatusText = "";
	break;
	
      case KEY_RIGHT:
	mFocusPos += mFocusStepSize->getStepSize();
	mFocuserDevice->setAbsPos(mFocusPos, 0 /* non-blocking */);
	mStatusText = "";
	break;
	
      case 27 /* ESC */:
	mFocuserDevice->abortMotion(0 /*non blocking*/);
	mStatusText = "Stopping focus motion.";
	break;
      }
    }
  };

  class ControlExposureT : public MenuEntryT {
  public:
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

  private:
    IndiCameraT * mCameraDevice;
    float mExposureTimeSec;
    ExposureModeT::TypeE mExposureMode;
    bool mExposureDone;
				      
  public:
    ControlExposureT(IndiCameraT * inCameraDevice, float inExposureTimeSec = 1) : mCameraDevice(inCameraDevice),
										  mExposureTimeSec(inExposureTimeSec),
										  mExposureMode(ExposureModeT::SINGLE),
										  mExposureDone(false) { }
    
    virtual const char * getName() const { return "Exposure time [s] :"; }
    virtual string getValueAsStr() const {
      stringstream ss;
      ss << mExposureTimeSec << " [" << ExposureModeT::asStr(mExposureMode) << "]";
      return ss.str();
    }
    virtual const char * getHelpText() const { return "ESC / a: Abort motion, l: Loop mode, s: Single mode"; }
    virtual void execute(int inKey) {
      switch(inKey) {
      case KEY_LEFT:
	mExposureMode = ExposureModeT::SINGLE;
	
	if (mExposureTimeSec <= 1 && mExposureTimeSec > 0.001) {
	  mExposureTimeSec /= 2;
	} else if (mExposureTimeSec > 1) {
	  mExposureTimeSec--;
	}
	mStatusText = "";
	break;
      
      case KEY_RIGHT:
	mExposureMode = ExposureModeT::SINGLE;
	
	if (mExposureTimeSec <= 1) {
	  mExposureTimeSec *= 2;
	} else if (mExposureTimeSec < 10000) {
	  mExposureTimeSec++;
	}
	mStatusText = "";
	break;
      
	// case KEY_ENTER:
	// 	//NOTE: Enter does not work for some reason...
	// 	//this->startExposure();
	// 	break;
      
      case 27: /*ESC*/
      case 'a':
	LOG(debug) << "Aborting exposure... Is exposure in progress? " << (mCameraDevice->isExposureInProgress() ? "yes" : "no") << endl;
	mExposureMode = ExposureModeT::SINGLE;
	
	if (mCameraDevice->isExposureInProgress()) {
	  LOG(debug) << "Ok, initiating abort procedure..." << endl;
	  mCameraDevice->abortExposure();
	  mExposureDone = false;
	  mStatusText = "Exposure aborted.";
	}
	break;

      case 'l':
	LOG(debug) << "Switching to LOOP mode..." << endl;
	mExposureDone = false;
	mExposureMode = ExposureModeT::LOOP;
	break;

      case 's':
	LOG(debug) << "Switching to SINGLE mode..." << endl;
	mExposureDone = false;
	mExposureMode = ExposureModeT::SINGLE;
	break;
      } /* end switch*/
    }
    inline float getExposureTime() const { return mExposureTimeSec; }
    inline ExposureModeT::TypeE getExposureMode() const { return mExposureMode; }
    inline bool getExposureDone() const { return mExposureDone; }
    inline void setExposureDone(bool inExposureDone) { mExposureDone = inExposureDone; }
  };


  class ControlBinningT : public MenuEntryT {
  private:
    IndiCameraT * mCameraDevice;
    BinningT mBinning;
    
  public:
    ControlBinningT(IndiCameraT * inCameraDevice, BinningT inBinning) : mCameraDevice(inCameraDevice), mBinning(inBinning) { }
    virtual const char * getName() const { return "Binning :"; }
    virtual BinningT getBinning() const { return mBinning; }
    virtual string getValueAsStr() const { stringstream ss; ss << mBinning.get<0>() << " x " << mBinning.get<1>(); return ss.str(); }
    virtual const char * getHelpText() const { return ""; }
    virtual void execute(int inKey) {
      switch(inKey) {
      case KEY_LEFT: {
	if (mBinning.get<0>() > 1  /* We just use hor binning here...*/) {
	  mBinning.get<0>()--;
	  mBinning.get<1>()--;
	}
	mStatusText = "";
	break;
      }
      case KEY_RIGHT: {
	BinningT maxBinning = mCameraDevice->getMaxBinning();
	if (mBinning.get<0>() < maxBinning.get<0>() /* We just use hor binning here...*/) {
	  mBinning.get<0>()++;
	  mBinning.get<1>()++;
	}
	mStatusText = "";
	break;
      }
      }
    }
  };

  class ControlCameraTemperatureT : public MenuEntryT {
  private:
    IndiCameraT * mCameraDevice;
    int mTemperature;
    
  public:
    ControlCameraTemperatureT(IndiCameraT * inCameraDevice, int inTemperature = -15) : mCameraDevice(inCameraDevice),
										       mTemperature(inTemperature) { }
    virtual const char * getName() const { return "Temperature [\0xf8 C] :"; }
    virtual BinningT getTemperature() const { return mTemperature; }
    virtual string getValueAsStr() const { stringstream ss; ss << mTemperature << "\0xf8 C ["
							       << (mCameraDevice->isCoolerEnabled() ? "ON" : "OFF") << "]"; return ss.str(); }
    virtual const char * getHelpText() const { return ""; }
    virtual void execute(int inKey) {
      switch(inKey) {
      case KEY_LEFT: {
	if (mTemperature >= mCameraDevice->getMinTemperature()) {
	  mTemperature--;
	}
	mStatusText = "";
	break;
      }
      case KEY_RIGHT: {
	if (mTemperature <= mCameraDevice->getMaxTemperature()) {
	  mTemperature++;
	}
	mStatusText = "";
	break;
      }
      }
    }
  };


  class FilterSelectT : public MenuEntryT {
  private:
    IndiFilterWheelT * mFilterWheelDevice;
    int mSelectedFilter;
    
  public:
    FilterSelectT(IndiFilterWheelT * inFilterWheelDevice, int inPos) : mFilterWheelDevice(inFilterWheelDevice),
								       mSelectedFilter(inPos) { }
    virtual const char * getName() const { return "Filter pos (dest) :"; }
    virtual BinningT getSelectedFilter() const { return mSelectedFilter; }
    virtual string getValueAsStr() const { stringstream ss; ss << mSelectedFilter; return ss.str(); }
    virtual const char * getHelpText() const { return ""; }
    virtual void execute(int inKey) {
      switch(inKey) {
      case KEY_LEFT:
	if (mSelectedFilter > mFilterWheelDevice->getMinPos()) {
	  mSelectedFilter--;
	} else {
	  mSelectedFilter = mFilterWheelDevice->getNumSlots();
	}

      // TODO / FIXME!: terminate called after throwing an instance of 'IndiFilterWheelIsBusyExceptionT'
      //                                         what():  Cannot set new filter position - filter wheel is currently busy.

	
	mFilterWheelDevice->setPos(mSelectedFilter, 0 /* non-blocking */);
	mStatusText = "";
	break;
	
      case KEY_RIGHT:
	if (mSelectedFilter < mFilterWheelDevice->getNumSlots()) {
	  mSelectedFilter++;
	} else {
	  mSelectedFilter = mFilterWheelDevice->getMinPos();
	}
	mFilterWheelDevice->setPos(mSelectedFilter, 0 /* non-blocking */);
	mStatusText = "";
	break;
      }
    }
  };
  
  
  DEF_Exception(FocusFinderPlugin);
  DEF_Exception(UnknownFocusFinderImpl);
  DEF_Exception(RequireOption);
  DEF_Exception(ImageDimension);
  DEF_Exception(WindowOutOfBounds);
  
  template <typename... Args> void
  mv_print(int x, int y, const char * c, Args&&... args) {
    move(y, x);
    clrtoeol();
    mvprintw(y, x, c, std::forward<Args>(args)...);
  }

  // TODO: Create ManualConsoleFocusCntl class or similar...??!
  void
  manualConsoleFocusCntl(const po::variables_map & inCmdLineMap, IndiCameraT * inCameraDevice, IndiFocuserT * inFocuserDevice, IndiFilterWheelT * inFilterWheelDevice, const FrameT<unsigned int> & inSelectionFrame, float inExposureTimeSec, BinningT inBinning, bool inFollowStar = true);
  
}; // end AT

#endif /* _FOCUSER_CONSOLE_UI_HPP_ */
