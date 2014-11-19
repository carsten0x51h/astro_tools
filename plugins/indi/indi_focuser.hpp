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

#ifndef _INDI_FOCUSER_HPP_
#define _INDI_FOCUSER_HPP_ _INDI_FOCUSER_HPP_

// See: http://www.boost.org/doc/libs/1_36_0/libs/tuple/doc/tuple_users_guide.html
#include <boost/tuple/tuple.hpp>

#include "indi_device.hpp"
#include "at_exception.hpp"

using namespace boost;

DEF_Exception(IndiFocuser);
DEF_Exception(IndiFocuserNotConnected);
DEF_Exception(IndiFocuserIsBusy);


/**
 * Focuser traits.
 *
 * TODO: Further props from Focuser Simulator - required?
 *
 * Presets
 *   Preset 1
 *   Preset 2
 *   Preset 3
 *
 * Goto
 *   Preset 1
 *   Preset 2
 *   Preset 3
 *
 * SEEING_SETTINGS
 *   SIM_SEEING
 *
 * FWHM
 *   SIM_FWHM
 */

// See: http://en.wikipedia.org/wiki/Curiously_recurring_template_pattern
// See: http://en.wikipedia.org/wiki/Template_metaprogramming#Static_polymorphism
struct FocuserTraitsT {
  static const DeviceTypeT::TypeE sDeviceType;

  struct VecPropsT {
    enum TypeE {
      CONNECTION,
      DRIVER_INFO,
      CONFIG_PROCESS,
      DEVICE_PORT,
      DEBUG,
      FOCUS_MOTION,
      FOCUS_SPEED,
      FOCUS_TIMER,
      ABS_FOCUS_POSITION,
      FOCUS_TEMPERATURE,  // special prop. from MoonLite focuser
      REL_FOCUS_POSITION, // special prop. from MoonLite focuser
      FOCUS_ABORT_MOTION, // special prop. from MoonLite focuser
      RESET, // special prop. from MoonLite focuser
      _Count
    };

    // TODO: Maybe specifying the type to string translation is enough!
    static const char * asStr(const TypeE & inType) {
      switch (inType) {
      case CONNECTION: return "CONNECTION";
      case DRIVER_INFO: return "DRIVER_INFO";
      case CONFIG_PROCESS: return "CONFIG_PROCESS";
      case DEVICE_PORT: return "DEVICE_PORT";
      case DEBUG: return "DEBUG";
      case FOCUS_MOTION: return "FOCUS_MOTION";
      case FOCUS_SPEED: return "FOCUS_SPEED";
      case FOCUS_TIMER: return "FOCUS_TIMER";
      case ABS_FOCUS_POSITION: return "ABS_FOCUS_POSITION";
      case FOCUS_TEMPERATURE: return "FOCUS_TEMPERATURE";
      case REL_FOCUS_POSITION: return "REL_FOCUS_POSITION";
      case FOCUS_ABORT_MOTION: return "FOCUS_ABORT_MOTION";
      case RESET: return "Reset"; // NOTE: Small letters...
      default: return "<?>";
      }
    }

    MAC_AS_TYPE(Type, E, _Count);

    static bool isOptional(const TypeE & inType) {
      switch (inType) {
      case FOCUS_TEMPERATURE:
      case REL_FOCUS_POSITION:
      case FOCUS_ABORT_MOTION:
      case RESET:
	return true;
      default:
    	return false;
      }
    }
  };


  struct PropsT {
    enum TypeE {
      CONNECT,
      DISCONNECT,
      PORT,
      NAME,
      EXEC,
      VERSION,
      ENABLE,
      DISABLE,
      FOCUS_INWARD,
      FOCUS_OUTWARD,
      FOCUS_SPEED_VALUE,
      FOCUS_TIMER_VALUE,
      FOCUS_ABSOLUTE_POSITION,
      CONFIG_LOAD,
      CONFIG_SAVE,
      CONFIG_DEFAULT,
      TEMPERATURE,                 // special prop. from MoonLite focuser
      RELATIVE_ABSOLUTE_POSITION,  // special prop. from MoonLite focuser
      ABORT,                       // special prop. from MoonLite focuser
      ZERO,                        // special prop. from MoonLite focuser
      _Count
    };

    
    static const char * asStr(const TypeE & inType) {
      switch (inType) {
      case CONNECT: return "CONNECT";
      case DISCONNECT: return "DISCONNECT";
      case PORT: return "PORT";
      case NAME: return "NAME";
      case EXEC: return "EXEC";
      case VERSION: return "VERSION";
      case ENABLE: return "ENABLE";
      case DISABLE: return "DISABLE";
      case FOCUS_INWARD: return "FOCUS_INWARD";
      case FOCUS_OUTWARD: return "FOCUS_OUTWARD";
      case FOCUS_SPEED_VALUE: return "FOCUS_SPEED_VALUE";
      case FOCUS_TIMER_VALUE: return "FOCUS_TIMER_VALUE";
      case FOCUS_ABSOLUTE_POSITION: return "FOCUS_ABSOLUTE_POSITION";
      case CONFIG_LOAD: return "CONFIG_LOAD";
      case CONFIG_SAVE: return "CONFIG_SAVE";
      case CONFIG_DEFAULT: return "CONFIG_DEFAULT";
      case TEMPERATURE: return "TEMPERATURE";
      case RELATIVE_ABSOLUTE_POSITION: return "RELATIVE_ABSOLUTE_POSITION";
      case ABORT: return "ABORT";
      case ZERO: return "Zero";
      default: return "<?>";
      }
    }
    
    // TODO: As macro...
    static typename PropsT::TypeE asType(const char * typeName) {
      for (size_t i=0; i < PropsT::_Count; ++i) {
	typename PropsT::TypeE type = static_cast<typename PropsT::TypeE> (i);
	if (! strcmp(typeName, PropsT::asStr(type))) {
	  return type;
	}
      }
      return PropsT::_Count;
    }

    static bool isOptional(const TypeE & inType) {
      switch (inType) {
      case PORT:
      case TEMPERATURE:
      case RELATIVE_ABSOLUTE_POSITION:
      case ABORT:
      case ZERO:
	return true;
      default:
    	return false;
      }
    }

  };
};

/**
 * Focuser directions.
 *
 */
struct FocusDirectionT {
  enum TypeE {
    INWARDS,
    OUTWARDS,
    _Count
  };
    
  static const char * asStr(const TypeE & inType) {
    switch (inType) {
    case INWARDS: return "INWARDS";
    case OUTWARDS: return "OUTWARDS";
    default: return "<?>";
    }
  }

  static TypeE invert(const TypeE & inCurDir) {
    return (inCurDir == INWARDS ? OUTWARDS : INWARDS);
  }
}; // end struct


/**
 * Focuser device.
 *
 */
class IndiFocuserT : public IndiDeviceT {
private:
  // We do not want device copies
  IndiFocuserT(const IndiFocuserT &);
  IndiFocuserT & operator=(const IndiFocuserT &); 

public:
  typedef FocuserTraitsT::VecPropsT VecPropsT;
  typedef FocuserTraitsT::PropsT PropsT;

  static unsigned int estimateMaxFocuserMovementDelayMs(unsigned int inSteps) {
    // NOTE: Assume at max 2 sec for 100 steps --> 20ms per step, we add another 2 seconds to be sure.
    return 20 * inSteps + 2000;
  }

  
  // TODO: Should not be allowed... otherwise... which device should be assigned?!
  IndiFocuserT(IndiClientT * inIndiClient, BaseDevice * inBaseDevice) : IndiDeviceT(inIndiClient, inBaseDevice, IndiFocuserT::getType()) {
  }
  static DeviceTypeT::TypeE getType() { return FocuserTraitsT::sDeviceType; }

  /**
   * Temperature
   *
   */
  inline double getTemperature() const { return this->getNumberVal<FocuserTraitsT>(VecPropsT::FOCUS_TEMPERATURE, PropsT::TEMPERATURE); }

  /**
   * Focus relative position
   */
  inline int getRelPos() const { return this->getNumberVal<FocuserTraitsT>(VecPropsT::REL_FOCUS_POSITION, PropsT::RELATIVE_ABSOLUTE_POSITION); }
  inline void setRelPos(unsigned int inRelPos, int inTimeoutMs = sDefaultTimeoutMs) {
    if (this->isMovementInProgess())
      throw IndiFocuserIsBusyExceptionT("Cannot set new relative position - focuser is currently moving.");

    this->sendNumberVal<FocuserTraitsT>(VecPropsT::REL_FOCUS_POSITION, PropsT::RELATIVE_ABSOLUTE_POSITION, inRelPos, inTimeoutMs);
  }

  /**
   * Focus absolute position
   *
   * TODO / Note: For MoonLite focuser the PropsT::... is RELATIVE_ABSOLUTE_POSITION --> use map...
   */
  inline int getAbsPos() const { return this->getNumberVal<FocuserTraitsT>(VecPropsT::ABS_FOCUS_POSITION, PropsT::FOCUS_ABSOLUTE_POSITION); }
  inline void setAbsPos(unsigned int inAbsPos, int inTimeoutMs = sDefaultTimeoutMs) {
    if (this->isMovementInProgess())
      throw IndiFocuserIsBusyExceptionT("Cannot set new absolute position - focuser is currently moving.");

    this->sendNumberVal<FocuserTraitsT>(VecPropsT::ABS_FOCUS_POSITION, PropsT::FOCUS_ABSOLUTE_POSITION, inAbsPos, inTimeoutMs);

    // if (inTimeoutMs > 0) {
    //   /**
    //    *  NOTE: For some reason the value returned from the indi client is not
    //    *       the one which should have been reached right after calling setNumberVal
    //    *       (i.e. if the state returns to OK / IDLE). Hence we wait for the values
    //    *       to be the same to be absoultely sure.
    //    */
    //   WAIT_MAX_FOR(getAbsPos() == inAbsPos, estimatedMaxFocuserDelayMs, "Hit timeout while waiting for focuser reaching position.");
    // }
  }

  inline bool isMovementInProgess() const {
    // TODO: Is this the correct prop. to check for busy state?!
    // TODO: How to handle error state?!
    INumberVectorProperty * nVecAbs = this->getNumberVec<FocuserTraitsT>(VecPropsT::ABS_FOCUS_POSITION);
    INumberVectorProperty * nVecRel = this->getNumberVec<FocuserTraitsT>(VecPropsT::REL_FOCUS_POSITION);
    return (nVecAbs->s == IPS_BUSY || nVecRel->s == IPS_BUSY);
  }

  inline void abortMotion(int inTimeoutMs = sDefaultTimeoutMs) {
    if (this->isMovementInProgess()) {
      this->sendSwitchVal<FocuserTraitsT>(VecPropsT::FOCUS_ABORT_MOTION, PropsT::ABORT, true, inTimeoutMs);
      WAIT_MAX_FOR(! this->isMovementInProgess(), inTimeoutMs, "Hit timeout setting while setting value.");
    }
  }

  inline void resetPosition(int inTimeoutMs = sDefaultTimeoutMs) {
    if (this->isMovementInProgess())
      throw IndiFocuserIsBusyExceptionT("Cannot reset focuser position - focuser is currently moving.");

    this->sendSwitchVal<FocuserTraitsT>(VecPropsT::RESET, PropsT::ZERO, true, inTimeoutMs);
    WAIT_MAX_FOR(this->getAbsPos() == 0, inTimeoutMs, "Hit timeout setting while setting value.");
  }



  /**
   * Focus direction
   *
   */
  inline FocusDirectionT::TypeE getFocusDirection() const {
    return (this->getSwitchVal<FocuserTraitsT>(VecPropsT::FOCUS_MOTION, PropsT::FOCUS_INWARD) ? FocusDirectionT::INWARDS : FocusDirectionT::OUTWARDS);
  }

  inline void setFocusDirection(FocusDirectionT::TypeE inDirection, int inTimeoutMs = sDefaultTimeoutMs) {
    if (this->isMovementInProgess())
      throw IndiFocuserIsBusyExceptionT("Cannot set new focuser direction - focuser is currently moving.");

    this->sendSwitchVal<FocuserTraitsT>(VecPropsT::FOCUS_MOTION, PropsT::FOCUS_INWARD, (inDirection == FocusDirectionT::INWARDS), inTimeoutMs);
  }

  inline FocusDirectionT::TypeE invertFocusDirection(int inTimeoutMs = sDefaultTimeoutMs) {
    const FocusDirectionT::TypeE newDirection = FocusDirectionT::invert(this->getFocusDirection());
    this->setFocusDirection(newDirection, inTimeoutMs);
    return newDirection;
  }


  /**
   * Device connection
   *
   */
  inline string getDevicePort() const { return this->getTextVal<FocuserTraitsT>(VecPropsT::DEVICE_PORT, PropsT::PORT); }
  inline void setDevicePort(const string & inDevicePort, int inTimeoutMs = sDefaultTimeoutMs) {
    this->sendTextVal<FocuserTraitsT>(VecPropsT::DEVICE_PORT, PropsT::PORT, inDevicePort.c_str(), inTimeoutMs);
  }
  

  /*
   * Higher level function
   *
   */
  void moveFocusBy(unsigned int inSteps, FocusDirectionT::TypeE inDirection, int inTimeoutMs = sDefaultTimeoutMs) {
    this->setFocusDirection(inDirection, inTimeoutMs);
    this->setRelPos(inSteps, inTimeoutMs);
    WAIT_MAX_FOR(! this->isMovementInProgess(), inTimeoutMs, "Hit timeout setting while moving focus to new position.");
  }

 

  //DEFINE_PROP_LISTENER(NewImageReceived, IBLOB*);
};

#endif // _INDI_FOCUSER_HPP_
