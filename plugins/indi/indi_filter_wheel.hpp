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

#ifndef _INDI_FILTER_WHEEL_HPP_
#define _INDI_FILTER_WHEEL_HPP_ _INDI_FILTER_WHEEL_HPP_

#include "indi_device.hpp"

DEF_Exception(IndiFilterWheelIsBusy);

struct FilterWheelTraitsT {
  static const DeviceTypeT::TypeE sDeviceType;

  struct VecPropsT {
    enum TypeE {
      CONNECTION,
      DRIVER_INFO,
      CONFIG_PROCESS,
      FILTER_SLOT,
      _Count
    };

    // TODO: Maybe specifying the type to string translation is enough!
    static const char * asStr(const TypeE & inType) {
      switch (inType) {
      case CONNECTION: return "CONNECTION";
      case DRIVER_INFO: return "DRIVER_INFO";
      case CONFIG_PROCESS: return "CONFIG_PROCESS";
      case FILTER_SLOT: return "FILTER_SLOT";
      default: return "<?>";
      }
    }

    MAC_AS_TYPE(Type, E, _Count);

    static bool isOptional(const TypeE & inType) {
      switch (inType) {
      default:
    	return false;
      }
    }
  };


  struct PropsT {
    enum TypeE {
      CONNECT,
      DISCONNECT,
      NAME,
      EXEC,
      VERSION,
      FILTER_SLOT_VALUE,
      _Count
    };

    
    static const char * asStr(const TypeE & inType) {
      switch (inType) {
      case CONNECT: return "CONNECT";
      case DISCONNECT: return "DISCONNECT";
      case NAME: return "NAME";
      case EXEC: return "EXEC";
      case VERSION: return "VERSION";
      case FILTER_SLOT_VALUE: return "FILTER_SLOT_VALUE";
      default: return "<?>";
      }
    }
    
    MAC_AS_TYPE(Type, E, _Count);

    static bool isOptional(const TypeE & inType) {
      switch (inType) {
      default:
    	return false;
      }
    }

  };
};


class IndiFilterWheelT : public IndiDeviceT {
private:
  // We do not want device copies
  IndiFilterWheelT(const IndiFilterWheelT &);
  IndiFilterWheelT & operator=(const IndiFilterWheelT &); 

public:
  typedef FilterWheelTraitsT::VecPropsT VecPropsT;
  typedef FilterWheelTraitsT::PropsT PropsT;

  // TODO: Should not be allowed... otherwise... which device should be assigned?!
  IndiFilterWheelT(IndiClientT * inIndiClient, BaseDevice * inBaseDevice) : IndiDeviceT(inIndiClient, inBaseDevice, IndiFilterWheelT::getType()) {
  }
  static DeviceTypeT::TypeE getType() { return FilterWheelTraitsT::sDeviceType; }

  /**
   * Filter position
   */
  inline int getPos() const { return this->getNumberVal<FilterWheelTraitsT>(VecPropsT::FILTER_SLOT, PropsT::FILTER_SLOT_VALUE); }
  inline void setPos(unsigned int inPos, int inTimeoutMs = sDefaultTimeoutMs) {
    if (this->isMovementInProgess())
      throw IndiFilterWheelIsBusyExceptionT("Cannot set new filter position - filter wheel is currently busy.");
    this->sendNumberVal<FilterWheelTraitsT>(VecPropsT::FILTER_SLOT, PropsT::FILTER_SLOT_VALUE, inPos, inTimeoutMs);
  }

  inline bool isMovementInProgess() const {
    // TODO: Is this the correct prop. to check for busy state?!
    // TODO: How to handle error state?!
    INumberVectorProperty * nVec = this->getNumberVec<FilterWheelTraitsT>(VecPropsT::FILTER_SLOT);
    return (nVec->s == IPS_BUSY);
  }

  inline size_t getNumSlots() const {
    const INumber & number = this->getNumber<FilterWheelTraitsT>(VecPropsT::FILTER_SLOT, PropsT::FILTER_SLOT_VALUE);
    return number.max;
  }
};

#endif // _INDI_FILTER_WHEEL_HPP_