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

#ifndef _INDI_UTILS_HPP_
#define _INDI_UTILS_HPP_ _INDI_UTILS_HPP_

#include "indiapi.h"
#include "indidevapi.h"
#include "indicom.h"
#include "baseclient.h"
#include "basedevice.h"

#include "at_exception.hpp"
#include "util.hpp"

DEF_Exception(IndiUtils);
DEF_Exception(PropNotFound);

struct IndiUtilsT {
public:
  static size_t sColNum;
  static size_t sColNumDiff;

  /**
   * Definition from indibase.h
   *   INDI_NUMBER - INumberVectorProperty
   *   INDI_SWITCH - ISwitchVectorProperty
   *   INDI_TEXT   - ITextVectorProperty
   *   INDI_LIGHT  - ILightVectorProperty
   *   INDI_BLOB   - IBLOBVectorProperty
   *   INDI_UNKNOWN
   */
  static inline const char * asStr(const INDI_PROPERTY_TYPE  & inIndiType) {
    switch(inIndiType) {
    case INDI_NUMBER: return "INDI_NUMBER";
    case INDI_SWITCH: return "INDI_SWITCH";
    case INDI_TEXT: return "INDI_TEXT";
    case INDI_LIGHT: return "INDI_LIGHT";
    case INDI_BLOB: return "INDI_BLOB";
    default: return "<?>";
    }
  }

  /**
   * Definition from indiapi.h
   * ISS_OFF - Switch is OFF
   * ISS_ON  - Switch is ON
   */
  static inline const char * asStr(const ISState & inISState) {
    switch(inISState) {
    case ISS_OFF: return "ISS_OFF";
    case ISS_ON: return "ISS_ON";
    default: return "<?>";
    }    
  }

  /**
   * Definition from indiapi.h
   * IPS_IDLE  - State is idle
   * IPS_OK    - State is ok
   * IPS_BUSY  - State is busy
   * IPS_ALERT - State is alert
   */
  static inline const char * asStr(const IPState & inIPState) {
    switch(inIPState) {
    case IPS_IDLE: return "IPS_IDLE";
    case IPS_OK: return "IPS_OK";
    case IPS_BUSY: return "IPS_BUSY";
    case IPS_ALERT: return "IPS_ALERT";
    default: return "<?>";
    }    
  }

  /**
   * Definition from indiapi.h
   * IP_RO - Read Only
   * IP_WO - Write Only
   * IP_RW - Read & Write
   */
  static inline const char * asStr(const IPerm & inIPerm) {
    switch(inIPerm) {
    case IP_RO: return "IP_RO";
    case IP_WO: return "IP_WO";
    case IP_RW: return "IP_RW";
    default: return "<?>";
    }    
  }

  /**
   * Definition from indiapi.h
    ISR_1OFMANY - Only 1 switch of many can be ON (e.g. radio buttons)
    ISR_ATMOST1 - There is only ONE switch
    ISR_NOFMANY - Any number of switches can be ON (e.g. check boxes)
   */
  static inline const char * asStr(const ISRule & inISRule) {
    switch(inISRule) {
    case ISR_1OFMANY: return "ISR_1OFMANY";
    case ISR_ATMOST1: return "ISR_ATMOST1";
    case ISR_NOFMANY: return "ISR_NOFMANY";
    default: return "<?>";
    }    
  }
};

ostream & operator<<(ostream & os, const INDI::Property &);
ostream & operator<<(ostream & os, const INumberVectorProperty &);
ostream & operator<<(ostream & os, const INumber &);
ostream & operator<<(ostream & os, const ISwitchVectorProperty &);
ostream & operator<<(ostream & os, const ISwitch &);
ostream & operator<<(ostream & os, const ITextVectorProperty &);
ostream & operator<<(ostream & os, const IText &);
ostream & operator<<(ostream & os, const ILightVectorProperty &);
ostream & operator<<(ostream & os, const ILight &);
ostream & operator<<(ostream & os, const IBLOBVectorProperty &);
ostream & operator<<(ostream & os, const IBLOB &);



#endif /* _INDI_UTILS_HPP_ */
