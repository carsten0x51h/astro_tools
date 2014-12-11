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

#include "indi_device.hpp"
#include "io_util.hpp"

// TODO: Maybe forward decl. is enough..
#include "indi_camera.hpp"
#include "indi_focuser.hpp"
#include "indi_filter_wheel.hpp"

const char * IndiDeviceT::sVecPropNameMapNvp = "vec_prop_name_map";
const char * IndiDeviceT::sPropNameMapNvp = "prop_name_map";

double IndiDeviceT::sInitialDevicePropertyPopulationDelaySec = 2.0;

int IndiDeviceT::sDefaultTimeoutMs = 5000;

// Further device functions...
// void addMessage(const char *msg);
// void setMediator(INDI::BaseMediator *med)
// INDI::BaseMediator * getMediator()
// INumberVectorProperty * getNumber(const char *name);
// ITextVectorProperty * getText(const char *name);
// ISwitchVectorProperty * getSwitch(const char *name);
// ILightVectorProperty * getLight(const char *name);
// IBLOBVectorProperty * getBLOB(const char *name);
// void * getRawProperty(const char *name, INDI_TYPE type = INDI_UNKNOWN);
// INDI::Property * getProperty(const char *name, INDI_TYPE type = INDI_UNKNOWN);

  // Check if device is of type ...
// TODO: Should this be moved to indi_client?!
DeviceTypeT::TypeE IndiDeviceT::getDeviceTypeFromProps() /*TODO: const?!getProperties... */ {
  const IndiPropVecT * props = mBaseDevice->getProperties();
  string uniqueDeviceId = getUniqueDeviceId();

  // TODO: extend...
  if (IndiDeviceT::satisfiesTraits<CameraTraitsT>(uniqueDeviceId, props)) return DeviceTypeT::CAMERA;
  else if (IndiDeviceT::satisfiesTraits<FocuserTraitsT>(uniqueDeviceId, props)) return DeviceTypeT::FOCUSER;
  else if (IndiDeviceT::satisfiesTraits<FilterWheelTraitsT>(uniqueDeviceId, props)) return DeviceTypeT::FILTER_WHEEL;
  else return DeviceTypeT::CUSTOM;
}



/**
 * Waits passed number of ms until device is connected, or pass 0 to not wait at all,
 * or pass -1 to wait forever. Throws a IndiDeviceTimeoutExceptionT if device is not
 * connected during this time.
 */
void IndiDeviceT::connect(int inTimeoutMs, bool inVerifyDeviceType) {
  AT_ASSERT(IndiDevice, mIndiClient, "Expected mIndiClient to be set.");
  AT_ASSERT(IndiDevice, mBaseDevice, "Expected mBaseDevice to be set.");

  // Sends CONNECTION - ISS_ON to the INDI server. Method returns directly
  // before the connection has been made and without evaluating possible errors.
  mIndiClient->connectDevice(mBaseDevice->getDeviceName());

  WAIT_MAX_FOR(mBaseDevice->isConnected(), inTimeoutMs, "Hit timeout connecting to INDI device.");

  LOG(debug) << "Waiting for props..." << endl;
  waitForInitialDevicePropertyPopulation();
  LOG(info) << "Ok, got " << dec << mBaseDevice->getProperties()->size() << " properties..." << endl;  

  if (inVerifyDeviceType) {
    DeviceTypeT::TypeE propDeviceType = getDeviceTypeFromProps();
    
    if (mType != propDeviceType) {
      stringstream exSs;
      exSs << "Device type determined by props=" << DeviceTypeT::asStr(propDeviceType)
	   << " does not match this device: " << DeviceTypeT::asStr(mType);
      LOG(error) << exSs.str() << endl;
      throw IndiDeviceTypeMismatchExceptionT(exSs.str().c_str());
    }
  }
}


/**
 * Waits passed number of ms until device is connected, or pass 0 to not wait at all,
 * or pass -1 to wait forever. Throws a IndiDeviceTimeoutExceptionT if device is not
 * disconnected during this time.
 */
void IndiDeviceT::disconnect(int inTimeoutMs) {
  AT_ASSERT(IndiDevice, mIndiClient, "Expected mIndiClient to be set.");
  AT_ASSERT(IndiDevice, mBaseDevice, "Expected mBaseDevice to be set.");

  // Sends CONNECTION - ISS_OFF to the INDI server. Method returns directly
  // before the connection has been made and without evaluating possible errors.
  mIndiClient->disconnectDevice(mBaseDevice->getDeviceName());

  WAIT_MAX_FOR(!mBaseDevice->isConnected(), inTimeoutMs, "Hit timeout disconnecting from INDI device.");
}


/* TODO: Replace string... - asStr enum? */
// double IndiDeviceT::getNumberVal(const string & inVecPropName, const string & inPropName) const {
//   AT_ASSERT(IndiDevice, mBaseDevice, "Expected mBaseDevice to be set.");
//   INumberVectorProperty * vecNumber = mBaseDevice->getNumber(inVecPropName.c_str());

//   if (! vecNumber) {
//     const string exStr = "Number vector property '" + inVecPropName  + "' not found.";
//     throw PropNotFoundExceptionT(exStr.c_str());
//   }

//   if (IP_WO == vecNumber->p) {
//     const string exStr = "Number vector property '" + inVecPropName  + "' is write-only.";
//     throw PropPermissionExceptionT(exStr.c_str());
//   }

//   return IndiUtilsT::extractNumber(vecNumber, inPropName).value;
// }


// TODO: Replace string... - asStr enum?!
// TODO: MACRO?!....
// TODO: Second method with enum-types?!....
// void IndiDeviceT::setNumberVal(const string & inVecPropName, const string & inPropName, double inNumber, int inTimeoutMs) {
//   AT_ASSERT(IndiDevice, mBaseDevice, "Expected mBaseDevice to be set.");
//   INumberVectorProperty * vecNumber = mBaseDevice->getNumber(inVecPropName.c_str());
  
//   if (! vecNumber) {
//     const string exStr = "Number vector property '" + inVecPropName  + "' not found.";
//     throw PropNotFoundExceptionT(exStr.c_str());
//   }

//   if (IP_RO == vecNumber->p) {
//     const string exStr = "Number vector property '" + inVecPropName  + "' is read-only.";
//     throw PropPermissionExceptionT(exStr.c_str());
//   }
 
//   IndiUtilsT::extractNumber(vecNumber, inPropName).value = inNumber;

//   mIndiClient->sendNewNumber(vecNumber);

//   WAIT_MAX_FOR(vecNumber->s != IPS_BUSY, inTimeoutMs, "Hit timeout setting number.");

//   if (IPS_ALERT == vecNumber->s) {
//     const string exStr = "Problem setting " + inVecPropName + " vector (ALERT state).";
//     throw SetPropValueFailedExceptionT(exStr.c_str());
//   }
// }


ostream & IndiDeviceT::print(ostream & os) const {
  const size_t cCol1 = IndiUtilsT::sColNum, cCol2 = 20;

  os << setw(cCol1) << " " << setw(cCol2) << "IndiDeviceT"         << left << endl
     << setw(cCol1) << " " << setw(cCol2) << "> deviceName: "      << mBaseDevice->getDeviceName() << endl
     << setw(cCol1) << " " << setw(cCol2) << "> driverName: "      << mBaseDevice->getDriverName() << endl
     << setw(cCol1) << " " << setw(cCol2) << "> driverExec: "      << mBaseDevice->getDriverExec() << endl
     << setw(cCol1) << " " << setw(cCol2) << "> isConnected: "     << (mBaseDevice->isConnected() ? "yes" : "no") << endl
     << setw(cCol1) << " " << setw(cCol2) << "> #Props: "          << mBaseDevice->getProperties()->size() << endl
     << setw(cCol1) << " " << setw(cCol2) << "> Device-Properties" << endl;

  // TODO: How to handle property changes by other thread?!??! 
  IndiUtilsT::sColNum += IndiUtilsT::sColNumDiff;
  for (IndiPropVecT::const_iterator it = mBaseDevice->getProperties()->begin(); it != mBaseDevice->getProperties()->end(); ++it) {
    INDI::Property * prop = *it;
    os << *prop;
  } // end for
  IndiUtilsT::sColNum -= IndiUtilsT::sColNumDiff;

  return os;
}

// Overloading IndiDevice output operator 
ostream & operator<<(ostream & os, const IndiDeviceT & inIndiDevice) {
  return inIndiDevice.print(os);
}
