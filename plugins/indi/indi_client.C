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

#include "indi_client.hpp"

#include "indi_device.hpp"
#include "indi_camera.hpp"
#include "indi_focuser.hpp"
#include "indi_filter_wheel.hpp"

const char * IndiClientT::sHostnameNvp = "hostname";
const char * IndiClientT::sPortNvp = "port";
const char * IndiClientT::sAutoConnectNvp = "auto_connect";
const char * IndiClientT::sDeviceNameMapNvp = "device_name_type_map";

double IndiClientT::sInitialDevicePopulationDelaySec = 2.0;
int IndiClientT::sDefaultTimeoutMs = 5000;
const char * IndiClientT::sDefaultIndiHostname = "localhost";
const size_t IndiClientT::sDefaultIndiPort = 7624;

IndiClientT::~IndiClientT() {
  this->disconnect();
}

IndiDeviceT * IndiClientT::createDeviceInstance(INDI::BaseDevice * inBaseDevice, DeviceTypeT::TypeE inDeviceType) {
  switch (inDeviceType) {
  case DeviceTypeT::CAMERA: return new IndiCameraT(this, inBaseDevice);
    case DeviceTypeT::FOCUSER: return new IndiFocuserT(this, inBaseDevice);
    case DeviceTypeT::FILTER_WHEEL: return new IndiFilterWheelT(this, inBaseDevice);
    case DeviceTypeT::TELESCOPE: return 0; /* TODO: Implement... */ /* MOUNT?! */
    case DeviceTypeT::CUSTOM: {
      LOG(warning) << "Could not classify device - creating common IndiDeviceT device (type " << DeviceTypeT::asStr(DeviceTypeT::CUSTOM) <<")." << endl;
      return new IndiDeviceT(this, inBaseDevice); /* No specific C interface for unrecognized device type -> fallback. */
    }
  default: {
    const string exStr = "Unknown device type '" + string(DeviceTypeT::asStr(inDeviceType)) + "' - no device instance created!"; 
    throw UnknownIndiDeviceTypeExceptionT(exStr.c_str());
  }
  } // end switch
  AT_ASSERT(IndiClient, false, "End of IndiClientT::createDeviceInstance should never be reached.");
}

IndiDeviceT * IndiClientT::getDevice(const char * inDeviceName, DeviceTypeT::TypeE inDeviceType) /*TODO: const?!*/ {
  BaseDevice * baseDevice = this->BaseClient::getDevice(inDeviceName);
  if (! baseDevice) {
    const string exStr = "Device '" + string(inDeviceName) + "' not found.";
    throw IndiDeviceNotFoundExceptionT(exStr.c_str());
  }
  
  IndiDeviceMapT::iterator it = mDeviceMap.find(baseDevice);
  if (it != mDeviceMap.end()) {
    IndiDeviceT * devInst = it->second;

    if (inDeviceType != devInst->getType()) {
      // Type has changed, delete and create new instance...
      LOG(debug) << "IndiClientT::getDevice - Device type has changed, delete and create new instance..." << endl;
      delete devInst;
      mDeviceMap[baseDevice] = createDeviceInstance(baseDevice, inDeviceType);
    } else {
      // Same type, just return existing instance.
      LOG(debug) << "IndiClientT::getDevice - Same device type, just return existing instance...." << endl;
      return devInst; // return ptr to IndiDeviceT in map
    }
  } else {
    // No device instance found, yet. Create new one...
    LOG(debug) << "IndiClientT::getDevice - No device instance found, yet. Create new one...." << endl;
    IndiDeviceT * newDevInst = createDeviceInstance(baseDevice, inDeviceType);
    mDeviceMap.insert(make_pair(baseDevice, newDevInst));
    return newDevInst;
  }
  return 0;
}

// TODO: Are those functions below required any longer?!
IndiCameraT * IndiClientT::getCamera(const string & inDeviceName) {
  return static_cast<IndiCameraT*>(this->getDevice(inDeviceName.c_str(), DeviceTypeT::CAMERA));
}

IndiFocuserT * IndiClientT::getFocuser(const string & inDeviceName) {
  return static_cast<IndiFocuserT*>(this->getDevice(inDeviceName.c_str(), DeviceTypeT::FOCUSER));
}

IndiFilterWheelT * IndiClientT::getFilterWheel(const string & inDeviceName) {
  return static_cast<IndiFilterWheelT*>(this->getDevice(inDeviceName.c_str(), DeviceTypeT::FILTER_WHEEL));
}


void IndiClientT::newDevice(INDI::BaseDevice * inBaseDevice) {
  AT_ASSERT(IndiClient, inBaseDevice, "Expecting inBaseDevice to be valid pointer.");
  LOG(debug) << "INDI CLIENT - New device received: " << inBaseDevice->getDeviceName() << endl;
  mNewDeviceListeners(inBaseDevice);
}

void IndiClientT::connect() {
  if (! mIndiServerConnected) {
    mAutoConnected = false;
    
    // TODO: Log... connecting client to hostname:port server...
    
    // According to the indi documentation, this function blocks until connection is either successul or unsuccessful.
    mIndiServerConnected = this->BaseClient::connectServer();
    
    if (! mIndiServerConnected) {
      throw IndiClientConnectFailedExceptionT();
    }
  }
  
  LOG(debug) << "Waiting for devices..." << flush;
  waitForInitialDevicePopulation();
  
  // typedef vector<INDI::BaseDevice *> BaseDeviceVecT;
  // const BaseDeviceVecT & baseDevices = this->BaseClient::getDevices();
  // LOG(debug) << "DONE. Have " << baseDevices.size() << " device" << (baseDevices.size() > 1 ? "s..." : "...") << endl;
  
  // AT_ASSERT(IndiClient, ! mDeviceMap.size(), "Expect mDeviceMap being empty.");
  // for (BaseDeviceVecT::const_iterator it = baseDevices.begin(); it != baseDevices.end(); ++it) {
  //   std::pair<IndiDeviceMapT::iterator, bool> ret = mDeviceMap.insert(make_pair(*it, this->createDeviceInstance(*it)));
  //   AT_ASSERT(IndiClient, ret.second, "Expect new device element to be inserted into map.");
  // }
}

void IndiClientT::disconnect() {
  if (mIndiServerConnected) {
    // We have to wait before closing due to a bug in libindi
    usleep(1000000 /* 1sec */);
    if (! this->BaseClient::disconnectServer()) {
      throw IndiClientDisconnectFailedExceptionT();
    }

    // We have to wait before closing due to a bug in libindi
    usleep(1000000 /* 1sec */);

    // Delete indi devices and clear all map entries
    for (IndiDeviceMapT::iterator it = mDeviceMap.begin(); it != mDeviceMap.end(); ++it) {
      delete it->second;
    }
    mDeviceMap.clear();
  }
}

ostream & IndiClientT::print(ostream & os) const {
  // TODO: Print devices and other details!!
  os << "IndiClient: " << getHost() << ":" << getPort() << endl;
  return os;
}

// Overloading IndiClient output operator 
ostream & operator<<(ostream & os, const IndiClientT & inIndiClient) {
  return inIndiClient.print(os);
}
