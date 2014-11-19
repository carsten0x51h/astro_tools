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

#ifndef _INDI_DEVICE_MANAGER_HPP_
#define _INDI_DEVICE_MANAGER_HPP_ _INDI_DEVICE_MANAGER_HPP_

/************************************************************
 * DeviceManager
 *
 * Purpose
 * -Program wide organization and management of (INDI) device connection
 * -More specific, allow access to devices from all parts of the program
 * -Provides a list of available INDI devices (names + types -> TODO: Is there a way to get the INDI device type?)
 * -Handles connect and disconnect of devices (i.e. error handling when device disconnects or is not available)
 * -Maps certain INDI devices to a "roles"
 * -Load and save the role mapping in a configuration file
 *
 * Aliases
 * -An alias is a kind of variable which provides access to a certain device under a given name
 * -An alias is for example "imaging camera", "guiding camera", "guiding focuser", "imaging focuser",
 *  "imaging filter wheel", "guiding filter wheel", "mount"...
 * -Technically, an alias consists of a name (like "imaging camera") and a device type which is accepted
 *  for this alias (e.g. camera, focuser, filterwheel, mount, ...)
 * -Each alias can be assigned 0..1 devices i.e. an alias can be empty
 * -There are n devices available and 0..m (m<=n) of them are coupled to aliases
 * -Each device can only be assigned to one alias at a time (?? Really ??)
 * -There are certain __default__ aliases defined which are coupled to __fixed__ names so that they can
 *  be used for example in python scripts but also hard coded in the C++ code
 * -In addition the user can define additional aliases (alias name and device type) which can then be
 *  used in additional python scripts
 *
 * Use Cases
 * 1.) Add / remove user alias
 * 2.) Get device alias(es) class
 * 3.) Get list of available devices (available does not say if device is connected or not)
 * 4.) Get device for alias
 * 5.) Couple / decouple alias and device
 * 6.) Load / save configuration
 *
 * 0..N Devices (connected / unconnected ------ Device name 0..N <-----------> 1 device-alias
 *        / asyncronous new /remove)
 */

#include <string.h>

// Serialization
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/array.hpp>
#include <fstream>

#include "at_exception.hpp"
#include "util.hpp"

using namespace std;
using namespace boost::serialization;

DEF_Exception(IndiDeviceManagerAliasDoesNotExist);


// TODO: Required?!
struct BaseAliasT {
  enum TypeE {
    IMAGE_CAMERA = 0,
    GUIDE_CAMERA,
    FOCUSER,
    FILTER_WHEEL,
    MOUNT,
    /* TODO: More to come */
    _Count
  };
  
  static inline const char * asStr(const TypeE & inType) {
    switch (inType) {
    case IMAGE_CAMERA: return "IMAGE_CAMERA";
    case GUIDE_CAMERA: return "GUIDE_CAMERA";
    case FOCUSER: return "FOCUSER";
    case FILTER_WHEEL: return "FILTER_WHEEL";
    case MOUNT: return "MOUNT"; // TODO: or TELESCOPE?!
    default: return "<?>";
    }
  }

  MAC_AS_TYPE(Type, E, _Count);

}; // end struct


// TODO: Later.... first compose interface...
// TODO: Overwrite == operator?! --> "Alias = deviceName"
// class DeviceAliasT : public std::string {
// };

class IndiDeviceManagerT {
private:
  typedef boost::array<string /* device name */, BaseAliasT::_Count> DeviceBaseAliasMapT;
  DeviceBaseAliasMapT mDeviceBaseAliasMap;
  typedef map<string /* alias --> DeviceAliasT class instead?? */, string /* device name */> DeviceUserAliasMapT; // 1 device-alias <--> 0..N Device names
  DeviceUserAliasMapT mDeviceUserAliasMap;

  static const char * sDeviceBaseAliasMapNvp;
  static const char * sDeviceUserAliasMapNvp;

public:
  IndiDeviceManagerT() {
  }


  /**
   * TODO
   */
  void assignAlias(const string & inAlias, const string & inDeviceName);

  /**
   * If alias found, returns copy of device name assigned to alias (empty string if no device name assigned).
   */
  bool lookupAlias(const string & inAlias, string * outDeviceName = 0) const;

  //addUserAlias();
  //removeUserAlias();

  // getDevice(inAlias);

  // getCamera(inAlias);
  // getFocuser(inAlias);
  // getFilterWheel(inAlias);
  // getMount(inAlias);

  // getGuideCamera();
  // getImageCamera();
  // getFocuser();
  // getFilterWheel();
  // getMount();
  //list<string> getAliases() {
  //}

  /**
   * Object serialization using boost:
   * See: http://www.boost.org/doc/libs/1_43_0/libs/serialization/doc/serialization.html
   * See: http://stackoverflow.com/questions/3279037/serializing-a-map-of-objects-to-xml-using-boostserialization
   */
  void loadConfig(const char * inFileName) {
    std::ifstream ifs(inFileName);
    boost::archive::xml_iarchive ia(ifs);
    ia >> make_nvp(sDeviceBaseAliasMapNvp, mDeviceBaseAliasMap);
    ia >> make_nvp(sDeviceUserAliasMapNvp, mDeviceUserAliasMap);
  }

  void saveConfig(const char * inFileName) {
    std::ofstream ofs(inFileName);
    boost::archive::xml_oarchive oa(ofs);
    oa << make_nvp(sDeviceBaseAliasMapNvp, mDeviceBaseAliasMap);
    oa << make_nvp(sDeviceUserAliasMapNvp, mDeviceUserAliasMap);
  }

};


//   void addUserDeviceAlias(const string & inDeviceAlias, inAcceptDeviceType (asString?!?! or INDI enum? or own enum type?) ) throws DeviceAliasAlreadyExists;
//   void removeUserDeviceAlias(string inDeviceAliasName) throws DeviceAliasDoesNotExist

// };

// UC 2.)
// const DeviceAliasT * getDeviceAliasByName(string inDeviceAliasName) const, returns 0 if role does not exist
// bool hasDeviceAlias(string inDeviceAliasName) const
// bool hasDeviceAlias(const DeviceAliasT * inDeviceAlias) const
// list<DeviceAliasT*> getDeviceAliases(roleType = DEFAULT, USER, ALL = DEFAULT | USER)

// UC 3.)
// list<INDI::Device*> getDevices(inIndiServerName = "" /*default from all servers*/, enum inDeviceType = ALL ??) const;

// UC 4.)
// INDI::Device * getDevice(string inAliasName) const, returns 0 if no device is assigned, throws DeviceAliasDoesNotExistT if alias does not exist

// UC 5.)
// void coupleDeviceToAlias(INDI::Device * ???, const RoleT * inDeviceAlias)
// void decoupleDeviceAliasPair(INDI::Device * ???)
// void decoupleDeviceAliasPair(const RoleT * inDeviceRole)

// UC 6.)
// void loadConfiguration(const string & inFilename) 
// void saveConfiguration(const string & inFilename)

// void connect(string inRoleName) throws
// void disconnect(string inRoleName) throws ?



// bool isConnected(string inRoleName)




// map DeviceRoleT <-> DeviceTypeT --> then we do not need a role class?!
// map DeviceRoleT <-> Device 
//


#endif /* _INDI_DEVICE_MANAGER_HPP_ */
