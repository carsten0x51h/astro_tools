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

#include "indi_device_manager.hpp"

const char * IndiDeviceManagerT::sDeviceBaseAliasMapNvp = "device_base_alias_map";
const char * IndiDeviceManagerT::sDeviceUserAliasMapNvp = "device_user_alias_map";

void IndiDeviceManagerT::assignAlias(const string & inAlias, const string & inDeviceName) {
  // Lookup in base alias map (boost array)
  BaseAliasT::TypeE typeIdx = BaseAliasT::asType(inAlias.c_str());
    
  if (typeIdx != BaseAliasT::_Count) {
    mDeviceBaseAliasMap[typeIdx] = inDeviceName;
    return; // return assigned device name
  }

  // Lookup in user alias map (stl map)
  DeviceUserAliasMapT::iterator itUserAlias = mDeviceUserAliasMap.find(inAlias);

  if (itUserAlias != mDeviceUserAliasMap.end()) {
    itUserAlias->second = inDeviceName;
    return;
  }

  // Alias not found
  throw IndiDeviceManagerAliasDoesNotExistExceptionT();
}


bool IndiDeviceManagerT::lookupAlias(const string & inAlias, string * outDeviceName) const {
  // Lookup in base alias map (boost array)
  BaseAliasT::TypeE typeIdx = BaseAliasT::asType(inAlias.c_str());
    
  if (typeIdx != BaseAliasT::_Count) {
    if (outDeviceName)
      *outDeviceName = mDeviceBaseAliasMap.at(typeIdx); // array access by type index
    return true; // return assigned device name
  }

  // Lookup in user alias map (stl map)
  DeviceUserAliasMapT::const_iterator itUserAlias = mDeviceUserAliasMap.find(inAlias);

  if (itUserAlias != mDeviceUserAliasMap.end()) {
    if (outDeviceName)
      *outDeviceName = itUserAlias->second;
    return true; // return assigned device name
  }
  return false; // No alias found
}
