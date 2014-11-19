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

#include <boost/test/unit_test.hpp>
#include "indi/indi_device_manager.hpp"

BOOST_AUTO_TEST_SUITE(IndiDeviceManager)

BOOST_AUTO_TEST_CASE(IndiDeviceManagerAssignAlias)
{
  IndiDeviceManagerT manager;

  // Alias assign check
  manager.assignAlias(BaseAliasT::asStr(BaseAliasT::IMAGE_CAMERA), "MY_DEVICE_NAME"); // No exception expected

  // Alias readout check
  string deviceName = "ABC";
  bool found = manager.lookupAlias(BaseAliasT::asStr(BaseAliasT::IMAGE_CAMERA), & deviceName);
  BOOST_CHECK_MESSAGE(found, "Expect alias to be found.");
  BOOST_CHECK_MESSAGE(! strcmp(deviceName.c_str(), "MY_DEVICE_NAME"), "Expect device name to match given value.");

  // Alias does not exist check...
  BOOST_CHECK_THROW(manager.assignAlias("ALIAS_NAME_THAT_DOES_NOT_EXIST", "MY_DEVICE_NAME"), IndiDeviceManagerAliasDoesNotExistExceptionT);
}

BOOST_AUTO_TEST_CASE(IndiDeviceManagerAddRemoveUserAlias)
{
  // TODO
}

BOOST_AUTO_TEST_CASE(IndiDeviceManagerLookupAlias)
{
  IndiDeviceManagerT manager;

  bool found = manager.lookupAlias("ALIAS_THAT_DOES_NOT_EXIST");
  BOOST_CHECK_MESSAGE(! found, "Expect alias not to be found.");

  string deviceName = "ABC";
  bool found2 = manager.lookupAlias("ALIAS_THAT_DOES_NOT_EXIST_TOO", & deviceName);
  BOOST_CHECK_MESSAGE(! found2, "Expect alias not to be found.");
  BOOST_CHECK_MESSAGE(! strcmp(deviceName.c_str(), "ABC"), "Expect supplied device name variable not to be modified.");

  string deviceName2 = "DEF";
  bool found3 = manager.lookupAlias(BaseAliasT::asStr(BaseAliasT::IMAGE_CAMERA), & deviceName2);
  BOOST_CHECK_MESSAGE(found3, "Expect alias to be found.");
  BOOST_CHECK_MESSAGE(strcmp(deviceName2.c_str(), "DEF"), "Expect deviceName to be changed.");
}

/**
 * TODO
 */
// IndiDeviceManagerAddRemoveDeviceAlias
BOOST_AUTO_TEST_CASE(IndiDeviceManagerSaveLoadConfig)
{
  IndiDeviceManagerT manager;

  manager.saveConfig("device_alias_map.cfg");

  //manager.assign("USER_ALIAS_1", "DEVICE_NAME_1");

  //BOOST_CHECK(false);
  // IndiServerConnectionT * newServerConn = manager.createConnection("Hostname1", 7624 /* port */, false /* AutoConnect */);
  // BOOST_CHECK(newServerConn);

  // // Destroy connection
  // BOOST_CHECK_THROW(manager.destroyConnection("NAME_THAT_DOES_NOT_EXIST"), IndiServerConnectionDoesNotExistExceptionT);
  // BOOST_REQUIRE(newServerConn);
  // string connName = newServerConn->getConnectionName();
  // manager.destroyConnection(connName);
  // BOOST_CHECK_MESSAGE(manager.getConnections().size() == 0, "No connections expected to be in the map.");
}

// TODO: Base names are not allowed as user alias names (duplicates).
// TODO: Duplicate user alias names are not allowed.
// TODO: Empty alias name is not allowed.

BOOST_AUTO_TEST_SUITE_END()
