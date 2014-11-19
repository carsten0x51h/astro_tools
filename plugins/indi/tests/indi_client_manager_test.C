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
#include "indi/indi_client_manager.hpp"

BOOST_AUTO_TEST_SUITE(IndiClientManager)

/**
 * Create and destroy INDI server
 */
BOOST_AUTO_TEST_CASE(IndiClientAddRemove)
{
  IndiClientManagerT manager;
  IndiClientT * newIndiClient = manager.createClient("Hostname1", 7624 /* port */, false /* AutoConnect */);
  BOOST_CHECK(newIndiClient);

  // Destroy connection
  BOOST_CHECK_THROW(manager.destroyClient("NAME_THAT_DOES_NOT_EXIST"), IndiClientDoesNotExistExceptionT);
  BOOST_REQUIRE(newIndiClient);
  cerr << "Calling newIndiClient->getConnectionName..." << endl;

  string connName = newIndiClient->getConnectionName();
  manager.destroyClient(connName);
  BOOST_CHECK_MESSAGE(manager.getClients().size() == 0, "No connections expected to be in the map.");
}

/**
 * Add same hostname and port twice.
 */
BOOST_AUTO_TEST_CASE(IndiClientAddDuplicate)
{
  IndiClientManagerT manager;
  manager.createClient("HOSTNAME_THAT_DOES_NOT_EXIST", 7624 /* port */, false /* AutoConnect */);
  BOOST_CHECK_THROW(manager.createClient("HOSTNAME_THAT_DOES_NOT_EXIST", 7624 /* port */, false /* AutoConnect */), IndiClientAlreadyExistsExceptionT);
}

/**
 * Get connection object pointer.
 */
BOOST_AUTO_TEST_CASE(IndiClientGetConnection)
{
  static const string myHostName = "my_hostname";
  
  IndiClientManagerT manager;
  IndiClientT * newIndiClient = manager.createClient(myHostName, 7624 /* port */, false /* AutoConnect */);
  BOOST_CHECK(newIndiClient);
  BOOST_CHECK(manager.getClient(newIndiClient->getConnectionName()) == newIndiClient);
}

/**
 * Load and save server configuration.
 */
BOOST_AUTO_TEST_CASE(IndiClientSaveLoadConfig)
{
  {
    IndiClientManagerT manager;
    IndiClientT * newIndiClient = manager.createClient("HOSTNAME_THAT_DOES_NOT_EXIST", 7624 /* port */, false /* AutoConnect */);
    BOOST_CHECK(newIndiClient);
    IndiClientT * newIndiClient2 = manager.createClient("ANOTHER_HOSTNAME_THAT_DOES_NOT_EXIST", 6666 /* port */, false /* AutoConnect */);
    BOOST_CHECK(newIndiClient2);
    
    // Save configuration
    manager.saveConfig("indi_server_manager_test.cfg");
  }

  {
    IndiClientManagerT manager;

    // Load configuration
    manager.loadConfig("indi_server_manager_test.cfg");
    BOOST_CHECK(manager.getClients().size() == 2);
    static const string connName = IndiClientT::getConnectionName("HOSTNAME_THAT_DOES_NOT_EXIST", 7624);
    IndiClientT * newIndiClient = manager.getClient(connName);
    BOOST_CHECK(newIndiClient);
    BOOST_CHECK(! strcmp(newIndiClient->getHost(), "HOSTNAME_THAT_DOES_NOT_EXIST"));
    BOOST_CHECK(newIndiClient->getPort() == 7624);
  }

  {
    IndiClientManagerT manager;
    BOOST_CHECK_THROW(manager.loadConfig("FILE_THAT_DOES_NOT_EXIST"), IndiClientManagerLoadConfigFailedExceptionT);
    BOOST_CHECK_THROW(manager.saveConfig("/TO/A/PATH/THAT/DOES/NOT/EXIST/FILE_THAT_DOES_NOT_EXIST"), IndiClientManagerSaveConfigFailedExceptionT);
  }
}

/**
 * Test connect/disconnect all server connections.
 */
BOOST_AUTO_TEST_CASE(IndiClientConnectDisconnectAll)
{
  {
    // Check if connect all fails
    IndiClientManagerT manager;
    manager.createClient("HOSTNAME_THAT_DOES_NOT_EXIST", 6666 /* port */, false /* AutoConnect */);
    manager.createClient("localhost", 7624 /* port */, false /* AutoConnect */);
    BOOST_CHECK_MESSAGE(! manager.connectAll(), "Expecting connectAll to fail.");
  }

  {
    // Check if connect all is succesful
    IndiClientManagerT manager;
    IndiClientT * newIndiClient = manager.createClient("localhost", 7624 /* port */, false /* AutoConnect */);
    BOOST_CHECK_MESSAGE(manager.connectAll(), "Expecting connectAll to succeed.");
    BOOST_CHECK_MESSAGE(newIndiClient->isConnected(), "Expecting client to be connected.");

    // Check if disconnect all succeeds
    manager.disconnectAll();
    BOOST_CHECK_MESSAGE(! newIndiClient->isConnected(), "Expecting client to be disconnected.");
  }
}

BOOST_AUTO_TEST_SUITE_END()
