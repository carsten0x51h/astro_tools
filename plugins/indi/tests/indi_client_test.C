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
#include "indi/indi_client.hpp"
#include "indi/indi_device.hpp"

BOOST_AUTO_TEST_SUITE(IndiClient)

/**
 * Connecting/Disconnectiong IndiClient
 */
BOOST_AUTO_TEST_CASE(IndiClientConnectDisconnect)
{
  IndiClientT * newIndiClient = new IndiClientT("HOSTNAME_THAT_DOES_NOT_EXIST", 7624 /* port */, false /* AutoConnect */);
  BOOST_CHECK(newIndiClient);
  BOOST_CHECK_THROW(newIndiClient->connect(), IndiClientConnectFailedExceptionT);
  
  newIndiClient->setServer("localhost", 7624);

  // Successful connect
  bool success = true;
  try {
    newIndiClient->connect();
  } catch(IndiClientConnectFailedExceptionT & exc) {
    success = false;
    cerr << "Client connection failed! Reason: " << exc.what() << endl; 
  } catch(...) {
    success = false;
    cerr << "Client connection failed! Reason unknown!" << endl; 
  }
  
  BOOST_CHECK_MESSAGE(success, "Connecting to localhost failed.");
  BOOST_CHECK_MESSAGE(newIndiClient->isConnected(), "Expected client to be connected to server.");

  // Successful disconnect
  newIndiClient->disconnect();
  BOOST_CHECK_MESSAGE(! newIndiClient->isConnected(), "Expected client to be not connected to server.");
  delete newIndiClient;
}

/**
 * Automatically connecting/disconnecting IndiClient
 */
BOOST_AUTO_TEST_CASE(IndiClientAutoConnect)
{
  IndiClientT * newIndiClient = new IndiClientT("localhost", 7624 /* port */, true /* AutoConnect */);

  BOOST_CHECK(newIndiClient);
  BOOST_CHECK_MESSAGE(newIndiClient->isConnected(), "Expected client to be connected to server.");
  BOOST_CHECK_MESSAGE(newIndiClient->getAutoConnected(), "Expected client to be auto-connected to server.");
  
  // Disconnect
  newIndiClient->disconnect();
  BOOST_CHECK_MESSAGE(! newIndiClient->isConnected(), "Expected client to be not connected to server.");
  
  // Manually reconnect
  newIndiClient->connect();
  BOOST_CHECK_MESSAGE(newIndiClient->isConnected(), "Expected client to be connected to server.");
  BOOST_CHECK_MESSAGE(! newIndiClient->getAutoConnected(), "Expected client to be not auto-connected to server.");

  delete newIndiClient;
}


/**
 * Test multiple connect / discconects to the server with the same INDI client.
 */
BOOST_AUTO_TEST_CASE(IndiClientMultipleConnectDisconnect)
{
  IndiClientT * newIndiClient = new IndiClientT("localhost", 7624 /* port */, false /* AutoConnect */);

  for(size_t i=0; i < 5; ++i) {
    // Connect
    bool success = true;
    try {
      newIndiClient->connect();
    } catch(IndiClientConnectFailedExceptionT & exc) {
      success = false;
      cerr << "Client connection failed! Reason: " << exc.what() << endl; 
    }
    BOOST_CHECK_MESSAGE(success, "Expecting client connection to be successful.");
    BOOST_CHECK_MESSAGE(newIndiClient->isConnected(), "Expecting client to be connected.");
    
    usleep(1000000);

    // Disconnect
    newIndiClient->disconnect();
    
    BOOST_CHECK_MESSAGE(! newIndiClient->isConnected(), "Expecting client to be disconnected.");
  } // end for

  delete newIndiClient;
}

/**
 * Get devices from INDI client.
 */
BOOST_AUTO_TEST_CASE(IndiClientGetDevices)
{
  IndiClientT * newIndiClient = new IndiClientT("localhost", 7624 /* port */, true /* AutoConnect */);
  BOOST_CHECK(newIndiClient);
  BOOST_CHECK_MESSAGE(newIndiClient->isConnected(), "Expected client to be connected to server.");

  const IndiDeviceMapT devices = newIndiClient->getDevices();

  BOOST_CHECK_MESSAGE(devices.size() == 4, "Expected number of devices equally 2.");
  BOOST_CHECK_MESSAGE(newIndiClient->getDevice("CCD Simulator") != 0, "Expected device to be valid.");

  cerr << "Devices:" << endl;
  for (IndiDeviceMapT::const_iterator it = devices.begin(); it != devices.end(); ++it) {
    cerr << "   > Device-Name: " << it->first->getDeviceName() << ", Device-Type: " << DeviceTypeT::asStr(it->second->getType()) << endl;
  }

  delete newIndiClient;
}


BOOST_AUTO_TEST_SUITE_END()
