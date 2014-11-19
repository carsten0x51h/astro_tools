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

#ifndef _INDI_CLIENT_MANAGER_HPP_
#define _INDI_CLIENT_MANAGER_HPP_ _INDI_CLIENT_MANAGER_HPP_

#include <string>
#include <iosfwd>

#include "at_exception.hpp"
#include "indi_client.hpp"

using namespace std;

/************************************************************
 *  IndiClientManager
 *  
 * Purpose
 * -Manage the connection to 1..n INDI servers
 * -Load / save configuration with server entries
 * -Connect / disconnect to INDI server
 * -Manage server exit / failure
 * -There is one INDI Client object per server connection
 * -Technically (INDI bug?) the creation of a new INDI Client is required after each disconnect
 * 
 * Use Cases
 * 1.) Add / remove INDI server
 * 2.) Get list of INDI servers
 * 3.) Get certain INDI server connection
 * 4.) Connect / disconnect to INDI servers
 * 5.) Load / save server configuration
 *
 ************************************************************/
DEF_Exception(IndiClientManagerLoadConfigFailed);
DEF_Exception(IndiClientManagerSaveConfigFailed);

class IndiClientManagerT {
private:
  IndiClientsT mIndiClients;
  static const char * sConnectionsNvp;

  static void createMapKey(const string & inHostName, int inPort, string * outKey) {
    stringstream ss;
    ss << inHostName << ":" << inPort;
    AT_ASSERT(IndiClient, outKey, "Destination string pointer expected to be != 0.");
    *outKey = ss.str(); // Copy string
  }


public:
  ~IndiClientManagerT() {
    for(IndiClientsT::iterator itConn = mIndiClients.begin(); itConn != mIndiClients.end(); ++itConn) {
      IndiClientT * conn = itConn->second;
      AT_ASSERT(IndiClient, conn, "Expect conn to be != 0.");
      conn->disconnect();
      delete conn;
      mIndiClients.erase(itConn);
    }
  }

  IndiClientT * createClient(const string & inHostName, int inPort, bool inAutoConnect) {
    string connName = IndiClientT::getConnectionName(inHostName, inPort);

    IndiClientsT::const_iterator itConn = mIndiClients.find(connName);
    if (itConn != mIndiClients.end())
      throw IndiClientAlreadyExistsExceptionT();

    IndiClientT * newConnection = new IndiClientT(inHostName, inPort, inAutoConnect);
    pair<IndiClientsT::iterator, bool> res = mIndiClients.insert(make_pair(connName, newConnection));

    AT_ASSERT(IndiClient, res.second, "Expect element to be inserted.");

    return newConnection;
  }

  void destroyClient(const string & inConnName) {
    IndiClientsT::iterator itConn = mIndiClients.find(inConnName);
    if (itConn == mIndiClients.end())
      throw IndiClientDoesNotExistExceptionT();

    IndiClientT * conn = itConn->second;
    AT_ASSERT(IndiClient, conn, "Expect conn to be != 0.");

    conn->disconnect();
    delete conn;
    mIndiClients.erase(itConn);
  }
  //void destroyClient(IndiClientT * inIndiClient); // optional... required?


  /**
   * Use default STL map interface to add / remove / iterate connections.
   */
  inline IndiClientsT & getClients() {
    return mIndiClients;
  }

  /**
   * Returns 0 if connection does not exist.
   */
  inline IndiClientT * getClient(const string & inConnName) {
    IndiClientsT::iterator serverIt = mIndiClients.find(inConnName);
    return (serverIt != mIndiClients.end() ? serverIt->second : 0);
  }

  /**
   * Returns 0 if connection does not exist.
   */
  inline const IndiClientT * getClient(const string & inConnName) const {
    IndiClientsT::const_iterator serverIt = mIndiClients.find(inConnName);
    return (serverIt != mIndiClients.end() ? serverIt->second : 0);
  }

  /**
   * Returns true, if all connects were successful.
   * Returns false if at least one connect failed.
   * If there are no connections, method returns true.
   */
  bool connectAll() {
    bool allConnSuccessful = true;

    for (IndiClientsT::iterator itConn = mIndiClients.begin(); itConn != mIndiClients.end(); ++itConn) {
      IndiClientT * conn = itConn->second;
      AT_ASSERT(IndiClient, conn, "Expect conn to be != 0.");

      if (conn->isConnected())
	continue;

      try {
	conn->connect();
      } catch(IndiClientConnectFailedExceptionT & exc) {
	// Ok, could just not be connected
	allConnSuccessful = false;
      }
    } // end for

    return allConnSuccessful;
  }
  
  /**
   * Returns true, if all disconnects were successful.
   * Returns false if at least one disconnect failed.
   * If there are no connections, method returns true.
   */
  bool disconnectAll() {
    bool allDisconnSuccessful = true;

    for (IndiClientsT::iterator itConn = mIndiClients.begin(); itConn != mIndiClients.end(); ++itConn) {
      IndiClientT * conn = itConn->second;
      AT_ASSERT(IndiClient, conn, "Expect conn to be != 0.");

      if (! conn->isConnected())
	continue;

      try {
	conn->disconnect();
      } catch(IndiClientConnectFailedExceptionT & exc) {
	// Ok, could just not be disconnected
	allDisconnSuccessful = false;
      }
    } // end for

    return allDisconnSuccessful;
  }

  void loadConfig(const char * inFileName) {
    std::ifstream ifs(inFileName);
    if (! ifs.good()) {
      throw IndiClientManagerLoadConfigFailedExceptionT();
    }
    boost::archive::xml_iarchive ia(ifs);
    ia >> make_nvp(sConnectionsNvp, mIndiClients);
  }

  void saveConfig(const char * inFileName) {
    std::ofstream ofs(inFileName);
    if (! ofs.good()) {
      throw IndiClientManagerSaveConfigFailedExceptionT();
    }
    boost::archive::xml_oarchive oa(ofs);
    oa << make_nvp(sConnectionsNvp, mIndiClients);
  }

};

#endif /* _INDI_CLIENT_MANAGER_HPP_ */
