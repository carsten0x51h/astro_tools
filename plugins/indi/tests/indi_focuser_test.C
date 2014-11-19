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
#include "indi/indi_camera.hpp"
#include "indi/indi_focuser.hpp"

BOOST_AUTO_TEST_SUITE(IndiFocuser)

/**
 * Get and set focuser device port.
 */
BOOST_AUTO_TEST_CASE(IndiFocuserGetSetDevicePort)
{
  IndiClientT * newIndiClient = new IndiClientT("localhost", 7624 /* port */, true /* AutoConnect */);
  
  BOOST_CHECK(newIndiClient);
  BOOST_CHECK_MESSAGE(newIndiClient->isConnected(), "Expected client to be connected to server.");

  IndiFocuserT * indiFocuser = newIndiClient->getFocuser("Focuser Simulator");
  BOOST_CHECK_MESSAGE(indiFocuser, "Expect indiFocuser to be set.");  

  // Check connect
  indiFocuser->disconnect(2000);
  BOOST_CHECK_MESSAGE(! indiFocuser->isConnected(), "Expected focuser to be disconnected within 2 seconds.");
  indiFocuser->connect(2000);
  BOOST_CHECK_MESSAGE(indiFocuser->isConnected(), "Expected focuser to be connected within 2 seconds.");


  string devPort = indiFocuser->getDevicePort();
  cerr << "devPort: " << devPort << endl;

  string newDevPort = "/dev/ttyUSB3";
  indiFocuser->setDevicePort(newDevPort);

  string newDevPortRd = indiFocuser->getDevicePort();
  cerr << "newDevPortRd: " << newDevPortRd << endl;


  // Disconnect
  indiFocuser->disconnect(2000);
  BOOST_CHECK_MESSAGE(! indiFocuser->isConnected(), "Expected device to be disconnected within 2 seconds.");
  
  delete newIndiClient;
}


/**
 * Get and set focuser direction and position.
 */
BOOST_AUTO_TEST_CASE(IndiFocuserGetSetDirectionAndPosition)
{
  IndiClientT * newIndiClient = new IndiClientT("localhost", 7624 /* port */, true /* AutoConnect */);
  
  BOOST_CHECK(newIndiClient);
  BOOST_CHECK_MESSAGE(newIndiClient->isConnected(), "Expected client to be connected to server.");

  IndiFocuserT * indiFocuser = newIndiClient->getFocuser("Focuser Simulator");
  BOOST_CHECK_MESSAGE(indiFocuser, "Expect indiFocuser to be set.");  

  // Check connect
  indiFocuser->disconnect(2000);
  BOOST_CHECK_MESSAGE(! indiFocuser->isConnected(), "Expected focuser to be disconnected within 2 seconds.");
  indiFocuser->connect(2000);
  BOOST_CHECK_MESSAGE(indiFocuser->isConnected(), "Expected focuser to be connected within 2 seconds.");

  cerr << "Focuser props:" << endl;
  indiFocuser->print(cout); // TODO: How to crosscheck output?


  // Rel pos
  //cerr << "relPos: " << indiFocuser->getRelPos() << endl;
  // TODO: SetRelPos...


  // Direction
  FocusDirectionT::TypeE focusDir = indiFocuser->getFocusDirection();
  BOOST_CHECK_MESSAGE(focusDir == FocusDirectionT::INWARDS, "Expecting focusDir == FocusDirectionT::INWARDS.");

  indiFocuser->setFocusDirection(FocusDirectionT::invert(focusDir));
  FocusDirectionT::TypeE newFocusDir = indiFocuser->getFocusDirection();
  BOOST_CHECK_MESSAGE(newFocusDir == FocusDirectionT::OUTWARDS, "Expecting newFocusDir == FocusDirectionT::OUTWARDS.");


  // Abs pos
  int absPos = indiFocuser->getAbsPos();
  BOOST_CHECK_MESSAGE(absPos == 50000, "Expecting absPos == 50000.");

  int newAbsPos = absPos + 100;
  indiFocuser->setAbsPos(newAbsPos);
  int newAbsPosRd = indiFocuser->getAbsPos();

  BOOST_CHECK_MESSAGE(newAbsPosRd == newAbsPos, "Expecting newAbsPosRd == newAbsPos.");
  BOOST_CHECK_MESSAGE(! indiFocuser->isMovementInProgess(), "Expecting no movement in progress.");


  // Disconnect
  indiFocuser->disconnect(2000);
  BOOST_CHECK_MESSAGE(! indiFocuser->isConnected(), "Expected device to be disconnected within 2 seconds.");
  
  delete newIndiClient;
}

// TODO: Add another check: abort focuser motion... does not work for simulator.... need MoonLite... - only perform this check if moonlite is available (connected to the computer...) -> also need to add indi driver...

// TODO: Add another check - moveFocusBy....



/**
 * Get and set MoonLite focuser direction and position.
 */
BOOST_AUTO_TEST_CASE(IndiFocuserMoonLiteGetSetDirectionAndPosition)
{
  static const int allowedDelta = 8; // NOTE: For some reason final position is not always exactly reached by MoonLite focuser

  IndiClientT * newIndiClient = new IndiClientT("localhost", 7624 /* port */, true /* AutoConnect */);
  
  BOOST_CHECK(newIndiClient);
  BOOST_CHECK_MESSAGE(newIndiClient->isConnected(), "Expected client to be connected to server.");

  IndiFocuserT * indiFocuser = newIndiClient->getFocuser("MoonLite");

  if (! indiFocuser) {
    cerr << "MoonLite focuser not present - MoonLite focuser tests not executed." << endl;
    return;
  }

  // Check connect
  indiFocuser->disconnect(2000);
  BOOST_CHECK_MESSAGE(! indiFocuser->isConnected(), "Expected focuser to be disconnected within 2 seconds.");
  indiFocuser->connect(2000);
  BOOST_CHECK_MESSAGE(indiFocuser->isConnected(), "Expected focuser to be connected within 2 seconds.");

  cerr << "Focuser props:" << endl;
  indiFocuser->print(cout); // TODO: How to crosscheck output?


  // Rel pos
  //cerr << "relPos: " << indiFocuser->getRelPos() << endl;
  // TODO: SetRelPos...


  // Direction
  FocusDirectionT::TypeE focusDir = indiFocuser->getFocusDirection();
  BOOST_CHECK_MESSAGE(focusDir == FocusDirectionT::INWARDS, "Expecting focusDir == FocusDirectionT::INWARDS.");

  indiFocuser->setFocusDirection(FocusDirectionT::invert(focusDir));
  FocusDirectionT::TypeE newFocusDir = indiFocuser->getFocusDirection();
  BOOST_CHECK_MESSAGE(newFocusDir == FocusDirectionT::OUTWARDS, "Expecting newFocusDir == FocusDirectionT::OUTWARDS.");


  // Set new abs pos...
  int absPos = indiFocuser->getAbsPos();
  cerr << "absPos: " << absPos << endl;

  int newAbsPos = absPos + 300;
  cerr << "newAbsPos: " << newAbsPos << endl;
  indiFocuser->setAbsPos(newAbsPos);

  BOOST_CHECK_MESSAGE(! indiFocuser->isMovementInProgess(), "Expecting no movement in progress.");
    
  usleep(100000); // Wait a moment due to a strange indi delay...

  int newAbsPosRd = indiFocuser->getAbsPos();
  cerr << "newAbsPosRd: " << newAbsPosRd << endl;

  BOOST_CHECK_MESSAGE(abs((int)newAbsPosRd - (int)newAbsPos) < allowedDelta, "Expecting abs(newAbsPosRd - newAbsPos) < allowedDelta.");



  // Just moove focuser back...
  indiFocuser->setAbsPos(absPos);
  BOOST_CHECK_MESSAGE(! indiFocuser->isMovementInProgess(), "Expecting no movement in progress.");
  usleep(100000); // Wait a moment due to a strange indi delay...

  int newAbsPosRd2 = indiFocuser->getAbsPos();
  cerr << "newAbsPosRd2: " << newAbsPosRd2 << endl;

  BOOST_CHECK_MESSAGE(abs((int)newAbsPosRd2 - (int)absPos) < allowedDelta, "Expecting abs(newAbsPosRd2 - absPos) < allowedDelta.");


  // Disconnect
  indiFocuser->disconnect(2000);
  BOOST_CHECK_MESSAGE(! indiFocuser->isConnected(), "Expected device to be disconnected within 2 seconds.");
  
  delete newIndiClient;
}


/**
 * Get MoonLite temperature.
 */
BOOST_AUTO_TEST_CASE(IndiFocuserMoonLiteGetTemperature)
{
  IndiClientT * newIndiClient = new IndiClientT("localhost", 7624 /* port */, true /* AutoConnect */);
  
  BOOST_CHECK(newIndiClient);
  BOOST_CHECK_MESSAGE(newIndiClient->isConnected(), "Expected client to be connected to server.");

  IndiFocuserT * indiFocuser = newIndiClient->getFocuser("MoonLite");

  if (! indiFocuser) {
    cerr << "MoonLite focuser not present - MoonLite focuser tests not executed." << endl;
    return;
  }

  // Check connect
  indiFocuser->disconnect(2000);
  BOOST_CHECK_MESSAGE(! indiFocuser->isConnected(), "Expected focuser to be disconnected within 2 seconds.");
  indiFocuser->connect(2000);
  BOOST_CHECK_MESSAGE(indiFocuser->isConnected(), "Expected focuser to be connected within 2 seconds.");

  // Get focuser temperature...
  cerr << "Focuser temperature: " << indiFocuser->getTemperature() << "°C" << endl;

  // Disconnect
  indiFocuser->disconnect(2000);
  BOOST_CHECK_MESSAGE(! indiFocuser->isConnected(), "Expected device to be disconnected within 2 seconds.");
  
  delete newIndiClient;
}


/**
 * Get MoonLite abortMotion.
 */
BOOST_AUTO_TEST_CASE(IndiFocuserMoonLiteAbortMotion)
{
  IndiClientT * newIndiClient = new IndiClientT("localhost", 7624 /* port */, true /* AutoConnect */);
  
  BOOST_CHECK(newIndiClient);
  BOOST_CHECK_MESSAGE(newIndiClient->isConnected(), "Expected client to be connected to server.");

  IndiFocuserT * indiFocuser = newIndiClient->getFocuser("MoonLite");

  if (! indiFocuser) {
    cerr << "MoonLite focuser not present - MoonLite focuser tests not executed." << endl;
    return;
  }

  // Check connect
  indiFocuser->disconnect(2000);
  BOOST_CHECK_MESSAGE(! indiFocuser->isConnected(), "Expected focuser to be disconnected within 2 seconds.");
  indiFocuser->connect(2000);
  BOOST_CHECK_MESSAGE(indiFocuser->isConnected(), "Expected focuser to be connected within 2 seconds.");

  // Get focuser temperature...
  int absPos = indiFocuser->getAbsPos();
  cerr << "absPos: " << absPos << endl;

  int newAbsPos = absPos - 300;
  cerr << "newAbsPos: " << newAbsPos << endl;
  indiFocuser->setAbsPos(newAbsPos, 0 /* do not wait at all */);

  BOOST_CHECK_MESSAGE(indiFocuser->isMovementInProgess(), "Expected focuser to be in motion.");

  cerr << "Aborting move..." << flush;
  indiFocuser->abortMotion();
  cerr << "DONE!" << endl;

  BOOST_CHECK_MESSAGE(! indiFocuser->isMovementInProgess(), "Expected focuser to be not in motion.");

  usleep(100000);

  cerr << "New focus pos: " << indiFocuser->getAbsPos() << endl;

  // Disconnect
  indiFocuser->disconnect(2000);
  BOOST_CHECK_MESSAGE(! indiFocuser->isConnected(), "Expected device to be disconnected within 2 seconds.");
  
  delete newIndiClient;
}


/**
 * Get MoonLite relative position test.
 */
BOOST_AUTO_TEST_CASE(IndiFocuserMoonLiteRelativePosTest)
{
  const unsigned int allowedDelta = 10;
  IndiClientT * newIndiClient = new IndiClientT("localhost", 7624 /* port */, true /* AutoConnect */);
  
  BOOST_CHECK(newIndiClient);
  BOOST_CHECK_MESSAGE(newIndiClient->isConnected(), "Expected client to be connected to server.");

  IndiFocuserT * indiFocuser = newIndiClient->getFocuser("MoonLite");

  if (! indiFocuser) {
    cerr << "MoonLite focuser not present - MoonLite focuser tests not executed." << endl;
    return;
  }

  // Check connect
  indiFocuser->disconnect(2000);
  BOOST_CHECK_MESSAGE(! indiFocuser->isConnected(), "Expected focuser to be disconnected within 2 seconds.");
  indiFocuser->connect(2000);
  BOOST_CHECK_MESSAGE(indiFocuser->isConnected(), "Expected focuser to be connected within 2 seconds.");

  // Get abs pos...
  int absPos = indiFocuser->getAbsPos();

  // Get current rel. pos and set new...
  const unsigned int delta = 100;
  indiFocuser->setRelPos(delta);

  // New abs pos...
  int  newAbsPos = indiFocuser->getAbsPos();
  BOOST_CHECK_MESSAGE(abs((int)newAbsPos - ((int)absPos + (int)delta)) < allowedDelta , "Expecting abs(newAbsPos - (absPos + delta)) < allowedDelta.");

  BOOST_CHECK_THROW(indiFocuser->setRelPos(-100), PropInvalidValueExceptionT);

  // Disconnect
  indiFocuser->disconnect(2000);
  BOOST_CHECK_MESSAGE(! indiFocuser->isConnected(), "Expected device to be disconnected within 2 seconds.");
  
  delete newIndiClient;
}


/**
 * Get MoonLite moveFocusBy test.
 */
BOOST_AUTO_TEST_CASE(IndiFocuserMoonLiteMoveFocusByTest)
{
  const unsigned int allowedDelta = 20;
  IndiClientT * newIndiClient = new IndiClientT("localhost", 7624 /* port */, true /* AutoConnect */);
  
  BOOST_CHECK(newIndiClient);
  BOOST_CHECK_MESSAGE(newIndiClient->isConnected(), "Expected client to be connected to server.");

  IndiFocuserT * indiFocuser = newIndiClient->getFocuser("MoonLite");

  if (! indiFocuser) {
    cerr << "MoonLite focuser not present - MoonLite focuser tests not executed." << endl;
    return;
  }

  // Check connect
  indiFocuser->disconnect(2000);
  BOOST_CHECK_MESSAGE(! indiFocuser->isConnected(), "Expected focuser to be disconnected within 2 seconds.");
  indiFocuser->connect(2000);
  BOOST_CHECK_MESSAGE(indiFocuser->isConnected(), "Expected focuser to be connected within 2 seconds.");

  const unsigned int delta = 300;

  // Inwards test
  int absPos = indiFocuser->getAbsPos();
  indiFocuser->moveFocusBy(delta, FocusDirectionT::INWARDS);
  usleep(1000000);

  int newAbsPos = indiFocuser->getAbsPos();
  cerr << "Inwards result - absPos: " << absPos << ", newAbsPos: " << newAbsPos << endl;
  cerr << "abs(newAbsPos - (absPos - delta)): " << abs((int)newAbsPos - ((int)absPos - (int)delta)) << endl;
  BOOST_CHECK_MESSAGE(abs((int)newAbsPos - ((int)absPos - (int)delta)) < allowedDelta, "Expexting abs(newAbsPos - (absPos - delta)) < allowedDelta.");

  // Outwards test
  int absPos2 = indiFocuser->getAbsPos();
  indiFocuser->moveFocusBy(delta, FocusDirectionT::OUTWARDS);
  usleep(1000000);

  int newAbsPos2 = indiFocuser->getAbsPos();
  cerr << "Outwards result - absPos2: " << absPos2 << ", newAbsPos2: " << newAbsPos2 << endl;
  cerr << "abs(newAbsPos2 - (absPos2 + delta)): " << abs((int)newAbsPos2 - ((int)absPos2 + (int)delta)) << endl;
  BOOST_CHECK_MESSAGE(abs((int)newAbsPos2 - ((int)absPos2 + (int)delta)) < allowedDelta, "Expexting abs(newAbsPos2 - (absPos2 + delta)) < allowedDelta.");


  // Disconnect
  indiFocuser->disconnect(2000);
  BOOST_CHECK_MESSAGE(! indiFocuser->isConnected(), "Expected device to be disconnected within 2 seconds.");
  
  delete newIndiClient;
}


BOOST_AUTO_TEST_SUITE_END()
