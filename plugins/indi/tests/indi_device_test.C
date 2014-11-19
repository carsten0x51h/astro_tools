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

using namespace INDI;

BOOST_AUTO_TEST_SUITE(IndiDevice)

/**
 * Connect / disconnect INDI device.
 */
BOOST_AUTO_TEST_CASE(IndiDeviceConnectDisconnect)
{
  IndiClientT * newIndiClient = new IndiClientT("localhost", 7624 /* port */, true /* AutoConnect */);

  BOOST_CHECK(newIndiClient);
  BOOST_CHECK_MESSAGE(newIndiClient->isConnected(), "Expected client to be connected to server.");

  IndiDeviceT * indiDevice = newIndiClient->getDevice("CCD Simulator");
  BOOST_CHECK_MESSAGE(indiDevice != 0, "Expected device to be valid.");

  // Check timeout exception
  indiDevice->disconnect(2000);
  BOOST_CHECK_MESSAGE(! indiDevice->isConnected(), "Expected device to be disconnected within 2 seconds.");

  BOOST_CHECK_THROW(indiDevice->connect(1), TimeoutExceptionT);
  indiDevice->disconnect();
  BOOST_CHECK_MESSAGE(! indiDevice->isConnected(), "Expected device to be disconnected.");

  // Check connect
  indiDevice->connect(2000);
  BOOST_CHECK_MESSAGE(indiDevice->isConnected(), "Expected device to be connected within 2 seconds.");
  
  // Check disconnect
  indiDevice->disconnect(2000);
  BOOST_CHECK_MESSAGE(! indiDevice->isConnected(), "Expected device to be disconnected within 2 seconds.");

  delete newIndiClient;
}


/**
 * Connect / disconnect INDI device multiple times.
 */
BOOST_AUTO_TEST_CASE(IndiDeviceMultipleConnectDisconnect)
{
  IndiClientT * newIndiClient = new IndiClientT("localhost", 7624 /* port */, true /* AutoConnect */);

  BOOST_CHECK(newIndiClient);
  BOOST_CHECK_MESSAGE(newIndiClient->isConnected(), "Expected client to be connected to server.");

  IndiDeviceT * indiDevice = newIndiClient->getDevice("CCD Simulator");
  BOOST_CHECK_MESSAGE(indiDevice != 0, "Expected device to be valid.");

  for (size_t i=0; i < 5; ++i) {
  // Check connect
    indiDevice->connect(2000);
    BOOST_CHECK_MESSAGE(indiDevice->isConnected(), "Expected device to be connected within 2 seconds.");
    
    // Check disconnect
    indiDevice->disconnect(2000);
    BOOST_CHECK_MESSAGE(! indiDevice->isConnected(), "Expected device to be disconnected within 2 seconds.");
  }

  delete newIndiClient;
}


/**
 * Print available properties
 */
BOOST_AUTO_TEST_CASE(IndiDeviceQueryAvailableDeviceProperties)
{
  IndiClientT * newIndiClient = new IndiClientT("localhost", 7624 /* port */, true /* AutoConnect */);

  BOOST_CHECK(newIndiClient);
  BOOST_CHECK_MESSAGE(newIndiClient->isConnected(), "Expected client to be connected to server.");

  IndiDeviceT * indiDevice = newIndiClient->getDevice("CCD Simulator");
  BOOST_CHECK_MESSAGE(indiDevice != 0, "Expected device to be valid.");

  // Check connect
  indiDevice->disconnect(2000);
  BOOST_CHECK_MESSAGE(! indiDevice->isConnected(), "Expected device to be disconnected within 2 seconds.");
  indiDevice->connect(2000);
  BOOST_CHECK_MESSAGE(indiDevice->isConnected(), "Expected device to be connected within 2 seconds.");

  // Check print function
  for (size_t i=0; i < 10; ++i) {
    indiDevice->print(cout); // TODO: How to crosscheck output?
  }

  // Check overloaded ouptut operator
  for (size_t i=0; i < 10; ++i) {
    cout << *indiDevice << endl; // TODO: How to crosscheck output?
  }
  delete newIndiClient;
}


/**
 * Get different value types from the INDI device.
 */
BOOST_AUTO_TEST_CASE(IndiDeviceGetValues)
{
  IndiClientT * newIndiClient = new IndiClientT("localhost", 7624 /* port */, true /* AutoConnect */);

  BOOST_CHECK(newIndiClient);
  BOOST_CHECK_MESSAGE(newIndiClient->isConnected(), "Expected client to be connected to server.");

  IndiDeviceT * indiDevice = newIndiClient->getDevice("CCD Simulator");
  BOOST_CHECK_MESSAGE(indiDevice != 0, "Expected device to be valid.");

  // Check connect
  indiDevice->disconnect(2000);
  BOOST_CHECK_MESSAGE(! indiDevice->isConnected(), "Expected device to be disconnected within 2 seconds.");
  indiDevice->connect(2000);
  BOOST_CHECK_MESSAGE(indiDevice->isConnected(), "Expected device to be connected within 2 seconds.");

  // Read number prop
  double nRes = indiDevice->getNumberVal("CCD_BINNING", "VER_BIN");
  BOOST_CHECK_MESSAGE(nRes == 1, "Expected CCD_BINNING->VER_BIN to be 1.");

  bool bRes = indiDevice->getSwitchVal("GUIDER_RAPID_GUIDE", "ENABLE");
  BOOST_CHECK_MESSAGE(! bRes, "Expected GUIDER_RAPID_GUIDE->ENABLE to be false.");

  const char * cRes = indiDevice->getTextVal("ACTIVE_DEVICES", "ACTIVE_TELESCOPE");
  BOOST_CHECK_MESSAGE(! strcmp(cRes, "Telescope Simulator"), "Expected ACTIVE_DEVICES->ACTIVE_TELESCOPE to be 'Telescope Simulator'.");

  // No Light prop, yet...
  // IPState res = indiDevice->getLightVal("GUIDER_FRAME_TYPE", "FRAME_LIGHT");
  // BOOST_CHECK_MESSAGE(res, "Expected GUIDER_FRAME_TYPE->FRAME_LIGHT to be true.");

  void * vRes = indiDevice->getBLOBVal("CCD1", "CCD1");
  BOOST_CHECK_MESSAGE(! vRes, "Expected CCD1->CCD1 to be 0."); // vRes is 0 if no image has been taken. 

  // Read non existing props
  BOOST_CHECK_THROW(indiDevice->getNumberVal("DOES_NOT_EXIST", "VER_BIN"), PropNotFoundExceptionT);
  BOOST_CHECK_THROW(indiDevice->getNumberVal("CCD_BINNING", "DOES_NOT_EXIST"), PropNotFoundExceptionT);

  BOOST_CHECK_THROW(indiDevice->getSwitchVal("DOES_NOT_EXIST", "ENABLE"), PropNotFoundExceptionT);
  BOOST_CHECK_THROW(indiDevice->getSwitchVal("GUIDER_RAPID_GUIDE", "DOES_NOT_EXIST"), PropNotFoundExceptionT);

  BOOST_CHECK_THROW(indiDevice->getTextVal("DOES_NOT_EXIST", "ACTIVE_TELESCOPE"), PropNotFoundExceptionT);
  BOOST_CHECK_THROW(indiDevice->getTextVal("ACTIVE_DEVICES", "DOES_NOT_EXIST"), PropNotFoundExceptionT);

  BOOST_CHECK_THROW(indiDevice->getLightVal("DOES_NOT_EXIST", "ANY_LIGHT"), PropNotFoundExceptionT);
  BOOST_CHECK_THROW(indiDevice->getLightVal("ANY_LIGHT", "DOES_NOT_EXIST"), PropNotFoundExceptionT);

  BOOST_CHECK_THROW(indiDevice->getBLOBVal("DOES_NOT_EXIST", "CCD1"), PropNotFoundExceptionT);
  BOOST_CHECK_THROW(indiDevice->getBLOBVal("CCD1", "DOES_NOT_EXIST"), PropNotFoundExceptionT);

  // Read WO number prop - No such "write-only" property, yet
  //BOOST_CHECK_THROW(indiDevice->getNumberChecked("CCD_BINNING", "VER_BIN"), IndiDevicePropPermissionExceptionT);

  indiDevice->disconnect(2000);
  BOOST_CHECK_MESSAGE(! indiDevice->isConnected(), "Expected device to be disconnected within 2 seconds.");

  delete newIndiClient;
}


/**
 * Set different value types to the INDI device.
 */
BOOST_AUTO_TEST_CASE(IndiDeviceSetValues)
{
  IndiClientT * newIndiClient = new IndiClientT("localhost", 7624 /* port */, true /* AutoConnect */);

  BOOST_CHECK(newIndiClient);
  BOOST_CHECK_MESSAGE(newIndiClient->isConnected(), "Expected client to be connected to server.");

  IndiDeviceT * indiDevice = newIndiClient->getDevice("CCD Simulator");
  BOOST_CHECK_MESSAGE(indiDevice != 0, "Expected device to be valid.");

  // Check connect
  indiDevice->disconnect(2000);
  BOOST_CHECK_MESSAGE(! indiDevice->isConnected(), "Expected device to be disconnected within 2 seconds.");
  indiDevice->connect(2000);
  BOOST_CHECK_MESSAGE(indiDevice->isConnected(), "Expected device to be connected within 2 seconds.");

  // Read number value, write number and re-read number value
  double nRes = indiDevice->getNumberVal("CCD_BINNING", "VER_BIN");
  BOOST_CHECK_MESSAGE(nRes == 1, "Expected value to be set to 1.");
  indiDevice->sendNumberVal("CCD_BINNING", "VER_BIN", 2);
  double nRes2 = indiDevice->getNumberVal("CCD_BINNING", "VER_BIN");
  BOOST_CHECK_MESSAGE(nRes2 == 2, "Expected value to be set to 2.");
  

  indiDevice->sendSwitchVal("GUIDER_RAPID_GUIDE", "ENABLE", false);
  bool bRes = indiDevice->getSwitchVal("GUIDER_RAPID_GUIDE", "ENABLE");
  BOOST_CHECK_MESSAGE(! bRes, "Expected value to be to false.");
  indiDevice->sendSwitchVal("GUIDER_RAPID_GUIDE", "ENABLE", true);
  bool bRes2 = indiDevice->getSwitchVal("GUIDER_RAPID_GUIDE", "ENABLE");
  BOOST_CHECK_MESSAGE(bRes2, "Expected value to be set to true.");


  const char * cRes = indiDevice->getTextVal("FILTER_NAME", "FILTER_SLOT_NAME_1");
  BOOST_CHECK_MESSAGE(! strcmp(cRes, "Red"), "Expected value to be set to 'Red'.");

  /**
   * Note: Do not wait since for some reason state remains in IPS_BUSY! Could
   * be a problem of CCD Simulator or more generic with Text property...
   * TODO: This is a problem in the CCD Simulator.... we may use a different device to test this...
   */
  const char * str = "MyColor";
  indiDevice->sendTextVal("FILTER_NAME", "FILTER_SLOT_NAME_1", str, 0 /* 0 sec */);
  usleep(1000000); 
  const char * cRes2 = indiDevice->getTextVal("FILTER_NAME", "FILTER_SLOT_NAME_1");
  BOOST_CHECK_MESSAGE(! strcmp(cRes2, str), "Expected value to be set to content for str.");


  // TODO: void * vRes = indiDevice->getBLOBVal("CCD1", "CCD1");


  // Write to RO property
  BOOST_CHECK_THROW(indiDevice->sendNumberVal("CCD_INFO", "CCD_MAX_X", 2.0), PropPermissionExceptionT);
  
  indiDevice->disconnect(2000);
  BOOST_CHECK_MESSAGE(! indiDevice->isConnected(), "Expected device to be disconnected within 2 seconds.");

  delete newIndiClient;
}


/**
 * Set different value to a 1OfMany switch.
 */
BOOST_AUTO_TEST_CASE(IndiDeviceSet1OfManySwitchForthAndBack)
{
  IndiClientT * newIndiClient = new IndiClientT("localhost", 7624 /* port */, true /* AutoConnect */);

  BOOST_CHECK(newIndiClient);
  BOOST_CHECK_MESSAGE(newIndiClient->isConnected(), "Expected client to be connected to server.");

  IndiDeviceT * indiDevice = newIndiClient->getDevice("CCD Simulator");
  BOOST_CHECK_MESSAGE(indiDevice != 0, "Expected device to be valid.");

  // Check connect
  indiDevice->disconnect(2000);
  BOOST_CHECK_MESSAGE(! indiDevice->isConnected(), "Expected device to be disconnected within 2 seconds.");
  indiDevice->connect(2000);
  BOOST_CHECK_MESSAGE(indiDevice->isConnected(), "Expected device to be connected within 2 seconds.");

  // Switch ISR_1OFMANY forth and back test - repeat 5 times...
  for (size_t i=0; i < 5; ++i) {
    indiDevice->sendSwitchVal("GUIDER_RAPID_GUIDE", "ENABLE", true); // Enable
    bool newValueEnable = indiDevice->getSwitchVal("GUIDER_RAPID_GUIDE", "ENABLE");
    bool newValueDisable = indiDevice->getSwitchVal("GUIDER_RAPID_GUIDE", "DISABLE");

    indiDevice->sendSwitchVal("GUIDER_RAPID_GUIDE", "ENABLE", false); // Set back to false
    bool finalValueEnable = indiDevice->getSwitchVal("GUIDER_RAPID_GUIDE", "ENABLE");
    bool finalValueDisable = indiDevice->getSwitchVal("GUIDER_RAPID_GUIDE", "DISABLE");

    BOOST_CHECK_MESSAGE(newValueEnable != newValueDisable, "Expected newValueEnable != newValueDisable.");
    BOOST_CHECK_MESSAGE(finalValueEnable != finalValueDisable, "Expected finalValueEnable != finalValueDisable.");

    BOOST_CHECK_MESSAGE(newValueEnable, "Expected newValueEnable to be set to true.");
    BOOST_CHECK_MESSAGE(! newValueDisable, "Expected newValueDisable to be set to false.");

    BOOST_CHECK_MESSAGE(! finalValueEnable, "Expected newValueEnable to be set to false.");
    BOOST_CHECK_MESSAGE(finalValueDisable, "Expected finalValueDisable to be set to true.");
  }

  // Set an n>2 1OFMANY switch to 0 -> expect exception...
  bool bRes = indiDevice->getSwitchVal("GUIDER_FRAME_TYPE", "FRAME_LIGHT");
  BOOST_CHECK_MESSAGE(bRes, "Expected GUIDER_FRAME_TYPE->FRAME_LIGHT to be true.");
  BOOST_CHECK_THROW(indiDevice->sendSwitchVal("GUIDER_FRAME_TYPE", "FRAME_LIGHT", false), SetPropValueFailedExceptionT);

  bool bRes2 = indiDevice->getSwitchVal("GUIDER_FRAME_TYPE", "FRAME_LIGHT");
  BOOST_CHECK_MESSAGE(bRes2, "Expected GUIDER_FRAME_TYPE->FRAME_LIGHT to be still true.");
  


  indiDevice->disconnect(2000);
  BOOST_CHECK_MESSAGE(! indiDevice->isConnected(), "Expected device to be disconnected within 2 seconds.");

  delete newIndiClient;
}


/**
 * Set different value to a NOfMany switch.
 */
BOOST_AUTO_TEST_CASE(IndiDeviceSetNOfManySwitchForthAndBack)
{
  IndiClientT * newIndiClient = new IndiClientT("localhost", 7624 /* port */, true /* AutoConnect */);

  BOOST_CHECK(newIndiClient);
  BOOST_CHECK_MESSAGE(newIndiClient->isConnected(), "Expected client to be connected to server.");

  IndiDeviceT * indiDevice = newIndiClient->getDevice("CCD Simulator");
  BOOST_CHECK_MESSAGE(indiDevice != 0, "Expected device to be valid.");

  // Check connect
  indiDevice->disconnect(2000);
  BOOST_CHECK_MESSAGE(! indiDevice->isConnected(), "Expected device to be disconnected within 2 seconds.");
  indiDevice->connect(2000);
  BOOST_CHECK_MESSAGE(indiDevice->isConnected(), "Expected device to be connected within 2 seconds.");

  // Switch ISR_NOFMANY forth and back test - repeat 5 times...
  // For this test RAPID_GUIDING must be enabled since we need the GUIDER_RAPID_GUIDE_SETUP property.
  cerr << "Enabling GUIDER_RAPID_GUIDE..." << endl;
  indiDevice->sendSwitchVal("GUIDER_RAPID_GUIDE", "ENABLE", true);

  for (size_t i=0; i < 5; ++i) {
    // Check 1
    indiDevice->sendSwitchVal("GUIDER_RAPID_GUIDE_SETUP", "AUTO_LOOP", true);
    indiDevice->sendSwitchVal("GUIDER_RAPID_GUIDE_SETUP", "SEND_IMAGE", false);
    indiDevice->sendSwitchVal("GUIDER_RAPID_GUIDE_SETUP", "SHOW_MARKER", false);
    
    bool bResAutoLoop = indiDevice->getSwitchVal("GUIDER_RAPID_GUIDE_SETUP", "AUTO_LOOP");
    bool bResSendImage = indiDevice->getSwitchVal("GUIDER_RAPID_GUIDE_SETUP", "SEND_IMAGE");
    bool bResShowMarker = indiDevice->getSwitchVal("GUIDER_RAPID_GUIDE_SETUP", "SHOW_MARKER");

    BOOST_CHECK_MESSAGE(bResAutoLoop, "Expected bResAutoLoop to be set to true.");
    BOOST_CHECK_MESSAGE(! bResSendImage, "Expected bResSendImage to be set to false.");
    BOOST_CHECK_MESSAGE(! bResShowMarker, "Expected bResShowMarker to be set to false.");

    // Check 2
    indiDevice->sendSwitchVal("GUIDER_RAPID_GUIDE_SETUP", "SEND_IMAGE", true);

    bResAutoLoop = indiDevice->getSwitchVal("GUIDER_RAPID_GUIDE_SETUP", "AUTO_LOOP");
    bResSendImage = indiDevice->getSwitchVal("GUIDER_RAPID_GUIDE_SETUP", "SEND_IMAGE");
    bResShowMarker = indiDevice->getSwitchVal("GUIDER_RAPID_GUIDE_SETUP", "SHOW_MARKER");
    
    BOOST_CHECK_MESSAGE(bResAutoLoop, "Expected bResAutoLoop to be set to true.");
    BOOST_CHECK_MESSAGE(bResSendImage, "Expected bResSendImage to be set to true.");
    BOOST_CHECK_MESSAGE(! bResShowMarker, "Expected bResShowMarker to be set to false.");

    // Check 3
    indiDevice->sendSwitchVal("GUIDER_RAPID_GUIDE_SETUP", "SHOW_MARKER", true);

    bResAutoLoop = indiDevice->getSwitchVal("GUIDER_RAPID_GUIDE_SETUP", "AUTO_LOOP");
    bResSendImage = indiDevice->getSwitchVal("GUIDER_RAPID_GUIDE_SETUP", "SEND_IMAGE");
    bResShowMarker = indiDevice->getSwitchVal("GUIDER_RAPID_GUIDE_SETUP", "SHOW_MARKER");
    
    BOOST_CHECK_MESSAGE(bResAutoLoop, "Expected bResAutoLoop to be set to true.");
    BOOST_CHECK_MESSAGE(bResSendImage, "Expected bResSendImage to be set to true.");
    BOOST_CHECK_MESSAGE(bResShowMarker, "Expected bResShowMarker to be set to true.");

    // Check 4
    indiDevice->sendSwitchVal("GUIDER_RAPID_GUIDE_SETUP", "AUTO_LOOP", false);

    bResAutoLoop = indiDevice->getSwitchVal("GUIDER_RAPID_GUIDE_SETUP", "AUTO_LOOP");
    bResSendImage = indiDevice->getSwitchVal("GUIDER_RAPID_GUIDE_SETUP", "SEND_IMAGE");
    bResShowMarker = indiDevice->getSwitchVal("GUIDER_RAPID_GUIDE_SETUP", "SHOW_MARKER");
    
    BOOST_CHECK_MESSAGE(! bResAutoLoop, "Expected bResAutoLoop to be set to false.");
    BOOST_CHECK_MESSAGE(bResSendImage, "Expected bResSendImage to be set to true.");
    BOOST_CHECK_MESSAGE(bResShowMarker, "Expected bResShowMarker to be set to true.");
  } // end for

  indiDevice->sendSwitchVal("GUIDER_RAPID_GUIDE", "ENABLE", false);

  indiDevice->disconnect(2000);
  BOOST_CHECK_MESSAGE(! indiDevice->isConnected(), "Expected device to be disconnected within 2 seconds.");

  delete newIndiClient;
}


static double numberHandler1 = 0, numberHandler2 = 0;
void myNumberChangeHandler(INumberVectorProperty * inVec) {
  if (! strcmp(inVec->name, "CCD_BINNING")) { numberHandler1 = IndiDeviceT::getNumberVal(inVec, "VER_BIN"); }
}
void myNumberChangeHandler2(INumberVectorProperty * inVec) {
    if (! strcmp(inVec->name, "CCD_BINNING")) { 
      cerr << "CALLED myNumberChangeHandler2(INumberVectorProperty * inVec=CCD_BINNING...)..." << endl;
      numberHandler2 = IndiDeviceT::getNumberVal(inVec, "VER_BIN");
    }
}

static bool switchHandler1 = false, switchHandler2 = false;
void mySwitchChangeHandler(ISwitchVectorProperty * inVec) {
  if (! strcmp(inVec->name, "GUIDER_RAPID_GUIDE")) { switchHandler1 = IndiDeviceT::getSwitchVal(inVec, "ENABLE"); }
}
void mySwitchChangeHandler2(ISwitchVectorProperty * inVec) {
  if (! strcmp(inVec->name, "GUIDER_RAPID_GUIDE")) { switchHandler2 = IndiDeviceT::getSwitchVal(inVec, "ENABLE"); }
}

static string textHandler1, textHandler2;
void myTextChangeHandler(ITextVectorProperty * inVec) {
  if (! strcmp(inVec->name, "FILTER_NAME")) {
    textHandler1 = IndiDeviceT::getTextVal(inVec, "FILTER_SLOT_NAME_1");
  }
}
void myTextChangeHandler2(ITextVectorProperty * inVec) {
  if (! strcmp(inVec->name, "FILTER_NAME")) {
    textHandler2 = IndiDeviceT::getTextVal(inVec, "FILTER_SLOT_NAME_1");
  }
}

static void * blobHandler1 = 0;
static void * blobHandler2 = 0;
void myBLOBChangeHandler(IBLOB * inVec) { if (! strcmp(inVec->name, "CCD1")) { blobHandler1 = inVec->blob; } }
void myBLOBChangeHandler2(IBLOB * inVec) { if (! strcmp(inVec->name, "CCD1")) { blobHandler2 = inVec->blob; } }

/**
 * Test the registerValuelistener/unregisterValuelistener functionality.
 */
BOOST_AUTO_TEST_CASE(IndiDeviceRegisterValueListener)
{
  IndiClientT * newIndiClient = new IndiClientT("localhost", 7624 /* port */, true /* AutoConnect */);

  BOOST_CHECK(newIndiClient);
  BOOST_CHECK_MESSAGE(newIndiClient->isConnected(), "Expected client to be connected to server.");

  IndiDeviceT * indiDevice = newIndiClient->getDevice("CCD Simulator");
  BOOST_CHECK_MESSAGE(indiDevice != 0, "Expected device to be valid.");

  // Check connect
  indiDevice->disconnect(2000);
  BOOST_CHECK_MESSAGE(! indiDevice->isConnected(), "Expected device to be disconnected within 2 seconds.");
  indiDevice->connect(2000);
  BOOST_CHECK_MESSAGE(indiDevice->isConnected(), "Expected device to be connected within 2 seconds.");

  // Register for number change of device...
  signals2::connection cn1 = indiDevice->registerNewNumberListener(boost::bind(& myNumberChangeHandler, _1));
  signals2::connection cn2 = indiDevice->registerNewNumberListener(boost::bind(& myNumberChangeHandler2, _1));

  signals2::connection cs1 = indiDevice->registerNewSwitchListener(boost::bind(& mySwitchChangeHandler, _1));
  signals2::connection cs2 = indiDevice->registerNewSwitchListener(boost::bind(& mySwitchChangeHandler2, _1));

  indiDevice->registerNewTextListener(boost::bind(& myTextChangeHandler, _1));
  indiDevice->registerNewTextListener(boost::bind(& myTextChangeHandler2, _1));

  // How to issue a light event?? There is no set method...
  //indiDevice->registerNewLightListener(boost::bind(& myLightChangeHandler, _1));
  //indiDevice->registerNewLightListener(boost::bind(& myLightChangeHandler2, _1));

  indiDevice->registerNewBLOBListener(boost::bind(& myBLOBChangeHandler, _1));
  indiDevice->registerNewBLOBListener(boost::bind(& myBLOBChangeHandler2, _1));


  // Write values
  indiDevice->sendNumberVal("CCD_BINNING", "VER_BIN", 2);
  indiDevice->sendSwitchVal("GUIDER_RAPID_GUIDE", "ENABLE", true);

  // TODO: Does not work for FILTER_NAME", "FILTER_SLOT_NAME_1... Maybe a problem in the driver? We may try another driver which has more text props...
  indiDevice->sendTextVal("FILTER_NAME", "FILTER_SLOT_NAME_1", "Pink", 0 /* 0 sec */);
  usleep(1000000); 

  // Issue an exposure to activate BLOB receiver...
  indiDevice->sendNumberVal("CCD_EXPOSURE", "CCD_EXPOSURE_VALUE", 2 /* 2 sec */, 5000 /* wait up to 5 seconds until exposure is completed */);
  usleep(1000000); // Exposure finished... Give BLOB handler a short moment to proceed...

  // Finally, check the expected results
  BOOST_CHECK_MESSAGE(numberHandler1 == 2, "Expected number to be set to 2 by event handler.");
  BOOST_CHECK_MESSAGE(numberHandler2 == 2, "Expected number to be set to 2 by event handler.");

  BOOST_CHECK_MESSAGE(switchHandler1, "Expected switch to be set to true by event handler.");
  BOOST_CHECK_MESSAGE(switchHandler2, "Expected switch to be set to true by event handler.");

  BOOST_CHECK_MESSAGE(! strcmp(textHandler1.c_str(), "Pink"), "Expected text to be set to 'Pink' by event handler.");
  BOOST_CHECK_MESSAGE(! strcmp(textHandler2.c_str(), "Pink"), "Expected text to be set to 'Pink' by event handler.");
  
  BOOST_CHECK_MESSAGE(blobHandler1, "Expected blobHandler1 to be set != 0.");
  BOOST_CHECK_MESSAGE(blobHandler2, "Expected blobHandler2 to be set != 0.");


  // Unregister handlers
  indiDevice->unregisterNewNumberListener(cn2);
  numberHandler1 = 0;
  numberHandler2 = 0;
  indiDevice->sendNumberVal("CCD_BINNING", "VER_BIN", 2);
  usleep(1000000); // Wait a moment...
  BOOST_CHECK_MESSAGE(numberHandler1 == 2, "Expected number to be set to 2 by event handler.");
  BOOST_CHECK_MESSAGE(numberHandler2 == 0, "Expected number to be 0.");


  // Unregister handlers
  indiDevice->sendSwitchVal("GUIDER_RAPID_GUIDE", "ENABLE", false);
  usleep(1000000); // Wait a moment...
  indiDevice->unregisterNewSwitchListener(cs2);
  switchHandler1 = false;
  switchHandler2 = false;
  indiDevice->sendSwitchVal("GUIDER_RAPID_GUIDE", "ENABLE", true);
  usleep(1000000); // Wait a moment...
  BOOST_CHECK_MESSAGE(switchHandler1, "Expected switch to be set to true by event handler.");
  BOOST_CHECK_MESSAGE(! switchHandler2, "Expected switch to be false.");


  /**
   * TODO: We may test unregister handler for Text, Light and BLOB as well...
   * but there are no such scenarios with CCD Simuator, yet. Furthermore,
   * the responsible code is MACRO code which is the same as for Number and Switch.
   * Hence, we accept this as a checking hole for now.
   */

  // Finished checking, disconnect from device
  indiDevice->disconnect(2000);
  BOOST_CHECK_MESSAGE(! indiDevice->isConnected(), "Expected device to be disconnected within 2 seconds.");

  delete newIndiClient;
}

/**
 * Get different vec prop types from the INDI device.
 * TODO: Extend CHECKS!!
 */
BOOST_AUTO_TEST_CASE(IndiDeviceGetVecValues)
{
  IndiClientT * newIndiClient = new IndiClientT("localhost", 7624 /* port */, true /* AutoConnect */);

  BOOST_CHECK(newIndiClient);
  BOOST_CHECK_MESSAGE(newIndiClient->isConnected(), "Expected client to be connected to server.");

  IndiDeviceT * indiDevice = newIndiClient->getDevice("CCD Simulator");
  BOOST_CHECK_MESSAGE(indiDevice != 0, "Expected device to be valid.");

  // Check connect
  indiDevice->disconnect(2000);
  BOOST_CHECK_MESSAGE(! indiDevice->isConnected(), "Expected device to be disconnected within 2 seconds.");
  indiDevice->connect(2000);
  BOOST_CHECK_MESSAGE(indiDevice->isConnected(), "Expected device to be connected within 2 seconds.");

  // Read number prop vec
  INumberVectorProperty * nVec = indiDevice->getNumberVec("CCD_BINNING");
  ISwitchVectorProperty * sVec = indiDevice->getSwitchVec("GUIDER_RAPID_GUIDE");
  ITextVectorProperty * tVec = indiDevice->getTextVec("FILTER_NAME");
  ILightVectorProperty * lVec = indiDevice->getLightVec("CCD_FRAME_TYPE"); // TODO: Why this cannot be found?!?!
  IBLOBVectorProperty * bVec = indiDevice->getBLOBVec("CCD1");

  BOOST_CHECK_MESSAGE(nVec, "Expected CCD_BINNING to be != 0.");
  BOOST_CHECK_MESSAGE(sVec, "Expected GUIDER_RAPID_GUIDE to be != 0.");
  BOOST_CHECK_MESSAGE(tVec, "Expected FILTER_NAME to be != 0.");
  BOOST_CHECK_MESSAGE(lVec, "Expected CCD_FRAME_TYPE to be != 0.");
  BOOST_CHECK_MESSAGE(bVec, "Expected CCD1 to be != 0.");

  // Check THROW if not found...
  BOOST_CHECK_THROW(indiDevice->getNumberVec("DOES_NOT_EXIST"), PropNotFoundExceptionT);
  BOOST_CHECK_THROW(indiDevice->getSwitchVec("DOES_NOT_EXIST"), PropNotFoundExceptionT);
  BOOST_CHECK_THROW(indiDevice->getTextVec("DOES_NOT_EXIST"), PropNotFoundExceptionT);
  BOOST_CHECK_THROW(indiDevice->getLightVec("DOES_NOT_EXIST"), PropNotFoundExceptionT);
  BOOST_CHECK_THROW(indiDevice->getBLOBVec("DOES_NOT_EXIST"), PropNotFoundExceptionT);

  // Finally, disconnect
  indiDevice->disconnect(2000);
  BOOST_CHECK_MESSAGE(! indiDevice->isConnected(), "Expected device to be disconnected within 2 seconds.");

  delete newIndiClient;
}



/**
 * Set different vec prop types from the INDI device.
 * TODO: Extend CHECKS!!
 */
BOOST_AUTO_TEST_CASE(IndiDeviceSetVecValues)
{
  IndiClientT * newIndiClient = new IndiClientT("localhost", 7624 /* port */, true /* AutoConnect */);

  BOOST_CHECK(newIndiClient);
  BOOST_CHECK_MESSAGE(newIndiClient->isConnected(), "Expected client to be connected to server.");

  IndiDeviceT * indiDevice = newIndiClient->getDevice("CCD Simulator");
  BOOST_CHECK_MESSAGE(indiDevice != 0, "Expected device to be valid.");

  // Check connect
  indiDevice->disconnect(2000);
  BOOST_CHECK_MESSAGE(! indiDevice->isConnected(), "Expected device to be disconnected within 2 seconds.");
  indiDevice->connect(2000);
  BOOST_CHECK_MESSAGE(indiDevice->isConnected(), "Expected device to be connected within 2 seconds.");

  // Read number prop vec
  INumberVectorProperty * nVec = indiDevice->getNumberVec("CCD_BINNING");
  indiDevice->sendNumberVec(nVec);

  ISwitchVectorProperty * sVec = indiDevice->getSwitchVec("GUIDER_RAPID_GUIDE");
  indiDevice->sendSwitchVec(sVec);

  ITextVectorProperty * tVec = indiDevice->getTextVec("FILTER_NAME");
  indiDevice->sendTextVec(tVec);

  // TODO: BLOB?

  // TODO: Is this check sufficient to check the set functionality? We should set a different vector object... and see if it was set properly!
  BOOST_CHECK_MESSAGE(nVec == indiDevice->getNumberVec("CCD_BINNING"), "Expected nVec == getNumberVec().");
  BOOST_CHECK_MESSAGE(sVec == indiDevice->getSwitchVec("GUIDER_RAPID_GUIDE"), "Expected sVec == getSwitchVec().");
  BOOST_CHECK_MESSAGE(tVec == indiDevice->getTextVec("FILTER_NAME"), "Expected tVec == getTextVec().");

  // Finally, disconnect
  indiDevice->disconnect(2000);
  BOOST_CHECK_MESSAGE(! indiDevice->isConnected(), "Expected device to be disconnected within 2 seconds.");

  delete newIndiClient;
}


/**
 * Invalid parameters tests.
 */
BOOST_AUTO_TEST_CASE(IndiDeviceInvalidParametersTests)
{
  IndiClientT * newIndiClient = new IndiClientT("localhost", 7624 /* port */, true /* AutoConnect */);

  BOOST_CHECK(newIndiClient);
  BOOST_CHECK_MESSAGE(newIndiClient->isConnected(), "Expected client to be connected to server.");

  IndiDeviceT * indiDevice = newIndiClient->getDevice("CCD Simulator");
  BOOST_CHECK_MESSAGE(indiDevice != 0, "Expected device to be valid.");

  // Check timeout exception
  indiDevice->disconnect(2000);
  BOOST_CHECK_MESSAGE(! indiDevice->isConnected(), "Expected device to be disconnected within 2 seconds.");

  // Check connect
  indiDevice->connect(2000);
  BOOST_CHECK_MESSAGE(indiDevice->isConnected(), "Expected device to be connected within 2 seconds.");

  // Set invalid number
  double oldBinning = indiDevice->getNumberVal("CCD_BINNING", "VER_BIN");
  BOOST_CHECK_THROW(indiDevice->sendNumberVal("CCD_BINNING", "VER_BIN", 200000), PropInvalidValueExceptionT);
  double newBinning = indiDevice->getNumberVal("CCD_BINNING", "VER_BIN");
  BOOST_CHECK_MESSAGE(oldBinning == newBinning, "Expecting oldBinning == newBinning, i.e. binning not changed.");
  
  // Check disconnect
  indiDevice->disconnect(2000);
  BOOST_CHECK_MESSAGE(! indiDevice->isConnected(), "Expected device to be disconnected within 2 seconds.");

  delete newIndiClient;
}


/**
 * TODO: Check custom prop. map mechanism
 */
BOOST_AUTO_TEST_CASE(IndiDeviceCheckCustomPropMaps)
{
  IndiClientT * newIndiClient = new IndiClientT("localhost", 7624 /* port */, true /* AutoConnect */);

  BOOST_CHECK(newIndiClient);
  BOOST_CHECK_MESSAGE(newIndiClient->isConnected(), "Expected client to be connected to server.");

  IndiDeviceT * indiDevice = newIndiClient->getDevice("CCD Simulator");
  BOOST_CHECK_MESSAGE(indiDevice != 0, "Expected device to be valid.");

  // Check timeout exception
  indiDevice->disconnect(2000);
  BOOST_CHECK_MESSAGE(! indiDevice->isConnected(), "Expected device to be disconnected within 2 seconds.");

  // Check connect
  indiDevice->connect(2000);
  BOOST_CHECK_MESSAGE(indiDevice->isConnected(), "Expected device to be connected within 2 seconds.");

  // Check 
  //PropNameMapT & vecPropMap = indiDevice->getVecPropMap();
  //PropNameMapT & propMap = indiDevice->getPropMap();
  // TODO!!!
  
  // Check disconnect
  indiDevice->disconnect(2000);
  BOOST_CHECK_MESSAGE(! indiDevice->isConnected(), "Expected device to be disconnected within 2 seconds.");

  delete newIndiClient;
}


BOOST_AUTO_TEST_SUITE_END()
