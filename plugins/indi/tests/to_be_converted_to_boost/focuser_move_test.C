#include <iostream>
#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include <unistd.h>
#include <time.h>
#include <memory>
#include <sys/types.h>
#include <sys/stat.h>

#include "../indi/indi_focuser_client.hpp"


void testSwitchVecHandler(ISwitchVectorProperty* inVecSwitch) {
  cerr << "################################# TEST HANDLER: value: " << inVecSwitch << endl;
  return;
}

void testMessageHandler(const char * msg) {
  cerr << "################################# TEST MSG HANDLER: " << msg << endl;
  return;
}


int main(int argc, char *argv[]) {

  //MoonLiteFocuserT focusClient;
  IndiFocuserClientT focusClient("MoonLite");

  try {
    // Register for centain events
    IndiFocuserClientT::SwitchVectorPropertyListenersT::ListenerT c = boost::bind(& testSwitchVecHandler, _1);
    focusClient.registerChangeListener(IndiFocuserClientT::VecPropNameT::CONNECTION, c);

    IndiFocuserClientT::MessageListenersT::ListenerT mh = boost::bind(& testMessageHandler, _1);
    focusClient.registerMessageListener(mh);

    // Class example...
    //IndiFocuserClientT::IndiServerConnectionStatusListenersT::ListenerT c = boost::bind(& FocuserCntlPanel::indiServerConnectionStatusChanged, this, _1);
    //focusClient.registerIndiServerConnectionChangeListener(c);


    // Connect client to INDI server
    const char * hostname = "localhost";
    size_t port = 7624;
    cout << "Setting server to " << hostname << ", port: " << port << "..." << flush;
    focusClient.setServer(hostname, port);
    cout << "DONE" << endl;

    cout << "Waiting while connecting to server..." << flush;
    while (! focusClient.isServerConnected()) {
      focusClient.connectServer();
      cout << "." << flush;
      sleep(1);
    }
    cout << "DONE" << endl; 

    // Connect device
    const char * devicePort = "/dev/ttyUSB0";
    cout << "Setting device port to " << devicePort << " ..." << flush;
    focusClient.setDevicePort(devicePort);
    cout << "DONE" << endl;
    string newDevicePort;
    focusClient.getDevicePort(& newDevicePort);
    cout << "New device port is " << newDevicePort << endl;


    cout << "Waiting for device being ready (received all properties)..." << flush;
    focusClient.waitForDeviceReady();
    cout << "DONE" << endl;


    cout << "Waiting for Focuser being connected." << flush;
    while(! focusClient.isDeviceConnected()) {
      focusClient.connectDevice();
      cout << "." << flush;
      sleep(1);
    }
    cout << "DONE" << endl;


    // Check if really powered on (measure temperature) (controller gets power from USB, rest of circuit externally!)
    // TODO!!!!!
    int tmp = focusClient.getTemperature();
    cerr << "############################# TMP: " << tmp << endl;


    // cout << "Wainting some time until props are populated (later do this on event...)" << endl;
    // sleep(3);
    // cout << "Ok, waited enough! Moving!" << endl;
    // TODO: Required?!!

    
    // Get current relative position of focus...
    cout << "Requesting current relative focus position..." << flush;
    cout << focusClient.getRelPos() << endl;
    cout << "DONE" << endl;

    // Changing focus direction to inwards
    cout << "Requesting changing focus direction to inwards." << flush;
    focusClient.setFocusDirection(IndiFocuserClientT::FocusDirectionT::INWARDS);
    cout << "DONE" << endl;

    // Move 100 steps inwards
    unsigned int steps = 100;
    cout << "Requesting move 100 steps." << flush;
    focusClient.setRelPos(steps); // Blocks until reached - or quits with exception, updtes are sent if registered to events... but cntl flow stays here!
    cout << "DONE" << endl;

    // Get current relative position of focus...
    cout << "Requesting current relative focus position..." << flush;
    cout << focusClient.getRelPos() << endl;
    cout << "DONE" << endl;

    // Changing focus direction to outwards
    cout << "Requesting changing focus direction to outwards." << flush;
    focusClient.setFocusDirection(IndiFocuserClientT::FocusDirectionT::OUTWARDS);
    cout << "DONE" << endl;

    // Move 100 steps outwards
    steps = 100;
    cout << "Requesting move 100 steps." << flush;
    focusClient.setRelPos(steps);
    cout << "DONE" << endl;

    // Get current relative position of focus...
    cout << "Requesting current relative focus position..." << flush;
    cout << focusClient.getRelPos() << endl;
    cout << "DONE" << endl;

    // Set / Get absolute position
    // TODO


    // Get current temperature...
    // TODO




    // Just wait for input
    char cc;
    cout << "Press btn to exit..." << endl;
    cin >> cc;

    // Disconnect device
    focusClient.disconnectDevice();
    focusClient.disconnectServer();

  } catch(exception & e) {
    cerr << "EXCEPTION: " << e.what() << endl;
  }

  return 0;
}

