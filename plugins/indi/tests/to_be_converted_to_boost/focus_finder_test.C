#include <iostream>
#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include <unistd.h>
#include <time.h>
#include <memory>
#include <sys/types.h>
#include <sys/stat.h>

#include "../indi/indi_camera_client.hpp"
#include "../indi/indi_focuser_client.hpp"
#include "focus_finder.hpp"


int main(int argc, char *argv[]) {

  IndiCameraClientT cameraClient("ATIK CCD Atik 383L+");
  IndiFocuserClientT focuserClient("MoonLite");
  FocusFinderT focusFinder;
  focusFinder.setCameraClient(& cameraClient);
  focusFinder.setFocuserClient(& focuserClient);

  // try {
  //   // Connect client to INDI server
  //   const char * hostname = "localhost";
  //   size_t port = 7624;
  //   cout << "Setting server to " << hostname << ", port: " << port << "..." << flush;
  //   focusFinder.setServer(hostname, port);
  //   cout << "DONE" << endl;

  //   cout << "Waiting while connecting to server..." << flush;
  //   while (! focusFinder.isServerConnected()) {
  //     focusFinder.connectServer();
  //     cout << "." << flush;
  //     sleep(1);
  //   }
  //   cout << "DONE" << endl; 

  //   // Connect device
  //   const char * devicePort = "/dev/ttyUSB0";
  //   cout << "Setting device port to " << devicePort << " ..." << flush;
  //   focusFinder.setDevicePort(devicePort);
  //   cout << "DONE" << endl;
  //   string newDevicePort;
  //   focusFinder.getDevicePort(& newDevicePort);
  //   cout << "New device port is " << newDevicePort << endl;


  //   cout << "Waiting for device being ready (received all properties)..." << flush;
  //   focusFinder.waitForDeviceReady();
  //   cout << "DONE" << endl;

  //   cout << "Waiting for Focuser being connected." << flush;
  //   while(! focusFinder.isDeviceConnected()) {
  //     focusFinder.connectDevice();
  //     cout << "." << flush;
  //     sleep(1);
  //   }
  //   cout << "DONE" << endl;






  //   // Just wait for input
  //   char cc;
  //   cout << "Press btn to exit..." << endl;
  //   cin >> cc;

  //   // Disconnect device
  //   focusFinder.disconnectDevice();
  //   focusFinder.disconnectServer();

  // } catch(exception & e) {
  //   cerr << "EXCEPTION: " << e.what() << endl;
  // }

  return 0;
}

