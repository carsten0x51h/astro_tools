#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include <unistd.h>
#include <time.h>
#include <memory>
#include <sys/types.h>
#include <sys/stat.h>

#include <CImg.h>

#include "../indi/indi_camera_client.hpp"

using namespace std;
using namespace cimg_library;

void testSwitchVecHandler(ISwitchVectorProperty* inVecSwitch) {
  cerr << "################################# TEST HANDLER: value: " << inVecSwitch << endl;
  return;
}

void testMessageHandler(const char * msg) {
  cerr << "################################# TEST MSG HANDLER: " << msg << endl;
  return;
}

void exposureTimeChangeHandler(INumberVectorProperty * inNumberVec) {
  if (! inNumberVec)
    return;

  cerr << "exposureTimeChangeHandler - New number vec: " << inNumberVec << ",name: " << inNumberVec->name << ", #entries: " << inNumberVec->nnp << endl;

  for (size_t i = 0; i < (inNumberVec->nnp); ++i) {
    cerr << "i: " << i << ", value: " << inNumberVec[i].np->value << endl;
  }
  return;
}

void blobHandler(IBLOB * inBlob) {
  cout << "BLOB received! - inBlob: " << inBlob << ", saving..." << flush;
  ofstream myfile;
  myfile.open ("received_img.fits", ios::out | ios::binary);
  myfile.write(static_cast<char *> (inBlob->blob), inBlob->bloblen);
  myfile.close();
  cout << "DONE" << endl;
}

int main(int argc, char *argv[]) {

  const char * deviceName = "ATIK CCD Atik 383L+";
  IndiCameraClientT cameraClient(deviceName);

  try {
    // Register for centain events
    IndiCameraClientT::SwitchVectorPropertyListenersT::ListenerT c = boost::bind(& testSwitchVecHandler, _1);
    cameraClient.registerChangeListener(IndiCameraClientT::VecPropNameT::CONNECTION, c);

    IndiCameraClientT::MessageListenersT::ListenerT mh = boost::bind(& testMessageHandler, _1);
    cameraClient.registerMessageListener(mh);

    IndiCameraClientT::NumberVectorPropertyListenersT::ListenerT nh = boost::bind(& exposureTimeChangeHandler, _1);
    cameraClient.registerChangeListener(IndiCameraClientT::VecPropNameT::CCD_EXPOSURE, nh);

    IndiCameraClientT::BLOBListenersT::ListenerT bh = boost::bind(& blobHandler, _1);
    cameraClient.registerChangeListener(IndiCameraClientT::VecPropNameT::CCD1, bh);


    /**
     * Connect client to INDI server
     */
    const char * hostname = "localhost";
    size_t port = 7624;
    cout << "Setting server to " << hostname << ", port: " << port << "..." << flush;
    cameraClient.setServer(hostname, port);
    cout << "DONE" << endl;

    cout << "Waiting while connecting to server..." << flush;
    while (! cameraClient.isServerConnected()) {
      cameraClient.connectServer();
      cout << "." << flush;
      sleep(1);
    }
    cout << "DONE" << endl; 


    /**
     * Wait until device is available
     */
    cout << "Waiting until device " << cameraClient.getDeviceName() << " gets available..." << flush;
    while (! cameraClient.isDeviceAvailable()) {
      cout << "." << flush;
      sleep(1);
    }


    /**
     * Connect to device
     */
    cout << "Waiting for Camera being connected." << flush;
    while(! cameraClient.isDeviceConnected()) {
      cameraClient.connectDevice();
      cout << "." << flush;
      sleep(1);
    }
    cout << "DONE" << endl;

    cout << "Waiting for device being ready (received all properties)..." << flush;
    cameraClient.waitForDeviceReady();
    cout << "DONE" << endl;
    cameraClient.setBLOBMode(B_ALSO, deviceName, NULL); // HACK: Move somewhere else?!


    /**
     * Get driver info
     */
    cout << "Getting camera driver info..." << flush;
    string name, exec, version;
    cameraClient.getDriverInfo(& name, & exec, & version);
    cout << "Received camera driver - name: " << name << ", exec: " << exec << ", version: " << version << endl;    


    /**
     * Get camera info
     */
    cout << "Getting camera info..." << flush;
    unsigned int maxW, maxH;
    cameraClient.getMaxResolution(& maxW, & maxH);

    double pixelSizeW, pixelSizeH;
    cameraClient.getPixelSize(& pixelSizeW, & pixelSizeH);

    unsigned int bitsPerPixel = cameraClient.getBitsPerPixel();
    cout << "maxW: " << maxW << ", maxH: " << maxH << ", pixelSizeW: " << pixelSizeW << ", pixelSizeH: " << pixelSizeH << ", bitsPerPixel: " << bitsPerPixel << " DONE" << endl;


    /**
     * Frame
     */
    unsigned int frameX = 0, frameY = 0, frameW = maxW, frameH = maxH;
    cout << "Set frame to (x,y) = (" << frameX << ", " << frameY << "), (w,h) = (" << frameW << ", " << frameH << ")..." << flush;
    cameraClient.setFrame(frameX, frameY, frameW, frameH);
    cout << "DONE" << endl;

    cout << "Verifying frame..." << flush;
    unsigned int newFrameX, newFrameY, newFrameW, newFrameH;
    cameraClient.getFrame(& newFrameX, & newFrameY, & newFrameW, & newFrameH);
    cout << "newFrameX: " << newFrameX << ", newFrameY: " << newFrameY << ", newFrameW: " << newFrameW << ", newFrameH: " << newFrameH <<  " DONE" << endl;


    /**
     * Binning
     */
    unsigned int horzBinning = 1, vertBinning = 1;
    cout << "Set binning to (horz,vert) = (" << horzBinning << ", " << vertBinning << ")" << flush;
    cameraClient.setBinning(horzBinning, vertBinning);
    cout << " DONE" << endl;

    cout << "Verifying binning..." << flush;
    unsigned int newHorzBinning, newVertBinning;
    cameraClient.getBinning(& newHorzBinning, & newVertBinning);
    cout << "newHorzBinning: " << newHorzBinning << ", newVertBinning: " << newVertBinning << " DONE" << endl;

    
    /**
     * Compression
     */
    cout << "Disabling image compression..." << flush;
    cameraClient.setCompressed(false /* no compression */); // blocking
    cout << "DONE" << endl;

    cout << "Verifying compression..." << flush;
    bool compressionState = cameraClient.isCompressed();
    cout << "Compression " << (compressionState ? "enabled" : "disabled") << " DONE" << endl;
    assert(! compressionState);


    /**
     * Shutter cntl
     */
    cout << "Shutter test... opening shutter..." << flush;
    cameraClient.setShutterState(true /* open shutter */); // blocking
    cout << "DONE" << endl;
    cout << "Verifying shutter state..." << flush;
    bool newShutterState = cameraClient.getShutterState();
    cout << "Shutter state is " << (newShutterState ? "open" : "closed") << "." << endl;
    assert(newShutterState); // expect shutter to be open
    cout << "Keeping shutter open 3 seconds..." << endl;
    sleep(3);
    cout << "Closing shutter..." << flush;
    cameraClient.setShutterState(false /* close shutter */); // blocking
    cout << "DONE" << endl;
    cout << "Verifying shutter state..." << flush;
    newShutterState = cameraClient.getShutterState();
    cout << "Shutter state is " << (newShutterState ? "open" : "closed") << "." << endl;
    assert(! newShutterState); // expect shutter to be closed


    /**
     * Frame type
     */
    FrameTypeT::TypeE frameType = FrameTypeT::LIGHT;
    cout << "Setting frame type to " << FrameTypeT::asStr(frameType) << "..." << flush;
    cameraClient.setFrameType(frameType);

    cout << "Verifying frametype..." << flush;
    FrameTypeT::TypeE newFrameType = cameraClient.getFrameType();
    cout << "New frame type..." << FrameTypeT::asStr(newFrameType) << " DONE" << endl;
    assert(frameType == newFrameType);


    /**
     * Cooler
     */
    // double desiredChipTemperature = 10;
    // cout << "Setting desired chip temperature to " << desiredChipTemperature << "..." << flush;
    // cameraClient.setTemperature(desiredChipTemperature);
    // cout << "DONE" << endl;

    // cout << "Getting camera chip temperature..." << flush;
    // double newCcdTemperature = cameraClient.getTemperature();
    // cout << "T=" << newCcdTemperature << "°C. DONE" << endl;

    // char f;
    // cout << "press key to activate cooler..." << endl;
    // cin >> f;


    // cout << "Enabling cooler..." << flush;
    // cameraClient.setCoolerEnabled(true); // blocking
    // cout << " DONE" << endl;

    // cout << "Verify cooler enabled..." << flush;
    // bool coolerEnabled = cameraClient.isCoolerEnabled();
    // cout << "New cooler enabled state: " << coolerEnabled << endl;
    // assert(coolerEnabled);


    // char g;
    // cout << "press key to disable cooler..." << endl;
    // cin >> g;


    // cout << "Disabling cooler..." << flush;
    // cameraClient.setCoolerEnabled(false); // blocking
    // cout << " DONE" << endl;

    // cout << "Verify cooler diabled..." << flush;
    // coolerEnabled = cameraClient.isCoolerEnabled();
    // cout << "New cooler enabled state: " << coolerEnabled << endl;
    // assert(! coolerEnabled);





    /**
     * Exposure
     */
    cout << "Getting exposure time..." << flush;
    double curExposureTime = cameraClient.getExposureTime();
    cout << "t=" << curExposureTime << " seconds...DONE" << endl;

    double exposureTime = 1; // 1 seconds
    cout << "Setting exposure time to " << exposureTime << " seconds..." << flush;
    cameraClient.setExposureTime(exposureTime, UpdatePolicyT::NON_BLOCKING); // Do not block!
    cout << "DONE" << endl;



    /**
     * Taking a series of different frames
     */
    CImg<float> img;
    cameraClient.takePicture(& img, 1 /*horz bin*/, 1 /*vert bin*/, 0 /*x*/, 0 /*y*/, 100 /*w*/, 100 /*h*/, FrameTypeT::LIGHT, 5 /*s*/, false /*no compr.*/);



    // Just wait for input
    char cc;
    cout << "Press btn to exit..." << endl;
    cin >> cc;

    // Disconnect device
    cameraClient.disconnectDevice();
    cameraClient.disconnectServer();

  } catch(exception & e) {
    cerr << "EXCEPTION: " << e.what() << endl;
  }

  return 0;
}
