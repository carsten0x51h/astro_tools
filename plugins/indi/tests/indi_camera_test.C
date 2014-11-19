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

// http://www.boost.org/doc/libs/1_35_0/libs/test/doc/components/utf/components/test_case/param_boost_function_tc.html

#include <boost/test/unit_test.hpp>

#include "indi/indi_client.hpp"
#include "indi/indi_camera.hpp"

BOOST_AUTO_TEST_SUITE(IndiCamera)

static const char * cameras[] = { "CCD Simulator" };

/**
 * Connect / disconnect INDI device.
 */
BOOST_AUTO_TEST_CASE(IndiCameraConnectDisconnect)
{
  IndiClientT * newIndiClient = new IndiClientT("localhost", 7624 /* port */, true /* AutoConnect */);
  
  BOOST_CHECK(newIndiClient);
  BOOST_CHECK_MESSAGE(newIndiClient->isConnected(), "Expected client to be connected to server.");

  IndiCameraT * indiCamera = newIndiClient->getCamera("CCD Simulator");
  BOOST_CHECK_MESSAGE(indiCamera, "Expect indiCamera to be set.");  

  // Check connect
  indiCamera->disconnect(2000);
  BOOST_CHECK_MESSAGE(! indiCamera->isConnected(), "Expected camera to be disconnected within 2 seconds.");
  indiCamera->connect(2000);
  BOOST_CHECK_MESSAGE(indiCamera->isConnected(), "Expected camera to be connected within 2 seconds.");
  indiCamera->disconnect(2000);
  BOOST_CHECK_MESSAGE(! indiCamera->isConnected(), "Expected device to be disconnected within 2 seconds.");

  delete newIndiClient;
}


/**
 * Get camera chip details.
 */
BOOST_AUTO_TEST_CASE(IndiCameraGetValues)
{
  IndiClientT * newIndiClient = new IndiClientT("localhost", 7624 /* port */, true /* AutoConnect */);
  
  BOOST_CHECK(newIndiClient);
  BOOST_CHECK_MESSAGE(newIndiClient->isConnected(), "Expected client to be connected to server.");

  IndiCameraT * indiCamera = newIndiClient->getCamera("CCD Simulator");
  BOOST_CHECK_MESSAGE(indiCamera, "Expect indiCamera to be set.");  

  // Check connect
  indiCamera->disconnect(2000);
  BOOST_CHECK_MESSAGE(! indiCamera->isConnected(), "Expected camera to be disconnected within 2 seconds.");
  indiCamera->connect(2000);
  BOOST_CHECK_MESSAGE(indiCamera->isConnected(), "Expected camera to be connected within 2 seconds.");

  // Get chip details
  DimensionT maxResolution = indiCamera->getMaxResolution();
  DimensionT pixelSize = indiCamera->getPixelSize();
  unsigned int bitsPerPixel = indiCamera->getBitsPerPixel();
  FrameT selectedFrame = indiCamera->getFrame();
  BinningT binning = indiCamera->getBinning();
  bool isCompressed = indiCamera->isCompressed();
  FrameTypeT::TypeE frameType = indiCamera->getFrameType();
  double exposureTime = indiCamera->getExposureTime(); /* sec */

  // Output dimension test
  cerr << "maxResolution: " << maxResolution << endl;
  cerr << "pixelSize: " << pixelSize << endl;
  cerr << "bitsPerPixel: " << bitsPerPixel << endl;
  cerr << "selectedFrame: " << selectedFrame << endl;
  cerr << "binning: " << binning << endl;
  cerr << "isCompressed: " << isCompressed << endl;
  cerr << "frameType: " << FrameTypeT::asStr(frameType) << endl;
  cerr << "exposureTime: " << exposureTime << endl;

  // Check
  BOOST_CHECK_MESSAGE(maxResolution.get<0>() == 1280 && maxResolution.get<1>() == 1024, "Expected maxResolution == 1280 x 1024.");
  BOOST_CHECK_MESSAGE(rough_eq(pixelSize.get<0>(), 5.2, 1.0e-6) && rough_eq(pixelSize.get<1>(), 5.2, 1.0e-6), "Expected pixelSize == 5.2 x 5.2.");
  BOOST_CHECK_MESSAGE(bitsPerPixel == 16, "Expected bitsPerPixel == 16.");
  BOOST_CHECK_MESSAGE(selectedFrame == FrameT(0, 0, 1280, 1024), "Expected frame to be (0, 0, 1280, 1024).");
  BOOST_CHECK_MESSAGE(binning == BinningT(1, 1), "Expected binning to be (1, 1).");
  BOOST_CHECK_MESSAGE(isCompressed, "Expected isCompressed to be 1.");
  BOOST_CHECK_MESSAGE(frameType == FrameTypeT::LIGHT, "Expected frameType == FrameTypeT::LIGHT.");
  BOOST_CHECK_MESSAGE(! exposureTime, "Expected exposureTime to be 0.");

  // Disconnect
  indiCamera->disconnect(2000);
  BOOST_CHECK_MESSAGE(! indiCamera->isConnected(), "Expected device to be disconnected within 2 seconds.");
  
  delete newIndiClient;
}


/**
 * Get camera chip details.
 */
BOOST_AUTO_TEST_CASE(IndiCameraSetValues)
{
  IndiClientT * newIndiClient = new IndiClientT("localhost", 7624 /* port */, true /* AutoConnect */);
  
  BOOST_CHECK(newIndiClient);
  BOOST_CHECK_MESSAGE(newIndiClient->isConnected(), "Expected client to be connected to server.");

  IndiCameraT * indiCamera = newIndiClient->getCamera("CCD Simulator");
  BOOST_CHECK_MESSAGE(indiCamera, "Expect indiCamera to be set.");  

  // Check connect
  indiCamera->disconnect(2000);
  BOOST_CHECK_MESSAGE(! indiCamera->isConnected(), "Expected camera to be disconnected within 2 seconds.");
  indiCamera->connect(2000);
  BOOST_CHECK_MESSAGE(indiCamera->isConnected(), "Expected camera to be connected within 2 seconds.");

  // Frame set
  FrameT oldFrame = indiCamera->getFrame();
  cerr << "oldFrame: " << oldFrame << endl;
  indiCamera->setFrame(10, 20, 150, 180);

  FrameT newFrame = indiCamera->getFrame();
  cerr << "newFrame: " << newFrame << endl;

  BOOST_CHECK_MESSAGE(newFrame == FrameT(10, 20, 150, 180), "Expected newFrame == FrameT(10, 20, 150, 180).");
  indiCamera->setFrame(oldFrame);
  FrameT newFrame2 = indiCamera->getFrame();
  BOOST_CHECK_MESSAGE(newFrame2 == oldFrame, "Expected newFrame2 == oldFrame.");

  // Binning set
  BinningT oldBinning = indiCamera->getBinning();
  indiCamera->setBinning(BinningT(2, 2));
  BinningT newBinning = indiCamera->getBinning();
  BOOST_CHECK_MESSAGE(newBinning == BinningT(2, 2), "Expected newBinning == BinningT(2, 2).");
  indiCamera->setBinning(oldBinning);
  usleep(10000);
  BinningT newBinning2 = indiCamera->getBinning();
  cerr << "oldBinning: " << oldBinning << ", newBinning2: " << newBinning2 << endl;
  BOOST_CHECK_MESSAGE(newBinning2 == oldBinning, "Expected newBinning2 == oldBinning.");

  // Compression
  bool oldCompression = indiCamera->isCompressed();
  indiCamera->setCompressed(! oldCompression);
  bool newCompression = indiCamera->isCompressed();
  BOOST_CHECK_MESSAGE(newCompression == ! oldCompression, "Expected newCompression == ! oldCompression.");
  indiCamera->setCompressed(oldCompression);
  bool newCompression2 = indiCamera->isCompressed();
  BOOST_CHECK_MESSAGE(newCompression2 == oldCompression, "Expected newCompression2 == oldCompression.");

  // Frame type
  FrameTypeT::TypeE oldFrameType = indiCamera->getFrameType();
  indiCamera->setFrameType(FrameTypeT::BIAS);
  FrameTypeT::TypeE newFrameType = indiCamera->getFrameType();
  BOOST_CHECK_MESSAGE(newFrameType == FrameTypeT::BIAS, "Expected newFrameType == FrameTypeT::BIAS.");
  indiCamera->setFrameType(oldFrameType);
  FrameTypeT::TypeE newFrameType2 = indiCamera->getFrameType();
  BOOST_CHECK_MESSAGE(newFrameType2 == oldFrameType, "Expected newFrameType2 == oldFrameType.");

  // Disconnect
  indiCamera->disconnect(2000);
  BOOST_CHECK_MESSAGE(! indiCamera->isConnected(), "Expected device to be disconnected within 2 seconds.");
  
  delete newIndiClient;
}


/**
 * Connect / disconnect INDI device.
 */
BOOST_AUTO_TEST_CASE(IndiCameraSpecialGetBlobTests)
{
  IndiClientT * newIndiClient = new IndiClientT("localhost", 7624 /* port */, true /* AutoConnect */);
  
  BOOST_CHECK(newIndiClient);
  BOOST_CHECK_MESSAGE(newIndiClient->isConnected(), "Expected client to be connected to server.");

  IndiCameraT * indiCamera = newIndiClient->getCamera("CCD Simulator");
  BOOST_CHECK_MESSAGE(indiCamera, "Expect indiCamera to be set.");  

  // Check connect
  indiCamera->disconnect(2000);
  BOOST_CHECK_MESSAGE(! indiCamera->isConnected(), "Expected camera to be disconnected within 2 seconds.");
  indiCamera->connect(2000);
  BOOST_CHECK_MESSAGE(indiCamera->isConnected(), "Expected camera to be connected within 2 seconds.");

  // Get blob without taking an image multiple times...
  CImg<float> img1, img2, img3;
  BOOST_CHECK_THROW(indiCamera->getImage(& img1), IndiCameraNoImageAvailableExceptionT);
  BOOST_CHECK_THROW(indiCamera->getImage(& img1), IndiCameraNoImageAvailableExceptionT);
  BOOST_CHECK_THROW(indiCamera->getImage(& img1), IndiCameraNoImageAvailableExceptionT);

  // Take a picture...
  const DimensionT maxResolution = indiCamera->getMaxResolution();
  const double cExpTimeSec = 1;
  const FrameT fullFrame(0, 0, maxResolution.get<0>(), maxResolution.get<1>());
  const BinningT binning(1, 1);
  FrameTypeT::TypeE frameType = FrameTypeT::LIGHT;
  bool compressed = false;
  indiCamera->takePicture(& img1, cExpTimeSec, fullFrame, frameType, binning, compressed);
  indiCamera->getImage(& img2);
  indiCamera->getImage(& img3);

  // Check
  BOOST_CHECK_MESSAGE(img1 == img2, "Expect img1 == img2.");
  BOOST_CHECK_MESSAGE(img2 == img3,  "Expect img2 == img3.");

  // Disconnect
  indiCamera->disconnect(2000);
  BOOST_CHECK_MESSAGE(! indiCamera->isConnected(), "Expected device to be disconnected within 2 seconds.");

  delete newIndiClient;
}


/**
 * Set invalid values to camera properties.
 */
BOOST_AUTO_TEST_CASE(IndiCameraSetInvalidValues)
{
  IndiClientT * newIndiClient = new IndiClientT("localhost", 7624 /* port */, true /* AutoConnect */);
  
  BOOST_CHECK(newIndiClient);
  BOOST_CHECK_MESSAGE(newIndiClient->isConnected(), "Expected client to be connected to server.");

  IndiCameraT * indiCamera = newIndiClient->getCamera("CCD Simulator");
  BOOST_CHECK_MESSAGE(indiCamera, "Expect indiCamera to be set.");  

  // Check connect
  indiCamera->disconnect(2000);
  BOOST_CHECK_MESSAGE(! indiCamera->isConnected(), "Expected camera to be disconnected within 2 seconds.");
  indiCamera->connect(2000);
  BOOST_CHECK_MESSAGE(indiCamera->isConnected(), "Expected camera to be connected within 2 seconds.");

  const DimensionT maxResolution = indiCamera->getMaxResolution();
  const int delta = 10;
  const FrameT tooBigFrame(-delta, -delta, maxResolution.get<0>() + delta, maxResolution.get<1>() + delta);
  FrameT oldFrame = indiCamera->getFrame();
  BOOST_CHECK_THROW(indiCamera->setFrame(tooBigFrame), PropInvalidValueExceptionT);
  FrameT newFrame = indiCamera->getFrame();
  BOOST_CHECK_MESSAGE(oldFrame == newFrame, "Expect oldFrame == newFrame.");

  // Disconnect
  indiCamera->disconnect(2000);
  BOOST_CHECK_MESSAGE(! indiCamera->isConnected(), "Expected device to be disconnected within 2 seconds.");
  
  delete newIndiClient;
}


/**
 * Get camera chip details.
 */
BOOST_AUTO_TEST_CASE(IndiCameraTakePictures)
{
  IndiClientT * newIndiClient = new IndiClientT("localhost", 7624 /* port */, true /* AutoConnect */);
  
  BOOST_CHECK(newIndiClient);
  BOOST_CHECK_MESSAGE(newIndiClient->isConnected(), "Expected client to be connected to server.");

  IndiCameraT * indiCamera = newIndiClient->getCamera("CCD Simulator");
  BOOST_CHECK_MESSAGE(indiCamera, "Expect indiCamera to be set.");  

  // Check connect
  indiCamera->disconnect(2000);
  BOOST_CHECK_MESSAGE(! indiCamera->isConnected(), "Expected camera to be disconnected within 2 seconds.");
  indiCamera->connect(2000);
  BOOST_CHECK_MESSAGE(indiCamera->isConnected(), "Expected camera to be connected within 2 seconds.");


  const DimensionT maxResolution = indiCamera->getMaxResolution();
  const double cExpTimeSec = 0.1;
  const FrameT fullFrame(0, 0, maxResolution.get<0>(), maxResolution.get<1>());

  FrameTypeT::TypeE frameType = FrameTypeT::LIGHT;
  bool compressed = false;

  // TODO: What is max image size in case of binning > 1?
  struct RecordInfoT {
    BinningT binning;
    FrameT frame;
    bool compressed;
    RecordInfoT(const BinningT & inBinning, const FrameT & inFrame, bool inCompressed) : binning(inBinning), frame(inFrame), compressed(inCompressed) {}
  };

  RecordInfoT recordInfos[] = {
    // Full frame with and without binning
    RecordInfoT(BinningT(1,1), FrameT(0, 0, 1280, 1024) /* Take frame */, false),
    RecordInfoT(BinningT(2,2), FrameT(0, 0, 1280, 1024) /* Take frame */, false),
    RecordInfoT(BinningT(3,3), FrameT(0, 0, 1280, 1024) /* Take frame */, false),
    RecordInfoT(BinningT(4,4), FrameT(0, 0, 1280, 1024) /* Take frame */, false),

    // Sub frame with and without binning
    RecordInfoT(BinningT(1,1), FrameT(50, 100, 25, 35) /* Take frame */, false),
    RecordInfoT(BinningT(2,2), FrameT(50, 100, 25, 35) /* Take frame */, false),
    RecordInfoT(BinningT(3,3), FrameT(50, 100, 25, 35) /* Take frame */, false),
    RecordInfoT(BinningT(4,4), FrameT(50, 100, 25, 35) /* Take frame */, false),

    // Different binnings
    RecordInfoT(BinningT(2,1), FrameT(0, 0, 1280, 1024) /* Take frame */, false),
    RecordInfoT(BinningT(1,2), FrameT(0, 0, 1280, 1024) /* Take frame */, false),
    RecordInfoT(BinningT(3,1), FrameT(0, 0, 1280, 1024) /* Take frame */, false),
    RecordInfoT(BinningT(1,3), FrameT(0, 0, 1280, 1024) /* Take frame */, false),
    RecordInfoT(BinningT(3,2), FrameT(0, 0, 1280, 1024) /* Take frame */, false),
    RecordInfoT(BinningT(2,3), FrameT(0, 0, 1280, 1024) /* Take frame */, false),

    // Different binnings without full frame
    RecordInfoT(BinningT(2,1), FrameT(50, 100, 25, 35) /* Take frame */, false),
    RecordInfoT(BinningT(1,2), FrameT(50, 100, 25, 35) /* Take frame */, false),
    RecordInfoT(BinningT(3,1), FrameT(50, 100, 25, 35) /* Take frame */, false),
    RecordInfoT(BinningT(1,3), FrameT(50, 100, 25, 35) /* Take frame */, false),
    RecordInfoT(BinningT(2,3), FrameT(50, 100, 25, 35) /* Take frame */, false),
    RecordInfoT(BinningT(3,2), FrameT(50, 100, 25, 35) /* Take frame */, false)
  };

  size_t numElements = sizeof(recordInfos) / sizeof(recordInfos[0]);

  for (size_t i=0; i < numElements; ++i) {
    CImg<float> img;
    const RecordInfoT & recordInfo = recordInfos[i];

    float expectedWidth = recordInfo.frame.get<2>() /* w */ / recordInfo.binning.get<0>();
    float expectedHeight = recordInfo.frame.get<3>() /* h */ / recordInfo.binning.get<1>();

    for (size_t j = 0; j < FrameTypeT::_Count; ++j) {
      FrameTypeT::TypeE ft = static_cast<FrameTypeT::TypeE>(j);
      // cerr << "Taking picture " << FrameTypeT::_Count * i + j + 1 << "/" << FrameTypeT::_Count * numElements << "...(frame: " << recordInfo.frame
      // 	   << ", binning: " << recordInfo.binning << ", compressed: " << (compressed ? "yes" : "no")
      // 	   << ", frame type: " << FrameTypeT::asStr(ft) << ")"
      // 	   << " --> expected image size(w: " << expectedWidth << ", h: " << expectedHeight << ")" << endl;
      
      indiCamera->takePicture(& img, cExpTimeSec, recordInfo.frame, ft, recordInfo.binning, recordInfo.compressed);

      //cerr << " --> img.width(): " << img.width() << ", img.height(): " << img.height() << endl;
      BOOST_CHECK_MESSAGE(img.width() == expectedWidth  && img.height() == expectedHeight, "Expected image size does not match.");
      //img.display();
    } // end for
  }

  // Disconnect
  indiCamera->disconnect(2000);
  BOOST_CHECK_MESSAGE(! indiCamera->isConnected(), "Expected device to be disconnected within 2 seconds.");
  
  delete newIndiClient;
}


/**
 * Abort exposure.
 */
BOOST_AUTO_TEST_CASE(IndiCameraStartAndAbortExposure)
{
  IndiClientT * newIndiClient = new IndiClientT("localhost", 7624 /* port */, true /* AutoConnect */);
  
  BOOST_CHECK(newIndiClient);
  BOOST_CHECK_MESSAGE(newIndiClient->isConnected(), "Expected client to be connected to server.");

  IndiCameraT * indiCamera = newIndiClient->getCamera("CCD Simulator");
  BOOST_CHECK_MESSAGE(indiCamera, "Expect indiCamera to be set.");  

  // Check connect
  indiCamera->disconnect(2000);
  BOOST_CHECK_MESSAGE(! indiCamera->isConnected(), "Expected camera to be disconnected within 2 seconds.");
  indiCamera->connect(2000);
  BOOST_CHECK_MESSAGE(indiCamera->isConnected(), "Expected camera to be connected within 2 seconds.");

  cerr << "Starting exposure..." << endl;
  indiCamera->startExposure(30);
  cerr << "Exposure in progress..." << endl;
  usleep(3000000 /* 3 sec. */);
  BOOST_CHECK_MESSAGE(indiCamera->isExposureInProgress(), "Expexting isExposureInProgress() == true.");

  cerr << "Aborting exposure..." << endl;
  indiCamera->abortExposure();
  cerr << "Exposure aborted." << endl;
  usleep(3000000 /* 3 sec. */); // sleep is required since ABORT property of CCD Simulator leaves BUSY state before abort is complete...
  BOOST_CHECK_MESSAGE(! indiCamera->isExposureInProgress(), "Expexting isExposureInProgress() == false.");

  // Disconnect
  indiCamera->disconnect(2000);
  BOOST_CHECK_MESSAGE(! indiCamera->isConnected(), "Expected device to be disconnected within 2 seconds.");
  
  delete newIndiClient;
}


/**
 * Shutter test.
 */
// TODO: Maybe better look at http://stackoverflow.com/questions/6425056/boost-test-library-run-suite-twice-with-different-parameters
// !!!
// Fixtures..


// void test_mask( int arg, int mask ) {
//     BOOST_MESSAGE( arg << " " << mask );

//     BOOST_CHECK( (arg & mask) != 0 );
// }

// struct sub_test_suite : public test_suite {
//   sub_test_suite() {
//     parameters_list.push_back( 1 );
//     parameters_list.push_back( 5 );
//     parameters_list.push_back( 6 );
//     parameters_list.push_back( 7 );
//     parameters_list.push_back( 140 );		
		
//     add( BOOST_PARAM_TEST_CASE( boost::function1<void,int>( bind( &test_mask, _1, 0x80 ) ), parameters_list.begin(), parameters_list.end() ) );
//   }
//   std::list<int> parameters_list;
// }




BOOST_AUTO_TEST_CASE(IndiCameraShutterTest)
{
  IndiClientT * newIndiClient = new IndiClientT("localhost", 7624 /* port */, true /* AutoConnect */);
  
  BOOST_CHECK(newIndiClient);
  BOOST_CHECK_MESSAGE(newIndiClient->isConnected(), "Expected client to be connected to server.");

  for (size_t i=0; i < sizeof(cameras) / sizeof(cameras[0]); ++i) {
    IndiCameraT * indiCamera = newIndiClient->getCamera(cameras[i]);
    BOOST_CHECK_MESSAGE(indiCamera, "Expect indiCamera to be set (name='" << cameras[i] << "'.");  

    // Check connect
    indiCamera->disconnect(2000);
    BOOST_CHECK_MESSAGE(! indiCamera->isConnected(), "Expected camera to be disconnected within 2 seconds.");
    indiCamera->connect(2000);
    BOOST_CHECK_MESSAGE(indiCamera->isConnected(), "Expected camera to be connected within 2 seconds.");


    cerr << "Current shutter state: " << (indiCamera->isShutterOpen() ? "open" : "closed") << endl;
    cerr << "Opening shutter..." << endl;
    indiCamera->setShutterOpen(true);
    BOOST_CHECK_MESSAGE(indiCamera->isShutterOpen(), "Expected shutter being open.");
    
    usleep(1000000); /* 1sec */
    
    cerr << "Closing shutter..." << endl;
    indiCamera->setShutterOpen(false);
    BOOST_CHECK_MESSAGE(! indiCamera->isShutterOpen(), "Expected shutter being closed.");

    // Disconnect
    indiCamera->disconnect(2000);
    BOOST_CHECK_MESSAGE(! indiCamera->isConnected(), "Expected device to be disconnected within 2 seconds.");
  }

  
  delete newIndiClient;
}

BOOST_AUTO_TEST_SUITE_END()
