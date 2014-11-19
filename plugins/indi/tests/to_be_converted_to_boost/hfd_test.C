/**
 Small Qt-Program to test the Hfd calculation and the HfdPanel
*/

// Qt Stuff
#include <Qt/qapplication.h>
#include <Qt/qmainwindow.h>

#include "ui/hfd_panel.hpp"

#include <iostream>
#include <CImg.h>
#include <CCfits/CCfits>

#include "hfd.hpp"
#include "fwhm.hpp"

using namespace std;
using namespace cimg_library;
using namespace CCfits;


// Qt helper function
void fitWindow(QMainWindow & stretchWindow, QWidget * stretchWidgetL) {
  stretchWindow.setCentralWidget(stretchWidgetL);
  
  QSizePolicy sizePolicy1(QSizePolicy::Expanding, QSizePolicy::Expanding);
  sizePolicy1.setHorizontalStretch(100);
  sizePolicy1.setVerticalStretch(100);
  sizePolicy1.setHeightForWidth(true);
  stretchWidgetL->setSizePolicy(sizePolicy1);
  
  stretchWindow.setMinimumSize(stretchWidgetL->width(), stretchWidgetL->height());
  stretchWindow.setGeometry(0,0, stretchWidgetL->width(), stretchWidgetL->height());
}



// CCfits helper function
// See http://heasarc.gsfc.nasa.gov/fitsio/ccfits/html/cookbook.html
bool readFile(CImg<float> & cimg, const string & inFilename, bool inVerboseMode = false) {
  FITS::setVerboseMode(inVerboseMode);
  try {
    std::auto_ptr<FITS> pInfile(new FITS(inFilename, Read, true));
    PHDU& image = pInfile->pHDU(); 
    
    // read all user-specifed, coordinate, and checksum keys in the image
    image.readAllKeys();

    if (inVerboseMode) {
      cout << image << endl;
    }

    // Set image dimensions
    cimg.resize(image.axis(0) /*x*/, image.axis(1) /*y*/, 1/*z - HACK*/, 1 /*1 color*/);

    // HACK / FIXME: At this point we assume that there is only 1 layer!
    std::valarray<unsigned long> imgData;
    image.read(imgData);

    // For now we create a copy... maybe there is a better way to directly read data into CImg, later...
    cimg_forXY(cimg, x, y) { cimg(x, cimg.height() - 1 - y) = imgData[cimg.offset(x, y)]; }

  } catch (FitsException&) {
    // will catch all exceptions thrown by CCfits, including errors found by cfitsio (status != 0)
    return false;
  }
  return true;
}


int main(int argc, char **argv) {
  if (argc < 2) {
    cerr << "Usage: hdf_test <img_filename>" << endl;
    return 1;
  }

  // Init the test app
  QApplication app(argc, argv);
  QMainWindow mainWindow;
  mainWindow.setGeometry(0,0, 300, 300);
  mainWindow.show();

  cout << "Opening image " << argv[1] << endl;

  // Load the test image
  CImg<float> image(10, 10, 1 /*HACK: z*/, 1);
  if(! readFile(image, argv[1], true /*verbose mode*/)) {
    cerr << "Error oepning FITS file." << endl;
    return 1;
  }

  // Do the HFD calculation  
  const unsigned int outerRadius = 15;
  HfdT hfd(image, outerRadius /*radius px*/, true);
  cout << "HFD calculated: " << hfd.getValue() << endl;

  //TODO: Calculate from pixels to arcsec?! --> see FWHM
  cout << "HFD['']" << FwhmT::pxToArcsec(hfd.getValue(), 1000.0 /* F mm*/, 5.4 /*pixelSize um*/, /*binningXY*/ 1) << endl;

  // DEBUG START
  // CImg<float> resImg = hfd.getResultImage();
  // CImgDisplay disp3(resImg, "resImg" );
  // while(! disp3.is_closed()) { CImgDisplay::wait(disp3); }
  // DGB END




  // Display the result
  HfdPanel hfdPanel(& mainWindow);
  hfdPanel.setGeometry(0,0, 300, 300);

  unsigned char centroidColor[] = { 0, 0, 255 };
  hfdPanel.setCentroidColor(centroidColor);

  unsigned char outerRingdColor[] = { 255, 255, 0 };
  hfdPanel.setOuterRingColor(outerRingdColor);

  hfdPanel.setHfd(hfd);
  hfdPanel.replot();

  fitWindow(mainWindow, & hfdPanel);
  app.exec();
  
  return 0;
}     
