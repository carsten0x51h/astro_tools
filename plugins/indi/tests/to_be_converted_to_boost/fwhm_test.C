/**
 Small Qt-Program to test FWHM calculation and the FWHM Panel
*/

// Qt Stuff
#include <Qt/qapplication.h>
#include <Qt/qmainwindow.h>

#include "ui/fwhm_panel.hpp"

#include <iostream>
#include <CImg.h>
#include <CCfits/CCfits>

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
  QMainWindow horzFwhmWindow, vertFwhmWindow;
  horzFwhmWindow.setGeometry(0,0, 200, 200);
  vertFwhmWindow.setGeometry(0,0, 200, 200);
  horzFwhmWindow.show();
  vertFwhmWindow.show();

  cout << "Opening image " << argv[1] << endl;

  // Load the test image
  CImg<float> image(10, 10, 1 /*HACK: z*/, 1);
  if(! readFile(image, argv[1], true /*verbose mode*/)) {
    cerr << "Error oepning FITS file." << endl;
    return 1;
  }
  
  // TODO: Reduce resolution of image to fixed size (e.g. 20x20 or 16x16...)

  // Do the FWHM calculation  
  FwhmT fwhmHorz(image, FwhmT::DirectionT::HORZ);
  FwhmT fwhmVert(image, FwhmT::DirectionT::VERT);

  // TODO: Move somewhere else...
  double fwhmHorzArcsec = FwhmT::pxToArcsec(fwhmHorz.getValue(), 1000.0 /* F mm*/, 5.4 /*pixelSize um*/, /*binningXY*/ 1);
  double fwhmVertArcsec = FwhmT::pxToArcsec(fwhmVert.getValue(), 1000.0 /* F mm*/, 5.4 /*pixelSize um*/, /*binningXY*/ 1);

  cerr << "horzFwhm: " << fwhmHorz.getValue() << "[px] = " << fwhmHorzArcsec << "['']" << ", MSE: " << fwhmHorz.getStandardDeviation() << endl;
  cerr << "vertFwhm: " << fwhmVert.getValue() << "[px] = " << fwhmVertArcsec << "['']" << ", MSE: " << fwhmVert.getStandardDeviation() << endl;


  float centerx, centery;
  fwhmHorz.getCentroid(& centerx, & centery);

  cerr << "Centroid (x,y) = (" << centerx << ", " << centery << ")" << endl;

  // Display the result
  FwhmPanel horzFwhmPanel(& horzFwhmWindow);
  horzFwhmPanel.setGeometry(0, 0, 200, 200);
  horzFwhmPanel.setFwhm(fwhmHorz);
  horzFwhmPanel.replot();
  fitWindow(horzFwhmWindow, & horzFwhmPanel);

  FwhmPanel vertFwhmPanel(& vertFwhmWindow);
  vertFwhmPanel.setGeometry(0, 0, 200, 200);
  vertFwhmPanel.setFwhm(fwhmVert);
  vertFwhmPanel.replot();
  fitWindow(vertFwhmWindow, & vertFwhmPanel);

  app.exec();
  
  return 0;
}     
