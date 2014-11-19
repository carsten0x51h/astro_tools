#include <iostream>
#include <CImg.h>
#include <CCfits/CCfits>

#include "centroid.hpp"

using namespace std;
using namespace cimg_library;
using namespace CCfits;


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
  // Load the test image
  CImg<float> image(10, 10, 1 /*HACK: z*/, 1);
  if(! readFile(image, "../test_data/star10.fits", true /*verbose mode*/)) {
    cerr << "Error oepning FITS file." << endl;
    return 1;
  }

  CImg<float> resImage;
  float xcenter, ycenter;
  CentroidCalcT::starCentroid(image, & xcenter, & ycenter, & resImage);

  cerr << "Centroid - x: " << xcenter << ", y: " << ycenter << endl;


  // Copy result image to rgb image 
  CImg<unsigned char> rgbImg(resImage.width(), resImage.height(), 1 /*z*/, 3);

  // TODO: Improve...... required?? Or directly equalize / norm image?! --> CImg function...
  double maxValue = 0;
  cimg_forXY(resImage, x, y) {
    if (maxValue < resImage(x,y)) {
      maxValue = resImage(x,y);
    }
  }
  
  cimg_forXY(resImage, x, y) {
    rgbImg(x, y, 0, 0 /*r*/) = 255.0 / maxValue * resImage(x, y);
    rgbImg(x, y, 0, 1 /*g*/) = 255.0 / maxValue * resImage(x, y);
    rgbImg(x, y, 0, 2 /*b*/) = 255.0 / maxValue * resImage(x, y);
  }
  
  // Mark centroid by cross
  const unsigned int lineLength = 4;
  unsigned char crossColor[] = {255,0,0};
  rgbImg.draw_line(xcenter - lineLength / 2, ycenter, xcenter + lineLength / 2, ycenter, crossColor, 1 /*opacity*/);
  rgbImg.draw_line(xcenter, ycenter - lineLength / 2, xcenter, ycenter + lineLength / 2, crossColor, 1 /*opacity*/);
  
  
  CImgDisplay disp1(rgbImg, "Img1" );
  while(! disp1.is_closed()) {
    CImgDisplay::wait(disp1);
  }
  
  return 0;
}     
