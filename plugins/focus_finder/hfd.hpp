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

/**
   Get all pixels inside a radius: http://stackoverflow.com/questions/14487322/get-all-pixel-array-inside-circle
   Algorithm: http://en.wikipedia.org/wiki/Midpoint_circle_algorithm
   HDF calculation: http://www005.upp.so-net.ne.jp/k_miyash/occ02/halffluxdiameter/halffluxdiameter_en.html

   Display histogram of original image
   See http://cimg.sourceforge.net/reference/structcimg__library_1_1CImg.html#ab102f4505a6fc92565367bf0ab91e655
   CImg<float> imageCopy(image);
   const CImg<float> imgHist = imageCopy.histogram(65535);
   imgHist.display_graph(0,3);
   TODO: Display histogram in Qt?!

   Equalize image (required? useful?)
   const CImg<float> img("reference.jpg"), res = img.get_equalize(256);
   (img,res).display();


   Remove noise - gaussian blur
   CImg <float> imageBlur(image);
   imageBlur.blur(15);
   cimg_forXY(image, x, y) {
   if (image(x, y) > imageBlur(x,y)) {
   image(x, y) = image(x,y) - imageBlur(x,y);
   } else {
   image(x, y) = 0;
   }
   }

   // Apply threshold to image
   const float th = 40; // TODO: Determine automatically
   cimg_forXY(image, x, y) {
   if (image(x,y) < th) {
   image(x,y) = 0;
   }
   }

   Edge detection
   const CImg<float> maskX(3,3,1,1, 1,0,-1, 2,0,-2,1,0,-1), maskY = maskX.get_transpose(), imgX = image.get_convolve(maskX), imgY = image.get_convolve(maskY);
   (imgX,imgY).display("ImgX, imgY");
   cimg_forXY(image, x, y) {
   image(x, y, 0) = sqrt( imgX(x,y,0) * imgX(x,y,0) + imgY(x,y,0) * imgY(x,y,0) );
   }
   image.normalize(0, 255).save("sobel_effect.jpg");
   return;

   Calculate COM - Center Of Mass
   unsigned int xcom, ycom;
   calcCenterOfMass(image, & xcom, & ycom);
*/

#ifndef _HFD_HPP_
#define _HFD_HPP_ _HFD_HPP_

#include <assert.h>
#include <CImg.h>
#include "centroid.hpp"


  //TODO: Move radius to last position as parameter, pass / calc default?! Pass in constructor at all?!, use / calc good default instead, allow setting it explicitely by set function... and allow access by get function... can default be assigned statically at all?! Or does it depend on image width?! --> No, not for stars!!!

class HfdT {
private:
  CImg<float> mResImage;
  map<size_t, double> mIntegralsByRadius;
  float mHfdValue;
  float mXCom, mYCom;
  float mRadius;

  // Example code from: http://stackoverflow.com/questions/14487322/get-all-pixel-array-inside-circle
  // TODO: We may move this to a "utility" function
  static bool insideCircle(int x /*pos of x*/, int y /*pos of y*/, int centerX, int centerY, unsigned int radius) {
    return (pow(x - centerX, 2.0) + pow(y - centerY, 2.0) <= pow(radius, 2.0));
  }

public:
  HfdT() : mHfdValue(0), mXCom(0), mYCom(0) {}
  HfdT(const CImg<float> & image, unsigned int radius, float inCenterX = -1, float inCenterY = -1, size_t inSizePx = 0) : mHfdValue(0), mXCom(0), mYCom(0) {
    this->set(image, radius, inCenterX, inCenterY, inSizePx);
  }

  void set(const CImg<float> & image, unsigned int radius, float inCenterX = -1, float inCenterY = -1, size_t inSizePx = 0) {
    // TODO / FIXME: Check the radius - ok?!
    float minImgDim = std::min(image.width(), image.height());
    mRadius = (2.0 * radius < minImgDim ? radius : floor(minImgDim / 2.0));
    //cerr << "HfdT() - minImgDim: " << minImgDim << ", radius: " << radius << ", mRadius: " << mRadius << endl;
    
    // DGB START
    //CImgDisplay disp1(image, "image" );
    //while(! disp1.is_closed()) { CImgDisplay::wait(disp1); }
    // DGB END

    // TODO: Can we avoid image copy??!
    //CImg<float> resImage;

    if (inCenterX < 0 || inCenterY < 0) {
      // // Calculate centroid
      //float cx, cy; // ignore
      //CentroidCalcT::starCentroid(image, & cx, & cy, & mResImage, true);
      mXCom = image.width() / 2;
      mYCom = image.height() / 2;
      mResImage = image;
      //cout << "HFD Centroid - x: " << mXCom << ", y: " << mYCom << endl;
    } else {
      if (! inSizePx) {
	cerr << "PLEASE SPECIFY size!" << endl;
	exit(1); // TODO: assert... / throw!
      }
      //cout << "inSizePx: " << inSizePx << endl;

      const int BACK = ceil(inSizePx / 2.0) - 1;
      int startX = inCenterX - BACK, startY = inCenterY - BACK;
      //cerr << "########## HFD WND - x: " << (startX + 1) << ", y: " << (startY + 1) << ", w: " << inSizePx << ", h: " << inSizePx << endl;

      //cerr << "############################################## MODIFYING HFD - mResImage: " << & mResImage << flush;
      mResImage = image.get_crop(startX+1, startY+1, startX + inSizePx, startY + inSizePx);
      //cerr << "DONE: &mResImage: " << &mResImage << endl;

      // DGB START
      //CImgDisplay disp7(image, "image - HFD image before copying to mResImage..." );
      //while(! disp7.is_closed()) { CImgDisplay::wait(disp7); }
      // DGB END

      // DGB START
      //CImgDisplay disp8(mResImage, "mResImage -  HFD image AFTER copying to mResImage..." );
      //while(! disp8.is_closed()) { CImgDisplay::wait(disp8); }
      // DGB END

      mXCom = BACK; // TODO: better solution?!
      mYCom = BACK;	
    }

    //CentroidCalcT::starCentroid(image, & mXCom, & mYCom, & mResImage);

    // Subtract median image
    double med = mResImage.median();
    cimg_forXY(mResImage, x, y) {
      mResImage(x, y) = (mResImage(x, y) > med ? mResImage(x, y) - med : 0);
    }
    
    //cerr << "Centroid - mXCom: " << mXCom << ", mYCom: " << mYCom << ", mRadius: " << mRadius
    //     << ", mResImage.width(): " << mResImage.width() << ", mResImage.height(): " << mResImage.height() << endl;
 
    // TODO: We may extend this to subpixel accuracy... Currently the accuracy is limited by the insideCircle function.
    mIntegralsByRadius.clear();

    for (size_t r = 1; r <= mRadius; ++r) {
      cimg_forXY(mResImage, x, y) {
    	if (insideCircle(x, y, mXCom, mYCom, r)) {
	  mIntegralsByRadius[r] += mResImage(x, y);
	  // rgbImg(x, y, 0, 0 /*r*/) = 255;
	  // rgbImg(x, y, 0, 1 /*g*/) = 0;
	  // rgbImg(x, y, 0, 2 /*b*/) = 0;
	}
      }

      // CImgDisplay disp3(rgbImg, "resImg");
      // while(! disp3.is_closed()) {
      // 	CImgDisplay::wait(disp3);
      // }

    }
    
    // cerr << "MAP SIZE: " << mIntegralsByRadius.size() << endl;
    // cerr << "------------------------------------- integralsByRadius -------------------------------------" << endl;
    // for (map<size_t, double>::const_iterator it = mIntegralsByRadius.begin(); it != mIntegralsByRadius.end(); ++it) {
    //   cerr << "> radius: " << it->first << ", flux: " << it->second << endl;
    // }

    // Linear interpolation
    mHfdValue = 2.0 * getHalfFluxRadiusLinInterp(); 
  }

  inline bool valid() const { return (mResImage.width() > 0 && mResImage.height() > 0); }


  // TODO: set new Image...?!
  inline float getValue() const { return mHfdValue; }
  inline void getCentroid(float * xcom, float * ycom) const { *xcom = mXCom; *ycom = mYCom; }

  //BUG - mResImage is empty!!! Why is that?!?!?!?!
  inline const CImg<float> & getResultImage() const { return mResImage; }
  inline float getRadius() const { return mRadius; }
  const map<size_t, double> & getHfdDist() const { return mIntegralsByRadius; }

  float getHalfFluxRadiusLinInterp() const {
    double halfFlux = mIntegralsByRadius.rbegin()->second /*total flux*/ / 2.0;

    // TODO: Better solution?
    size_t rHalfFit = 0;
    double fitFlux = 0;
    for (map<size_t, double>::const_iterator rIt = mIntegralsByRadius.begin(); rIt != mIntegralsByRadius.end(); ++rIt) {
      if (rIt->second >= halfFlux) {
    	rHalfFit = rIt->first;
    	fitFlux = rIt->second;
    	break;
      }
    }
    double halfFluxRadiusLinInterp = rHalfFit * halfFlux / fitFlux; 
    //cerr << "halfFlux: " << halfFlux << ", fitFlux: " << fitFlux << ", rHalfFit: " << rHalfFit << ", halfFluxRadiusLinInterp: " << halfFluxRadiusLinInterp << endl;

    return halfFluxRadiusLinInterp;
  }
};

#endif // _HFD_HPP_
