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
   http://www.astro-imaging.com/Tutorial/MatchingCCD.html --> FWHM = 1.02 * (wavelength) * (Focal Ratio)
   http://www.stanmooreastro.com/pixel_size.htm
   --> As theory predicts, there is actually slight resolution degradation when FWHM = 3.0 pixels and significant degradation when FWHM = 1.5 pixels.
   --> As discussed above, typical amateur equipment and conditions can be expected to produce images with FWHM between 2.0 and 3.5 arcsec. 
   --> Thus a pixel size of 0.5 to 1.5 arcsec can be considered ideal for optimizing resolution.

   This is the full width of the image at half maximum value, or full-width half-maximum, or FWHM.
   It is a simple and well-defined number which can be used to compare the quality of images obtained
   under different observing conditions. In the usual sort of astronomical image, the FWHM is measured
   for a selection of stars in the frame and the "seeing" or image quality is reported as the mean value. 
   
   Calculating FWHM:
   FWHM = sigma * 1.66510922; // 1.66510922 = 2*sqrt(ln(2)) -- WHY? Similar proof but factor of sqrt(2) (?): http://mathworld.wolfram.com/GaussianFunction.html

   -Non-Linear-Curve Fitting! --> gsl
   See http://en.wikipedia.org/wiki/Levenberg%E2%80%93Marquardt_algorithm
*/

// TODOs: Remove DirectionT from Fwhm?!, cleanup centroid and Fwhm class!! New constructor, remove old code, duplicated code,...

#ifndef _FWHM_HPP_
#define _FWHM_HPP_ _FWHM_HPP_

#include <CImg.h>
#include <vector>
#include <math.h>
#include <iostream>

#include <boost/algorithm/string/case_conv.hpp>

#include "centroid.hpp"
#include "at_exception.hpp"
#include "fitgsl_tmpl.hpp"


using namespace std;
using namespace cimg_library;

DEF_Exception(Fwhm);

namespace AT {
  struct DirectionT {
    enum TypeE {
      HORZ,
      VERT,
      _Count
    };
      
    static const char * asStr(const TypeE & inType) {
      switch(inType) {
      case HORZ: return "HORZ";
      case VERT: return "VERT";
      default: return "<?>";
      }
    }
      
    MAC_AS_TYPE(Type, E, _Count);
  };




  static vector<float>
  extractLine(const CImg<float> & inImage, const DirectionT::TypeE & inDirection) {
    CImg<float> img(inImage); // Make a copy
    PositionT center(img.width() / 2, img.height() / 2);
    vector<float> values;

    // Subtract median image
    double med = img.median();
    cimg_forXY(img, x, y) {
      img(x, y) = (img(x, y) > med ? img(x, y) - med : 0);
    }

    // Extract slices through centroid for profiles
    switch(inDirection) {
    case DirectionT::HORZ: {
      values.resize(img.width());
      LOG(trace) << "DirectionT::HORZ sliced values - width: " << img.width() << ": " << endl;
      cimg_forX(img, x) { 
	LOG(trace) << "x: " << x << ", y: " << (int) center.get<1>() << " -> value: " << img(x, center.get<1>()) << endl;
	values[x] = img(x, (int) center.get<1>());
      }
      break;
    }
    case DirectionT::VERT: {
      values.resize(img.height());
      LOG(trace) << "DirectionT::VERT sliced values - height: " << img.height() << ": " << endl;
      cimg_forY(img, y) {
	LOG(trace) << "x: " << (int) center.get<0>() << ", y: " << y << " -> value: " << img(center.get<0>(), y) << endl;
	values[y] = img((int) center.get<0>(), y);
      }
      break;
    }
    default: {
      AT_ASSERT(Fwhm, false, "Invalid direction.");
    }
    }
    return values;
  }

  static vector<float>
  extractLine(const CImg<float> & inImage, const DirectionT::TypeE & inDirection, const FrameT & inFrame) {
    // TODO: Check if in bounds?!
    // TODO: is +1 correct?!
    CImg<float> resImage = inImage.get_crop(inFrame.get<0>() + 1 /* x */, inFrame.get<1>() + 1 /* y */,
					    inFrame.get<1>() /* x */ + inFrame.get<2>() /* w */,
					    inFrame.get<1>() /* y */ + inFrame.get<3>() /* h */);

    return extractLine(resImage, inDirection);
  }

  // TODO: Do not pass back vector as copy!
  // TODO: Move to utils?!
  static vector<float>
  extractLine(const CImg<float> & inImage, const DirectionT::TypeE & inDirection, PositionT inCenter, size_t inWindowSizePx) {
    AT_ASSERT(Fwhm, inWindowSizePx > 0, "Specify inWindowSizePx > 0.");
    
    // TODO: Check if in bounds?!
    FrameT imgWindow = centerPosToFrame(inCenter, inWindowSizePx);
    return extractLine(inImage, inDirection, imgWindow);
  }



  class FwhmT {  
  private:
    typedef CurveFitTmplT<GaussianFitTraitsT> GaussMatcherT;
    
  public:    
    // TODO: Calc FWHM ["] from FWHM [px] --> Needs F?! --> Move out of here?!--> Diff class?!
    // See http://www.paulmcgale.co.uk/tech.htm
    // See http://www.wilmslowastro.com/software/formulae.htm  
    static double pxToArcsec(double px, double inFocalLength /*mm*/, DimensionT inPixelSize /*um*/, BinningT inBinning) {
      const double pixelSizeEq = sqrt(inPixelSize.get<0>() * inPixelSize.get<1>());
      const double arcsecPerPixel = (206.2648 * (double) inBinning.get<0>() * (double) inBinning.get<1>() * pixelSizeEq) / inFocalLength;
      return arcsecPerPixel * px;
    }
    
  private:
    vector<float> mImgValues, mFitValues;
    GaussMatcherT::CurveParamsT::TypeT mGaussParms;
    
    static const double SIGMA_TO_FWHM;
    static const size_t MAX_PTS;
    
    static float calcGaussianValue(const GaussMatcherT::CurveParamsT::TypeT & inGaussParms, float x);
    static void fitValues(const vector<float> & imgValues, vector<float> * outFitValues, GaussMatcherT::CurveParamsT::TypeT * outGaussParms, double inEpsAbs = 1e-2, double inEpsRel = 1e-2);
    
  public:
    FwhmT() { }
    FwhmT(const vector<float> & inValues, double inEpsAbs = 1e-2, double inEpsRel = 1e-2);
    void set(const vector<float> & inValues, double inEpsAbs = 1e-2, double inEpsRel = 1e-2);
    
    inline bool valid() const { return (mImgValues.size() > 0 && mFitValues.size() > 0); }
    
    static inline double sigmaToFwhm(double sigma) { return FwhmT::SIGMA_TO_FWHM * sigma; }
    static inline double fwhmToSigma(double sigma) { return sigma / FwhmT::SIGMA_TO_FWHM; }
    
    inline float getValue() const { return sigmaToFwhm(mGaussParms[GaussMatcherT::CurveParamsT::W_IDX]); }
    
    inline const vector<float> & getImgValues() const { return mImgValues; }
    inline const vector<float> & getFitValues() const { return mFitValues; }
    inline float calcGaussianValue(float x) const { return calcGaussianValue(mGaussParms, x); }
    float getStandardDeviation() const;

    ostream & print(ostream & os) const;
    friend ostream & operator<<(ostream & os, const FwhmT & inFwhm);
  };
  
  void validate(boost::any & v, const vector<string> & values, DirectionT::TypeE * target_type, int);
}; // end namespace AT

#endif // _FWHM_HPP_


