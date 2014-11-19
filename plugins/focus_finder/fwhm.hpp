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

#include "fitgsl.hpp"
#include "centroid.hpp"

#include "at_exception.hpp"


using namespace std;
using namespace cimg_library;

DEF_Exception(Fwhm);
DEF_Exception(FwhmCurveFitting);

struct GaussParmsT {
  float b, p, c, w, area;
  size_t numFitPts;

  GaussParmsT() : b(0), p(0), c(0), w(0), area(0), numFitPts(0) { }
  ostream & print(ostream & os) {
    os << "b: " << b << ", p: " << p << ", c: " << c << ", w: " << w << ", area: " << area << ", numFitPts: " << numFitPts << endl;
    return os;
  }
};

// TODO: Pull out DirectionT from Fwhm?! instead pass vector of values...?!value itself is independent frm image, horz and vert!!!!!!!
class FwhmT {  
public:
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

  // TODO: Calc FWHM ["] from FWHM [px] --> Needs F?! --> Move out of here?!--> Diff class?!
  // See http://www.paulmcgale.co.uk/tech.htm
  // See http://www.wilmslowastro.com/software/formulae.htm  
  static double pxToArcsec(double px, double inFocalLength /*mm*/, DimensionT inPixelSize /*um*/, BinningT inBinning) {
    const double pixelSizeEq = sqrt(inPixelSize.get<0>() * inPixelSize.get<1>());
    const double arcsecPerPixel = (206.2648 * (double) inBinning.get<0>() * (double) inBinning.get<1>() * pixelSizeEq) / inFocalLength;
    return arcsecPerPixel * px;
  }

private:
  float mXCom, mYCom;
  vector<float> mImgValues, mFitValues;
  GaussParmsT mGaussParms;
  
  static const double SIGMA_TO_FWHM;
  static const size_t MAX_PTS;

  static float calcGaussValue(const GaussParmsT & gaussParms, float x) {
    return fitgsl_fx(gaussParms.b, gaussParms.p, gaussParms.c, gaussParms.w, x);
  }

  static void fitValues(const vector<float> & imgValues, vector<float> * outFitValues, GaussParmsT * gaussParms) {
    outFitValues->resize(imgValues.size());
    vector<float> & fitValues = (*outFitValues);
    float A[4]; // Contains the coefficients of the fitted line
    
    LOG(trace) << "Sliced values: ";
    for (vector<float>::const_iterator it = imgValues.begin(); it != imgValues.end(); ++it)
      LOG(trace) << *it << "; ";
    LOG(trace) << endl;
    
    // Allocate temporary data buffer
    fitgsl_data	* dat = fitgsl_alloc_data(imgValues.size());
    
    // Fill data
    for(unsigned int i = 0; i < imgValues.size(); ++i) {
      dat->pt[i].x = i;
      dat->pt[i].y = imgValues[i];
    }
	
    // Do the LM fit
    int err = fitgsl_lm(dat, A);

    if (err) {
      stringstream ss;
      ss << "FwhmT::fitValues - fitgsl_lm() returned non-zero status: " << err << endl; 
      throw FwhmCurveFittingExceptionT(ss.str().c_str());
    }

    // Transfer over the data to the global storage. Plot the true points onto the graph as well.
    size_t i;
    for(i = 0; i < imgValues.size() && i < FwhmT::MAX_PTS; ++i) {
      fitValues[i] = fitgsl_fx(A[FITGSL_B_INDEX], A[FITGSL_P_INDEX], A[FITGSL_C_INDEX], A[FITGSL_W_INDEX], dat->pt[i].x);
    }

    // Free allocated data
    fitgsl_free_data(dat);
    
    // Return values
    gaussParms->area = sqrt(2.0 * M_PI) * A[FITGSL_P_INDEX] * A[FITGSL_W_INDEX];
    gaussParms->b = A[FITGSL_B_INDEX];
    gaussParms->p = A[FITGSL_P_INDEX];
    gaussParms->c = A[FITGSL_C_INDEX];
    gaussParms->w = A[FITGSL_W_INDEX];
    gaussParms->numFitPts = i - 1;
  }
  

public:
  FwhmT() : mXCom(0), mYCom(0) { }

  // TODO: Add further constructors... vector<double> ... just row of data...
  FwhmT(const CImg<float> & image, const DirectionT::TypeE & inDirection, float inCenterX = -1, float inCenterY = -1, size_t inSizePx = 0);
  inline bool valid() const { return (mImgValues.size() > 0 && mFitValues.size() > 0); }

  static inline double sigmaToFwhm(double sigma) { return FwhmT::SIGMA_TO_FWHM * sigma; }
  static inline double fwhmToSigma(double sigma) { return sigma / FwhmT::SIGMA_TO_FWHM; }

  inline float getValue() const { return sigmaToFwhm(mGaussParms.w); }
  inline void getCentroid(float * x, float * y) const { *x = mXCom; *y = mYCom; }

  inline const vector<float> & getImgValues() const { return mImgValues; }
  inline const vector<float> & getFitValues() const { return mFitValues; }
  inline float calcGaussianValue(float x) const { return calcGaussValue(mGaussParms, x); }

  float getStandardDeviation() const;

  // TODO: overload << operator...
};

void validate(boost::any & v, const vector<string> & values, FwhmT::DirectionT::TypeE * target_type, int);

#endif // _FWHM_HPP_


