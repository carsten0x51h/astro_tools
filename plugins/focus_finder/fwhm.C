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

#include "fwhm.hpp"

namespace AT {


  class ImgAccessorT : public DataAccessorT {
  public:
    ImgAccessorT(const vector<float> & inImgValues) : mImgValues(inImgValues) {}
    virtual size_t size() const { return mImgValues.size(); }
    virtual DataPointT operator()(size_t inIdx) const {
      DataPointT dp(inIdx, mImgValues.at(inIdx));
      return dp;
    }
  private:
    const vector<float> & mImgValues;
  };




  const double FwhmT::SIGMA_TO_FWHM = 1.66510922; // 2.0 * sqrt(ln(2.0))
  const size_t FwhmT::MAX_PTS = 1000; // Max number of data points


  ostream & operator<<(ostream & os, const FwhmT & inFwhm) {
    return inFwhm.print(os);
  }

  FwhmT::FwhmT(const CImg<float> & image, const DirectionT::TypeE & inDirection, double inEpsAbs, double inEpsRel, float inCenterX, float inCenterY, /*TODO: use PositionT */size_t inSizePx) {
    this->set(image, inDirection, inEpsAbs, inEpsRel, inCenterX, inCenterY, inSizePx);
  }


  void FwhmT::set(const CImg<float> & image, const DirectionT::TypeE & inDirection, double inEpsAbs, double inEpsRel, float inCenterX, float inCenterY, size_t inSizePx) {
    //cout << "FwhmT - direction: " << DirectionT::asStr(inDirection) << endl;

    mDirection = inDirection;
  
    // TODO: Can we avoid image copy??!
    CImg<float> resImage;
  
    if (inCenterX < 0 || inCenterY < 0) {
      // Calculate centroid
      //CentroidCalcT::starCentroid(image, & mXCom, & mYCom, & resImage);
      mXCom = image.width() / 2;
      mYCom = image.height() / 2;
      resImage = image;
      //cout << "FWHM Centroid - x: " << mXCom << ", y: " << mYCom << endl;
    } else {
      if (! inSizePx) {
	// TODO: Require any longer?
	throw FwhmExceptionT("Specify size.");
      }
    
      const int BACK = ceil(inSizePx / 2.0) - 1;
      int startX = (int)inCenterX - BACK, startY = (int)inCenterY - BACK;
    
      //cout << "inSizePx: " << inSizePx << "--> BACK: " << BACK << ", inCenterX: " << inCenterX << ", inCenterY: " << inCenterY << ", startX: " << startX << ", startY: " << startY << endl;
    
      resImage = image.get_crop(startX+1, startY+1, startX + inSizePx, startY + inSizePx);
    
      //cerr << "resImage - width(): " << resImage.width() << ", height: " << resImage.width() << endl;
    
      // DGB START
      // CImgDisplay disp1(resImage, "fwhm-image" );
      // while(! disp1.is_closed()) { CImgDisplay::wait(disp1); }
      // DGB END
    
    
      // TODO: Improve?!
      mXCom = BACK;
      mYCom = BACK;
    }
    
    // Subtract median image
    double med = resImage.median();
    cimg_forXY(resImage, x, y) {
      resImage(x, y) = (resImage(x, y) > med ? resImage(x, y) - med : 0);
    }


    // TODO / FIXME!!!!!!!!!!!!!!!!!!!! centroid should be returned with respect to passed image - not to result image!!!!

    // Extract slices through centroid for profiles
    switch(inDirection) {
    case DirectionT::HORZ: {
      mImgValues.resize(resImage.width());
      LOG(trace) << "DirectionT::HORZ sliced values - width: " << resImage.width() << ": " << endl;
      cimg_forX(resImage, x) { 
	LOG(trace) << "x: " << x << ", mYCom: " << (int)mYCom << " -> value: " << resImage(x, mYCom) << endl;
	mImgValues[x] = resImage(x, (int)mYCom);
      }
      break;
    }
    case DirectionT::VERT: {
      mImgValues.resize(resImage.height());
      LOG(trace) << "DirectionT::VERT sliced values - height: " << resImage.height() << ": " << endl;
      cimg_forY(resImage, y) {
	LOG(trace) << "y: " << y << ", mXCom: " << (int)mXCom << " -> value: " << resImage(mXCom, y) << endl;
	mImgValues[y] = resImage((int)mXCom, y);
      }
      break;
    }
    default: {
      return;
    }
    }
    
    // TODO: Calculate the SNR?!?!?!!! Values from Fwhm do not seem to be suitable to recognize what kind of region is selected ... ^^ strange!
    
    // Fit horz. & vert. values
    FwhmT::fitValues(mImgValues, & mFitValues, & mGaussParms, inEpsAbs, inEpsRel);
  }


  void
  FwhmT::fitValues(const vector<float> & imgValues, vector<float> * outFitValues, GaussMatcherT::ParamsT * outGaussParms, double inEpsAbs, double inEpsRel) {
    AT_ASSERT(Fwhm, outGaussParms, "outGaussParms not set!");
    AT_ASSERT(Fwhm, outFitValues, "outFitValues not set!");

    outFitValues->resize(imgValues.size());

    // Do the LM fit - TODO: this function should throw if it does not succeed?!
    int err = GaussMatcherT::fitGslLevenbergMarquart(ImgAccessorT(imgValues), outGaussParms, inEpsAbs, inEpsRel);
  
    if (err) {
      stringstream ss;
      ss << "FwhmT::fitValues - fitgsl_lm() returned non-zero status: " << err << endl; 
      throw CurveFitExceptionT(ss.str().c_str());
    }
  
    // Transfer over the data to the global storage. Plot the true points onto the graph as well.
    vector<float> & fitValues = (*outFitValues);
    for(size_t i = 0; i < imgValues.size() && i < FwhmT::MAX_PTS; ++i) {
      fitValues[i] = GaussianFitTraitsT::fx(i /*TODO: was - still works? dataPoints[i].x*/, *outGaussParms);
    }
  }

  float
  FwhmT::calcGaussValue(const GaussMatcherT::ParamsT & inGaussParms, float x) {
    return GaussianFitTraitsT::fx(x, inGaussParms);
  }


  /**
     Method to calculate the standrard deviation from values on the curve
     to values measured values (image values).

     See http://de.wikipedia.org/wiki/Mittlere_quadratische_Abweichung
  */

  // TODO / FIXME: Seems to give wrong values!!
  float
  FwhmT::getStandardDeviation() const {
    float mse = 0;
    size_t x = 0;
    for (vector<float>::const_iterator it = mImgValues.begin(); it != mImgValues.end(); ++it, ++x) {
      //cout << "x: " << x << ", Calculated value: " << calcGaussianValue(x) << ", measured value: " << *it << ", square error: " << pow(calcGaussianValue(x) - *it, 2.0) << endl;

      mse += pow(calcGaussianValue(x) - *it, 2.0);
    }

    mse = sqrt(mse / ((float) mImgValues.size() - 1.0));
    return mse;
  }




  void validate(boost::any & v, const vector<string> & values, FwhmT::DirectionT::TypeE * target_type, int) {
    using namespace boost::program_options;
    
    validators::check_first_occurrence(v);
    string s = validators::get_single_string(values);
    boost::to_upper(s);
    FwhmT::DirectionT::TypeE type = FwhmT::DirectionT::asType(s.c_str());

    if (type != FwhmT::DirectionT::_Count) {
      v = any(type);
    } else {
      throw validation_error(validation_error::invalid_option_value);
    }
  }

}; // end namespace AT
