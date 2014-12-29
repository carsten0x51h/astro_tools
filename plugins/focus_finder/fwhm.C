//TODO: Test all constructors / set methods with existing test data!

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

  // Class which provides access to image vector values by curve fit algorithm.
  class ImgAccessorT {
  public:
    typedef vector<float> TypeT;
    static DataPointT getDataPoint(size_t inIdx, TypeT::const_iterator inIt) {
      DataPointT dp(inIdx, *inIt);
      return dp;
    }
  };


  // DirectionT validator for boost cmd line parser
  void validate(boost::any & v, const vector<string> & values, DirectionT::TypeE * target_type, int) {
    using namespace boost::program_options;
    
    validators::check_first_occurrence(v);
    string s = validators::get_single_string(values);
    boost::to_upper(s);
    DirectionT::TypeE type = DirectionT::asType(s.c_str());

    if (type != DirectionT::_Count) {
      v = any(type);
    } else {
      throw validation_error(validation_error::invalid_option_value);
    }
  }


  const double FwhmT::SIGMA_TO_FWHM = 1.66510922; // 2.0 * sqrt(ln(2.0))
  const size_t FwhmT::MAX_PTS = 1000;             // Max number of data points


  ostream & operator<<(ostream & os, const FwhmT & inFwhm) {
    return inFwhm.print(os);
  }

  FwhmT::FwhmT(const vector<float> & inValues, double inEpsAbs, double inEpsRel) {
    this->set(inValues, inEpsAbs, inEpsRel);
  }

  void
  FwhmT::set(const vector<float> & inValues, double inEpsAbs, double inEpsRel) {
    FwhmT::fitValues(inValues, & mFitValues, & mGaussParms, inEpsAbs, inEpsRel);
  }

  void
  FwhmT::fitValues(const vector<float> & imgValues, vector<float> * outFitValues, GaussMatcherT::CurveParamsT::TypeT * outGaussParms, double inEpsAbs, double inEpsRel) {
    AT_ASSERT(Fwhm, outGaussParms, "outGaussParms not set!");
    AT_ASSERT(Fwhm, outFitValues, "outFitValues not set!");

    outFitValues->resize(imgValues.size());

    // Do the LM fit, throws CurveFitExceptionT if error boundaries are specified and NOT satisfied
    GaussMatcherT::fitGslLevenbergMarquart<ImgAccessorT>(imgValues, outGaussParms, inEpsAbs, inEpsRel);
  
    // Transfer over the data to the global storage. Plot the true points onto the graph as well.
    vector<float> & fitValues = (*outFitValues);
    for(size_t i = 0; i < imgValues.size() && i < FwhmT::MAX_PTS; ++i) {
      fitValues[i] = GaussianFitTraitsT::fx(i /*TODO: was - still works? dataPoints[i].x*/, *outGaussParms);
    }
  }

  float
  FwhmT::calcGaussianValue(const GaussMatcherT::CurveParamsT::TypeT & inGaussParms, float x) {
    return GaussianFitTraitsT::fx(x, inGaussParms);
  }

  /**
   *  Method to calculate the standrard deviation from values on the curve
   *  to values measured values (image values).
   *
   * See http://de.wikipedia.org/wiki/Mittlere_quadratische_Abweichung
   */
  float
  FwhmT::getStandardDeviation() const {
    float mse = 0;
    size_t x = 0;
    for (vector<float>::const_iterator it = mImgValues.begin(); it != mImgValues.end(); ++it, ++x) {
      mse += pow(this->calcGaussianValue(x) - *it, 2.0);
    }
    
    mse = sqrt(mse / ((float) mImgValues.size() - 1.0));
    return mse;
  }

  ostream &
  FwhmT::print(ostream & os, bool inPrintDetails) const {
    os << "FWHM=" << this->getValue() << "\"" 
       << ", b=" << mGaussParms[GaussMatcherT::CurveParamsT::B_IDX]
       << ", p=" << mGaussParms[GaussMatcherT::CurveParamsT::P_IDX]
       << ", c=" << mGaussParms[GaussMatcherT::CurveParamsT::C_IDX]
       << ", w=" << mGaussParms[GaussMatcherT::CurveParamsT::W_IDX] << "]..." << endl;
    
    if (inPrintDetails) {
      os << ", Img values: ";
      for (vector<float>::const_iterator it = mImgValues.begin(); it != mImgValues.end(); ++it) { os << *it << "; "; }
      os << ", Fit values: ";
      for (vector<float>::const_iterator it = mFitValues.begin(); it != mFitValues.end(); ++it) { os << *it << "; "; }
    }

    return os;
  }

}; // end namespace AT
