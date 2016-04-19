/*************************************************************************** *
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

#ifndef __FITGSL_TMPL_HPP__
#define __FITGSL_TMPL_HPP__ __FITGSL_TMPL_HPP__

/* Include standard and GSL headers */
#include <stdlib.h>
#include <stdio.h>
#include <string.h> // memset
#include <math.h>
#include <array>

#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_multifit_nlin.h>


using namespace std;

namespace AT {

  DEF_Exception(CurveFit);
  

  // TODO: Should we move this to the tmpl?!
  struct DataPointT {
    float x;
    float y;
    DataPointT(float inX = 0, float inY = 0) : x(inX), y(inY) {}
  };

  typedef vector<DataPointT> DataPointsT;

  struct GslMultiFitDataT {
    float y;
    float sigma;
    DataPointT pt;
  };

  typedef vector<GslMultiFitDataT> GslMultiFitParmsT;



  template <class FitTraitsT>
  class CurveFitTmplT {
  private:

  public:
    typedef typename FitTraitsT::CurveParamsT CurveParamsT;

    /**
     * Fits the specified data to the function, storing the coefficients in 'results'.
     * DataAccessor allows specifying how x,y data is accessed.
     * See http://en.wikipedia.org/wiki/Approximation_error for expl. of rel and abs errors.
     */
    template<typename DataAccessorT> static int  
    fitGslLevenbergMarquart(const typename DataAccessorT::TypeT & inData, typename CurveParamsT::TypeT * outResults,
			    double inEpsAbs, double inEpsRel, size_t inNumMaxIter = 500, bool inExOnError = true) {
      AT_ASSERT(CurveFit, outResults, "Result vector not set!");
      
      GslMultiFitParmsT gslMultiFitParms(inData.size());
      LOG(trace) << "fitGslLevenbergMarquart... numDataPoints: " << inData.size() << endl;
      
      // Fill in the parameters
      for (typename DataAccessorT::TypeT::const_iterator it = inData.begin(); it != inData.end(); ++it) {
	size_t idx = std::distance(inData.begin(), it);
	const DataPointT & dataPoint = DataAccessorT::getDataPoint(idx, it);
	gslMultiFitParms[idx].y     = dataPoint.y;
	gslMultiFitParms[idx].sigma = 0.1f;
	gslMultiFitParms[idx].pt    = dataPoint;
	LOG(trace) << " > dataPoint idx=" << idx << ", x: " << dataPoint.x << ", y: " << dataPoint.y << endl;
      }
      
      // Fill in function info
      gsl_multifit_function_fdf f;
      f.f      = FitTraitsT::gslFx;
      f.df     = FitTraitsT::gslDfx;
      f.fdf    = FitTraitsT::gslFdfx;
      f.n      = inData.size();
      f.p      = FitTraitsT::CurveParamsT::_Count;
      f.params = & gslMultiFitParms;
      
      
      gsl_vector * guess = gsl_vector_alloc(FitTraitsT::CurveParamsT::_Count);  // Allocate the guess vector
      
      FitTraitsT::makeGuess(gslMultiFitParms, guess);  // Make initial guesses based on the data
      
      // Create a Levenberg-Marquardt solver with n data points and m parameters
      gsl_multifit_fdfsolver * solver = gsl_multifit_fdfsolver_alloc(gsl_multifit_fdfsolver_lmsder,
								     inData.size(), FitTraitsT::CurveParamsT::_Count);
      
      AT_ASSERT(CurveFit, solver, "Solver is NULL!");
      
      gsl_multifit_fdfsolver_set(solver, & f, guess);  // Initialize the solver
      
      int status, i = 0;
      
      // Iterate to to find a result
      do {
      i++;
      status = gsl_multifit_fdfsolver_iterate(solver); // returns 0 in case of success
      if (status) {  break; }
      status = gsl_multifit_test_delta(solver->dx, solver->x, inEpsAbs, inEpsRel);
      } while (status == GSL_CONTINUE && i < inNumMaxIter);
      
      // Store the results to be returned to the user (copy from gsl_vector to result structure)
      for (size_t i = 0; i < FitTraitsT::CurveParamsT::_Count; ++i) {
	typename FitTraitsT::CurveParamsT::TypeE idx = static_cast<typename FitTraitsT::CurveParamsT::TypeE>(i);
	(*outResults)[idx] = gsl_vector_get(solver->x, idx);
      }
      
      // Free GSL memory
      gsl_multifit_fdfsolver_free(solver);
      gsl_vector_free(guess);
      
      return status;
    }
  };



  /**
   * Gaussian fit traits
   */
  class GaussianFitTraitsT {
  private:
  
  public:
    // TODO: Put this to a macro, just supply list if enum! / Or create a template and supply enum type as T!
    struct CurveParamsT {
      // b = base, p = peak, c = center in x, w = mean width (FWHM)
      enum TypeE { B_IDX = 0, P_IDX, C_IDX, W_IDX, _Count };
      struct TypeT : public std::array<float, TypeE::_Count> {
	TypeT(const gsl_vector * inVec = 0) {
	  for (size_t i = 0; i < TypeE::_Count; ++i) {
	    TypeE idx = static_cast<TypeE>(i);
	    (*this)[i] = (inVec ? gsl_vector_get(inVec, idx) : 0);
	  }
	}
      };
    };


    /* Makes a guess for b, p, c and w based on the supplied data */
    //static void makeGuess(const DataAccessorT & inDataAccessor, gsl_vector * guess) {
    static void makeGuess(const GslMultiFitParmsT & inData, gsl_vector * guess) {
      AT_ASSERT(CurveFit, inData.size() > 1, "inData.size() <= 1!");

      size_t numDataPoints = inData.size();
      float y_mean = 0;
      float y_max = inData.at(0).pt.y;
      float c = inData.at(0).pt.x;
    
      for(size_t i = 0; i < numDataPoints; ++i) {
	const DataPointT & dataPoint = inData.at(i).pt;

	y_mean += dataPoint.y;
      
	if(y_max < dataPoint.y) {
	  y_max = dataPoint.y;
	  c = dataPoint.x;
	}
      }

      y_mean /= (float) numDataPoints;
      float w = (inData.at(numDataPoints - 1).pt.x - inData.at(0).pt.x) / 10.0;
    
      gsl_vector_set(guess, CurveParamsT::B_IDX, y_mean);
      gsl_vector_set(guess, CurveParamsT::P_IDX, y_max);
      gsl_vector_set(guess, CurveParamsT::C_IDX, c);
      gsl_vector_set(guess, CurveParamsT::W_IDX, w);
    }
  
    /* y = b + p * exp(-0.5f * ((t - c) / w) * ((t - c) / w)) */
    static float fx(float x, const CurveParamsT::TypeT & inParms) {
      float b = inParms[CurveParamsT::B_IDX];
      float p = inParms[CurveParamsT::P_IDX];
      float c = inParms[CurveParamsT::C_IDX];
      float w = inParms[CurveParamsT::W_IDX];
      float t = ((x - c) / w);
      t *= t;
      return (b + p * exp(-0.5f * t));
    }

    /* Calculates f(x) = b + p * e^[0.5*((x-c)/w)] for each data point. */
    static int gslFx(const gsl_vector * x, void * inGslParams, gsl_vector * outResultVec) {    
      CurveParamsT::TypeT curveParams(x);     // Store the current coefficient values
      const GslMultiFitParmsT * gslParams = ((GslMultiFitParmsT*) inGslParams); // Store parameter values
      
      //Execute Levenberg-Marquart on f(x)
      for(size_t i = 0; i < gslParams->size(); ++i) {
	const GslMultiFitDataT & gslData = gslParams->at(i);
	float yi = GaussianFitTraitsT::fx((float) gslData.pt.x, curveParams);
	LOG(trace) << "Gaussian - gslFx - i: " << i << ", gslData.pt.x: " << gslData.pt.x
		   << ", yi: " << yi << ", gslData.y: " << gslData.y << ", gslData.sigma: " << gslData.sigma << endl; 
	
	gsl_vector_set(outResultVec, i, (yi - gslData.y) / gslData.sigma);
      }
      return GSL_SUCCESS;
    }


    /* Calculates the Jacobian (derivative) matrix of f(x) = b + p * e^[0.5*((x-c)/w)^2] for each data point */
    static int gslDfx(const gsl_vector * x, void * params, gsl_matrix * J) {
    
      // Store parameter values
      const GslMultiFitParmsT * gslParams = ((GslMultiFitParmsT*) params);
    
      // Store current coefficients
      float p = gsl_vector_get(x, CurveParamsT::P_IDX);
      float c = gsl_vector_get(x, CurveParamsT::C_IDX);
      float w = gsl_vector_get(x, CurveParamsT::W_IDX);
    
      // Store non-changing calculations
      float w2 = w * w;
      float w3 = w2 * w;
    
      for(size_t i = 0; i < gslParams->size(); ++i) {
	const GslMultiFitDataT & gslData = gslParams->at(i);
	float x_minus_c = (gslData.pt.x - c);
	float e = exp(-0.5f * (x_minus_c / w) * (x_minus_c / w));
      
	gsl_matrix_set(J, i, CurveParamsT::B_IDX, 1 / gslData.sigma);
	gsl_matrix_set(J, i, CurveParamsT::P_IDX, e / gslData.sigma);
	gsl_matrix_set(J, i, CurveParamsT::C_IDX, (p * e * x_minus_c) / (gslData.sigma * w2));
	gsl_matrix_set(J, i, CurveParamsT::W_IDX, (p * e * x_minus_c * x_minus_c) / (gslData.sigma * w3));
      }
    
      return GSL_SUCCESS;
    }
  
    /* Invokes f(x) and f'(x) */
    static int gslFdfx(const gsl_vector * x, void * params, gsl_vector *f, gsl_matrix *J) {
      gslFx(x, params, f);
      gslDfx(x, params, J);
    
      return GSL_SUCCESS;
    }
  };


  /**
   * Parabel fit traits
   */
  class ParabelFitTraitsT {
  private:
  
  public:
    struct CurveParamsT {
      enum TypeE { A_IDX = 0, B_IDX, C_IDX, _Count };
      struct TypeT : public std::array<float, TypeE::_Count> {
	TypeT(const gsl_vector * inVec = 0) {
	  for (size_t i = 0; i < TypeE::_Count; ++i) {
	    TypeE idx = static_cast<TypeE>(i);
	    (*this)[i] = (inVec ? gsl_vector_get(inVec, idx) : 0);
	  }
	}
      };
    };

    /* Makes a guess for a, b and c based on the supplied data */
    static void makeGuess(const GslMultiFitParmsT & inData, gsl_vector * guess) {
      AT_ASSERT(CurveFit, inData.size(), "inData.size() == 0!");

      float xMin = inData.at(0).pt.x;
      float yMin = inData.at(0).pt.y;

      // Find minimum y value...
      for (size_t i = 0; i < inData.size(); ++i) {
	const DataPointT & dataPoint = inData.at(i).pt;
	if (dataPoint.y < yMin) {
	  xMin = dataPoint.x;
	  yMin = dataPoint.y;
	}
      }

      LOG(debug) << "xMin: " << xMin << ", yMin: " << yMin << endl;

      // Guessing of parameters a,b and c using prominent coordinates of the curve
      const float x1 = inData.at(0).pt.x; // Left value
      const float y1 = inData.at(0).pt.y;
      const float x2 = xMin;
      const float y2 = yMin;
      const float x3 = inData.back().pt.x; // Right value
      const float y3 = inData.back().pt.y;

      const float x1x2sq = x1 * x1 - x2 * x2;
      const float x1x3sq = x1 * x1 - x3 * x3;

      const float bNumerator = (y1 - y2) * x1x3sq - y1 * (x1 * x1 - x2 * x2) + y3 * x1x2sq;
      const float bDenominator = (x1 - x3) * x1x2sq - (x1 - x2) * x1x3sq;

      const float bGuess = -bNumerator / bDenominator;
      const float aGuess = (y1 - y2 - bGuess * (x1 - x2)) / x1x2sq;
      const float cGuess = y2 - aGuess * x2 * x2 - bGuess * x2;

      LOG(debug) << "Guessing a=" << aGuess << ", b=" << bGuess << ", c=" << cGuess << endl;

      gsl_vector_set(guess, CurveParamsT::A_IDX, aGuess);
      gsl_vector_set(guess, CurveParamsT::B_IDX, bGuess);
      gsl_vector_set(guess, CurveParamsT::C_IDX, cGuess);
    }
  
    /* y = a * x^2 + b * x + c */
    static float fx(float x, const CurveParamsT::TypeT & inParms) {
      float a = inParms[CurveParamsT::A_IDX];
      float b = inParms[CurveParamsT::B_IDX];
      float c = inParms[CurveParamsT::C_IDX];
      float x2 = x * x;
      return (a * x2 + b * x + c);
    }

    /* Calculates f(x) = a*x^2 + b*x + c for each data point */
    static int gslFx(const gsl_vector * x, void * params, gsl_vector * f) {    
      /* Store the parameter data */
      const GslMultiFitParmsT * gslParams = ((GslMultiFitParmsT*) params);
    
      /* Store the current coefficient values */
      CurveParamsT::TypeT resParams(x);

      /* Execute Levenberg-Marquart on f(x) */
      for(size_t i = 0; i < gslParams->size(); ++i) {
	const GslMultiFitDataT & gslData = gslParams->at(i);
	const float yi = ParabelFitTraitsT::fx((float) gslData.pt.x, resParams);
	gsl_vector_set(f, i, (yi - gslData.y) / gslData.sigma);
      }
    
      return GSL_SUCCESS;
    }

    /**
     * Calculates the Jacobian (derivative) matrix of f(x) = a*x^2 + b*x + c for each data point.
     * -> Fitting function f_i = [(a*x_i^2 + b*x_i + c) - y_i] / sigma_i
     * -> J = 1/sigma * (x^2, x, 1)
     *
     * See: http://de.wikipedia.org/wiki/Jacobi-Matrix
     * See: http://de.wikipedia.org/wiki/Levenberg-Marquardt-Algorithmus
     */
    static int gslDfx(const gsl_vector * x, void * params, gsl_matrix * J) {
    
      /* Store parameter values */
      const GslMultiFitParmsT * gslParams = ((GslMultiFitParmsT*) params);

      /* Store current coefficients */
      float a = gsl_vector_get(x, CurveParamsT::A_IDX);
      float b = gsl_vector_get(x, CurveParamsT::B_IDX);
      float c = gsl_vector_get(x, CurveParamsT::C_IDX);
    
      for(size_t i = 0; i < gslParams->size(); ++i) {
	const GslMultiFitDataT & gslData = gslParams->at(i);
	const float oneBySigma = 1.0f / gslData.sigma;
	const float x = gslData.pt.x;

	gsl_matrix_set(J, i, CurveParamsT::A_IDX, oneBySigma * x * x);
	gsl_matrix_set(J, i, CurveParamsT::B_IDX, oneBySigma * x);
	gsl_matrix_set(J, i, CurveParamsT::C_IDX, oneBySigma);
      }
    
      return GSL_SUCCESS;
    }
  
    /* Invokes f(x) and f'(x) */
    static int gslFdfx(const gsl_vector * x, void * params, gsl_vector * f, gsl_matrix * J) {
      // TODO: Just for optmization we may directly compute f -> df... for now we just call the respective functions...
      ParabelFitTraitsT::gslFx(x, params, f);
      ParabelFitTraitsT::gslDfx(x, params, J);
    
      return GSL_SUCCESS;
    }
  };
}; // end namespace AT

#endif	/* __FITGSL_TMPL_HPP__ */
