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

#ifndef __FITGSL_TMPL_HPP__
#define __FITGSL_TMPL_HPP__ __FITGSL_TMPL_HPP__

/* Include standard and GSL headers */
#include <stdlib.h>
#include <stdio.h>
#include <string.h> // memset
#include <math.h>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_multifit_nlin.h>

#include <array>

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

  class DataAccessorT {
  public:
    virtual size_t size() const = 0;
    virtual DataPointT operator()(size_t inIdx) const = 0;
  };




  template <class FitTraitsT>
  class CurveFitTmplT {
  private:

  public:
    typedef typename FitTraitsT::ParamsT ParamsT;
    typedef typename FitTraitsT::ParamsIdxT IdxT;

    /**
     * Fits the specified data to the function, storing the coefficients in 'results'.
     *
     * See http://en.wikipedia.org/wiki/Approximation_error for expl. of rel and abs errors.
     */
    static int fitGslLevenbergMarquart(const DataAccessorT & inDataAccessor, ParamsT * outResults, double epsabs, double epsrel) {
      AT_ASSERT(CurveFit, outResults, "Result vector not set!");

      // Fill the params
      const size_t numDataPoints = inDataAccessor.size();
      GslMultiFitParmsT gslMultiFitParms(numDataPoints);
      
      LOG(trace) << "fitGslLevenbergMarquart... numDataPoints: " << numDataPoints << endl;

      for(size_t i = 0; i < numDataPoints; ++i) {
	DataPointT dataPoint = inDataAccessor(i);
	gslMultiFitParms[i].y     = dataPoint.y;
	gslMultiFitParms[i].sigma = 0.1f;
	gslMultiFitParms[i].pt    = dataPoint; // TODO: we pass a copy here... we may improve this...

	LOG(trace) << " > dataPoint i=" << i << ", x: " << dataPoint.x << ", y: " << dataPoint.y << endl;
      }

      // Fill in function info
      gsl_multifit_function_fdf f;
      f.f      = FitTraitsT::gslFx;
      f.df     = FitTraitsT::gslDfx;
      f.fdf    = FitTraitsT::gslFdfx;
      f.n      = numDataPoints;
      f.p      = FitTraitsT::ParamsIdxT::_Count;
      f.params = & gslMultiFitParms;
    
      // Allocate the guess vector
      gsl_vector * guess = gsl_vector_alloc(FitTraitsT::ParamsIdxT::_Count);
    
      // Make initial guesses based on the data
      FitTraitsT::makeGuess(inDataAccessor, guess);
    
      // Create a Levenberg-Marquardt solver with n data points and m parameters
      const gsl_multifit_fdfsolver_type * solverType = gsl_multifit_fdfsolver_lmsder;
      gsl_multifit_fdfsolver * solver = gsl_multifit_fdfsolver_alloc(solverType, numDataPoints, FitTraitsT::ParamsIdxT::_Count);

      AT_ASSERT(CurveFit, solver, "Solver is NULL!");

      // Initialize the solver
      gsl_multifit_fdfsolver_set(solver, & f, guess);
    
      // Reset i to count iterations
      int status, i = 0;
    
      // Iterate to to find results
      do {
	i++;
	status = gsl_multifit_fdfsolver_iterate(solver);
      
	// TODO: use print from fitgsl.C....
	//print_state(i, solver);
      
	if (status != 0) {
	  break;
	}
	status = gsl_multifit_test_delta(solver->dx, solver->x, epsabs, epsrel);
      
      } while (status == GSL_CONTINUE && i < 500);
    
      // Store the results to be returned to the user (copy from gsl_vector to result structure)
      for (size_t i=0; i < FitTraitsT::ParamsIdxT::_Count; ++i) {
	typename FitTraitsT::ParamsIdxT::TypeE idx = static_cast<typename FitTraitsT::ParamsIdxT::TypeE>(i);
	(*outResults)[idx] = gsl_vector_get(solver->x, idx);
      }

      // Free GSL memory
      gsl_multifit_fdfsolver_free(solver);
      gsl_vector_free(guess);
    
      return (status == 0 ? 0 : -1);
    }
  };



  /**
   * Gaussian fit traits
   */
  class GaussianFitTraitsT {
  private:
  
  public:
    struct ParamsIdxT {
      enum TypeE {
	B_IDX = 0, // base
	P_IDX,     // peak (in y direction) ? - TODO: ~area...
	C_IDX,     // center (in x direction)
	W_IDX,     // mean width (FWHM) 
	_Count
      };

      static const char * asStr(const TypeE & inType) {
	switch (inType) {
	case B_IDX: return "B_IDX";
	case P_IDX: return "P_IDX";
	case C_IDX: return "C_IDX";
	case W_IDX: return "W_IDX";
	default: return "<?>";
	}
      }
    };
    typedef std::array<float, ParamsIdxT::_Count> ParamsT;


    /* Makes a guess for b, p, c and w based on the supplied data */
    static void makeGuess(const DataAccessorT & inDataAccessor, gsl_vector * guess) {
      AT_ASSERT(CurveFit, inDataAccessor.size() > 1, "inDataAccessor.size() <= 1!");
      size_t numDataPoints = inDataAccessor.size();
      float y_mean = 0;
      float y_max = inDataAccessor(0).y;
      float c = inDataAccessor(0).x;
    
      for(size_t i = 0; i < numDataPoints; ++i) {
	y_mean += inDataAccessor(i).y;
      
	if(y_max < inDataAccessor(i).y) {
	  y_max = inDataAccessor(i).y;
	  c = inDataAccessor(i).x;
	}
      }

      y_mean /= (float) numDataPoints;
      float w = (inDataAccessor(numDataPoints - 1).x - inDataAccessor(0).x) / 10.0;
    
      gsl_vector_set(guess, ParamsIdxT::B_IDX, y_mean);
      gsl_vector_set(guess, ParamsIdxT::P_IDX, y_max);
      gsl_vector_set(guess, ParamsIdxT::C_IDX, c);
      gsl_vector_set(guess, ParamsIdxT::W_IDX, w);
    }
  
    /* y = b + p * exp(-0.5f * ((t - c) / w) * ((t - c) / w)) */
    static float fx(float x, const ParamsT & inParms) {
      float b = inParms[ParamsIdxT::B_IDX];
      float p = inParms[ParamsIdxT::P_IDX];
      float c = inParms[ParamsIdxT::C_IDX];
      float w = inParms[ParamsIdxT::W_IDX];
      float t = ((x - c) / w);
      t *= t;
      return (b + p * exp(-0.5f * t));
    }

    /* Calculates f(x) = b + p * e^[0.5*((x-c)/w)] for each data point */
    // TODO: Stupid naming - parms vs resParms...
    static int gslFx(const gsl_vector * x, void * params /* Passed from fitgsl_lm... */, gsl_vector * f) {    
      /* Store parameter values */
      const GslMultiFitParmsT * gslParams = ((GslMultiFitParmsT*) params);
    
      /* Store the current coefficient values */
      ParamsT resParams; // TODO: We may create a constructor which takes a gsl_vector for init!
      for (size_t i=0; i < ParamsIdxT::_Count; ++i) {
	ParamsIdxT::TypeE idx = static_cast<ParamsIdxT::TypeE>(i);
	resParams[i] = gsl_vector_get(x, idx);
      }

      /* Execute Levenberg-Marquart on f(x) */
      for(size_t i = 0; i < gslParams->size(); ++i) {
	const GslMultiFitDataT & gslData = gslParams->at(i);
	float yi = GaussianFitTraitsT::fx((float) gslData.pt.x, resParams);

	LOG(trace) << "Gaussian - gslFx - i: " << i << ", gslData.pt.x: " << gslData.pt.x
		   << ", yi: " << yi << ", gslData.y: " << gslData.y << ", gslData.sigma: " << gslData.sigma << endl; 

	gsl_vector_set(f, i, (yi - gslData.y) / gslData.sigma);
      }
    
      return GSL_SUCCESS;
    }

    /* Calculates the Jacobian (derivative) matrix of f(x) = b + p * e^[0.5*((x-c)/w)^2] for each data point */
    static int gslDfx(const gsl_vector * x, void * params, gsl_matrix * J) {
    
      /* Store parameter values */
      const GslMultiFitParmsT * gslParams = ((GslMultiFitParmsT*) params);
    
      /* Store current coefficients */
      float p = gsl_vector_get(x, ParamsIdxT::P_IDX);
      float c = gsl_vector_get(x, ParamsIdxT::C_IDX);
      float w = gsl_vector_get(x, ParamsIdxT::W_IDX);
    
      /*  Store non-changing calculations */
      float w2 = w * w;
      float w3 = w2 * w;
    
      for(size_t i = 0; i < gslParams->size(); ++i) {
	const GslMultiFitDataT & gslData = gslParams->at(i);
	float x_minus_c = (gslData.pt.x - c);
	float e = exp(-0.5f * (x_minus_c / w) * (x_minus_c / w));
      
	gsl_matrix_set(J, i, ParamsIdxT::B_IDX, 1 / gslData.sigma);
	gsl_matrix_set(J, i, ParamsIdxT::P_IDX, e / gslData.sigma);
	gsl_matrix_set(J, i, ParamsIdxT::C_IDX, (p * e * x_minus_c) / (gslData.sigma * w2));
	gsl_matrix_set(J, i, ParamsIdxT::W_IDX, (p * e * x_minus_c * x_minus_c) / (gslData.sigma * w3));
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
    struct ParamsIdxT {
      enum TypeE {
	A_IDX = 0,
	B_IDX,
	C_IDX,
	_Count
      };

      static const char * asStr(const TypeE & inType) {
	switch (inType) {
	case A_IDX: return "A_IDX";
	case B_IDX: return "B_IDX";
	case C_IDX: return "C_IDX";
	default: return "<?>";
	}
      }
    };
    typedef std::array<float, ParamsIdxT::_Count> ParamsT;


    /* Makes a guess for a, b and c based on the supplied data */
    static void makeGuess(const DataAccessorT & inDataAccessor, gsl_vector * guess) {
      AT_ASSERT(CurveFit, inDataAccessor.size(), "inDataAccessor.size() == 0!");

      float xMin = inDataAccessor(0).x;
      float yMin = inDataAccessor(0).y;

      // Find minimum y value...
      for (size_t i = 0; i < inDataAccessor.size(); ++i) {
	const DataPointT & dataPoint = inDataAccessor(i);
	if (dataPoint.y < yMin) {
	  xMin = dataPoint.x;
	  yMin = dataPoint.y;
	}
      }

      cerr << "xMin: " << xMin << ", yMin: " << yMin << endl;

      // TODO: Improve guess!
      float aGuess = 1;
      float bGuess = -2.0 * aGuess * xMin;
      float cGuess = yMin + aGuess * xMin * xMin;

      cerr << "Guessing a=" << aGuess << ", b=" << bGuess << ", c=" << cGuess << endl;

      gsl_vector_set(guess, ParamsIdxT::A_IDX, aGuess);
      gsl_vector_set(guess, ParamsIdxT::B_IDX, bGuess);
      gsl_vector_set(guess, ParamsIdxT::C_IDX, cGuess);
    }
  
    /* y = a * x^2 + b * x + c */
    static float fx(float x, const ParamsT & inParms) {
      float a = inParms[ParamsIdxT::A_IDX];
      float b = inParms[ParamsIdxT::B_IDX];
      float c = inParms[ParamsIdxT::C_IDX];
      float x2 = x * x;
      return (a * x2 + b * x + c);
    }

    /* Calculates f(x) = a*x^2 + b*x + c for each data point */
    // TODO: Stupid naming - parms vs resParms...
    static int gslFx(const gsl_vector * x, void * params, gsl_vector * f) {    
      /* Store the parameter data */
      const GslMultiFitParmsT * gslParams = ((GslMultiFitParmsT*) params);
    
      /* Store the current coefficient values */
      ParamsT resParams; // TODO: We may create a constructor which takes a gsl_vector for init!
      for (size_t i = 0; i < ParamsIdxT::_Count; ++i) {
	ParamsIdxT::TypeE idx = static_cast<ParamsIdxT::TypeE>(i);
	resParams[i] = gsl_vector_get(x, idx);
      }

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
      float a = gsl_vector_get(x, ParamsIdxT::A_IDX);
      float b = gsl_vector_get(x, ParamsIdxT::B_IDX);
      float c = gsl_vector_get(x, ParamsIdxT::C_IDX);
    
      for(size_t i = 0; i < gslParams->size(); ++i) {
	const GslMultiFitDataT & gslData = gslParams->at(i);
	const float oneBySigma = 1.0f / gslData.sigma;
	const float x = gslData.pt.x;

	gsl_matrix_set(J, i, ParamsIdxT::A_IDX, oneBySigma * x * x);
	gsl_matrix_set(J, i, ParamsIdxT::B_IDX, oneBySigma * x);
	gsl_matrix_set(J, i, ParamsIdxT::C_IDX, oneBySigma);
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
