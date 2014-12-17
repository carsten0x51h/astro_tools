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

/* 2-dimensional coordinate with floating-point values */
// TODO: Question is if this is generic or not! For now we put it here!
typedef struct _fitgsl_point2df {
  float x;
  float y;
} fitgsl_point2df;

/* Data storage structure */
// TODO: Is this common?
typedef struct _fitgsl_data {
  int n;
  fitgsl_point2df *pt;
} fitgsl_data;

/* Function parameter, passed to each of the f(x), f'(x) and fdf(x) */
// // TODO: Is this common?
typedef struct _fitgsl_fnparams {
  int n;
  float *y;
  float *sigma;
  fitgsl_point2df *pt;
} fitgsl_fnparams;


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
  static void makeGuess(const fitgsl_data * dat, gsl_vector * guess) {
    float y_mean = 0;
    float y_max = dat->pt[0].y;
    float c = dat->pt[0].x;
    
    for(int i = 0; i < dat->n; ++i) {
      y_mean += dat->pt[i].y;
      
      if(y_max < dat->pt[i].y) {
	y_max = dat->pt[i].y;
	c = dat->pt[i].x;
      }
    }
    y_mean /= (float)dat->n;
    float w = (dat->pt[dat->n-1].x - dat->pt[0].x) / 10.0;
    
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
    /* Store the parameter data */
    int n = ((fitgsl_fnparams*)params)->n;
    float * y = ((fitgsl_fnparams*)params)->y;
    float * sigma = ((fitgsl_fnparams*)params)->sigma;
    fitgsl_point2df * pt = ((fitgsl_fnparams*)params)->pt;
    
    /* Store the current coefficient values */
    ParamsT resParams; // TODO: We may create a constructor which takes a gsl_vector for init!
    for (size_t i=0; i < ParamsIdxT::_Count; ++i) {
      ParamsIdxT::TypeE idx = static_cast<ParamsIdxT::TypeE>(i);
      resParams[i] = gsl_vector_get(x, idx);
    }

    /* Execute Levenberg-Marquart on f(x) */
    for(int i = 0; i < n; ++i) {
      float yi = GaussianFitTraitsT::fx((float) pt[i].x, resParams);
      gsl_vector_set(f, i, (yi - y[i]) / sigma[i]);
    }
    
    return GSL_SUCCESS;
  }

  /* Calculates the Jacobian (derivative) matrix of f(x) = b + p * e^[0.5*((x-c)/w)^2] for each data point */
  static int gslDfx(const gsl_vector *x, void * params, gsl_matrix *J) {
    
    /* Store parameter values */
    int n = ((fitgsl_fnparams*)params)->n;
    float * sigma = ((fitgsl_fnparams*)params)->sigma;
    fitgsl_point2df * pt = ((fitgsl_fnparams*)params)->pt;
    
    /* Store current coefficients */
    float p = gsl_vector_get(x, ParamsIdxT::P_IDX);
    float c = gsl_vector_get(x, ParamsIdxT::C_IDX);
    float w = gsl_vector_get(x, ParamsIdxT::W_IDX);
    
    /*  Store non-changing calculations */
    float w2 = w * w;
    float w3 = w2 * w;
    
    for(int i = 0; i < n; ++i) {
      float x_minus_c = (pt[i].x - c);
      float e = exp(-0.5f * (x_minus_c / w) * (x_minus_c / w));
      
      gsl_matrix_set(J, i, ParamsIdxT::B_IDX, 1 / sigma[i]);
      gsl_matrix_set(J, i, ParamsIdxT::P_IDX, e / sigma[i]);
      gsl_matrix_set(J, i, ParamsIdxT::C_IDX, (p * e * x_minus_c) / (sigma[i] * w2));
      gsl_matrix_set(J, i, ParamsIdxT::W_IDX, (p * e * x_minus_c * x_minus_c) / (sigma[i] * w3));
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
  static void makeGuess(const fitgsl_data * dat, gsl_vector * guess) {
    float x_min = dat->pt[0].x;
    float y_min = dat->pt[0].y;

    // Find minimum y value...
    for(int i = 0; i < dat->n; ++i) {
      if(y_min > dat->pt[i].y) {
    	y_min = dat->pt[i].y;
    	x_min = dat->pt[i].x;
      }
    }

    cerr << "x_min: " << x_min << ", y_min: " << y_min << endl;

    // TODO: Improve guess!
    float a_guess = 1;
    float b_guess = -2.0 * a_guess * x_min;
    float c_guess = y_min + a_guess * x_min * x_min;

    cerr << "Guessing a=" << a_guess << ", b=" << b_guess << ", c=" << c_guess << endl;

    gsl_vector_set(guess, ParamsIdxT::A_IDX, a_guess);
    gsl_vector_set(guess, ParamsIdxT::B_IDX, b_guess);
    gsl_vector_set(guess, ParamsIdxT::C_IDX, c_guess);
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
    int n = ((fitgsl_fnparams*)params)->n;
    float * y = ((fitgsl_fnparams*)params)->y;
    float * sigma = ((fitgsl_fnparams*)params)->sigma;
    fitgsl_point2df * pt = ((fitgsl_fnparams*)params)->pt;
    
    /* Store the current coefficient values */
    ParamsT resParams; // TODO: We may create a constructor which takes a gsl_vector for init!
    for (size_t i=0; i < ParamsIdxT::_Count; ++i) {
      ParamsIdxT::TypeE idx = static_cast<ParamsIdxT::TypeE>(i);
      resParams[i] = gsl_vector_get(x, idx);
    }

    /* Execute Levenberg-Marquart on f(x) */
    for(int i = 0; i < n; ++i) {
      float yi = ParabelFitTraitsT::fx((float) pt[i].x, resParams);
      gsl_vector_set(f, i, (yi - y[i]) / sigma[i]);
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
  static int gslDfx(const gsl_vector *x, void * params, gsl_matrix *J) {
    
    /* Store parameter values */
    int n = ((fitgsl_fnparams*)params)->n;
    float * sigma = ((fitgsl_fnparams*)params)->sigma;
    fitgsl_point2df * pt = ((fitgsl_fnparams*)params)->pt;
    
    /* Store current coefficients */
    float a = gsl_vector_get(x, ParamsIdxT::A_IDX);
    float b = gsl_vector_get(x, ParamsIdxT::B_IDX);
    float c = gsl_vector_get(x, ParamsIdxT::C_IDX);
    
    
    for(int i = 0; i < n; ++i) {
      float oneBySigma = 1.0f / sigma[i];
      float x = pt[i].x;

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


DEF_Exception(FunctionFit); // TODO: One of the two enough?! Or better naming?!
DEF_Exception(CurveFitting);


template <class FitTraitsT>
class FunctionFitTmplT {
private:

public:
  typedef typename FitTraitsT::ParamsT ParamsT;
  typedef typename FitTraitsT::ParamsIdxT IdxT;

  /* Allocates a 'data' structure with n points */
  // TODO: User should not care about that!!!
  static fitgsl_data * fitgsl_alloc_data(int n) {
    fitgsl_data *d;
    
    /* Validate the number of points */
    if( n < 0 )
      return NULL;
    
    /* Allocate the structure */
    if( (d = (fitgsl_data*)malloc(sizeof(fitgsl_data))) == NULL )
      return NULL;
    
    /* Allocate the data point memory */
    if( (d->pt = (fitgsl_point2df*)malloc(n * sizeof(fitgsl_point2df))) == NULL) {
      free(d);
      return NULL;
    }	
    
    /* Initialize structure data */
    d->n	= n;
    memset(d->pt, 0, n * sizeof(fitgsl_point2df));
    
    return d;
  }

  /* Free a 'data' structure */
  // TODO: User should not care about that!!!
  static void fitgsl_free_data(fitgsl_data *dat) {
    free(dat->pt);
    free(dat);
  }


  /* Fits the specified data to the function, storing the coefficients in 'results' */
  // TODO: Rename function...
  // TODO: Add logging!
  // Return type should be void, throws in case of exception!
  // See http://en.wikipedia.org/wiki/Approximation_error for expl. of rel and abs errors.
  static int fitgsl_lm(const fitgsl_data * dat, ParamsT * outResults, double epsabs, double epsrel) {
    
    AT_ASSERT(FunctionFit, outResults, "Result vector not set!");

    // Store and allocate parameter info
    fitgsl_fnparams p; // Passed to gsl_multifit_function_fdf and filled there! 
    p.n     = dat->n;
    p.y     = (float*)malloc(sizeof(float) * dat->n);
    p.sigma = (float*)malloc(sizeof(float) * dat->n);
    p.pt    = dat->pt;
    
    // Fill in function info
    gsl_multifit_function_fdf f;
    f.f      = FitTraitsT::gslFx;
    f.df     = FitTraitsT::gslDfx;
    f.fdf    = FitTraitsT::gslFdfx;
    f.n      = dat->n;
    f.p      = FitTraitsT::ParamsIdxT::_Count;
    f.params = & p;
    
    // Copy the data into the function parameter object
    for(int i = 0; i < dat->n; ++i) {
      p.y[i]     = dat->pt[i].y;
      p.sigma[i] = 0.1f;
    }
    
    // Allocate the guess vector
    gsl_vector * guess = gsl_vector_alloc(FitTraitsT::ParamsIdxT::_Count);
    
    // Make initial guesses based on the data
    FitTraitsT::makeGuess(dat, guess);

    
    // Create a Levenberg-Marquardt solver with n data points and m parameters
    const gsl_multifit_fdfsolver_type * solverType = gsl_multifit_fdfsolver_lmsder;
    gsl_multifit_fdfsolver * solver = gsl_multifit_fdfsolver_alloc(solverType, dat->n, FitTraitsT::ParamsIdxT::_Count);

    AT_ASSERT(FunctionFit, solver, "Solver is NULL!");

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


    // Fit-Index
    // { 
    //   gsl_matrix *covar = gsl_matrix_alloc (4, 4);
    //   gsl_multifit_covar (solver->J, 0.0, covar);
    //   double chi = gsl_blas_dnrm2(solver->f);
    //   double dof = dat->n - 4;
    //   double c = GSL_MAX_DBL(1, chi / sqrt(dof)); 
    
    //   printf("chisq/dof = %g\n",  pow(chi, 2.0) / dof);
    //   printf ("B = %.5f +/- %.5f\n", results[FITGSL_B_INDEX], c*sqrt(gsl_matrix_get(covar, FITGSL_B_INDEX, FITGSL_B_INDEX)));
    //   printf ("P = %.5f +/- %.5f\n", results[FITGSL_P_INDEX], c*sqrt(gsl_matrix_get(covar, FITGSL_P_INDEX, FITGSL_P_INDEX)));
    //   printf ("C = %.5f +/- %.5f\n", results[FITGSL_C_INDEX], c*sqrt(gsl_matrix_get(covar, FITGSL_C_INDEX, FITGSL_C_INDEX)));
    //   printf ("W = %.5f +/- %.5f\n", results[FITGSL_W_INDEX], c*sqrt(gsl_matrix_get(covar, FITGSL_W_INDEX, FITGSL_W_INDEX)));
    //   gsl_matrix_free (covar);
    // }  
    
    // Free parameter memory
    free(p.y);
    free(p.sigma);
    
    // Free GSL memory
    gsl_multifit_fdfsolver_free(solver);
    gsl_vector_free(guess);
    
    return (status == 0 ? 0 : -1);
  }
  

};


#endif	/* __FITGSL_TMPL_HPP__ */
