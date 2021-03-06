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

#ifndef _VCURVE_TMPL_H_
#define _VCURVE_TMPL_H_ _VCURVE_TMPL_H_

#include <iosfwd>
#include <map>
#include <limits>

#include "at_exception.hpp"

using namespace std;

namespace AT {

  DEF_Exception(VCurve);


  /**
   * The VCurve is a container for X-Y coordinate pairs.
   * X represents the focuser relative position in steps.
   * Y represents the focus measure (HFD or FWHM - a simple double value in arcsec).
   * Allow only one Y-value per X value -> A VCurve can be implemented as a map<X, Y>.
   * VCurves can be added if they have exactly the same indices in X direction.
   * A VCurve can be divided by a scalar value i.e. each Y value is divided by the same scalar value.
   * The values of a VCurve can be printed to the console.
   * A VCurve is serializable, we may use the boost XML writer or implement a simple (x,y) format.
   * The data types for X and Y can be suplied by two template parameters.
   * It is possible to insert a value X-Y pair to a VCurve (standard map interface).
   * It is possible to remove a value X-Y pair from a VCurve (standard map interface).
   * TODO: Clarify ! We need index access by x -> no map possible!
   *
   * idx    focus pos    fitness
   *  [0] -> [1234]    -> [1.55]
   *   .       .           .
   *   .       .           .
   *  [9] -> [23444]   -> [2.35]
   *
   * vector< pair<x, y> >  -> x can be contained multiple times...
   * 
   * Use of boost multi_index_containers http://www.boost.org/doc/libs/1_47_0/libs/multi_index/doc/tutorial/index.html#rationale
   * We need access from idx 0...n to the fitness value AND from focus pos to the fitness value!
   */

  template<typename X, typename Y>
  class VCurveTmplT : public map<X, Y> {
  private:

    struct AddSubVCurvesT {
      enum TypeE {
	ADD_VCURVES,
	SUB_VCURVES,
	_Count
      };
      
      static const char * asStr(const TypeE & inType) {
	switch (inType) {
	case ADD_VCURVES: return "ADD_VCURVES";
	case SUB_VCURVES: return "SUB_VCURVES";
	default: return "<?>";
	}
      }
    }; // end struct
    
    static inline void addSub(typename AddSubVCurvesT::TypeE inVCurveType, const VCurveTmplT & c1, const VCurveTmplT & c2, VCurveTmplT * outResVCurve) {
      LOG(debug) << "Entering VCurve addSub(" << AddSubVCurvesT::asStr(inVCurveType) << ", ...)..." << endl;

      if (! outResVCurve) {
	throw VCurveExceptionT("No outVCurve set.");
      }

      if (c1.empty() && c2.empty()) {
	LOG(debug) << "c1 and c2 empty... nothing to add/sub." << endl;
      } else if (c1.empty()) {
	// Only c1 empty --> outResVCurve = c2, care about sign of c2
	int sign = (inVCurveType == AddSubVCurvesT::SUB_VCURVES ? -1 : 1);
	LOG(debug) << "c1 is empty... assign c2 with sign " << sign << "." << endl;
	*outResVCurve = sign * c2; // use of operator*
      } else if (c2.empty()) {
	// Only c2 empty --> outResVCurve = c1, sign of c2 does not matter since vcurve is empty
	LOG(debug) << "c2 is empty... assign c1, ignore sign." << endl;
	*outResVCurve = c1;
      } else {
	// None of both empty --> check if equal size and same indices...
	LOG(debug) << "c1 and c2 not empty... assign c1, ignore sign." << endl;

	if (c1.size() != c2.size()) {
	  stringstream ssExc;
	  ssExc << "Number of VCurve entries have to be identical for add operation - c1: " << c1.size() << ", c2: " << c2.size() << ".";
	  const string tmpStr = ssExc.str();
	  throw VCurveExceptionT(tmpStr.c_str());
	}
	
	for (typename VCurveTmplT::const_iterator c1It = c1.begin(); c1It != c1.end(); ++c1It) {
	  typename VCurveTmplT::const_iterator c2It = c2.find(c1It->first);
	  
	  if (c2It == c2.end()) {
	    throw VCurveExceptionT("X indices of VCurves have to be identical for add operation.");
	  }
	  
	  (*outResVCurve)[c1It->first] = (inVCurveType == AddSubVCurvesT::ADD_VCURVES ? c1It->second + c2It->second : c1It->second - c2It->second);
	}
      }
      return;
    }
    
  public:
    VCurveTmplT() {
    }
    
    /*
     * Min / Max functions for X and Y values
     */
    inline X getMinX() const {
      if (this->empty())
	return 0;
      //throw VCurveExceptionT("VCurve empty, no minX.");
      return this->begin()->first;
    }
    inline X getMaxX() const {
      if (this->empty())
	return 0;
      //throw VCurveExceptionT("VCurve empty, no maxX.");
      return this->rbegin()->first;
    }
    inline Y getMinY(X * outPos = 0) const {
      Y curMin = std::numeric_limits<Y>::max();
      for (typename VCurveTmplT::const_iterator it = this->begin(); it != this->end(); ++it) {
	if (curMin > it->second) {
	  curMin = it->second;
	  
	  if (outPos)
	    *outPos = it->first;
	}
      }
      return curMin;
    }
    inline Y getMaxY(X * outPos = 0) const {
      Y curMax = std::numeric_limits<Y>::min();
      for (typename VCurveTmplT::const_iterator it = this->begin(); it != this->end(); ++it) {
	if (curMax < it->second) {
	  curMax = it->second;
	  if (outPos)
	    *outPos = it->first;
	}
      }
      return curMax;
    }

    /*
     * Overloading operators for simple VCurve calculations
     */
    VCurveTmplT operator+(const VCurveTmplT & inOtherVCurve) {
      VCurveTmplT newVCurve;
      addSub(AddSubVCurvesT::ADD_VCURVES, *this, inOtherVCurve, & newVCurve);
      return newVCurve;
    }

    VCurveTmplT operator+=(const VCurveTmplT & inOtherVCurve) {
      addSub(AddSubVCurvesT::ADD_VCURVES, *this, inOtherVCurve, this);
      return *this;
    }

    VCurveTmplT operator-(const VCurveTmplT & inOtherVCurve) {
      VCurveTmplT newVCurve;
      addSub(AddSubVCurvesT::SUB_VCURVES, *this, inOtherVCurve, & newVCurve);
      return newVCurve;
    }

    VCurveTmplT operator-=(const VCurveTmplT & inOtherVCurve) {
      addSub(AddSubVCurvesT::SUB_VCURVES, *this, inOtherVCurve /* this one is subtracted */, this);
      return *this;
    }

  
    VCurveTmplT operator/(const double & inScalar) {
      if (! inScalar)
	throw VCurveExceptionT("Dividing a VCurve by 0 is not allowed.");

      VCurveTmplT newCurve(*this); // Create a copy of VCurve before dividing
      for (typename VCurveTmplT::iterator it = newCurve.begin(); it != newCurve.end(); ++it) {
	it->second /= inScalar;
      }
      return newCurve;
    }

    VCurveTmplT operator/=(const double & inScalar) {
      if (! inScalar)
	throw VCurveExceptionT("Dividing a VCurve by 0 is not allowed.");

      for (typename VCurveTmplT::iterator it = this->begin(); it != this->end(); ++it) {
	it->second /= inScalar;
      }
      return *this;
    }

    // VCurveTmplT * scalar
    VCurveTmplT operator*(const double & inScalar) {
      VCurveTmplT newCurve(*this); // Create a copy of VCurve before multiplying
      for (typename VCurveTmplT::iterator it = newCurve.begin(); it != newCurve.end(); ++it) {
	it->second *= inScalar;
      }
      return newCurve;
    }

    VCurveTmplT operator*=(const double & inScalar) {
      for (typename VCurveTmplT::iterator it = this->begin(); it != this->end(); ++it) {
	it->second *= inScalar;
      }
      return *this;
    }

    bool operator==(const VCurveTmplT & inOtherVCurve) {
      if (this->size() != inOtherVCurve.size())
	return false;

      for (typename VCurveTmplT::const_iterator c1It = this->begin(); c1It != this->end(); ++c1It) {
	typename VCurveTmplT::const_iterator c2It = inOtherVCurve.find(c1It->first);
      
	if (c2It == inOtherVCurve.end())
	  return false;

	// FIXME: Comparing double values will probably fail this way...
	if (c1It->first != c2It->first || c1It->second != c2It->second)
	  return false;
      }
      return true;
    }

    //TODO: void saveToStream();
    //TODO: void loadFroMStream();
  };

  template<typename X, typename Y>
  ostream & operator<<(ostream & os, const VCurveTmplT<X, Y> & vcurve) {
    if (!vcurve.size()) {
      os << "VCurve is empty." << endl;
    } else {
      os << "VCurve (" << vcurve.size() << " entries)" << endl;
      for (typename VCurveTmplT<X,Y>::const_iterator it = vcurve.begin(); it != vcurve.end(); ++it) {
	os << "(x, y) = (" << it->first << ", " << it->second << ")" << endl;
      }
    }
    return os;
  }

  // scalar * VCurveTmplT
  template<typename X, typename Y>
  VCurveTmplT<X,Y> operator*(const double & inScalar, const VCurveTmplT<X, Y> & inVCurve) {
    VCurveTmplT<X, Y> newCurve(inVCurve); // Create a copy of VCurve before multiplying
    for (typename VCurveTmplT<X, Y>::iterator it = newCurve.begin(); it != newCurve.end(); ++it) {
      it->second *= inScalar;
    }
    return newCurve;
  }

} // end AT

#endif // _VCURVE_TMPL_H_
