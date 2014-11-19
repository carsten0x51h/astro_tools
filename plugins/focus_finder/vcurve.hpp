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
// http://www.youtube.com/watch?v=oiawjpuVglo
#include <iostream>
#include <map>
#include <exception>
#include <limits>

using namespace std;


class VCurveExceptionT : public std::exception {
public:
  // TODO: Change to stream....
  VCurveExceptionT(const string & inExceptionMsg) : mMsg(inExceptionMsg) {
  }
  ~VCurveExceptionT() throw() {}
  const char * what() const throw() { return mMsg.c_str(); }

private:
  string mMsg;
};


/**
 * The VCurve is a container for X-Y coordinate pairs.
 * X represents the focuser relative position in steps.
 * Y represents the focus measure (HFD or FWHM - a simple double value in arcsec).
 * Allow only one Y-value per X value -> A VCurve can be implemented as a map<X, Y>.
 * VCurves can be added if they have exactly the same indices in X direction.
 * A VCurve can be divided by a scalar value i.e. each Y value is divided by the same scalar value.
 * The values of a VCurve can be printed to the console.
 * A VCurve is serializable, we may use the boost XML writer or implement a simple (x,y) format.
 * A VCurve has a method to calculate the mean square linear for down- and up directions (we may use gsl).
 * A VCurve has a method to calculate the "optimal" focus X position from the given values (requirement: # of values?).
 * A the data types for X and Y can be suplied by two template parameters.
 * It is possible to add a value X-Y pair to a VCurve (standard map interface).
 * It is possible to remove a value X-Y pair from a VCurve (standard map interface).
 */
template<typename X, typename Y>
class VCurveT : public map<X, Y> {
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


  static inline void addSub(typename AddSubVCurvesT::TypeE inVCurveType, const VCurveT & c1, const VCurveT & c2, VCurveT * outResVCurve) {
    VCurveT newCurve;

    // Check if both VCurves can be added
    if (c1.size() != c2.size())
      throw VCurveExceptionT("Number of VCurve entries have to be identical for add operation.");

    for (typename VCurveT::const_iterator c1It = c1.begin(); c1It != c1.end(); ++c1It) {
      typename VCurveT::const_iterator c2It = c2.find(c1It->first);

      if (c2It == c2.end())
	throw VCurveExceptionT("X indices of VCurves have to be identical for add operation.");

      if (! outResVCurve)
	return;

      (*outResVCurve)[c1It->first] = (inVCurveType == AddSubVCurvesT::ADD_VCURVES ? c1It->second + c2It->second : c1It->second - c2It->second);
    }
    return;
  }

public:
  VCurveT() {
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
    for (typename VCurveT::const_iterator it = this->begin(); it != this->end(); ++it) {
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
    for (typename VCurveT::const_iterator it = this->begin(); it != this->end(); ++it) {
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
  VCurveT operator+(const VCurveT & inOtherVCurve) {
    VCurveT newVCurve;
    addSub(AddSubVCurvesT::ADD_VCURVES, *this, inOtherVCurve, & newVCurve);
    return newVCurve;
  }

  VCurveT operator-(const VCurveT & inOtherVCurve) {
    VCurveT newVCurve;
    addSub(AddSubVCurvesT::SUB_VCURVES, *this, inOtherVCurve, & newVCurve);
    return newVCurve;
  }
  
  VCurveT operator/(const double & inScalar) {
    if (! inScalar)
      throw VCurveExceptionT("Dividing a VCurve by 0 is not allowed.");

    VCurveT newCurve(*this); // Create a copy of VCurve before dividing
    for (typename VCurveT::iterator it = newCurve.begin(); it != newCurve.end(); ++it) {
      it->second /= inScalar;
    }
    return newCurve;
  }

  // VCurveT * scalar
  VCurveT operator*(const double & inScalar) {
    VCurveT newCurve(*this); // Create a copy of VCurve before multiplying
    for (typename VCurveT::iterator it = newCurve.begin(); it != newCurve.end(); ++it) {
      it->second *= inScalar;
    }
    return newCurve;
  }

  bool operator==(const VCurveT & inOtherVCurve) {
    if (this->size() != inOtherVCurve.size())
      return false;

    for (typename VCurveT::const_iterator c1It = this->begin(); c1It != this->end(); ++c1It) {
      typename VCurveT::const_iterator c2It = inOtherVCurve.find(c1It->first);
      
      if (c2It == inOtherVCurve.end())
	return false;

      // Note: Comparing double values will probably fail this way...
      if (c1It->first != c2It->first || c1It->second != c2It->second)
	return false;
    }
    return true;
  }

  //TODO: void saveToStream();
  //TODO: void loadFroMStream();
};

template<typename X, typename Y>
ostream & operator<<(ostream & os, const VCurveT<X, Y> & vcurve) {
  if (!vcurve.size()) {
    os << "VCurve is empty." << endl;
  } else {
    os << "VCurve (" << vcurve.size() << " entries)" << endl;
    for (typename VCurveT<X,Y>::const_iterator it = vcurve.begin(); it != vcurve.end(); ++it) {
      os << "(x, y) = (" << it->first << ", " << it->second << ")" << endl;
    }
  }
  return os;
}

// scalar * VCurveT
template<typename X, typename Y>
VCurveT<X,Y> operator*(const double & inScalar, const VCurveT<X, Y> & inVCurve) {
  VCurveT<X, Y> newCurve(inVCurve); // Create a copy of VCurve before multiplying
  for (typename VCurveT<X, Y>::iterator it = newCurve.begin(); it != newCurve.end(); ++it) {
    it->second *= inScalar;
  }
  return newCurve;
}

#endif // _VCURVE_TMPL_H_
