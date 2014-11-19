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

#ifndef _UTIL_HPP_
#define _UTIL_HPP_ _UTIL_HPP_

#include <iosfwd>
#include <limits> // To get epsilon - see: http://stackoverflow.com/questions/17333/most-effective-way-for-float-and-double-comparison

#include <ctime>
#include <sys/time.h>
#include <unistd.h>

#include <boost/tuple/tuple.hpp>
#include <boost/tuple/tuple_io.hpp>
#include "boost/tuple/tuple_comparison.hpp"

#include "at_exception.hpp"

// TODO: Put everything to AT namespace?!?!?!?!

using namespace std;
using namespace boost;

#define START_MEASURE(__name__)			\
  struct timeval __name__##start;		\
  gettimeofday(& __name__##start, NULL);	\
  
#define MEASURE(__name__, __resms__)					\
  {									\
    struct timeval __name__##end;					\
    long __name__##secs, __name__##usecs;				\
    gettimeofday(&__name__##end, NULL);					\
    __name__##secs  = __name__##end.tv_sec  - __name__##start.tv_sec;	\
    __name__##usecs = __name__##end.tv_usec - __name__##start.tv_usec;	\
    __resms__ = ((__name__##secs) * 1000 + __name__##usecs / 1000.0) + 0.5; \
  }									\

DEF_Exception(Timeout);

#define WAIT_MAX_FOR(__cond__, __timeout__, __excmsg__)			\
  if (__timeout__) {							\
     START_MEASURE(__timeout);						\
     bool __hitTimeout = false;						\
     while (!(__cond__) && ! __hitTimeout) {				\
       long __timeMs;							\
       usleep(1000);							\
       MEASURE(__timeout, __timeMs);					\
       __hitTimeout = ( __timeout__ >= 0 ? (__timeout__ <= __timeMs) : false); \
     }									\
     if (__hitTimeout)							\
       throw TimeoutExceptionT(__excmsg__);				\
  }									\


#define WAIT_MAX_FOR_PRINT(__cond__, __timeout__, __quiet__, __pmsg__, __excmsg__) \
  if (__timeout__) {							\
    static size_t pos = 0;						\
    static const char * progressChars = "/-\\";				\
    START_MEASURE(__timeout__);						\
    bool __hitTimeout = false;						\
    while (! (__cond__) && ! __hitTimeout) {				\
      long __timeMs;							\
      usleep(1000);							\
      MEASURE(__timeout__, __timeMs);					\
      __hitTimeout = (__timeout__ >= 0 ? (__timeout__ <= __timeMs) : false); \
      if(! (__quiet__)) {						\
	size_t idx = (++pos) % (strlen(progressChars));			\
	cout << "\r" << progressChars[idx] << " Please wait..." << __pmsg__ << flush; \
	if (__timeout__ > 0)						\
	  cout << (unsigned int) (100.0 * __timeMs / __timeout__) << "%" << flush; \
      }									\
      usleep(1000);							\
    }									\
    cout << endl;							\
    if (__hitTimeout)							\
      throw TimeoutExceptionT(__excmsg__);				\
  }									\

/**
 * Floating point comparison function.
 */
template <class T>
bool rough_eq(T lhs, T rhs, T epsilon = std::numeric_limits<T>::epsilon()) { return fabs(lhs - rhs) < epsilon; }


/**
 * DimensionT structure (W x H).
 */
typedef boost::tuple<double, double> DimensionT;

/**
 * PositionT structure (X x Y).
 */
typedef boost::tuple<double, double> PositionT;

/**
 * FrameT structure (X x Y x W x H).
 */
typedef boost::tuple<int, int, unsigned int, unsigned int> FrameT;

/**
 * BinningT structure (X, Y).
 */
typedef boost::tuple<unsigned int, unsigned int> BinningT;

/**
 * Helper class to encapsulate hostname and port.
 */
class HostnameAndPortT {
private:
  string mHostname;
  size_t mPort;

public:
  HostnameAndPortT(const string & inHostname, size_t inPort) : mHostname(inHostname), mPort(inPort) { }
  const string & getHostname() const { return mHostname; }
  size_t getPort() const { return mPort; }
  ostream & print(ostream & os) const {
    os << mHostname << ":" << mPort;
    return os;
  }
  friend ostream & operator<<(ostream & os, const HostnameAndPortT & inHostnameAndPort);

};

ostream & operator<<(ostream & os, const HostnameAndPortT & inHostnameAndPort);



/**
 * outResFrame contains subframe (FrameT) of intersection with given frame and chip dimension.
 * Returns true if there is any intersection, otherwise false if frame lies out of image bounds.
 */
static bool intersect(const DimensionT & inDimension, const FrameT & inFrame, FrameT * outResFrame = 0) {
  typedef boost::tuple<int, int, int, int> RectT;
  RectT rect1(0 /*x1*/, 0 /*y1*/, inDimension.get<0>() /*x2 = width*/, inDimension.get<1>() /*y2 = height*/);
  RectT rect2(inFrame.get<0>() /*x1*/, inFrame.get<1>() /*y1*/, inFrame.get<0>() + inFrame.get<2>() /*x2=x1+w*/,
	      inFrame.get<1>() + inFrame.get<3>() /*y2=y1+h*/);

  RectT rectRes(max(rect1.get<0>()/*x1*/, rect2.get<0>()/*x1*/), max(rect1.get<1>()/*y1*/, rect2.get<1>()/*y1*/),
		min(rect1.get<2>()/*x2*/, rect2.get<2>()/*x2*/), min(rect1.get<3>()/*y2*/, rect2.get<3>()/*y2*/));
  
  if (rectRes.get<0>()/*x1*/ >= rectRes.get<2>()/*x2*/ || rectRes.get<1>()/*y1*/ >= rectRes.get<3>()/*y2*/)
    return false;

  if (outResFrame)
    *outResFrame = FrameT(rectRes.get<0>()/*x1*/, rectRes.get<1>()/*y1*/,
			  rectRes.get<2>()/*x2*/ - rectRes.get<0>()/*x1*/,
			  rectRes.get<3>()/*y2*/ - rectRes.get<1>()/*y1*/);

  return true;
}

static bool isWindowInBounds(DimensionT inImgSize, PositionT inCentroid, unsigned int inWindowSize) {
  const unsigned int halfWindowSize = inWindowSize / 2; // TODO: use >> 1 operator instead?! What if windowSize is even?
  FrameT inputFrame(inCentroid.get<0>() - halfWindowSize /*x*/, inCentroid.get<1>() - halfWindowSize /*y*/, inWindowSize /*w*/, inWindowSize /*h*/);
  FrameT resultFrame;
  bool hasIntersection = intersect(inImgSize, inputFrame, & resultFrame);
  return (inputFrame == resultFrame && hasIntersection);
}


#define MAC_AS_TYPE(Type, E, Count)				\
    static inline Type##E as##Type(const char * inX) {		\
	for (size_t i = 0; i < Count; ++i) {			\
	    Type##E type = static_cast<Type##E>(i);		\
	    if (! strcasecmp(inX, asStr(type))) {		\
		return type;					\
	    }							\
	}							\
	return Count;						\
    }

#endif /* _UTIL_HPP_ */
