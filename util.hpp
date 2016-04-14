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
#include <boost/tuple/tuple_comparison.hpp>

#include <CImg.h>

#include "at_exception.hpp"
#include "at_logging.hpp"

using namespace std;
using namespace boost;
using namespace cimg_library;

DEF_Exception(Timeout);

// NOTE: Defines are not bound to any namespace...
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
	  cout << (unsigned int) ((__timeout__ - __timeMs) / 1000.0) << "s to timeout" << flush; \
      }									\
      usleep(1000);							\
    }									\
    cout << endl;							\
    if (__hitTimeout)							\
      throw TimeoutExceptionT(__excmsg__);				\
  }									\

#define MAC_AS_TYPE(Type, E, Count)			\
  static inline Type##E as##Type(const char * inX) {	\
    for (size_t i = 0; i < Count; ++i) {		\
      Type##E type = static_cast<Type##E>(i);		\
      if (! strcasecmp(inX, asStr(type))) {		\
	return type;					\
      }							\
    }							\
    return Count;					\
  }

  
/**
 * Floating point comparison function.
 */
template <class T>
bool rough_eq(T lhs, T rhs, T epsilon = std::numeric_limits<T>::epsilon()) { return fabs(lhs - rhs) < epsilon; }


/**
 * DimensionT structure (W x H).
 */
//typedef boost::tuple<double, double> DimensionT;
template <typename T> using DimensionT = boost::tuple<T, T>;

/**
 * PointT structure (X x Y).
 *
 */
//typedef boost::tuple<double, double> PointT;
template <typename T> using PointT = boost::tuple<T, T>;

/**
 * FrameT structure (X x Y x W x H).
 */
//typedef boost::tuple<int, int, unsigned int, unsigned int> FrameT;
template <typename T> using FrameT = boost::tuple<T, T, T, T>;

/**
 * BinningT structure (X, Y).
 */
// TODO: Formulate as template..
typedef boost::tuple<unsigned int, unsigned int> BinningT;

/**
 * Helper class to encapsulate hostname and port.
 */
class HostnameAndPortT {
private:
  string mHostname;
  size_t mPort;

public:
  HostnameAndPortT(const string & inHostname = "", size_t inPort = 0) : mHostname(inHostname), mPort(inPort) { }
  const string & getHostname() const { return mHostname; }
  size_t getPort() const { return mPort; }
  std::ostream & print(ostream & os) const {
    os << mHostname << ":" << mPort;
    return os;
  }
  friend std::ostream & operator<<(std::ostream & os, const HostnameAndPortT & inHostnameAndPort);
};

ostream & operator<<(ostream & os, const HostnameAndPortT & inHostnameAndPort);



/**
 * outResFrame contains subframe (FrameT) of intersection with given frame and chip dimension.
 * Returns true if there is any intersection, otherwise false if frame lies out of image bounds.
 */
static bool
intersect(const DimensionT<float> & inDimension, const FrameT<float> & inFrame, FrameT<float> * outResFrame = 0)
{
  typedef boost::tuple<float, float, float, float> RectT; // TODO: Define elsewhere? Or use Frame? What is difference between RectT and FrameT?????
  RectT rect1(0 /*x1*/, 0 /*y1*/, inDimension.get<0>() /*x2 = width*/, inDimension.get<1>() /*y2 = height*/);
  RectT rect2(inFrame.get<0>() /*x1*/, inFrame.get<1>() /*y1*/, inFrame.get<0>() + inFrame.get<2>() /*x2=x1+w*/,
	      inFrame.get<1>() + inFrame.get<3>() /*y2=y1+h*/);
  
  RectT rectRes(max(rect1.get<0>()/*x1*/, rect2.get<0>()/*x1*/), max(rect1.get<1>()/*y1*/, rect2.get<1>()/*y1*/),
		min(rect1.get<2>()/*x2*/, rect2.get<2>()/*x2*/), min(rect1.get<3>()/*y2*/, rect2.get<3>()/*y2*/));

  if (rectRes.get<0>()/*x1*/ >= rectRes.get<2>()/*x2*/ || rectRes.get<1>()/*y1*/ >= rectRes.get<3>()/*y2*/)
    return false;

  if (outResFrame)
    *outResFrame = FrameT<float>(rectRes.get<0>()/*x1*/, rectRes.get<1>()/*y1*/,
				 rectRes.get<2>()/*x2*/ - rectRes.get<0>()/*x1 -> W=x2-x1*/,
				 rectRes.get<3>()/*y2*/ - rectRes.get<1>()/*y1 -> H=y2-y1*/);  
  return true;
}

/**
* Get all pixels inside a radius: http://stackoverflow.com/questions/14487322/get-all-pixel-array-inside-circle
* Algorithm: http://en.wikipedia.org/wiki/Midpoint_circle_algorithm
*/
static bool
insideCircle(float inX /*pos of x*/, float inY /*pos of y*/, float inCenterX, float inCenterY, float inRadius) {
  return (pow(inX - inCenterX, 2.0) + pow(inY - inCenterY, 2.0) <= pow(inRadius, 2.0));
}

static FrameT<float>
centerPosToFrame(const PointT<float> & inCentroid, float inWidth, float inHeight) {
  // TODO: Or should we round here to int? Check if -1 is correct...
  //unsigned int halfWindowWidth = ceil(inWidth / 2.0f) - 1;
  //unsigned int  halfWindowHeight = ceil(inHeight / 2.0f) - 1;
  float halfWindowWidth = inWidth / 2.0f;
  float halfWindowHeight = inHeight / 2.0f;
  return FrameT<float>(inCentroid.get<0>() /*cx*/ - halfWindowWidth, inCentroid.get<1>() /*cy*/ - halfWindowHeight, inWidth, inHeight);
}

static FrameT<float>
centerPosToFrame(const PointT<float> & inCentroid, float inWindowSize) {
  return centerPosToFrame(inCentroid, inWindowSize, inWindowSize);
}

static PointT<float>
frameToCenterPos(const FrameT<float> & inFrame) {
  return PointT<float>(inFrame.get<0>() /*x0*/ + (inFrame.get<2>() /*w*/ / 2.0), inFrame.get<1>() /*y0*/ + (inFrame.get<3>() /*h*/ / 2.0));
}

static float
distance(const PointT<float> & inPoint1, const PointT<float> & inPoint2) {
  return sqrt(pow(inPoint1.get<0>() - inPoint2.get<0>(), 2.0) + pow(inPoint1.get<1>() - inPoint2.get<1>(), 2.0));
}

/* Returns true if the selection frame is fully inside the bounds. */
static bool
insideBounds(const DimensionT<float> & inDimension, const FrameT<float> & inSelectionFrame) {
  FrameT<float> resultFrame;
  bool hasIntersection = intersect(inDimension, inSelectionFrame, & resultFrame);

  // NOTE: For comparison of float/double we cannot use operator==... 
  static const float epsilon = 0.001;
  bool equal = rough_eq(inSelectionFrame.get<0>(), resultFrame.get<0>(), epsilon) &&
    rough_eq(inSelectionFrame.get<1>(), resultFrame.get<1>(), epsilon) &&
    rough_eq(inSelectionFrame.get<2>(), resultFrame.get<2>(), epsilon) &&
    rough_eq(inSelectionFrame.get<3>(), resultFrame.get<3>(), epsilon);
  
  bool ret = (equal && hasIntersection);

  LOG(trace) << "insideBounds - selectionFrame: " << inSelectionFrame << ", resultFrame: " << resultFrame
	     << ", equal: " << equal << ", hasIntersection: " << hasIntersection << ", ret-value: " << ret << endl;
  return ret;
}

static bool
insideBounds(const DimensionT<float> & inDimension, const PointT<float> & inCenter, float inWindowSize) {
  FrameT<float> selectionFrame = centerPosToFrame(inCenter, inWindowSize);
  return insideBounds(inDimension, selectionFrame);
}

static FrameT<float>
rectify(const FrameT<float> & inFrame) {
  // float border = 3;
  // float border2 = 2.0 * border;
  // float width = fabs(inFrame.get<0>() - inFrame.get<2>()) + border2;
  // float height = fabs(inFrame.get<1>() - inFrame.get<3>()) + border2;
  // float L = max(width, height);
  // float x0 = inFrame.get<0>() - (fabs(width - L) / 2.0) - border;
  // float y0 = inFrame.get<1>() - (fabs(height - L) / 2.0) - border;
  // return FrameT<float>(x0, y0, x0 + L, y0 + L);
  float border = 3;
  float L = max(inFrame.get<2>() /*width*/, inFrame.get<3>() /*height*/);
  return FrameT<float>(inFrame.get<0>() - border, inFrame.get<1>() - border, L + 2.0 * border, L + 2.0 * border);
}



/**
 * Helper class - horizontal and vertical.
 */
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


/**
 * Helper class - relative and absolute coordinates. 
 */
struct CoordTypeT {
  enum TypeE {
    RELATIVE,
    ABSOLUTE,
    _Count
  };
  
  static const char * asStr(const TypeE & inType) {
    switch (inType) {
    case RELATIVE: return "RELATIVE";
    case ABSOLUTE: return "ABSOLUTE";
    default: return "<?>";
    }
  }
};


DEF_Exception(ExtractLine);

template<DirectionT::TypeE D> static vector<float>
extractLine(const CImg<float> & inImage) {
  CImg<float> img(inImage); // Make a copy
  PointT<float> center(img.width() / 2, img.height() / 2);
  vector<float> values;

  // Extract slices through centroid for profiles
  switch(D) {
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
    AT_ASSERT(ExtractLine, false, "Invalid direction.");
  }
  }
  return values;
}

template<DirectionT::TypeE D> static vector<float>
extractLine(const CImg<float> & inImage, const FrameT<float> & inFrame) {
  // TODO: Check if in bounds?!
  // TODO: is +1 correct?!
  CImg<float> resImage = inImage.get_crop(inFrame.get<0>() + 1 /* x */, inFrame.get<1>() + 1 /* y */,
					  inFrame.get<1>() /* x */ + inFrame.get<2>() /* w */,
					  inFrame.get<1>() /* y */ + inFrame.get<3>() /* h */);

  return extractLine<D>(resImage);
}

// TODO: Do not pass back vector as copy!
template<DirectionT::TypeE D> static vector<float>
extractLine(const CImg<float> & inImage, PointT<float> inCenter, size_t inWindowSizePx) {
  AT_ASSERT(ExtractLine, inWindowSizePx > 0, "Specify inWindowSizePx > 0.");
    
  // TODO: Check if in bounds?!
  FrameT<float> imgWindow = centerPosToFrame(inCenter, inWindowSizePx);
  return extractLine<D>(inImage, imgWindow);
}



static void
getMinMaxPixel(const CImg<float> & inImg, const FrameT<int> * inFrame = 0, CoordTypeT::TypeE inCoordType = CoordTypeT::ABSOLUTE,
	       float * outMin = 0, PointT<unsigned int> * outMinPixelPos = 0,float * outMax = 0, PointT<unsigned int> * outMaxPixelPos = 0) { 

  // If frame is give, we crop...
  CImg<float> cropImg;
  if (inFrame) {
    cropImg = inImg.get_crop(inFrame->get<0>(), inFrame->get<1>(),
			     inFrame->get<0>() + inFrame->get<2>() - 1,
			     inFrame->get<1>() + inFrame->get<3>() - 1);
  }
  const CImg<float> & searchImg = (inFrame ? cropImg : inImg);
  
  PointT<unsigned int> minPos, maxPos;
  float min = std::numeric_limits<float>::max();
  float max = 0;
  
  // NOTE: First minimum and maximum counts
  cimg_forXY(cropImg, x, y) {
    const float & value = cropImg(x,y);
    if (min > value) {
      min = value;
      minPos = PointT<unsigned int>(x, y);
    }
    if (max < value) {
      max = value;
      maxPos = PointT<unsigned int>(x, y);
    }
  }
  
  if (outMin) { *outMin = min; }
  if (outMinPixelPos) {
    *outMinPixelPos = minPos;
    
    if (inCoordType == CoordTypeT::ABSOLUTE && inFrame) {
      outMinPixelPos->get<0>() += inFrame->get<0>();
      outMinPixelPos->get<1>() += inFrame->get<1>();
    }
  }
  if (outMax)  { *outMax = max; }
  if (outMaxPixelPos) {
    *outMaxPixelPos = maxPos;

    if (inCoordType == CoordTypeT::ABSOLUTE && inFrame) {
      outMaxPixelPos->get<0>() += inFrame->get<0>();
      outMaxPixelPos->get<1>() += inFrame->get<1>();
    }
  }
}

static void
getMaxPixel(const CImg<float> & inImg, const FrameT<int> * inFrame = 0, CoordTypeT::TypeE inCoordType = CoordTypeT::ABSOLUTE,
	    float * outMax = 0, PointT<unsigned int> * outMaxPixelPos = 0) {
  getMinMaxPixel(inImg, inFrame, inCoordType, 0 /* outMin */, 0 /* outMinPixelPos */, outMax, outMaxPixelPos);
}

static void
getMinPixel(const CImg<float> & inImg, const FrameT<int> * inFrame = 0, CoordTypeT::TypeE inCoordType = CoordTypeT::ABSOLUTE,
	    float * outMin = 0, PointT<unsigned int> * outMinPixelPos = 0) {
  getMinMaxPixel(inImg, inFrame, inCoordType, outMin, outMinPixelPos, 0 /* outMax */, 0 /* outMaxPixelPos */);
}

/**
 * Little draw helper function to draw a cross...
 */
static void
drawCross(CImg<unsigned char> * inoutImg, float inX, float inY, const unsigned char * inColor, size_t inCrossSize = 3, float inScaleFactor = 1.0, size_t inOpacity = 1) {
  inoutImg->draw_line(floor(inScaleFactor * (inX - inCrossSize + 1) + 0.5), floor(inScaleFactor * (inY + 1) + 0.5),
		      floor(inScaleFactor * (inX + inCrossSize + 1) + 0.5), floor(inScaleFactor * (inY + 1) + 0.5),
		      inColor, inOpacity);
  
  inoutImg->draw_line(floor(inScaleFactor * (inX + 1) + 0.5), floor(inScaleFactor * (inY - inCrossSize + 1) + 0.5),
		      floor(inScaleFactor * (inX + 1) + 0.5), floor(inScaleFactor * (inY + inCrossSize + 1) + 0.5),
		      inColor, inOpacity);
}
static void
drawCross(CImg<unsigned char> * inoutImg, const PointT<float> & inPos, const unsigned char * inColor, size_t inCrossSize = 3, float inScaleFactor = 1.0, size_t inOpacity = 1) {
  drawCross(inoutImg, inPos.get<0>(), inPos.get<1>(), inColor, inCrossSize, inScaleFactor, inOpacity); 
}

static void
drawFrame(CImg<unsigned char> * inoutImg, const FrameT<float> & inFrame, const unsigned char inColor[3], float inScaleFactor = 1.0) {
  int x1 = floor(inScaleFactor * inFrame.get<0>() + 0.5);
  int y1 = floor(inScaleFactor * inFrame.get<1>() + 0.5);
  int x2 = floor(inScaleFactor * (inFrame.get<0>() + inFrame.get<2>()) + 0.5);
  int y2 = floor(inScaleFactor * (inFrame.get<1>() + inFrame.get<3>()) + 0.5);
  inoutImg->draw_rectangle(x1, y1, x2, y2, inColor, 1 /*opacity*/, ~0 /*pattern*/);
}


template <typename T>
using PointLstT = std::list<PointT<T> >;


DEF_Exception(MeanCalc);

template <typename T>
static void
calcMeans(const PointLstT<T> & inDataPoints, T * outMeanX, T * outMeanY) {
  AT_ASSERT(MeanCalc, outMeanX && outMeanY, "outMeanX and outMeanY have to be set.");
  T sumX = 0, sumY = 0;
  for (typename PointLstT<T>::const_iterator it = inDataPoints.begin(); it != inDataPoints.end(); ++it) {
    sumX += it->get<0>();
    sumY += it->get<1>();
  }
  
  *outMeanX = sumX / inDataPoints.size();
  *outMeanY = sumY / inDataPoints.size();
}


// Calculate correlation Corr (Corelation coefficient (NOTE: Needs linear depenceny between variables))
// TODO: Move to util.h - make template
template <typename T>
static T correlation(const PointLstT<T> & inDataPoints) {
  T mean_x = 0, mean_y = 0;
  calcMeans(inDataPoints, & mean_x, & mean_y);

  T numeratorSum = 0, denominatorXSum = 0, denominatorYSum = 0;
    
  for (typename PointLstT<T>::const_iterator it = inDataPoints.begin(); it != inDataPoints.end(); ++it) {
    const T & xi = it->get<0>();
    const T & yi = it->get<1>();

    T diff_x = xi - mean_x;
    T diff_y = yi - mean_y;
      
    numeratorSum += diff_x * diff_y;
    denominatorXSum += pow(diff_x, 2.0f);
    denominatorYSum += pow(diff_y, 2.0f);
  }
    
  T numerator = numeratorSum / inDataPoints.size();
  T denominatorX = sqrt(denominatorXSum / inDataPoints.size());
  T denominatorY = sqrt(denominatorYSum / inDataPoints.size());

  return numerator / (denominatorX * denominatorY);
}



DEF_Exception(Line);
DEF_Exception(LineIntersection);

template <typename T>
class LineT {
private:
  T mA1;
  T mA0;

public:
  LineT(const PointLstT<T> & inDataPoints) {
    LineT::calcBestFitLine(inDataPoints, & mA1, & mA0);
  }
  LineT(const T & inA1 = 0, const T & inA0 = 0) : mA1(inA1),mA0(inA0) {}
  const T & getA1() const { return mA1; }
  const T & getA0() const { return mA0; }
  T f(const T & inX) const { return (mA1 * inX + mA0); }
  
  ostream & print(ostream & os) {
    os << "y(x) = " << mA1 << "*x + " << mA0 << endl;
    return os;
  }

  // TODO: Add non-static version - intersect with LineT... and with a1,a0 as sep. params...
  static PointT<T> calcIntersectionPoint(const LineT<T> & inL1, const LineT<T> & inL2) {
    return LineT::calcIntersectionPoint(inL1.mA1, inL1.mA0, inL2.mA1, inL2.mA0);
  }

  static PointT<T> calcIntersectionPoint(const T & inA1, const T & inA0, const T & inB1, const T & inB0) {
    PointT<T> sp;
    
    if (inB1 - inA1) {
      sp.get<0>() = (inA0 - inB0) / (inB1 - inA1);
      sp.get<1>() = inA1 * sp.get<0>() + inA0;
    } else {
      if (inA0 == inB0) {
	throw LineIntersectionExceptionT("Lines are equal. Infinite number of intersections.");
      } else {
	throw LineIntersectionExceptionT("Lines are parallel and have no intersection.");
      }
    }
    return sp;
  }
  
  static void
  calcBestFitLine(const PointLstT<T> & inDataPoints, T * outA1, T * outA0) {
    T mean_x = 0, mean_y = 0;
    calcMeans<T>(inDataPoints, & mean_x, & mean_y);
    
    T top = 0, bottom = 0;
    for (typename PointLstT<T>::const_iterator it = inDataPoints.begin(); it != inDataPoints.end(); ++it) {
      top += (it->get<0>() - mean_x) * (it->get<1>() - mean_y);
      bottom += pow(it->get<0>() - mean_x, 2.0);
    }

    AT_ASSERT(Line, outA1, "outA1 expected to be set.");
    AT_ASSERT(Line, outA0, "outA0 expected to be set.");
    
    *outA1 = top / bottom;
    *outA0 = mean_y - (*outA1) * mean_x;
  }
  
  friend inline std::ostream& operator<< (std::ostream& os, LineT<T> & inLine) {
    return inLine.print(os);
  }
};




#endif /* _UTIL_HPP_ */
