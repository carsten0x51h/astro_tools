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

#ifndef _FOCUS_CURVE_HPP_
#define _FOCUS_CURVE_HPP_ _FOCUS_CURVE_HPP_

#include <map>

#include "fwhm.hpp"
#include "hfd.hpp"
#include "util.hpp"

#include "focus_finder_common.hpp"

namespace AT {
// // TODO: Rename to StarDataT ?!
//   // TODO: Use as FocusFindStatusDataT above? Or at least as member??
//   struct FocusFinderDataSetT {
//     HfdT mHfd;
//     FwhmT mFwhmHorz;
//     FwhmT mFwhmVert;
//     FocusFinderDataSetT() {};
//     FocusFinderDataSetT(const HfdT & inHfd, const FwhmT & inFwhmHorz, const FwhmT & inFwhmVert) : mHfd(inHfd), mFwhmHorz(inFwhmHorz), mFwhmVert(inFwhmVert) {}
//   };

  
  DEF_Exception(FocusCurve);
  
  class FocusCurveT {
  public:
    typedef std::function<float(const CImg<float> &)> FocusMeasureFuncT;

    static FocusMeasureFuncT sHfdStrategy;
    static FocusMeasureFuncT sFwhmHorzStrategy;
    static FocusMeasureFuncT sFwhmVertStrategy;
    static FocusMeasureFuncT sFwhmMeanStrategy;
    static FocusMeasureFuncT sMaxPixelStrategy;

    
    typedef map<int /*absPos*/, float /*e.g. HFD value*/> PosToFocusMeasureT;
    
  private:
    PosToFocusMeasureT mPosToFocusMeasure;
    FocusMeasureFuncT mFocusMeasureFunc;
    
  public:
    FocusCurveT(FocusMeasureFuncT inFocusMeasureFunc) : mFocusMeasureFunc(inFocusMeasureFunc) { }
    FocusCurveT(const PosToImgMapT & inPosToImgMap, FocusMeasureFuncT inFocusMeasureFunc) : mFocusMeasureFunc(inFocusMeasureFunc) {
      // Fill internal map based on inPosToImgMap
      for (PosToImgMapT::const_iterator it = inPosToImgMap.begin(); it != inPosToImgMap.end(); ++it) {
	mPosToFocusMeasure[it->first /*focus pos*/] = mFocusMeasureFunc(it->second) /*focus measure*/;
      }
    }

    // TODO: Add function to add single samples...
    // TODO: Add clear function?
    
    inline PosToFocusMeasureT & getData() { return mPosToFocusMeasure; }
    inline const PosToFocusMeasureT & getData() const { return mPosToFocusMeasure; }

    // TODO: Replace LineFitTypeT::TypeE by "methods" - e.g. also ABS_MIN
    PointT<float>
    calcOptFocusPos(LineFitTypeT::TypeE inLineFitType, LineT<float> * outLine1, LineT<float> * outLine2) const;

    static PointT<float>
    mapToImgCoords(const PointT<float> & inCurveCoords, float inWidth /*image width*/, float inHeight /*image height*/,
		   const PointT<float> & inMinXY, const PointT<float> & inMaxXY, size_t inBorderPx);

    static void
    getBounds(const FocusCurveT & inFocusCurve, PointT<float> * outMin, PointT<float> * outMax);

    inline void
    getBounds(PointT<float> * outMin, PointT<float> * outMax) const { getBounds(*this, outMin, outMax); }

    inline void
    add(float inFocusPos, const CImg<float> & inImgFrame) {
      mPosToFocusMeasure.insert(make_pair(inFocusPos, mFocusMeasureFunc(inImgFrame)));
    }

    inline void
    clear() { mPosToFocusMeasure.clear(); }
    

    // TODO: Should be replaced by calcOptFocusPos(ABS_MIN)...??
    // TODO: Improve code below... more generic...
    PointT<float>
    getMinValPoint() const {
      float minVal = numeric_limits<float>::max();
      int pos;
      
      for(PosToFocusMeasureT::const_iterator it = mPosToFocusMeasure.begin(); it != mPosToFocusMeasure.end(); ++it) {
	if (minVal > it->second) {
	  minVal = it->second;
	  pos = it->first; // Corresponding position
	}
      }
      return PointT<float>(pos, minVal);
    }

    
    
    static CImg<unsigned char>
    genView(const FocusCurveT & inFocusCurve, size_t inWidth, size_t inHeight, bool inDrawBestFit, const LineT<float> * inLineL, const LineT<float> * inLineR, const PointT<float> * inSp);
    
    CImg<unsigned char>
    genView(size_t inWidth, size_t inHeight, bool inDrawBestFit, const LineT<float> * inLineL = 0, const LineT<float> * inLineR = 0, const PointT<float> * inSp = 0) const {
      // TODO: inLineL and inLineR could be class members...? 
      LineT<float> line1, line2;
      PointT<float> sp;
      
      if (inDrawBestFit) {
	sp = calcOptFocusPos(LineFitTypeT::OLS /*TODO: Should this be a member of the FocusCurveT??*/, & line1, & line2);
	return FocusCurveT::genView(*this, inWidth, inHeight, inDrawBestFit, & line1, & line2, & sp);
      }
      
      return FocusCurveT::genView(*this, inWidth, inHeight, inDrawBestFit, 0, 0, 0);
    }
  };
};

#endif /* _FOCUS_CURVE_HPP_ */
