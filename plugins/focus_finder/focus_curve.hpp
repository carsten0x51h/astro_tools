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
    typedef map<int /*absPos*/, float /*e.g. HFD value*/> SegmentT;

  private:
    SegmentT mSegments[2];
    
  public:
    SegmentT & getSegment(size_t inSegNo) {
      AT_ASSERT(FocusCurve, inSegNo == 0 || inSegNo == 1, "Invalid segment!");
      return mSegments[inSegNo];
    }
    const SegmentT & getSegment(size_t inSegNo) const {
      AT_ASSERT(FocusCurve, inSegNo == 0 || inSegNo == 1, "Invalid segment!");
      return mSegments[inSegNo];
    }

    PointT<float>
    calcOptFocusPos(LineFitTypeT::TypeE inLineFitType, LineT<float> * outLine1, LineT<float> * outLine2) const;

    static PointT<float>
    mapToImgCoords(const PointT<float> & inCurveCoords, float inWidth, float inHeight, float inMinX, float inMaxX, float inMinY, float inMaxY);

    static CImg<unsigned char>
    genView(const FocusCurveT & inFocusCurve, size_t inWidth, size_t inHeight, bool inDrawBestFit, const LineT<float> * inLineL, const LineT<float> * inLineR);
    
    CImg<unsigned char>
    genView(size_t inWidth, size_t inHeight, bool inDrawBestFit, const LineT<float> * inLineL = 0, const LineT<float> * inLineR = 0) const {
      // TODO: inLineL and inLineR could be class members...? 
      LineT<float> line1, line2;
      PointT<float> sp;
      
      if (inDrawBestFit) {
	sp = calcOptFocusPos(LineFitTypeT::OLS /*TODO: Should this be a member of the FocusCurveT??*/, & line1, & line2);
	return FocusCurveT::genView(*this, inWidth, inHeight, inDrawBestFit, & line1, & line2);
      }
      
      return FocusCurveT::genView(*this, inWidth, inHeight, inDrawBestFit, 0, 0);
    }
  };
};

#endif /* _FOCUS_CURVE_HPP_ */
