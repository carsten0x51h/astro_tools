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

#include "focus_curve.hpp"

namespace AT {

  // TODO: Too many parameters... NEED NEW DESIGN...   maybe class / part of class with props as members...?!
  PointT<float>
  FocusCurveT::mapToImgCoords(const PointT<float> & inCurveCoords, float inWidth, float inHeight,
			      float inMinX, float inMaxX, float inMinY, float inMaxY) {
    
    const float facX = (inMaxX == inMinX ? 1.0f : inWidth / (inMaxX - inMinX)); 
    const float facY = (inMaxY == inMinY ? 1.0f : inHeight / (inMaxY - inMinY)); 
    
    float xMap = (inMaxX == inMinX ? 0.5f * inWidth : facX * (inCurveCoords.get<0>() - inMinX) );
    float yMap = (inMaxY == inMinY ? 0.5f * inHeight : inHeight - (facY * (inCurveCoords.get<1>() - inMinY)));

    return PointT<float>(xMap, yMap);
  }

  // TODO: We may pass in img as pointer...
  CImg<unsigned char>
  FocusCurveT::genView(const FocusCurveT & inFocusCurve, size_t inWidth, size_t inHeight, bool inDrawBestFit,
		       const LineT<float> * inLineL, const LineT<float> * inLineR) {
    CImg<unsigned char> img(inWidth, inHeight, 1 /*size_z*/, 3 /*3 channels - RGB*/);
    const unsigned char red[3] = { 255, 0, 0 }, blue[3] = { 0, 0, 255 };

    img.fill(0);
    
    // TODO: Maybe put this code to a generic helper function...
    float minX = std::numeric_limits<float>::max();
    float maxX = std::numeric_limits<float>::min();
    float minY = std::numeric_limits<float>::max();
    float maxY = std::numeric_limits<float>::min();

    // Find min and max
    for (size_t i = 0; i < 2; ++i) {
      const SegmentT & recordedStarData = inFocusCurve.getSegment(i);
      for (SegmentT::const_iterator it = recordedStarData.begin(); it != recordedStarData.end(); ++it) {
  	int pos = it->first;
  	float value = it->second;
  	if (pos < minX) { minX = pos; }
  	if (pos > maxX) { maxX = pos; }
  	if (value < minY) { minY = value; }
  	if (value > maxY) { maxY = value; }
      }
    }

    // Draw points
    for (size_t i = 0; i < 2; ++i) {
      const SegmentT & recordedStarData = inFocusCurve.getSegment(i);
      for (SegmentT::const_iterator it = recordedStarData.begin(); it != recordedStarData.end(); ++it) {
  	PointT<float> dp(it->first, it->second);
  	drawCross(& img, FocusCurveT::mapToImgCoords(dp, inWidth, inHeight, minX, maxX, minY, maxY), red, 3.0 /*cross size*/, 1.0);
      }
    }

    // Draw lines
    if (inLineR) {
      for (int i = minX; i < maxX; ++i) {
  	PointT<float> mapPoint = FocusCurveT::mapToImgCoords(PointT<float>(i, inLineR->f(i)), inWidth, inHeight, minX, maxX, minY, maxY);
  	img.draw_point(mapPoint.get<0>(), mapPoint.get<1>(), red, 1 /*opacity*/);
      }
    }

    if (inLineL) {
      for (int i = minX; i < maxX; ++i) {
  	PointT<float> mapPoint = FocusCurveT::mapToImgCoords(PointT<float>(i, inLineL->f(i)), inWidth, inHeight, minX, maxX, minY, maxY);
  	img.draw_point(mapPoint.get<0>(), mapPoint.get<1>(), red, 1 /*opacity*/);
      }
    }
    return img;
  }

  PointT<float>
  FocusCurveT::calcOptFocusPos(LineFitTypeT::TypeE inLineFitType, LineT<float> * outLine1 = 0, LineT<float> * outLine2 = 0) const {

    // Convert inFocusCurve to PointLst<float> and create lines...
    // TODO: Converter might be moved to FocusCurveT...
    PointLstT<float> hfdPoints[2];
    LineT<float> bestFitLines[2];
    
    for (size_t i = 0; i < 2; ++i) {
      const FocusCurveT::SegmentT & currSeg = mSegments[i];

      LOG(info) << "calcOptFocusPos - mSegments[" << i << "].size(): " << currSeg.size() << endl;

      for (typename FocusCurveT::SegmentT::const_iterator it = currSeg.begin(); it != currSeg.end(); ++it) {
  	hfdPoints[i].push_back(PointT<float>(it->first, it->second));
	LOG(debug) << "(x, y) = (" << it->first << ", " << it->second << ")" << endl;
      }
      bestFitLines[i].set(hfdPoints[i], inLineFitType);

      LOG(info) << "Line " << i << " - A1: " << bestFitLines[i].getA1() << ", A0: " << bestFitLines[i].getA0() << endl;
    }

    // Optionally, return a copy of the lines
    if (outLine1) { *outLine1 = bestFitLines[0]; }
    if (outLine2) { *outLine2 = bestFitLines[1]; }
    
    // Calculate intersection point.
    // NOTE: Throws LineIntersectionExceptionT in case lines are parallel - cannot be handled here.
    PointT<float> sp;
    sp = LineT<float>::calcIntersectionPoint(bestFitLines[0], bestFitLines[1]);
    LOG(info) << "SP: " << sp << endl;
    return sp;
  }

};



