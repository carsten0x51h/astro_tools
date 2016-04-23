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
  FocusCurveT::mapToImgCoords(const PointT<float> & inCurveCoords, float inWidth /*image width*/, float inHeight /*image height*/,
			      const PointT<float> & inMinXY, const PointT<float> & inMaxXY, size_t inBorderPx) {

    float width = inWidth - 2 * inBorderPx;
    float height = inHeight - 2 * inBorderPx; 
    
    const float facX = (inMaxXY.get<0>() == inMinXY.get<0>() ? 1.0f : width / (inMaxXY.get<0>() - inMinXY.get<0>())); 
    const float facY = (inMaxXY.get<1>() == inMinXY.get<1>() ? 1.0f : height / (inMaxXY.get<1>() - inMinXY.get<1>())); 
    
    float xMap = (inMaxXY.get<0>() == inMinXY.get<0>() ? 0.5f * width : facX * (inCurveCoords.get<0>() - inMinXY.get<0>()) );
    float yMap = (inMaxXY.get<1>() == inMinXY.get<1>() ? 0.5f * height : inHeight - (facY * (inCurveCoords.get<1>() - inMinXY.get<1>())));

    return PointT<float>(xMap + inBorderPx, yMap - inBorderPx);
  }

  void
  FocusCurveT::getBounds(const FocusCurveT & inFocusCurve, PointT<float> * outMin, PointT<float> * outMax) {
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

    if (outMin) {
      outMin->get<0>() = minX;
      outMin->get<1>() = minY;
    }
    if (outMax) {
      outMax->get<0>() = maxX;
      outMax->get<1>() = maxY;
    }
  }
  
  // TODO: We may pass in img as pointer...
  CImg<unsigned char>
  FocusCurveT::genView(const FocusCurveT & inFocusCurve, size_t inWidth, size_t inHeight, bool inDrawBestFit,
		       const LineT<float> * inLineL, const LineT<float> * inLineR, const PointT<float> * inSp) {
    const size_t borderPx = 40;
    const size_t fontHeight = 25;
    CImg<unsigned char> img(inWidth, inHeight, 1 /*size_z*/, 3 /*3 channels - RGB*/);
    const unsigned char red[3] = { 255, 0, 0 }, green[3] = { 0, 255, 0 }, blue[3] = { 0, 0, 255 }, black[3] = { 0, 0, 0 }, yellow[3] = { 255, 255, 0 };

    img.fill(0);

    PointT<float> minXY, maxXY;
    getBounds(inFocusCurve, & minXY, & maxXY);    

    if (inSp && minXY.get<1>() > inSp->get<1>()) {
      minXY.get<1>() = inSp->get<1>();
    }
    
    // Draw points
    for (size_t i = 0; i < 2; ++i) {
      const SegmentT & recordedStarData = inFocusCurve.getSegment(i);
      for (SegmentT::const_iterator it = recordedStarData.begin(); it != recordedStarData.end(); ++it) {

	PointT<float> mapPt = FocusCurveT::mapToImgCoords(PointT<float>(it->first, it->second), inWidth, inHeight, minXY, maxXY, borderPx);
	drawCross(& img, mapPt, green, 5.0 /*cross size*/, 1.0);
      }
    }

    // Draw lines
    if (inLineR) {
      for (int i = minXY.get<0>(); i < maxXY.get<0>(); ++i) {
  	PointT<float> mapPoint = FocusCurveT::mapToImgCoords(PointT<float>(i, inLineR->f(i)), inWidth, inHeight, minXY, maxXY, borderPx);
  	img.draw_point(mapPoint.get<0>(), mapPoint.get<1>(), red, 1 /*opacity*/);
      }
    }

    if (inLineL) {
      for (int i = minXY.get<0>(); i < maxXY.get<0>(); ++i) {
  	PointT<float> mapPoint = FocusCurveT::mapToImgCoords(PointT<float>(i, inLineL->f(i)), inWidth, inHeight, minXY, maxXY, borderPx);
  	img.draw_point(mapPoint.get<0>(), mapPoint.get<1>(), red, 1 /*opacity*/);
      }
    }

    // Draw focus line
    if (inSp) {
      stringstream ss;
      PointT<float> mapPoint = FocusCurveT::mapToImgCoords(*inSp, inWidth, inHeight, minXY, maxXY, borderPx);
      ss << inSp->get<0>() << " HFD=" << inSp->get<1>();
      img.draw_line(mapPoint.get<0>(), 0, mapPoint.get<0>(), inHeight, yellow, 1 /*opacity*/);
      img.draw_text(mapPoint.get<0>(), borderPx, ss.str().c_str(), yellow, black, 1 /*opacity*/, fontHeight); 
    }
    
    // Draw 3 focus positions at the bottom (interpolated)
    const size_t focusPosTextHeight = inHeight -fontHeight;
    img.draw_text(borderPx /* left */, focusPosTextHeight, std::to_string((int)minXY.get<0>()).c_str(), blue, black, 1 /*opacity*/, fontHeight); 
    img.draw_text(inWidth / 2 /* center */, focusPosTextHeight, std::to_string((int) ((maxXY.get<0>() + minXY.get<0>()) / 2)).c_str(), blue, black, 1 /*opacity*/, fontHeight); 
    img.draw_text(inWidth - 2*borderPx /* right */, focusPosTextHeight, std::to_string((int)maxXY.get<0>()).c_str(), blue, black, 1 /*opacity*/, fontHeight); 

    return img;
  }

  PointT<float>
  FocusCurveT::calcOptFocusPos(LineFitTypeT::TypeE inLineFitType, LineT<float> * outLine1 = 0, LineT<float> * outLine2 = 0) const {

    // HACK! Put all points together and to a MEDIAN... otherwise lines are weighted wrong because start position is never exactly in the center!!!
    // TODO -> Maybe we can apply this to the overall data structure - segment is then no longer required... -> also good for parabel!

    FocusCurveT::SegmentT bothCurves;

    for (size_t i = 0; i < 2; ++i) {
      const FocusCurveT::SegmentT & currSeg = mSegments[i];
      for (typename FocusCurveT::SegmentT::const_iterator it = currSeg.begin(); it != currSeg.end(); ++it) {
	bothCurves[it->first] = it->second;
	LOG(debug) << "HACK - BOTH_CURVES - (x, y) = (" << it->first << ", " << it->second << ")" << endl;
      }
    }
    LOG(info) << "bothCurves size: " << bothCurves.size() << endl;

    
    PointLstT<float> hfdPoints[2];
    LineT<float> bestFitLines[2];
    
    if (bothCurves.size() % 2 == 0 /*even*/) {
      LOG(info) << "EVEN..." << endl;
      int numPerLine = bothCurves.size() / 2;
      int counter = 0;
      for (typename FocusCurveT::SegmentT::const_iterator it = bothCurves.begin(); it != bothCurves.end(); ++it,++counter) {
	if (counter < numPerLine) {
	  hfdPoints[0].push_back(PointT<float>(it->first, it->second));
	} else {
	  hfdPoints[1].push_back(PointT<float>(it->first, it->second));
	}
      }      
    } else {
      // Share the center point...
      LOG(info) << "ODD..." << endl;
      int numPerLine = floor(bothCurves.size() / 2);

      int counter = 0;
      for (typename FocusCurveT::SegmentT::const_iterator it = bothCurves.begin(); it != bothCurves.end(); ++it,++counter) {
	if (counter < numPerLine) {
	  hfdPoints[0].push_back(PointT<float>(it->first, it->second));
	} else if (counter == numPerLine) {
	  // Share the point...
	  hfdPoints[0].push_back(PointT<float>(it->first, it->second));
	  hfdPoints[1].push_back(PointT<float>(it->first, it->second));	  
	} else {
	  hfdPoints[1].push_back(PointT<float>(it->first, it->second));
	}
      }      
    }
    bestFitLines[0].set(hfdPoints[0], inLineFitType);
    bestFitLines[1].set(hfdPoints[1], inLineFitType);
    

    
    // Convert inFocusCurve to PointLst<float> and create lines...
    // TODO: Converter might be moved to FocusCurveT...


    // PointLstT<float> hfdPoints[2];
    // LineT<float> bestFitLines[2];
    
    // for (size_t i = 0; i < 2; ++i) {
    //   const FocusCurveT::SegmentT & currSeg = mSegments[i];

    //   LOG(info) << "calcOptFocusPos - mSegments[" << i << "].size(): " << currSeg.size() << endl;

    //   for (typename FocusCurveT::SegmentT::const_iterator it = currSeg.begin(); it != currSeg.end(); ++it) {
    // 	hfdPoints[i].push_back(PointT<float>(it->first, it->second));
    // 	LOG(debug) << "(x, y) = (" << it->first << ", " << it->second << ")" << endl;
    //   }
    //   bestFitLines[i].set(hfdPoints[i], inLineFitType);

    //   LOG(info) << "Line " << i << " - A1: " << bestFitLines[i].getA1() << ", A0: " << bestFitLines[i].getA0() << endl;
    // }

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



