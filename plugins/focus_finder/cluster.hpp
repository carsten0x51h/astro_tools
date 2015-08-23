/*************************************************************************** *
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

#ifndef __CLUSTER_HPP__
#define __CLUSTER_HPP__ __CLUSTER_HPP__

#include <list>
#include <set>

#include "util.hpp"

namespace AT {

  class ClusterT {
  private:
    typedef set<PointT<int> > PointSetT;
    typedef list<PointT<int> > PointListT;
  
    /**
     * Removes all white neighbours around pixel from whitePixels
     * if they exist and adds them to pixelsToBeProcessed.
     */
    static void
    getAndRemoveNeighbours(const PointT<int> & inCurPixelPos, PointSetT * inoutWhitePixels, PointListT * outPixelsToBeProcessed)
    {
      const size_t _numPixels = 8, _x = 0, _y = 1;
      const int offsets[_numPixels][2] = { { -1, -1 }, { 0, -1 }, { 1, -1 },
					   { -1,  0 },            { 1,  0 },
					   { -1,  1 }, { 0,  1 }, { 1,  1 } };
    
      for (size_t p = 0; p < _numPixels; ++p) {
	PointT<int> curPixPos(inCurPixelPos.get<0>() + offsets[p][_x], inCurPixelPos.get<1>() + offsets[p][_y]);
	PointSetT::iterator itPixPos = inoutWhitePixels->find(curPixPos);
      
	if (itPixPos != inoutWhitePixels->end()) {
	  outPixelsToBeProcessed->push_back(*itPixPos);
	  inoutWhitePixels->erase(itPixPos); // Remove white pixel from "white set" since it has been processed, now
	}
      }
    }
  
  public:
    static void
    calc(const CImg<float> & inImg, list<FrameT<int> > * outStarInfos) {
      PointSetT whitePixels;
    
      cimg_forXY(inImg, x, y) {
	if (inImg(x, y)) {
	  whitePixels.insert(whitePixels.end(), PointT<int>(x, y));
	}
      }
    
      // Iterate over white pixels as long as set is not empty
      while (whitePixels.size()) {
	PointListT pixelsToBeProcessed;

	// Pick first white pixel and add it to the process queue.
	// Also remove it from the set to mark that is has been processed.
	PointSetT::iterator itWhitePixPos = whitePixels.begin();
	pixelsToBeProcessed.push_back(*itWhitePixPos);
	whitePixels.erase(itWhitePixPos);
      
	FrameT<float> frame(inImg.width(), inImg.height(), 0, 0);
      
	while(! pixelsToBeProcessed.empty()) {
	  PointT<int> curPixelPos = pixelsToBeProcessed.front();

	  // Determine boundaries (min max in x and y directions)
	  //LOG(trace) << "Current pixel pos=" << curPixelPos << ", current frame: " << frame << endl;
	  float x2 = frame.get<0>() + frame.get<2>(); /*x2=x1+w*/
	  float y2 = frame.get<1>() + frame.get<3>(); /*y2=y1+h*/
	
	  if (curPixelPos.get<0>() /*x*/ < frame.get<0>() /*x1*/)  { frame.get<0>() = curPixelPos.get<0>() - 1; }
	  if (curPixelPos.get<0>() /*x*/ > x2 /*x2*/)              { frame.get<2>() = fabs(frame.get<0>() - curPixelPos.get<0>()) + 2; }
	  if (curPixelPos.get<1>() /*y*/ < frame.get<1>() /*y1*/)  { frame.get<1>() = curPixelPos.get<1>() - 1; }
	  if (curPixelPos.get<1>() /*y*/ > y2 /*y2*/)              { frame.get<3>() = fabs(frame.get<1>() - curPixelPos.get<1>()) + 2; }

	  //LOG(trace) << "--> New frame: " << frame << endl;

	  getAndRemoveNeighbours(curPixelPos, & whitePixels, & pixelsToBeProcessed);
	  pixelsToBeProcessed.pop_front();
	}
	LOG(debug) << "New cluster detected: " << frame << endl;
	outStarInfos->push_back(frame);
      }
    }
  };
} // end AT

#endif
