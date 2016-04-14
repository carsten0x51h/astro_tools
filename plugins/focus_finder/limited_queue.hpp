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

#ifndef _LIMITED_QUEUE_HPP_
#define _LIMITED_QUEUE_HPP_ _LIMITED_QUEUE_HPP_

#include <queue>
#include <algorithm> // max_element
#include <iostream>
#include <CImg.h>

#include "at_exception.hpp"
#include "at_logging.hpp"
#include "util.hpp"

using namespace std;
using namespace cimg_library;

DEF_Exception(LimitedQueue);

namespace AT {

  template <typename T>
  class LimitedQueueT {  
  private:
    size_t mMaxNumElements;
    deque<T> mContainer;

  public:
    typedef typename deque<T>::iterator iterator;
    typedef typename deque<T>::const_iterator const_iterator;
    
    LimitedQueueT(size_t inMaxNumElements = 50) : mMaxNumElements(inMaxNumElements) { }

    inline size_t maxNumElements() const { return mMaxNumElements; }
    inline size_t numElements() const { return mContainer.size(); }
    inline size_t empty() const { return mContainer.empty(); }
    inline void push(const T & inValue) {
      if (mContainer.size() < mMaxNumElements) {
	mContainer.push_back(inValue);
      } else {
	mContainer.pop_front();
	mContainer.push_back(inValue);
      }
    }
    inline float average() const {
      float average = 0;
      for (typename deque<T>::const_iterator it = mContainer.begin(); it != mContainer.end(); ++it) {
	average += *it;
      }
      return (mContainer.size() > 0 ? average / (float) mContainer.size() : 0);
    }

    inline T min() const { return (mContainer.size() > 0 ? *std::min_element(mContainer.begin(), mContainer.end()) : 0); }
    inline T max() const { return (mContainer.size() > 0 ? *std::max_element(mContainer.begin(), mContainer.end()) : 0); }
    
    iterator begin() { return mContainer.begin(); }
    iterator end() { return mContainer.end(); }
    const_iterator begin() const { return mContainer.begin(); }
    const_iterator end() const {return mContainer.end(); }

    ostream & print(ostream & os) const;
    template <typename SclassT>
    friend ostream & operator<<(ostream & os, const LimitedQueueT<SclassT> & inLimitedQueue);
    
    static CImg<unsigned char>
    genView(size_t inWidth, size_t inHeight, const deque<T> & inContainer, const T & inMax, const T & inMin, size_t inMaxNumElements, const T & inAverage) {
      LOG(debug) << "LimitedQueueT..." << endl;

      CImg<unsigned char> rgbImg(inWidth, inHeight, 1 /*depth*/, 3 /*3 channels - RGB*/);
      rgbImg.fill(0);

      for (typename deque<T>::const_iterator it = inContainer.begin(); it != inContainer.end(); ++it) {
	// Calculate height - inHeight -- max()
	float y = (float) (*it - (float) inMin) / ((float) inMax ? (float) fabs(inMax - inMin) : 1.0f);
	int yDraw = inHeight - (float) inHeight * y;

	int dist = distance(inContainer.begin(), it);
	int xDraw = (float) inWidth * ((float) dist / (float) inMaxNumElements);
	
	// Calculate width - inWidth
	const unsigned char green[3] = { 0, 255, 0 };
	const size_t cCrossSize = 3;
	drawCross(& rgbImg, xDraw, yDraw, green, cCrossSize);
      }
      return rgbImg; // Make a copy...
    }

    CImg<unsigned char> genView(size_t inWidth, size_t inHeight) const { return LimitedQueueT::genView(inWidth, inHeight, mContainer, max(), min(), maxNumElements(), average()); }
  };
  
}; // end namespace AT

#endif // _LIMITED_QUEUE_HPP_
