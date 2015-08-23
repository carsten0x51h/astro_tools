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

#ifndef _NORMALIZE_HPP_
#define _NORMALIZE_HPP_ _NORMALIZE_HPP_

#include <CImg.h>

using namespace cimg_library;

namespace AT {

  // IDEA: We may offer different normalizations...
  CImg<unsigned char>
  normalize(const CImg<float> & inImg, long inNumBuckets, float inPercent = 10.0)
  {
    // TODO: We can avoid copy by passing a pointer..
    CImg<unsigned char> normalizedImage(inImg.width(), inImg.height());
    const CImg<float> histogram = inImg.get_histogram(inNumBuckets);
    
    float sum = 0, prod = 0;
    cimg_forX(histogram, x) {
      prod = x * histogram(x);
      sum += histogram(x);
    }
    float cog = prod / sum;

    float delta = (inPercent / 100.0) * inNumBuckets;
    float min = (cog - delta < 0 ? 0 : cog - delta);
    float max = (cog - delta > inNumBuckets ? inNumBuckets : cog + delta);
	
    cimg_forXY(inImg, x, y) {
      float value = 255.0 * (inImg(x,y) - min) / (max - min);
      unsigned char valueCut = (value > 255 ? 255 : value);
      normalizedImage(x, y) = valueCut;
    }

    return normalizedImage;
  }
}; // end AT

#endif
