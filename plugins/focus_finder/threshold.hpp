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

#ifndef __THRESHOLD_HPP__
#define __THRESHOLD_HPP__ __THRESHOLD_HPP__

#include <CImg.h>

namespace AT {
  DEF_Exception(Threshold);

  // TODO: static function? functor? class IF? As tmpl? 
  class ThresholdT {
  private:
    static float
    thresholdOtsu(const CImg<float> & inImg, long inBitPix) {
      CImg<> hist = inImg.get_histogram(pow(2.0, inBitPix));
    
      float sum = 0;
      cimg_forX(hist, pos) { sum += pos * hist[pos]; }
    
      float numPixels = inImg.width() * inImg.height();
      float sumB = 0, wB = 0, max = 0.0;
      float threshold1 = 0.0, threshold2 = 0.0;
    
      cimg_forX(hist, i) {
	wB += hist[i];
      
	if (! wB) { continue; }    
      
	float wF = numPixels - wB;
      
	if (! wF) { break; }
      
	sumB += i * hist[i];
      
	float mF = (sum - sumB) / wF;
	float mB = sumB / wB;
	float diff = mB - mF;
	float bw = wB * wF * pow(diff, 2.0);
      
	if (bw >= max) {
	  threshold1 = i;
	  if (bw > max) {
	    threshold2 = i;
	  }
	  max = bw;            
	}
      } // end loop
    
      return (threshold1 + threshold2) / 2.0;
    }

  public:
    struct ThresholdTypeT {
      enum TypeE {
	OTSU,
	_Count
      };
    
      static const char * asStr(const TypeE & inType) {
	switch (inType) {
	case OTSU: return "OTSU";
	default: return "<?>";
	}
      }
      MAC_AS_TYPE(Type, E, _Count);  
    };

    static float
    calc(const CImg<float> & inImage, long inBitPix, ThresholdTypeT::TypeE inThresholdType) {
      switch(inThresholdType) {
      case ThresholdTypeT::OTSU:
	return ThresholdT::thresholdOtsu(inImage, inBitPix);
      default:
	AT_ASSERT(Threshold, false, "Unsupported ThresholdTypeT.");
      }
    }
  };
} // end namespace


#endif // __THRESHOLD_HPP__
