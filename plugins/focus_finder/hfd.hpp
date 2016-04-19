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

/**
 *
 * Get all pixels inside a radius: http://stackoverflow.com/questions/14487322/get-all-pixel-array-inside-circle
 * Algorithm:                      http://en.wikipedia.org/wiki/Midpoint_circle_algorithm
 * HDF calculation:                http://www005.upp.so-net.ne.jp/k_miyash/occ02/halffluxdiameter/halffluxdiameter_en.html
 *                                 http://www.cyanogen.com/help/maximdl/Half-Flux.htm
 *
 * NOTE: Currently the accuracy is limited by the insideCircle function (-> sub-pixel accuracy).
 * NOTE: The HFD is estimated in case there is no flux (HFD ~ sqrt(2) * inOuterDiameter / 2).
 * NOTE: The outer diameter is usually a value which depends on the properties of the optical
 *       system and also on the seeing conditions. The HFD value calculated depends on this
 *       outer diameter value.
 *
 * TODO: Implement scale up.
 * TODO: Add LOG msgs to HfdT.
 */

#ifndef _HFD_HPP_
#define _HFD_HPP_ _HFD_HPP_

#include <assert.h>
#include <CImg.h>
#include "centroid.hpp"
#include "at_exception.hpp"

namespace AT {
  DEF_Exception(Hfd);

  class HfdT {
  private:
    CImg<float> mImg;
    float mHfdValue;
    unsigned int mOuterDiameter;
    float mScaleFactor;
  
  public:
    static const unsigned int outerHfdDiameter = 21; // TODO: Calc?! - depends on pixel size and focal length (and seeing...)
    static const float defaultScaleFactor;
  
    HfdT() : mHfdValue(0), mOuterDiameter(outerHfdDiameter), mScaleFactor(defaultScaleFactor)  {}
    HfdT(const CImg<float> & inImage, const PointT<float> & inCenter, unsigned int inOuterDiameter = outerHfdDiameter,
	 bool inSubMean = true, float inScaleFactor = defaultScaleFactor) {
      this->set(inImage, inCenter, inOuterDiameter, inSubMean, inScaleFactor);
    }
    HfdT(const CImg<float> & inImage, unsigned int inOuterDiameter = outerHfdDiameter,
	 bool inSubMean = true, float inScaleFactor = defaultScaleFactor) {
      this->set(inImage, inOuterDiameter, inSubMean, inScaleFactor);
    }

    inline void
    set(const CImg<float> & inImage, const PointT<float> & inCenter, unsigned int inOuterDiameter = outerHfdDiameter,
	bool inSubMean = true, float inScaleFactor = defaultScaleFactor) {
      mHfdValue = HfdT::calc(inImage, inCenter, inOuterDiameter, & mImg, inSubMean, inScaleFactor);
      mOuterDiameter = inOuterDiameter;
      mScaleFactor = inScaleFactor;
    }
  
    inline void
    set(const CImg<float> & inImage, unsigned int inOuterDiameter = outerHfdDiameter,
	bool inSubMean = true, float inScaleFactor = defaultScaleFactor) {
      mHfdValue = HfdT::calc(inImage, inOuterDiameter, & mImg, inSubMean, inScaleFactor);
      mOuterDiameter = inOuterDiameter;
      mScaleFactor = inScaleFactor;
    }

    static float
    calc(const CImg<float> & inImage, const PointT<float> & inCenter, unsigned int inOuterDiameter = outerHfdDiameter,
	 CImg<float> * outCenteredImg = 0, bool inSubMean = true, float inScaleFactor = defaultScaleFactor);

    static float
    calc(const CImg<float> & inImage, unsigned int inOuterDiameter = outerHfdDiameter,
	 CImg<float> * outCenteredImg = 0, bool inSubMean = true, float inScaleFactor = 1.0)
    {
      int centerX = ceil(inImage.width() / 2.0);
      int centerY = ceil(inImage.height() / 2.0);
      return calc(inImage, PointT<float>(centerX, centerY), inOuterDiameter, outCenteredImg, inSubMean, inScaleFactor);
    }

    static CImg<unsigned char>
    genView(const CImg<float> & inImage, float inHfdValue, unsigned int inOuterDiameter = outerHfdDiameter, float inScaleFactor = defaultScaleFactor);  

    inline bool valid() const { return (mHfdValue > 0 && mImg.width() > 0 && mImg.height() > 0); }
    inline float getValue() const { return mHfdValue; }
    inline const CImg<float> & getResultImage() const { return mImg; }
    inline float getOuterDiameter() const { return mOuterDiameter; }
    CImg<unsigned char> genView() const { return HfdT::genView(mImg, mHfdValue, mOuterDiameter, mScaleFactor); }

    // TODO: Required??? inline void getCentroid(float * xcom, float * ycom) const { *xcom = mXCom; *ycom = mYCom; }
  };

}

#endif // _HFD_HPP_


















									  

