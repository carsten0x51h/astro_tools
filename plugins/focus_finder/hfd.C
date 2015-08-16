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

#include "hfd.hpp"

namespace AT {
  const float HfdT::defaultScaleFactor = 4.0;

  float
  HfdT::calc(const CImg<float> & inImage, const PointT<float> & inCenter, unsigned int inOuterDiameter,
       CImg<float> * outCenteredImg, bool inSubMean, float inScaleFactor)
  {  
    // Calculate centered frame
    FrameT<float> centeredSelectionFrame = centerPosToFrame(inCenter, inOuterDiameter /*window size*/);
    
    // Check if selection hits any image boundary (corner cases)...
    if (! insideBounds(DimensionT<float>(inImage.width(), inImage.height()), centeredSelectionFrame)) {
      stringstream ss;
      ss << "Centered frame hits image bounds (center=" << inCenter << ", outer diameter=" << inOuterDiameter
	 << " =>  selection frame=" << centeredSelectionFrame << " > image dimension (w=" << inImage.width()
	 << ", h=" << inImage.height() << "))." << endl;

	throw HfdExceptionT(ss.str().c_str());
    }
    
    // Crop the image
    CImg<float> subImg = inImage.get_crop(centeredSelectionFrame.get<0>() /*x0*/,
					  centeredSelectionFrame.get<1>() /*y0*/,
					  centeredSelectionFrame.get<0>() + centeredSelectionFrame.get<2>() /*x1 = x0 + w*/,
					  centeredSelectionFrame.get<1>() + centeredSelectionFrame.get<3>() /*y1 = y0 + h*/);

    // Sub mean image if desired
    if (inSubMean) {
      double mean = subImg.mean();
      cimg_forXY(subImg, x, y) {
	subImg(x, y) = (subImg(x, y) < mean ? 0 : subImg(x, y) - mean);
      }
    }
    
    // TODO: Check that iScaleFactor is at least 1
    // TODO: Scale up image if necessary

    // Sum up all pixel values in whole circle
    float outerRadius = (float) inOuterDiameter / 2.0;
    float sum = 0, sumDist = 0;
    cimg_forXY(subImg, x, y) {
      if (insideCircle(x, y, outerRadius /*centerX*/, outerRadius /*centerY*/, outerRadius)) {
	sum += subImg(x, y);
	sumDist += subImg(x, y) * sqrt(pow((float) x - outerRadius /*centerX*/, 2.0f) + pow((float) y - outerRadius /*centerX*/, 2.0f));
      }
    }

    // Make a copy of the image part which was used for calculation
    if (outCenteredImg) {
      // TODO: Zoom image? 
      *outCenteredImg = subImg;
    }
    
    // NOTE: Multiplying with 2 is required since actually just the HFR is calculated above
    // TODO: Does inScaleFactor has to be taken into account in the following calculation!!!???
    return (sum ? 2.0 * sumDist / sum : sqrt(2.0) * outerRadius);
  }

  CImg<unsigned char>
  HfdT::genView(const CImg<float> & inImage, float inHfdValue, unsigned int inOuterDiameter, float inScaleFactor)
  {
    // Create RGB image from fits file to paint boundaries and centroids (just for visualization)
    CImg<unsigned char> rgbImg(inImage.width(), inImage.height(), 1 /*depth*/, 3 /*3 channels - RGB*/);    
    const float min = inImage.min();
    const float mm = inImage.max() - min;

    cimg_forXY(inImage, x, y) {
      int value = 255.0 * (inImage(x,y) - min) / mm;
      rgbImg(x, y, 0 /*red*/) = value;
      rgbImg(x, y, 1 /*green*/) = value;
      rgbImg(x, y, 2 /*blue*/) = value;
    }

    // Scale image
    rgbImg.resize(inScaleFactor * inImage.width(), inScaleFactor * inImage.height(),
		  -100 /*size_z*/, -100 /*size_c*/, 1 /*interpolation_type*/);

    // Draw center
    const unsigned char red[3] = { 255, 0, 0 }, white[3] = { 255, 255, 255 }, black[3] = { 0, 0, 0 }, yellow[3] = { 255, 255, 0 };
    const size_t cCrossSize = 3;
    PointT<float> center((float) inImage.width() / 2.0, (float)inImage.height() / 2.0);

    rgbImg.draw_line(floor(inScaleFactor * (center.get<0>() - cCrossSize) + 0.5), floor(inScaleFactor * center.get<1>() + 0.5),
		     floor(inScaleFactor * (center.get<0>() + cCrossSize) + 0.5), floor(inScaleFactor * center.get<1>() + 0.5), red, 1 /*opacity*/);
    
    rgbImg.draw_line(floor(inScaleFactor * center.get<0>() + 0.5), floor(inScaleFactor * (center.get<1>() - cCrossSize) + 0.5),
		     floor(inScaleFactor * center.get<0>() + 0.5), floor(inScaleFactor * (center.get<1>() + cCrossSize) + 0.5), red, 1 /*opacity*/);

    // Draw HFD
    rgbImg.draw_circle(floor(inScaleFactor * center.get<0>() + 0.5), floor(inScaleFactor * center.get<1>() + 0.5),
		       inScaleFactor * inOuterDiameter / 2, yellow, 1 /*pattern*/, 1 /*opacity*/);
    
    rgbImg.draw_circle(floor(inScaleFactor * center.get<0>() + 0.5), floor(inScaleFactor * center.get<1>() + 0.5),
		       inScaleFactor * inHfdValue / 2, yellow, 1 /*pattern*/, 1 /*opacity*/);
	
    // Draw text
    ostringstream oss;
    oss.precision(4);
    oss << "HFD="<< inHfdValue << endl;
    
    rgbImg.draw_text(floor(inScaleFactor * center.get<0>() + 0.5), floor(inScaleFactor * center.get<1>() + 0.5),
		     oss.str().c_str(), white /*fg color*/, black /*bg color*/, 0.7 /*opacity*/, 9 /*font-size*/);

    return rgbImg; // Make a copy...
  }

  

}
