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

#include <boost/algorithm/string/case_conv.hpp>

#include "centroid.hpp"

namespace AT {
  const float CentroidT::defaultScaleFactor = 4.0;

  void
  CentroidT::calc(const CImg<float> & inImg, const FrameT<float> & inSelectionFrame, PointT<float> * outCenter, CImg<float> * outImg,
		  CoordTypeT::TypeE inCoordType, CentroidTypeT::TypeE inCalcType, float inZoomFactor, bool inSubMean) {
    AT_ASSERT(Centroid, outCenter, "outCenter expected to be set.");
    
    PointT<float> relCenter;
    
    // Check if selection hits any image boundary...
    if (! insideBounds(DimensionT<float>(inImg.width(), inImg.height()), inSelectionFrame)) {
      throw CentroidExceptionT("Frame hits image bounds.");
    }
      
    // Extract sub-image, check corner cases...
    CImg<float> subImg = inImg.get_crop(inSelectionFrame.get<0>() /*x0*/,
					inSelectionFrame.get<1>() /*y0*/,
					inSelectionFrame.get<0>() + inSelectionFrame.get<2>() - 1 /*x1=x0+w*/,
					inSelectionFrame.get<1>() + inSelectionFrame.get<3>() - 1/*y1=y0+h*/);

    // Noise reduction
    // AD noise reduction --> In: Loaded image, Out: Noise reduced image
    // http://cimg.sourceforge.net/reference/structcimg__library_1_1CImg.html
    CImg<float> & aiImg = subImg.blur_anisotropic(130.0f, /*amplitude*/
						  0.7f, /*sharpness*/
						  0.3f, /*anisotropy*/
						  0.6f, /*alpha*/
						  1.1f, /*sigma*/
						  0.8f, /*dl*/
						  30,   /*da*/
						  2,    /*gauss_prec*/
						  0,    /*interpolation_type*/
						  false /*fast_approx*/
						  );

    // Sub mean image if desired
    if (inSubMean) {
      double mean = aiImg.mean();
      cimg_forXY(aiImg, x, y) {
	aiImg(x, y) = (aiImg(x, y) < mean ? 0 : aiImg(x, y) - mean);
      }
    }
    
    // TODO: "zoom" (interpolate) image by factor...

    switch(inCalcType) {
    case CentroidTypeT::IWC: {
      calcIntensityWeightedCenter(aiImg, & relCenter); // center is relative
      break;
    }
    case CentroidTypeT::IWC_SUB: {
      PointT<float> centerIwc;
      calcIntensityWeightedCenter(aiImg, & centerIwc); // center is relative
      calcCentroidSubPixel(aiImg, centerIwc, & relCenter, 20 /*numIterations*/);
      break;
    }
    case CentroidTypeT::MOMENT2: {
      calcCentroid2ndMoment(aiImg, & relCenter); // center is relative
      break;
    }
    default: {
      AT_ASSERT(Centroid, false, "Invalid CentroidTypeT.");
    }
    }
      
    // Calculate new centered frame
    PointT<float> absCenter(relCenter.get<0>() + inSelectionFrame.get<0>(), relCenter.get<1>() + inSelectionFrame.get<1>());
    FrameT<float> centeredSelectionFrameAbs = centerPosToFrame(absCenter, inSelectionFrame.get<2>() /*w*/, inSelectionFrame.get<3>() /*h*/);

    // Check if selection hits any image boundary (corner cases)...
    if (! insideBounds(DimensionT<float>(inImg.width(), inImg.height()), centeredSelectionFrameAbs)) {
      throw CentroidExceptionT("Centered frame hits image bounds.");
    }
    
    /**
     * We return a copy of the zoomed (interpolated), centered image
     * with the boundaries of the passed frame. The boundaries are
     * checked after "re-centering" again.
     */
    if (outImg) {     
      // Extract sub-image
      CImg<float> centeredSubImg = inImg.get_crop(centeredSelectionFrameAbs.get<0>() /*x0*/,
						  centeredSelectionFrameAbs.get<1>() /*y0*/,
						  centeredSelectionFrameAbs.get<0>() + centeredSelectionFrameAbs.get<2>() - 1/*x1 = x0 + w - 1*/,
						  centeredSelectionFrameAbs.get<1>() + centeredSelectionFrameAbs.get<3>() - 1/*y1 = y0 + h - 1*/);

      // TODO: Zoom image

      // Finally, return a copy
      *outImg = centeredSubImg;
    }
    
    // Transform to absolute coordinates if required
    *outCenter = (CoordTypeT::ABSOLUTE == inCoordType ? absCenter : relCenter);
    LOG(info) << "calc - final centroid coords (x y)=" << *outCenter << endl;
  }


  CImg<unsigned char>
  CentroidT::genView(const CImg<float> & inImage, const PointT<float> & inCenter, float inScaleFactor) {
    // TODO: Move this to a "genView" function!
    CImg<unsigned char> rgbImg(inImage.width(), inImage.height(), 1 /*depth*/, 3 /*3 channels - RGB*/);    

    // TODO: We may use the normalize function instead...
    const float min = inImage.min();
    const float mm = inImage.max() - min;

    cimg_forXY(inImage, x, y) {
      int value = 255.0 * (inImage(x,y) - min) / mm;
      rgbImg(x, y, 0 /*red*/) = value;
      rgbImg(x, y, 1 /*green*/) = value;
      rgbImg(x, y, 2 /*blue*/) = value;
    }

    // Draw center cross
    const unsigned char red[3] = { 255, 0, 0 };
    const size_t cCrossSize = 3;

    // Scale image
    rgbImg.resize(inScaleFactor * rgbImg.width(), inScaleFactor * rgbImg.height(),
		  -100 /*size_z*/, -100 /*size_c*/, 1 /*interpolation_type*/);

    // TODO: We may use a generic draw-cross function since this is needed multiple times...
    rgbImg.draw_line(floor(inScaleFactor * (inCenter.get<0>() - cCrossSize) + 0.5), floor(inScaleFactor * inCenter.get<1>() + 0.5),
		     floor(inScaleFactor * (inCenter.get<0>() + cCrossSize) + 0.5), floor(inScaleFactor * inCenter.get<1>() + 0.5), red, 1 /*opacity*/);
    rgbImg.draw_line(floor(inScaleFactor * inCenter.get<0>() + 0.5), floor(inScaleFactor * (inCenter.get<1>() - cCrossSize) + 0.5),
		     floor(inScaleFactor * inCenter.get<0>() + 0.5), floor(inScaleFactor * (inCenter.get<1>() + cCrossSize) + 0.5), red, 1 /*opacity*/);

    return rgbImg; // Make a copy...
  }

  
  /**
   * NOTE: This validate belongs to AT::CentroidT::CentroidTypeT.
   */
  void validate(boost::any & v, const vector<string> & values, typename CentroidT::CentroidTypeT::TypeE * target_type, int) {
    using namespace boost::program_options;
    
    validators::check_first_occurrence(v);
    string s = validators::get_single_string(values);
    boost::to_upper(s);
    typename CentroidT::CentroidTypeT::TypeE type = CentroidT::CentroidTypeT::asType(s.c_str());

    if (type != CentroidT::CentroidTypeT::_Count) {
      v = any(type);
    } else {
      throw validation_error(validation_error::invalid_option_value);
    }
  }

};
