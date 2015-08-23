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

#ifndef __STAR_FRAME_SELECTOR_HPP__
#define __STAR_FRAME_SELECTOR_HPP__ __STAR_FRAME_SELECTOR_HPP__

#include "util.hpp"
#include "at_exception.hpp"

#include "threshold.hpp"
#include "cluster.hpp"
#include "normalize.hpp"

namespace AT {

  DEF_Exception(StarFrameSelector);
  

  // TODO: Add class methods as well
  // TODO: Generate an image with the different frames, the clicked pos. and the calculated centroid plotted.
  class StarFrameSelectorT {
  public:
    /**
     * Avaliable star selection methods
     */
    struct StarSelectionTypeT {
      enum TypeE {
	PROXIMITY,
	CLUSTERING,
	_Count
      };
      
      static const char * asStr(const TypeE & inType) {
	switch(inType) {
	case PROXIMITY: return "PROXIMITY";
	case CLUSTERING: return "CLUSTERING";
	default: return "<?>";
	}
      }
      
      MAC_AS_TYPE(Type, E, _Count);
    };


    static FrameT<unsigned int>
    calc(const CImg<float> & inImg, long bitPix, const string & starSelectMethod, typename StarSelectionTypeT::TypeE inStarRecognitionMethod,
	 typename CentroidT::CentroidTypeT::TypeE inCentroidMethod, unsigned int inFrameSize = 25)
    {
      DimensionT<int> imageDimension(inImg.width(), inImg.height());
      PointT<float> selectionCenter;
      FrameT<unsigned int> selectedFrame;
    
      if (! strcmp(starSelectMethod.c_str(), "auto")) {
	// Create a binary image to prepare clustering
	float th = ThresholdT::calc(inImg, bitPix, ThresholdT::ThresholdTypeT::OTSU);
	LOG(debug) << "Calculated threshold: " << th << endl;
	  
	CImg<float> thresholdImg(inImg); // Create a copy
	thresholdImg.threshold(th);
	  
	// // DEBUG START
	// CImgDisplay thDsp(thresholdImg, "TH IMG...");
	// while (! thDsp.is_closed()) {
	// 	thDsp.wait();
	// }
	// // DEBUG END
	  
	// Perform star clustering to determine interesting regions
	// TODO: Enabled clustering should not exit program!! Comment in...
	list<FrameT<int> > selectionList;
	ClusterT::calc(thresholdImg, & selectionList);
      
	AT_ASSERT(StarFrameSelector, selectionList.size(), "Automatic star selection needs at least one star.");
	LOG(debug) << "Automatically select 1 star out of " << selectionList.size() << "..." << endl;

	// TODO: Implement automatic star selection - For now we pick just the first one......
	// IDEA: Pick first (best? -> closest to center, brightness at max/2?) star from cluster list... / throw if more than one star?
	selectedFrame = *selectionList.begin();
      
      } else if (! strcmp(starSelectMethod.c_str(), "display")) {
	// Normalize image
	// TODO: Calculate value below instead of hard-coding it...
	float maxPossiblePixelValue = 65535.0;
	CImg<unsigned char> normalizedImage(normalize(inImg, maxPossiblePixelValue, 5.0 /*5%*/));

	// NOTE: See http://cimg.eu/reference/structcimg__library_1_1CImgDisplay.html
	// TODO: We will use our own graphical star-selector later - with zoom, value preview etc.
	CImgDisplay dispStarSelect(normalizedImage, "Select a star (click left)...");

	while (! dispStarSelect.is_closed()) {
	  dispStarSelect.wait();

	  if (dispStarSelect.button() & 1) { // Left button clicked.
	  
	    // NOTE:  If the image is bigger than the screen / window it is stretched and the mouse position
	    //        returned has to be computed accordingly.
	    int x = (float) inImg.width() / (float) dispStarSelect.width() * (float)dispStarSelect.mouse_x();
	    int y = (float) inImg.height() / (float) dispStarSelect.height() * (float)dispStarSelect.mouse_y();
	    selectionCenter = PointT<float>(x, y);
	    LOG(debug) << "Left click - position (x,y)=" << selectionCenter << endl;

	    if (inStarRecognitionMethod == StarSelectionTypeT::PROXIMITY) {
	      LOG(debug) << "Proximity star selection..." << endl;

	      // Calc frame of specified size around selectionCenter
	      // NOTE: For now we just look around in 3 times the frame size for the maximum pixel
	      FrameT<int> starSearchFrame = centerPosToFrame(selectionCenter, 3.0 * inFrameSize);
	      LOG(debug) << "starSearchFrame: " << starSearchFrame << endl;
	    
	      if (! insideBounds(imageDimension, starSearchFrame)) {
		LOG(warning) << "Selected frame hits image boundary." << endl;
		continue;
	      }

	      // Find pixel with highest value (if multiple, pick first, print saturation warning)
	      float maxPixelValue = 0;
	      PointT<unsigned int> maxPixelPos;
	      getMaxPixel(inImg, & starSearchFrame, CoordTypeT::ABSOLUTE, & maxPixelValue, & maxPixelPos);
	      LOG(debug) << "max pixel pos: " << maxPixelPos << ", value: " << maxPixelValue << endl;

	      // Re-center frame of same size around max pixel pos.
	      // NOTE: maxRecenteredFrame cannot hit the boundary because search frame was 3 * inFrameSize.
	      FrameT<int> maxRecenteredFrame = centerPosToFrame(maxPixelPos, inFrameSize);
	      LOG(debug) << "maxRecenteredFrame: " << maxRecenteredFrame << endl;
	    
	      // Calc centroid given the new frame
	      PointT<float> centroid;
	      CentroidT::calc(inImg, maxRecenteredFrame, & centroid, 0 /*centeredImg not required*/, CoordTypeT::ABSOLUTE, inCentroidMethod);
	    
	      // Re-center frame to calculated centroid
	      // NOTE: centroidRecenteredFrame cannot hit the boundary because search frame was 3 * inFrameSize.
	      FrameT<int> centroidRecenteredFrame = centerPosToFrame(centroid, inFrameSize);
	      LOG(debug) << "centroidRecenteredFrame: " << centroidRecenteredFrame << endl;
	    
	      // TODO: How to detect if there is no valid star detected?? Should this actually be done here?
	      selectedFrame = centroidRecenteredFrame;

	      // Exit loop after successful selection of frame.
	      break;
	    } else if (inStarRecognitionMethod == StarSelectionTypeT::CLUSTERING) {
	      // Create a binary image to prepare clustering
	      float th = ThresholdT::calc(inImg, bitPix, ThresholdT::ThresholdTypeT::OTSU);
	      LOG(debug) << "Calculated threshold: " << th << endl;
	  
	      CImg<float> thresholdImg(inImg); // Create a copy
	      thresholdImg.threshold(th);
	  
	      // Perform star clustering to determine interesting regions
	      list<FrameT<int> > selectionList;
	      ClusterT::calc(thresholdImg, & selectionList);
      
	      LOG(debug) << "Automatically select 1 star out of " << selectionList.size() << "..." << endl;

	      if (selectionList.size()) {
		LOG(debug) << "Selecting closest star to position " << selectionCenter << "..." << endl;
	
		// Pick closest star from cluster list...
		// NOTE: We may solve this by a list sort with predicate instead... but this would require to change the list 
		float minDist = std::numeric_limits<float>::max();
	
		for (list<FrameT<int> >::const_iterator it = selectionList.begin(); it != selectionList.end(); ++it) {
		  PointT<float> clusterCenter = frameToCenterPos(*it);
		  float dist = distance(clusterCenter, selectionCenter);
		  if (dist < minDist) {
		    minDist = dist;
		    selectedFrame = *it;
		  }
		}
		LOG(debug) << "Selected closest star frame: " << selectedFrame << endl;
	      } else {
		// No star recognized...
		LOG(warning) << "No star recognized with cluster method. Keeping selected frame." << endl;
		selectedFrame = centerPosToFrame(selectionCenter, inFrameSize);
	      }

	      // Exit loop in both cases.
	      break;
	    } else {
	      AT_ASSERT(StarFrameSelector, false, "Unknown star recognition method.");
	    }

	  
	    // DEBUG START - Show, what has been selected!!
	    // CImg<float> cropImg = normalizedImage.get_crop(selectedFrame.get<0>(), selectedFrame.get<1>(),
	    // 						 selectedFrame.get<0>() + selectedFrame.get<2>() - 1,
	    // 						 selectedFrame.get<1>() + selectedFrame.get<3>() - 1);
	    // CImgDisplay cropDsp(cropImg, "DEBUG - CROP IMG...");
	    // while (! cropDsp.is_closed()) {
	    //   cropDsp.wait();
	    // }
	    // DEBUG END
	  }
	} // end while
      } else {
	// Check if valid position
	boost::any v;
	vector<string> values;
	values.push_back(starSelectMethod);
	validate(v, values, & selectionCenter, 0);
	selectionCenter = any_cast<PointT<float> >(v); // throws boost::bad_any_cast
      
	// Just create a frame with the given window size at exactly the given position...
	selectedFrame = centerPosToFrame(selectionCenter, inFrameSize);
      }

      // Finally check again, if selected frame is not going to cut any image boundaries
      if (! insideBounds(imageDimension, selectedFrame)) {
	stringstream ss;
	ss << "Selected frame hits image boundary - image bounds: " << imageDimension
	   << ", starSearchFrame: " << selectedFrame << endl;
      
	LOG(error) << ss.str().c_str() << endl;

	// TODO: Rename exception...
	throw StarFrameSelectorExceptionT(ss.str().c_str());
      }
      return selectedFrame;
    }
  };


  /**
   * NOTE: This validate belongs to AT::StarFrameSelectorT::StarSelectionTypeT.
   */
  void validate(boost::any & v, const vector<string> & values, typename StarFrameSelectorT::StarSelectionTypeT::TypeE * target_type, int) {
    using namespace boost::program_options;
    
    validators::check_first_occurrence(v);
    string s = validators::get_single_string(values);
    boost::to_upper(s);
    typename StarFrameSelectorT::StarSelectionTypeT::TypeE type = StarFrameSelectorT::StarSelectionTypeT::asType(s.c_str());

    if (type != StarFrameSelectorT::StarSelectionTypeT::_Count) {
      v = any(type);
    } else {
      throw validation_error(validation_error::invalid_option_value);
    }
  }
  
  
};

  
#endif // __STAR_FRAME_SELECTOR_HPP__
