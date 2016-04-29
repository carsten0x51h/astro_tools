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


#include <boost/algorithm/string/case_conv.hpp>

#include "util.hpp"
#include "at_exception.hpp"

#include "threshold.hpp"
#include "cluster.hpp"
#include "normalize.hpp"
#include "centroid.hpp"

namespace AT {

  DEF_Exception(StarFrameSelector);
  

  // TODO: Add class methods as well
  // TODO: Generate an image with the different frames, the clicked pos. and the calculated centroid plotted.
  class StarFrameSelectorT {
  public:
    /**
     * Avaliable star recognion methods
     */
    struct StarRecognitionTypeT {
      enum TypeE {
	PROXIMITY,
	CLUSTERING,
	NONE,
	_Count
      };
      
      static const char * asStr(const TypeE & inType) {
	switch(inType) {
	case PROXIMITY: return "PROXIMITY";
	case CLUSTERING: return "CLUSTERING";
	case NONE: return "NONE";
	default: return "<?>";
	}
      }
      
      MAC_AS_TYPE(Type, E, _Count);
    };
  
    static bool
    calc(const CImg<float> & inImg, long bitPix, PointT<float> inSelectionCenter, FrameT<unsigned int> * outSelectedFrame,
	 typename StarRecognitionTypeT::TypeE inStarRecognitionMethod,
	 typename CentroidT::CentroidTypeT::TypeE inCentroidMethod, unsigned int inFrameSize = 25)
    {
      DimensionT<int> imageDimension(inImg.width(), inImg.height());
      FrameT<unsigned int> selectedFrame;

      LOG(debug) << "Beginning " << StarRecognitionTypeT::asStr(inStarRecognitionMethod) << " frame selection..." << endl;
      
      if (inStarRecognitionMethod == StarRecognitionTypeT::PROXIMITY) {
	// Calc frame of specified size around selectionCenter
	// NOTE: For now we just look around in 3 times the frame size for the maximum pixel

	// HACK!!!! IMPROVE - unify handling of frames, centers rounding etc...
	// NOTE: AtikDriver rounds a frameSize of 93 in binning 2 mode to 46 (rounds low)...
	//       This produces an imageDimension which is one pixel too small compared to the starSearchFrame...
	//       NEED A CLEAN WAY TO HANDLE THAT... 
	//int frameSize = (((int)inSelectionCenter.get<0>() % 2 == 0) ? (3.0 * inFrameSize - 1) : (3.0 * inFrameSize));
	//LOG(debug) << "inSelectionCenter.get<0>(): " << inSelectionCenter.get<0>() << " --> frameSize: " << frameSize << endl;
	// HACL DOES NOT WORK...
	float frameSize = inFrameSize;
	
	FrameT<int> starSearchFrame = centerPosToFrame(inSelectionCenter, frameSize);
	LOG(error) << "inSelectionCenter: " << inSelectionCenter << ", starSearchFrame: " << starSearchFrame << endl;
	LOG(error) << "imageDimension: " << imageDimension << endl;
	// TODO...
	
	if (! insideBounds(imageDimension, starSearchFrame)) {
	  LOG(debug) << "Frame hits image boundary." << endl;
	  return false;
	}

	// Find pixel with highest value (if multiple, pick first, print saturation warning)
	float maxPixelValue = 0;
	PointT<unsigned int> maxPixelPos;
	getMaxPixel(inImg, & starSearchFrame, CoordTypeT::ABSOLUTE, & maxPixelValue, & maxPixelPos);
	LOG(debug) << "max pixel pos: " << maxPixelPos << ", value: " << maxPixelValue << endl;

	// Re-center frame of same size around max pixel pos.
	// NOTE: maxRecenteredFrame cannot hit the boundary because search frame was 3 * inFrameSize.
	// NOTE: We decided to use int instead of float since sub-pixel accuracy is not needed here.
	FrameT<int> maxRecenteredFrame = centerPosToFrame(maxPixelPos, frameSize);
	LOG(debug) << "maxRecenteredFrame: " << maxRecenteredFrame << endl;
	
	// Calc centroid given the new frame
	CentroidT centroidObj(inImg, maxRecenteredFrame, inCentroidMethod);
	PointT<float> centroid(centroidObj.getCentroid(CoordTypeT::ABSOLUTE));
	
	// DEBUG START - Remove again
	// CImgDisplay thDsp3(centroidObj.genView(), "STAR-FRAME SELECTOR - CENTROID");
	// while (! thDsp3.is_closed()) {
	//   thDsp3.wait();
	// }
	// DEBUG END
	
	// Re-center frame to calculated centroid
	// NOTE: centroidRecenteredFrame cannot hit the boundary because search frame was 3 * inFrameSize.
	FrameT<int> centroidRecenteredFrame = centerPosToFrame(centroid, frameSize);
	LOG(debug) << "centroidRecenteredFrame: " << centroidRecenteredFrame << endl;
	    
	// TODO: How to detect if there is no valid star detected?? Should this actually be done here?
	selectedFrame = centroidRecenteredFrame;
      } else if (inStarRecognitionMethod == StarRecognitionTypeT::CLUSTERING) {
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
	  LOG(debug) << "Selecting closest star to position " << inSelectionCenter << "..." << endl;
	
	  // Pick closest star from cluster list...
	  // NOTE: We may solve this by a list sort with predicate instead... but this would require to change the list 
	  float minDist = std::numeric_limits<float>::max();
	
	  for (list<FrameT<int> >::const_iterator it = selectionList.begin(); it != selectionList.end(); ++it) {
	    PointT<float> clusterCenter = frameToCenterPos(*it);
	    float dist = distance(clusterCenter, inSelectionCenter);
	    if (dist < minDist) {
	      minDist = dist;
	      selectedFrame = *it;
	    }
	  }
	  LOG(debug) << "Selected closest star frame: " << selectedFrame << endl;
	} else {
	  // No star recognized...
	  LOG(warning) << "No star recognized with cluster method. Keeping selected frame." << endl;
	  selectedFrame = centerPosToFrame(inSelectionCenter, inFrameSize);
	}
      } else if(inStarRecognitionMethod == StarRecognitionTypeT::NONE) {
	// NOTE: In this case the click position is directly converted into a frame
	//       with the click position as its center.
	selectedFrame = centerPosToFrame(inSelectionCenter, inFrameSize);
	LOG(debug) << "Star recognition disabled. Frame: " << selectedFrame << endl;
      } else {
	AT_ASSERT(StarFrameSelector, false, "Unknown star recognition method.");
      }

      if (outSelectedFrame) {
	*outSelectedFrame = selectedFrame;
      }
      return true;
    }
  };


  /**
   * NOTE: This validate belongs to AT::StarFrameSelectorT::StarRecognitionTypeT.
   */
  void validate(boost::any & v, const vector<string> & values, typename StarFrameSelectorT::StarRecognitionTypeT::TypeE * target_type, int);  
  
};

  
#endif // __STAR_FRAME_SELECTOR_HPP__
