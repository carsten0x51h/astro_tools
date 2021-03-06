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


#ifndef _FOCUS_FINDER_COMMON_HPP_
#define _FOCUS_FINDER_COMMON_HPP_ _FOCUS_FINDER_COMMON_HPP_

#include <CImg.h>

#include "fwhm.hpp"
#include "hfd.hpp"
#include "vcurve.hpp"

#include "star_frame_selector.hpp"

namespace AT {  

  // TODO: Later, move to SequenceRecorderT.....
  typedef map<float /*pos*/, CImg<float> > PosToImgMapT;

  static const int cSelectionFrameSize = 31; // TODO: Make configure by using the windowSize parameter (which already exist...)...
  static const int cImageFrameSize = 3 * cSelectionFrameSize;

  static FrameT<float> getSelectionFrame(PointT<float> inCenterPosFF) {
    return centerPosToFrame(inCenterPosFF, cSelectionFrameSize, cSelectionFrameSize);
  }
  static FrameT<float> getImageFrame(PointT<float> inCenterPosFF) {
    return centerPosToFrame(inCenterPosFF, cImageFrameSize, cImageFrameSize);
  }

  
  struct TaskStateT {
    enum TypeE {
      READY,
      RUNNING,
      ABORTED,
      FINISHED,
      _Count
    };

    static const char * asStr(const TypeE & inType) {
      switch (inType) {
      case READY: return "READY";
      case RUNNING: return "RUNNING";
      case ABORTED: return "ABORTED";
      case FINISHED: return "FINISHED";
      default: return "<?>";
      }
    }
    MAC_AS_TYPE(Type, E, _Count);
  };

  
  // TODO: Question: Should this static function be part of the FocusFinderImpl? Actually it is more generic...
  static void
  calcStarValues(CImg<float> & inFrameImage, const BinningT & inBinning = BinningT(1,1), float * outDx = 0, float * outDy = 0, HfdT * outHfd = 0, FwhmT * outFwhmHorz = 0, FwhmT * outFwhmVert = 0, int * outMaxPixValue = 0) {

    // TODO / FIXME: In case of binning, the image needs to be scaled up to hit the minimum resolution requirements
    // TODO
    // TODO
    // TODO
    // Instread the corresponding window-sizes might be split in half.... TEST WHAT IS BETTER!!!
    CImg<float> zoomedImgFrame(inFrameImage);
    zoomedImgFrame.resize(inFrameImage.width() * inBinning.get<0>(), inFrameImage.height() * inBinning.get<1>());

    
    // Post process image... we assume that the star did not move too far from the image center
    // NOTE: Boundaries of currSubImage are based on currImageFrameFF.
    PointT<float> assumedCenter((float) zoomedImgFrame.width() / 2.0, (float) zoomedImgFrame.height() / 2.0);
    FrameT<unsigned int> newSelectionFrameIF;

    LOG(trace) << "inFrameImage.width(): " << inFrameImage.width() << ", inFrameImage.height(): " << inFrameImage.height()
	       << ", binning: " << inBinning << " --> zoomedImgFrame.width(): " << zoomedImgFrame.width()
	       << ", zoomedImgFrame.height(): " << zoomedImgFrame.height()
	       << " --> assumedCenter: " << assumedCenter << endl;

    bool insideBounds = StarFrameSelectorT::calc(zoomedImgFrame, 0 /*bitPix - TODO / HACK: not needed */,
						 assumedCenter, & newSelectionFrameIF,
						 StarFrameSelectorT::StarRecognitionTypeT::PROXIMITY,
						 CentroidT::CentroidTypeT::IWC, cSelectionFrameSize /*frameSize*/);
	  
    AT_ASSERT(StarFrameSelector, insideBounds, "Expected frame to be inside bounds.");	  

    PointT<float> newCenterPosIF = frameToCenterPos(newSelectionFrameIF);
    
    if (outDx) {
      *outDx = newCenterPosIF.get<0>() - assumedCenter.get<0>();
    }
    if (outDy) {
      *outDy = newCenterPosIF.get<1>() - assumedCenter.get<1>();
    }
    
    // Calculate star data
    // ------------------------------------------------------------------------------------------------------------ //
    CImg<float> subImg = zoomedImgFrame.get_crop(newSelectionFrameIF.get<0>() /*x0*/,
						 newSelectionFrameIF.get<1>() /*y0*/,
						 newSelectionFrameIF.get<0>() + newSelectionFrameIF.get<2>() - 1/*x1=x0+w-1*/,
						 newSelectionFrameIF.get<1>() + newSelectionFrameIF.get<3>() - 1/*y1=y0+h-1*/);
    if (outHfd) {
      try {
	// TODO: HFD value INCREASES if coming to focus using the simulator... !!! Maybe a simulator problem?! --> need real test!!!
	outHfd->set(subImg); // NOTE: HfdT takes image center as centroid, it does not matter if image is bigger
      } catch(std::exception & exc) {
	LOG(warning) << "HFD calculation failed! Details: " << exc.what() << endl;
      }
    }
    
    // Subtract median image
    double med = subImg.median();
    CImg<float> imageSubMed(subImg.width(), subImg.height());
    cimg_forXY(subImg, x, y) {
      imageSubMed(x, y) = (subImg(x, y) > med ? subImg(x, y) - med : 0);
    }

    if (outFwhmHorz) {
      try {
	outFwhmHorz->set(extractLine<DirectionT::HORZ>(imageSubMed));
      } catch(std::exception & exc) {
	LOG(warning) << "FWHM(horz) calculation failed!"  << endl;
      }
    }

    if (outFwhmVert) {
      try {
	outFwhmVert->set(extractLine<DirectionT::VERT>(imageSubMed));
      } catch(std::exception & exc) {
	LOG(warning) << "FWHM(horz) calculation failed!"  << endl;
      }
    }

    if (outMaxPixValue) {
      *outMaxPixValue = (int)zoomedImgFrame.max();
    }
  }
















  /////////////////////////////////////////////////////////////
  // NELOW HERE IS OLD STUFF....
  /////////////////////////////////////////////////////////////
  /*
   * StarDataT
   */
  class StarDataT {
    
  private:
    FwhmT mFwhmHorz;
    FwhmT mFwhmVert;
    HfdT mHfd;

  public:
    inline const FwhmT & getFwhmHorz() const { return mFwhmHorz; }
    inline FwhmT & getFwhmHorz() { return mFwhmHorz; }
    inline const FwhmT & getFwhmVert() const { return mFwhmVert; }
    inline FwhmT & getFwhmVert() { return mFwhmVert; }
    inline const HfdT & getHfd() const { return mHfd; }
    inline HfdT & getHfd() { return mHfd; }

    static inline bool isValidStar(const CImg<float> & inImg) {
      if (! inImg.size()) {
	return false;
      }

      bool validStar;

      // Use centroid calculation to determine if there is a valid star selected
      try {
	PointT<float> centerPos(inImg.width() / 2, inImg.height() / 2);
	//PointT<float> centroid = CentroidT::calc(inImg, centerPos, inImg.width(), CoordTypeT::ABSOLUTE);

	PointT<float> centroid;
	CentroidT::calc(inImg, centerPos, inImg.width(), & centroid);
	
	LOG(trace) << dec << "isValidStar() - centroid: " << centroid << endl;
	validStar = true;
      } catch(CentroidExceptionT & exc) {
	validStar = false;
      }
      return validStar;
    }

    /*
     * Currently the fitness is just the sum of all values - which we would
     * like to minimize. Later we might make this configurable by for example
     * just using FWHM or HFD or something else...
     */
    // TODO / FIXME! HFD sometines returns NAN!!! Can happen if outer radius is too small?!?!?
    //inline double getFitness() const { return mFwhmHorz.getValue() + mFwhmVert.getValue() + mHfd.getValue(); }
    //inline double getFitness() const { return mFwhmHorz.getValue() + mFwhmVert.getValue(); }
    // TODO / FIXME: Even worse... FWHM values are better (smaller) if almost only noise and worse (bigger) if star looks good!!!! --> only HFD!
    inline double getFitness() const { return mHfd.getValue(); }

    ostream & print(ostream & os) const {
      // TODO: pass printDetail?! Or remove mFwhmHorz.getValue()... and just call mFwhmHorz.print(detail)?
      os << "Fitness: " << getFitness()
	 << ", FWHM_horz=" << mFwhmHorz.getValue() << " [" << mFwhmHorz /*detail?!*/ << "]"
	 << ", FWHM_vert=" << mFwhmVert.getValue() << " [" << mFwhmVert << "]"
	 << ", HFD=" << mHfd.getValue() << endl;
      return os;
    }
    friend ostream & operator<<(ostream & os, const StarDataT & inStarData);
  };

  ostream & operator<<(ostream & os, const StarDataT & inStarData);


  /**
   * Focus finder data.
   */
  class FocusFinderDataT {
  private:
    int mAbsPos;
    StarDataT mStarData;
    float mProgress;
    string mUpdMsg;

  public:
    FocusFinderDataT(int inAbsPos, const StarDataT & inStarData, float inProgress, const string & inUpdMsg /*, TODO: mVCurve*/) : mAbsPos(inAbsPos), mStarData(inStarData), mProgress(inProgress), mUpdMsg(inUpdMsg) { }
    inline int getAbsPos() const { return mAbsPos; };
    inline const StarDataT & getStarData() const { return mStarData; };
    inline float getProgress() const { return mProgress; }
    inline const string & getUpdMsg() const { return mUpdMsg; }
  };


  struct MinMaxFocusPosT {
    enum TypeE {
      MIN_FOCUS_POS,
      MAX_FOCUS_POS,
      _Count
    };
    
    static const char * asStr(const TypeE & inType) {
      switch (inType) {
      case MIN_FOCUS_POS: return "MIN_FOCUS_POS";
      case MAX_FOCUS_POS: return "MAX_FOCUS_POS";
      default: return "<?>";
      }
    }
  }; // end struct


  /**
   * Focus finder interface.
   */
  struct FocusFinderInfoT {
    struct StateT {
      enum TypeE {
	READY,
	RUNNING,
	FINISHED_SUCCESS,
	FINISHED_FAILURE,
	FINISHED_ABORTED,
	_Count
      };
    
      static const char * asStr(const TypeE & inType) {
	switch (inType) {
	case READY: return "READY";
	case RUNNING: return "RUNNING";
	case FINISHED_SUCCESS: return "FINISHED_SUCCESS";
	case FINISHED_FAILURE: return "FINISHED_FAILURE";
	case FINISHED_ABORTED: return "FINISHED_ABORTED";
	default: return "<?>";
	}
      }
    }; // end struct

    FocusFinderInfoT() : mProgress(0), mState(StateT::READY) {}

    int mProgress;
    StateT::TypeE mState;
  };


  
  /**
   * Focus finder interface.
   */
  // TODO: To be removed
  class FocusFinderT {
  public:
    struct StatusT {
      enum TypeE {
  	RUNNING,
  	FINISHED_SUCCESS,
  	FINISHED_FAILURE,
  	_Count
      };
    
      static const char * asStr(const TypeE & inType) {
  	switch (inType) {
  	case RUNNING: return "RUNNING";
  	case FINISHED_SUCCESS: return "FINISHED_SUCCESS";
  	case FINISHED_FAILURE: return "FINISHED_FAILURE";
  	default: return "<?>";
  	}
      }
    }; // end struct
    
  private:
    StatusT::TypeE mStatus;
    
  public:    
    FocusFinderT() : mStatus(StatusT::RUNNING) {}
    inline StatusT::TypeE getStatus() const { return mStatus; }

    // TODO: Do we need both??
    virtual void step(const CImg<float> & inImage) = 0;
    virtual void findFocus() = 0;
  };
}; // end AT namespace

#endif /* _FOCUS_FINDER_COMMON_HPP_ */
