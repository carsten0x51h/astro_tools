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

#include "fwhm.hpp"
#include "hfd.hpp"
#include "vcurve.hpp"

namespace AT {
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

    static inline bool isValidStar(const CImg<float> & inImg, float inMinPsnr = 70) {
      if (! inImg.size()) {
	return false;
      }

      bool validStar;

      // Use centroid calculation to determine if there is a valid star selected
      try {
	PositionT centerPos(inImg.width() / 2, inImg.height() / 2);
	PositionT centroid = CentroidCalcT::starCentroid(inImg, centerPos, inImg.width(), CoordTypeT::ABSOLUTE);
	cout << dec << "Centroid: " << centroid << endl;
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
    inline double getFitness() const { return mFwhmHorz.getValue() + mFwhmVert.getValue() + mHfd.getValue(); }

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

  public:
    FocusFinderDataT(int inAbsPos, const StarDataT & inStarData /*, TODO: mVCurve*/) : mAbsPos(inAbsPos), mStarData(inStarData) { }
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
  class FocusFinderT {
  public:
    virtual void findFocus() = 0;
  };
}; // end AT namespace

#endif /* _FOCUS_FINDER_COMMON_HPP_ */