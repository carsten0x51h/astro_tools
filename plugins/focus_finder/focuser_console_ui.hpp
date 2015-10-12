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

#ifndef _FOCUSER_CONSOLE_UI_HPP_
#define _FOCUSER_CONSOLE_UI_HPP_ _FOCUSER_CONSOLE_UI_HPP_

#include "at_plugin.hpp"
#include "at_exception.hpp"

using namespace std;

namespace AT {

  class MenuEntryT {
  protected:
    string mStatusText;
    
  public:
    MenuEntryT() : mStatusText("") {}
    inline const string & getStatusText() { return mStatusText; };
    virtual void execute(int inKey) = 0;
    virtual const char* getName() const = 0;
    virtual string getValueAsStr() const = 0;
  };

  class SeparatorT : public MenuEntryT {
    virtual const char * getName() const { return ""; }
    virtual string getValueAsStr() const { return ""; }
    virtual void execute(int inKey) { }
  };

  class SetFocusStepSizeT : public MenuEntryT {
  private:
    int mFocusStepSize;
    const int cStepFactor = 10;
    
  public:
    SetFocusStepSizeT() : mFocusStepSize(10) {}
    virtual const char * getName() const { return "Step size :"; }
    virtual float getStepSize() const { return mFocusStepSize; }
    virtual string getValueAsStr() const { stringstream ss; ss << mFocusStepSize; return ss.str(); }
    virtual void execute(int inKey) {
      switch(inKey) {
      case KEY_LEFT: {
	if (mFocusStepSize > 1) { mFocusStepSize =  mFocusStepSize / cStepFactor; }
	mStatusText = "";
	break;
      }
      case KEY_RIGHT: {
	if (mFocusStepSize < 10000) { mFocusStepSize =  mFocusStepSize * cStepFactor; }
	mStatusText = "";
	break;
      }
      }
    }
  };
  
  class MoveFocusT : public MenuEntryT {
  private:
    IndiFocuserT * mFocuserDevice;
    SetFocusStepSizeT * mFocusStepSize;
    int mFocusPos;
    
  public:
    MoveFocusT(IndiFocuserT * inFocuserDevice, SetFocusStepSizeT * inFocusStepSize) : mFocuserDevice(inFocuserDevice), mFocusStepSize(inFocusStepSize) {
      mFocusPos = mFocuserDevice->getAbsPos();
    }
    virtual const char * getName() const { return "Focus pos (dest) :"; }
    virtual float getFocusPos() const { return mFocusPos; }
    virtual string getValueAsStr() const { stringstream ss; ss << mFocusPos; return ss.str(); }
    virtual void execute(int inKey) {
      switch(inKey) {
      case KEY_LEFT: {
	mFocusPos -= mFocusStepSize->getStepSize();
	mFocuserDevice->setAbsPos(mFocusPos, 0 /* non-blocking */);
	mStatusText = "";
	break;
      }
      case KEY_RIGHT: {
	mFocusPos += mFocusStepSize->getStepSize();
	mFocuserDevice->setAbsPos(mFocusPos, 0 /* non-blocking */);
	mStatusText = "";
	break;
      }
      case 27: {
	//mv_print(statusPosX, statusPosY, "Stopping focus motion.");
	mStatusText = "Stopping focus motion.";
	mFocuserDevice->abortMotion(0 /*non blocking*/);
      }
      }
    }
  };

  class ControlExposureT : public MenuEntryT {
  public:
    struct ExposureModeT {
      enum TypeE {
	SINGLE,
	LOOP,
	_Count
      };

      static const char * asStr(const TypeE & inType) {
	switch (inType) {
	case SINGLE: return "SINGLE";
	case LOOP: return "LOOP";
	default: return "<?>";
	}
      }

      MAC_AS_TYPE(Type, E, _Count);
    };

  private:
    IndiCameraT * mCameraDevice;
    float mExposureTimeSec;
    ExposureModeT::TypeE mExposureMode;
    bool mExposureDone;
				      
  public:
    ControlExposureT(IndiCameraT * inCameraDevice, float inExposureTimeSec = 1) : mCameraDevice(inCameraDevice),
										  mExposureTimeSec(inExposureTimeSec),
										  mExposureMode(ExposureModeT::SINGLE),
										  mExposureDone(false) { }
    
    virtual const char * getName() const { return "Exposure time [s] :"; }
    virtual string getValueAsStr() const {
      stringstream ss;
      ss << mExposureTimeSec << " [" << ExposureModeT::asStr(mExposureMode) << "]";
      return ss.str();
    }
    virtual void execute(int inKey) {
      switch(inKey) {
      case KEY_LEFT:
	mExposureMode = ExposureModeT::SINGLE;
	
	if (mExposureTimeSec <= 1 && mExposureTimeSec > 0.001) {
	  mExposureTimeSec /= 2;
	} else if (mExposureTimeSec > 1) {
	  mExposureTimeSec--;
	}
	mStatusText = "";
	break;
      
      case KEY_RIGHT:
	mExposureMode = ExposureModeT::SINGLE;
	
	if (mExposureTimeSec <= 1) {
	  mExposureTimeSec *= 2;
	} else if (mExposureTimeSec < 10000) {
	  mExposureTimeSec++;
	}
	mStatusText = "";
	break;
      
	// case KEY_ENTER:
	// 	//NOTE: Enter does not work for some reason...
	// 	//this->startExposure();
	// 	break;
      
      case 27: /*ESC*/
      case 'a':
	LOG(debug) << "Aborting exposure... Is exposure in progress? " << (mCameraDevice->isExposureInProgress() ? "yes" : "no") << endl;
	mExposureMode = ExposureModeT::SINGLE;
	
	if (mCameraDevice->isExposureInProgress()) {
	  LOG(debug) << "Ok, initiating abort procedure..." << endl;
	  mCameraDevice->abortExposure();
	  mExposureDone = false;
	  mStatusText = "Exposure aborted.";
	}
	break;

      case 'l':
	LOG(debug) << "Switching to LOOP mode..." << endl;
	mExposureDone = false;
	mExposureMode = ExposureModeT::LOOP;
	break;

      case 's':
	LOG(debug) << "Switching to SINGLE mode..." << endl;
	mExposureDone = false;
	mExposureMode = ExposureModeT::SINGLE;
	break;
      } /* end switch*/
    }
    inline float getExposureTime() const { return mExposureTimeSec; }
    inline ExposureModeT::TypeE getExposureMode() const { return mExposureMode; }
    inline bool getExposureDone() const { return mExposureDone; }
    inline void setExposureDone(bool inExposureDone) { mExposureDone = inExposureDone; }
  };


  class ControlBinningT : public MenuEntryT {
  private:
    IndiCameraT * mCameraDevice;
    BinningT mBinning;
    
  public:
    ControlBinningT(IndiCameraT * inCameraDevice, BinningT inBinning) : mCameraDevice(inCameraDevice), mBinning(inBinning) { }
    virtual const char * getName() const { return "Binning :"; }
    virtual BinningT getBinning() const { return mBinning; }
    virtual string getValueAsStr() const { stringstream ss; ss << mBinning.get<0>() << " x " << mBinning.get<1>(); return ss.str(); }
    virtual void execute(int inKey) {
      switch(inKey) {
      case KEY_LEFT: {
	if (mBinning.get<0>() > 1  /* We just use hor binning here...*/) {
	  mBinning.get<0>()--;
	  mBinning.get<1>()--;
	}
	mStatusText = "";
	break;
      }
      case KEY_RIGHT: {
	BinningT maxBinning = mCameraDevice->getMaxBinning();
	if (mBinning.get<0>() < maxBinning.get<0>() /* We just use hor binning here...*/) {
	  mBinning.get<0>()++;
	  mBinning.get<1>()++;
	}
	mStatusText = "";
	break;
      }
      }
    }
  };

  class ControlCameraTemperatureT : public MenuEntryT {
  private:
    IndiCameraT * mCameraDevice;
    int mTemperature;
    
  public:
    ControlCameraTemperatureT(IndiCameraT * inCameraDevice, int inTemperature = -15) : mCameraDevice(inCameraDevice),
										       mTemperature(inTemperature) { }
    virtual const char * getName() const { return "Temperature [\0xf8 C] :"; }
    virtual BinningT getTemperature() const { return mTemperature; }
    virtual string getValueAsStr() const { stringstream ss; ss << mTemperature << "\0xf8 C ["
							       << (mCameraDevice->isCoolerEnabled() ? "ON" : "OFF") << "]"; return ss.str(); }
    virtual void execute(int inKey) {
      switch(inKey) {
      case KEY_LEFT: {
	if (mTemperature >= mCameraDevice->getMinTemperature()) {
	  mTemperature--;
	}
	mStatusText = "";
	break;
      }
      case KEY_RIGHT: {
	if (mTemperature <= mCameraDevice->getMaxTemperature()) {
	  mTemperature++;
	}
	mStatusText = "";
	break;
      }
      }
    }
  };


  class FilterSelectT : public MenuEntryT {
  private:
    IndiFilterWheelT * mFilterWheelDevice;
    int mSelectedFilter;
    
  public:
    FilterSelectT(IndiFilterWheelT * inFilterWheelDevice, int inPos) : mFilterWheelDevice(inFilterWheelDevice),
								       mSelectedFilter(inPos) { }
    virtual const char * getName() const { return "Filter pos (dest) :"; }
    virtual BinningT getSelectedFilter() const { return mSelectedFilter; }
    virtual string getValueAsStr() const { stringstream ss; ss << mSelectedFilter; return ss.str(); }
    virtual void execute(int inKey) {
      switch(inKey) {
      case KEY_LEFT:
	if (mSelectedFilter > mFilterWheelDevice->getMinPos()) {
	  mSelectedFilter--;
	} else {
	  mSelectedFilter = mFilterWheelDevice->getNumSlots();
	}
	mFilterWheelDevice->setPos(mSelectedFilter, 0 /* non-blocking */);
	mStatusText = "";
	break;
	
      case KEY_RIGHT:
	if (mSelectedFilter < mFilterWheelDevice->getNumSlots()) {
	  mSelectedFilter++;
	} else {
	  mSelectedFilter = mFilterWheelDevice->getMinPos();
	}
	mFilterWheelDevice->setPos(mSelectedFilter, 0 /* non-blocking */);
	mStatusText = "";
	break;
      }
    }
  };
  
  
  DEF_Exception(FocusFinderPlugin);
  DEF_Exception(UnknownFocusFinderImpl);
  DEF_Exception(RequireOption);
  DEF_Exception(ImageDimension);
  DEF_Exception(WindowOutOfBounds);
  
  template <typename... Args> void
  mv_print(int x, int y, const char * c, Args&&... args) {
    move(y, x);
    clrtoeol();
    mvprintw(y, x, c, std::forward<Args>(args)...);
  }

  // TODO: Create ManualConsoleFocusCntl class or similar...??!
  static void
  manualConsoleFocusCntl(const po::variables_map & inCmdLineMap, IndiCameraT * inCameraDevice, IndiFocuserT * inFocuserDevice,
			 IndiFilterWheelT * inFilterWheelDevice, const FrameT<unsigned int> & inSelectionFrame, float inExposureTimeSec,
			 BinningT inBinning, bool inFollowStar = true)
  {
    const unsigned int statusPosX = 5;
    const unsigned int statusPosY = 27;
    const size_t infoColumn = 50;

    // TODO FIXME: getSelectionFrame() needs to take care of border!! Use borderFactor below...
    const float borderFactor = 3.0; // TODO: Pass as argument?!
    PointT<float> centerPos = frameToCenterPos(inSelectionFrame);
    FrameT<float> imageFrame = centerPosToFrame(centerPos, borderFactor * inSelectionFrame.get<2>(), borderFactor * inSelectionFrame.get<3>());

    bool haveFocalDistance = (inCmdLineMap.count("focal_distance") > 0);
    bool havePixelSize = (inCmdLineMap.count("pixel_size") > 0);
    unsigned int focalDistance = inCmdLineMap["focal_distance"].as<unsigned int>();
    const DimensionT<float> & pixelSize = inCmdLineMap["pixel_size"].as<DimensionT<float> >();
    
    LOG(trace) << "Initial selectionFrame(by click): " << inSelectionFrame
	       << ", centerPos: " << centerPos << ", initial imageFrame: " << imageFrame << endl;
    
    CImgDisplay currentImageDisp, currentFwhmHorzDisp, currentFwhmVertDisp, currentHfdDisp;
    CImg<float> image;
    int key;

    int focusPos = inFocuserDevice->getAbsPos();
    float hfdValue = -1;
    float fwhmHorzValue = -1;
    float fwhmVertValue = -1;
    float maxPixelValue = -1;
    
    initscr();
    crmode();
    keypad(stdscr, TRUE);
    noecho();
    clear();
    move(7 /*y*/,5 /*x*/);
    refresh();

    timeout(10);
    key = getch();

    // Build the menu
    vector<MenuEntryT *> menuActions;
    
    SetFocusStepSizeT * focusStepSize = new SetFocusStepSizeT();
    ControlExposureT * controlExposure = new ControlExposureT(inCameraDevice, inExposureTimeSec);
    ControlBinningT * controlBinning = new ControlBinningT(inCameraDevice, inBinning);
    menuActions.push_back(focusStepSize);
    menuActions.push_back(new MoveFocusT(inFocuserDevice, focusStepSize));
    menuActions.push_back(new SeparatorT());
    menuActions.push_back(new SeparatorT());

    menuActions.push_back(controlExposure);
    menuActions.push_back(controlBinning);
    if (inCameraDevice->hasCooler()) {
      menuActions.push_back(new ControlCameraTemperatureT(inCameraDevice, inCameraDevice->getTemperature()));
    }

    if (inFilterWheelDevice) {
      menuActions.push_back(new SeparatorT());
      menuActions.push_back(new SeparatorT());
      menuActions.push_back(new FilterSelectT(inFilterWheelDevice, inFilterWheelDevice->getPos()));
    }
    
    int position = 0;
    int numEntries = menuActions.size();
    
    while(key != 'q') {

      if (! inCameraDevice->isExposureInProgress()) {
	// Get previous image (if any)
	inCameraDevice->getImage(& image);

	LOG(trace) << "Received image [subframe: " << imageFrame << "] from camera - size: " << image.width() << " x " << image.height() << endl;
	LOG(trace) << "image.width(): " << image.width() << ", image.height(): " << image.height() << endl;
	LOG(trace) << "controlBinning->getBinning().get<0>(): " << controlBinning->getBinning().get<0>()
		   << ", controlBinning->getBinning().get<1>(): " << controlBinning->getBinning().get<1>() << endl;

	if (image.width() == imageFrame.get<2>() && image.height() == imageFrame.get<3>()) {
	  // Perform re-centroiding i.e. re-set selectionFrame in order to follow the star
	  // ------------------------------------------------------------------------------------------------------------ //
	  FrameT<unsigned int> newFrame;

	  // IDEA: Pass current image, re-center using given method
	  PointT<float> assumedCenter((float) image.width() / 2.0, (float) image.height() / 2.0); // We assume that the star did not move too far from the image center

	  LOG(trace) << "image.width(): " << image.width() << ", image.height(): " << image.height()
		     << ", --> assumed center: " << assumedCenter << endl;

	  // TODO: Do not pass StarFrameSelectorT::StarRecognitionTypeT::PROXIMITY,CentroidT::CentroidTypeT::IWC...
	  //       Instead pass what has been selected by parameters..
	  bool insideBounds = StarFrameSelectorT::calc(image, 0 /*bitPix - TODO / HACK: not needed */,
						       assumedCenter, & newFrame,
						       StarFrameSelectorT::StarRecognitionTypeT::PROXIMITY,
						       CentroidT::CentroidTypeT::IWC, inSelectionFrame.get<2>() /*frameSize*/);

	  AT_ASSERT(StarFrameSelector, insideBounds, "Expected frame to be inside bounds.");
 
	  // Calculate new imageFrame
	  PointT<float> newLocalCenterPos = frameToCenterPos(newFrame); // In "imageFrame" coordinates
	  float deltaX = ((float) imageFrame.get<2>() / 2.0) - newLocalCenterPos.get<0>();
	  float deltaY = ((float) imageFrame.get<3>() / 2.0) - newLocalCenterPos.get<1>();
	  PointT<float> oldCenterPos = frameToCenterPos(imageFrame); // In "full frame" coordinates
	  PointT<float> newCenterPos(oldCenterPos.get<0>() - deltaX, oldCenterPos.get<1>() - deltaY); // In "full frame" coordinates
	  FrameT<unsigned int> newImageFrame = centerPosToFrame(newCenterPos, imageFrame.get<2>(), imageFrame.get<3>());

	  LOG(debug) << "Old imageFrame: " << imageFrame << ", oldCenterPos: " << oldCenterPos << endl;
	  LOG(trace) << "frameSize used: " << inSelectionFrame.get<2>() << ", assumedCenter: " << assumedCenter
		     << ", newFrame: " << newFrame << " --> newLocalCenterPos: " << newLocalCenterPos
		     << ", --> deltaX: " << deltaX << ", deltaY: " << deltaY << endl;
	  LOG(debug) << "New imageFrame: " << newImageFrame << ", newCenterPos: " << newCenterPos << endl;

	  // TODO / FIXME: Check if it hits boundaries!!??
	  
	  // Get sub-frame for analysis
	  // ------------------------------------------------------------------------------------------------------------ //
	  // Convert to imageFrame coordinates
	  FrameT<float> selectionFrame(newFrame);

	  CImg<float> subImg = image.get_crop(selectionFrame.get<0>() /*x0*/,
	   				      selectionFrame.get<1>() /*y0*/,
	   				      selectionFrame.get<0>() + selectionFrame.get<2>() - 1/*x1=x0+w-1*/,
	   				      selectionFrame.get<1>() + selectionFrame.get<3>() - 1/*y1=y0+h-1*/);

	  // Calculate star data
	  // ------------------------------------------------------------------------------------------------------------ //
	  // TODO: try catch?
	  try {
	    // TODO: HFD value INCREASES if coming to focus using the simulator... !!! Maybe a simulator problem?! --> need real test!!!
	    HfdT hfd(subImg); // NOTE: HfdT takes image center as centroid, it does not matter if image is bigger
	    hfdValue = hfd.getValue();
	    currentHfdDisp.display(hfd.genView());
	  } catch(std::exception & exc) {
	    LOG(warning) << "HFD calculation failed!"  << endl;
	  }

	  // Subtract median image
	  double med = subImg.median();
	  CImg<float> imageSubMed(subImg.width(), subImg.height());
	  cimg_forXY(subImg, x, y) {
	    imageSubMed(x, y) = (subImg(x, y) > med ? subImg(x, y) - med : 0);
	  }
	  
	  try {
	    FwhmT fwhmHorz(extractLine<DirectionT::HORZ>(imageSubMed));
	    fwhmHorzValue = fwhmHorz.getValue();
	    currentFwhmHorzDisp.display(fwhmHorz.genView());
	  } catch(std::exception & exc) {
	    LOG(warning) << "FWHM(horz) calculation failed!"  << endl;
	  }
	  
	  try {
	    FwhmT fwhmVert(extractLine<DirectionT::VERT>(imageSubMed));
	    fwhmVertValue = fwhmVert.getValue();
	    currentFwhmVertDisp.display(fwhmVert.genView());
	  } catch(std::exception & exc) {
	    LOG(warning) << "FWHM(horz) calculation failed!"  << endl;
	  }
	  
	  maxPixelValue = subImg.max();

	  
	  // DEBUG START
	  // CImg<unsigned char> tmpNormalizedImage(normalize(subImg, 65535.0, 5.0));
	  // CImgDisplay thDsp(tmpNormalizedImage, "SUB-IMG...");
	  // while (! thDsp.is_closed()) {
	  //   thDsp.wait();
	  // }
	  // CImg<unsigned char> hfdView = hfd.genView();
	  // CImgDisplay thDsp2(hfdView, "HFD-VIEW...");
	  // while (! thDsp2.is_closed()) {
	  //   thDsp2.wait();
	  // }
	  // DEBUG END
	  
	  // Finally, scale and display the image
	  // ------------------------------------------------------------------------------------------------------------ //
	  // TODO: Move this code below to a genView function of e.g. CentroidT? Does this make sense??
	  // TODO: Calculate value below instead of hard-coding it...
	  float maxPossiblePixelValue = 65535.0;
	  CImg<unsigned char> normalizedImage(normalize(image, maxPossiblePixelValue, 5.0 /*TODO: HACK FIXME! Should not be hardcoded*/));

	  // Zoom image so that the star can be seen better...
	  // NOTE: RGB because we may add additional things in color to indicate for example the centroid / frame...
	  const float scaleFactor = 3.0;
	  
	  CImg<unsigned char> rgbImg(normalizedImage.width(), normalizedImage.height(), 1 /*depth*/, 3 /*3 channels - RGB*/);    
	  cimg_forXY(normalizedImage, x, y) {
	    int value = normalizedImage(x,y);
	    rgbImg(x, y, 0 /*red*/) = value;
	    rgbImg(x, y, 1 /*green*/) = value;
	    rgbImg(x, y, 2 /*blue*/) = value;
	  }

	  rgbImg.resize(scaleFactor * normalizedImage.width(), scaleFactor * normalizedImage.height(),
			-100 /*size_z*/, -100 /*size_c*/, 1 /*interpolation_type*/);

	  // Draw selected region
	  const unsigned char red[3] = { 255, 0, 0 }, green[3] = { 0, 255, 0 }, blue[3] = { 0, 0, 255 };
	  rgbImg.draw_rectangle(floor(scaleFactor * selectionFrame.get<0>() + 0.5), floor(scaleFactor * selectionFrame.get<1>() + 0.5),
	  			floor(scaleFactor * (selectionFrame.get<0>() + selectionFrame.get<2>()) + 0.5),
	  			floor(scaleFactor * (selectionFrame.get<1>() + selectionFrame.get<3>()) + 0.5),
	  			green, 1 /*opacity*/, ~0 /*pattern*/);

	  // Draw center
	  const size_t cCrossSize = 3;

	  // TODO: Write "drawCross" function...
	  // PointT<float> localNewCenterPos(newCenterPos.get<0>() - imageFrame.get<0>() - selectionFrame.get<0>(),
	  // 				  newCenterPos.get<1>() - imageFrame.get<1>() - selectionFrame.get<1>());
	  PointT<float> localOldCenterPos(oldCenterPos.get<0>() - imageFrame.get<0>(),
	   				  oldCenterPos.get<1>() - imageFrame.get<1>());
	  // TODO:  +1 ?!!
	  // rgbImg.draw_line(floor(scaleFactor * (localOldCenterPos.get<0>() - cCrossSize + 1) + 0.5), floor(scaleFactor * (localOldCenterPos.get<1>() + 1) + 0.5),
	  // 		   floor(scaleFactor * (localOldCenterPos.get<0>() + cCrossSize + 1) + 0.5), floor(scaleFactor * (localOldCenterPos.get<1>() + 1) + 0.5),
	  // 		   blue, 1 /*opacity*/);
	  
	  // rgbImg.draw_line(floor(scaleFactor * (localOldCenterPos.get<0>() + 1) + 0.5), floor(scaleFactor * (localOldCenterPos.get<1>() - cCrossSize + 1) + 0.5),
	  // 		   floor(scaleFactor * (localOldCenterPos.get<0>() + 1) + 0.5), floor(scaleFactor * (localOldCenterPos.get<1>() + cCrossSize + 1) + 0.5),
	  // 		   blue, 1 /*opacity*/);


	  
	  // PointT<float> localNewCenterPos(newCenterPos.get<0>() - imageFrame.get<0>(),
	  //  				  newCenterPos.get<1>() - imageFrame.get<1>());

	  // rgbImg.draw_line(floor(scaleFactor * (localNewCenterPos.get<0>() - cCrossSize + 1) + 0.5), floor(scaleFactor * (localNewCenterPos.get<1>() + 1) + 0.5),
	  // 		   floor(scaleFactor * (localNewCenterPos.get<0>() + cCrossSize + 1) + 0.5), floor(scaleFactor * (localNewCenterPos.get<1>() + 1) + 0.5),
	  // 		   red, 1 /*opacity*/);
	  
	  // rgbImg.draw_line(floor(scaleFactor * (localNewCenterPos.get<0>() + 1) + 0.5), floor(scaleFactor * (localNewCenterPos.get<1>() - cCrossSize + 1) + 0.5),
	  // 		   floor(scaleFactor * (localNewCenterPos.get<0>() + 1) + 0.5), floor(scaleFactor * (localNewCenterPos.get<1>() + cCrossSize + 1) + 0.5),
	  // 		   red, 1 /*opacity*/);

	  rgbImg.draw_line(floor(scaleFactor * (localOldCenterPos.get<0>() - cCrossSize) + 0.5), floor(scaleFactor * localOldCenterPos.get<1>() + 0.5),
			   floor(scaleFactor * (localOldCenterPos.get<0>() + cCrossSize) + 0.5), floor(scaleFactor * localOldCenterPos.get<1>() + 0.5),
			   blue, 1 /*opacity*/);
	  
	  rgbImg.draw_line(floor(scaleFactor * localOldCenterPos.get<0>() + 0.5), floor(scaleFactor * (localOldCenterPos.get<1>() - cCrossSize) + 0.5),
			   floor(scaleFactor * localOldCenterPos.get<0>() + 0.5), floor(scaleFactor * (localOldCenterPos.get<1>() + cCrossSize) + 0.5),
			   blue, 1 /*opacity*/);

	  
	  PointT<float> localNewCenterPos(newCenterPos.get<0>() - imageFrame.get<0>(),
	   				  newCenterPos.get<1>() - imageFrame.get<1>());

	  rgbImg.draw_line(floor(scaleFactor * (localNewCenterPos.get<0>() - cCrossSize) + 0.5), floor(scaleFactor * localNewCenterPos.get<1>() + 0.5),
			   floor(scaleFactor * (localNewCenterPos.get<0>() + cCrossSize) + 0.5), floor(scaleFactor * localNewCenterPos.get<1>() + 0.5), red, 1 /*opacity*/);
	  rgbImg.draw_line(floor(scaleFactor * localNewCenterPos.get<0>() + 0.5), floor(scaleFactor * (localNewCenterPos.get<1>() - cCrossSize) + 0.5),
			   floor(scaleFactor * localNewCenterPos.get<0>() + 0.5), floor(scaleFactor * (localNewCenterPos.get<1>() + cCrossSize) + 0.5), red, 1 /*opacity*/);

	  
	  currentImageDisp.display(rgbImg);
	  
	  if (inFollowStar && controlExposure->getExposureMode() != ControlExposureT::ExposureModeT::SINGLE) {
	    // If star following is enabled, the camera sub-frame is re-centered after each image
	    imageFrame = newImageFrame;
	  }
	} else {
	  LOG(debug) << "Received something unexpected! Expected image size: " << imageFrame.get<2>() << " x " << imageFrame.get<3>()
		     << " but received " << image.width() << " x " << image.height() << " -> ignoring." << endl;
	}
	
	// TODO: Should this be moved to the menu items?!
	LOG(trace) << "setBinning(" << controlBinning->getBinning() << ")..." << endl;
	inCameraDevice->setBinning(controlBinning->getBinning());

	//TODO: setBinning after first selection of imageFrame causes CentroidException (Frame hit simage bounds...) -> need to recalc imageFrame(?) - based on old binning and given new binning!
	LOG(trace) << "setBinnedFrame(" << imageFrame << ", " << controlBinning->getBinning() << ")..." << endl;
	inCameraDevice->setBinnedFrame(imageFrame, controlBinning->getBinning());
	
	LOG(trace) << "setFrameType(" << FrameTypeT::asStr(FrameTypeT::LIGHT) << ")..." << endl;
	inCameraDevice->setFrameType(FrameTypeT::LIGHT);
	LOG(trace) << "setCompressed(false)..." << endl;
	inCameraDevice->setCompressed(false); // No compression when moving image to CImg

	if (! controlExposure->getExposureDone() || controlExposure->getExposureMode() == ControlExposureT::ExposureModeT::LOOP) {
	  LOG(trace) << "startExposure(" << inExposureTimeSec << "s)..." << endl;
	  inCameraDevice->startExposure(controlExposure->getExposureTime()); // Non-blocking call...!!
	  controlExposure->setExposureDone(true);
	}
      } // end if - ! exposure in progress


      
      switch(key) {
      case KEY_UP:
	while (position > 0) {
	  position--;
	  bool isSeparator =  ! strcmp(menuActions[position]->getName(), "");
	  if (! isSeparator) { break; }
	}
	break;
	
      case KEY_DOWN:
	while (position < numEntries - 1) {
	  position++;
	  bool isSeparator =  ! strcmp(menuActions[position]->getName(), "");
	  if (! isSeparator) { break; }
	}
	break;
	
      case 9: /* TAB */
	bool isSeparator;
	do {
	  position++;
	  position = position % numEntries;
	  
	  isSeparator =  ! strcmp(menuActions[position]->getName(), "");
	} while (isSeparator);
	break;

      default: {
	menuActions[position]->execute(key);
	break;
      }
      } /* switch */

      const int cTopMenuBorder = 1;
      const int cLeftMenuBorder = 10;
      const int cLeftMenuWidth = 20;
      
      for (vector<MenuEntryT *>::iterator it = menuActions.begin(); it != menuActions.end(); ++it) {
	const MenuEntryT * menuEntry = *it;
	
	int pos = std::distance(menuActions.begin(), it);

	if (pos == position) { attron(A_STANDOUT); }
	mv_print(cLeftMenuBorder, cTopMenuBorder + pos, "%s", menuEntry->getName());
	if (pos == position) { attroff(A_STANDOUT); }

	mv_print(cLeftMenuBorder + cLeftMenuWidth, cTopMenuBorder + pos, "%s", menuEntry->getValueAsStr().c_str());
      }

      
      // Update data
      string focuserStatus = (inFocuserDevice->isMovementInProgess() ? "busy" : "ready"); 
      mv_print(infoColumn, 1, "Focus status: %s", focuserStatus.c_str());
      mv_print(infoColumn, 2, "Focus pos (is): %d", inFocuserDevice->getAbsPos());
      if (inFocuserDevice->supportsTemperature()) {
	mv_print(infoColumn, 3, "Focus temp: %d\0xf8 C", inFocuserDevice->getTemperature());
      }
      
      stringstream ssCameraStatus;
      if (inCameraDevice->isExposureInProgress()) {
	ssCameraStatus << "exposing..." << inCameraDevice->getExposureTime() << "s";
      } else {
	ssCameraStatus << "ready";
      }

      mv_print(infoColumn, 5, "Camera status: %s", ssCameraStatus.str().c_str());

      if (inCameraDevice->hasCooler()) {
	mv_print(infoColumn, 6, "Cooler status: ");
	mv_print(infoColumn, 7, "Temperature: ");
      }

      if (inFilterWheelDevice) {
	stringstream ssFilterWheelStatus;
	if (inFilterWheelDevice->isMovementInProgess()) {
	  ssFilterWheelStatus << "moving... is: " << inFilterWheelDevice->getPos();
	} else {
	  ssFilterWheelStatus << "ready";
	}
	mv_print(infoColumn, 9, "Filter status: %s", ssFilterWheelStatus.str().c_str());
      }
      
      
      if (hfdValue > 0) {
	stringstream hfdArcSecSs;
	if (haveFocalDistance && havePixelSize) {
	  hfdArcSecSs << " = " << FwhmT::pxToArcsec(hfdValue, focalDistance, pixelSize, controlBinning->getBinning()) << "\"";
	}
	mv_print(infoColumn, 15, "HFD: %fpx%s", hfdValue, hfdArcSecSs.str().c_str());
      } else {
	mv_print(infoColumn, 15, "HFD: n.a.");
      }

      if (fwhmHorzValue > 0) {
	stringstream fwhmHorzArcSecSs;

	if (haveFocalDistance && havePixelSize) {
	  fwhmHorzArcSecSs << " = " << FwhmT::pxToArcsec(fwhmHorzValue, focalDistance, pixelSize, controlBinning->getBinning()) << "\"";
	}
	mv_print(infoColumn, 16, "FWHM(horz): %fpx%s", fwhmHorzValue, fwhmHorzArcSecSs.str().c_str());
      } else {
	mv_print(infoColumn, 16, "FWHM(horz): n.a.");
      }
      
      if (fwhmVertValue > 0) {
	stringstream fwhmVertArcSecSs;

	if (haveFocalDistance && havePixelSize) {
	  fwhmVertArcSecSs << " = " << FwhmT::pxToArcsec(fwhmVertValue, focalDistance, pixelSize, controlBinning->getBinning()) << "\"";
	}
	mv_print(infoColumn, 17, "FWHM(vert): %fpx%s", fwhmVertValue, fwhmVertArcSecSs.str().c_str());
      } else {
	mv_print(infoColumn, 17, "FWHM(vert): n.a.");
      }

      if (maxPixelValue > 0) {
	mv_print(infoColumn, 18, "MAX: %f", maxPixelValue);
      } else {
	mv_print(infoColumn, 18, "MAX: n.a.");
      }
      mv_print(5, 25, "Focus finder. Press 'q' to quit");
      
    
      refresh();
      timeout(10);
      key = getch();
    } /* while */

    // Cleanup menu
    for (vector<MenuEntryT *>::iterator it = menuActions.begin(); it != menuActions.end(); ++it) {
      delete *it;
    }
    
    endwin();
  }

}; // end AT

#endif /* _FOCUSER_CONSOLE_UI_HPP_ */
