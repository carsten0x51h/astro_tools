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

#include <boost/thread.hpp>

#include "focuser_console_ui.hpp"
#include "star_frame_selector.hpp"
#include "focus_finder_parabel_fit_impl.hpp"
#include "centroid.hpp"
#include "fwhm.hpp"
#include "hfd.hpp"
#include "at_console_display.hpp"
#include "at_console_menu.hpp"

using namespace std;
using namespace boost;

namespace AT {

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


  void
  genSelectionView(const CImg<float> & inCurrSubImg, int dx, int dy,
		   CImg<unsigned char> * outRgbImg, size_t inBitsPerPixel = 16, float inScaleFactor = 3.0) {
    const unsigned char red[3] = { 255, 0, 0 }, green[3] = { 0, 255, 0 }, blue[3] = { 0, 0, 255 };
    const size_t cCrossSize = 3;

    // Finally, generate the image for the user...
    float maxPossiblePixelValue = pow(2.0, inBitsPerPixel);
					     
    CImg<unsigned char> normalizedImage(normalize(inCurrSubImg, maxPossiblePixelValue, 5.0 /*TODO: HACK FIXME! Should not be hardcoded*/));

    // NOTE: RGB because we may add additional things in color to indicate for example the centroid / frame...
    CImg<unsigned char> & rgbImgRef = *outRgbImg;
    rgbImgRef.resize(normalizedImage.width(), normalizedImage.height(), 1 /*size_z*/, 3 /*3 channels - RGB*/, 1 /*interpolation_type*/);
    
    cimg_forXY(normalizedImage, x, y) {
      int value = normalizedImage(x,y);
      rgbImgRef(x, y, 0 /*red*/) = value;
      rgbImgRef(x, y, 1 /*green*/) = value;
      rgbImgRef(x, y, 2 /*blue*/) = value;
    }
	    
    rgbImgRef.resize(inScaleFactor * normalizedImage.width(), inScaleFactor * normalizedImage.height(),
		     -100 /*size_z*/, -100 /*size_c*/, 1 /*interpolation_type*/);

    // Draw the "new" frame - this is the frame from which the next "imageFrame" will be calculated (using
    // the "border" - in this case factor 3. This new "imageFrame" will be the boundary for the next image
    // taken from the camera. From the drawm selection the HFD, FWHM etc. are calculated.
    PointT<float> starCenter(inCurrSubImg.width() / 2 + dx, inCurrSubImg.height() / 2 + dy);

    drawCross(& rgbImgRef, starCenter, red, cCrossSize, inScaleFactor);
    drawFrame(& rgbImgRef, getSelectionFrame(starCenter), red, inScaleFactor);
  }


  
  struct StartAbortT {
    enum TypeE {
      START,
      ABORT,
      _Count
    };

    static const char * asStr(const TypeE & inType) {
      switch (inType) {
      case START: return "START";
      case ABORT: return "ABORT";
      default: return "<?>";
      }
    }
    MAC_AS_TYPE(Type, E, _Count);
  };


  struct CoolerStateT {
    enum TypeE {
      DISABLED,
      ENABLED,
      _Count
    };

    static const char * asStr(const TypeE & inType) {
      switch (inType) {
      case DISABLED: return "DISABLED";
      case ENABLED: return "ENABLED";
      default: return "<?>";
      }
    }
    MAC_AS_TYPE(Type, E, _Count);
  };
  
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


  /******************************************************************************
   * EXPOSURE TASK
   ******************************************************************************/
  struct ExposureCntlDataT {
    float exposureTime;
    PointT<float> centerPosFF;
    BinningT binning;
  };

  struct ExposureStatusDataT {
    TaskStateT::TypeE taskState;
    int progress;
    int secondsLeft;
    ExposureStatusDataT() : taskState(TaskStateT::READY), progress(0), secondsLeft(0) {}
  };


  // NOTE: We may move this to a class if states are required 
  void exposeTask(IndiCameraT * inCameraDevice, atomic<ExposureCntlDataT> * cntlData, atomic<ExposureStatusDataT> * statusData, CImg<float> * imageData) {
    ExposureStatusDataT status;
    status.taskState = TaskStateT::RUNNING;
    statusData->store(status);  // updates the variable safely

    ExposureCntlDataT cntl = cntlData->load();
  
    LOG(trace) << "setBinning(" << cntl.binning << ")..." << endl;
    inCameraDevice->setBinning(cntl.binning);

    //TODO: setBinning after first selection of imageFrame causes CentroidException (Frame hit simage bounds...)
    //      -> need to recalc imageFrame(?) - based on old binning and given new binning!
    LOG(trace) << "setBinnedFrame(" << getImageFrame(cntl.centerPosFF) << ", " << cntl.binning << ")..." << endl;
    inCameraDevice->setBinnedFrame(getImageFrame(cntl.centerPosFF), cntl.binning);
	
    LOG(trace) << "setFrameType(" << FrameTypeT::asStr(FrameTypeT::LIGHT) << ")..." << endl;
    inCameraDevice->setFrameType(FrameTypeT::LIGHT);

    LOG(trace) << "setCompressed(false)..." << endl;
    inCameraDevice->setCompressed(false); // No compression when moving image to CImg
  
    inCameraDevice->startExposure(cntl.exposureTime); // Non-blocking call
    // NOTE: All up to here could be done before statrting the thread!

  
    while (inCameraDevice->isExposureInProgress()) {
      // Update status...
      status.secondsLeft = inCameraDevice->getExposureTime();
      status.progress = 100.0 * (1.0 - ((float) status.secondsLeft / (float)cntl.exposureTime));
      statusData->store(status);  // updates the variable safely

      try {
	chrono::milliseconds dura(200);
	this_thread::sleep_for(dura); // Sleeps for a bit - this is a predefined interruption point
      } catch(boost::thread_interrupted const& ) {
	inCameraDevice->abortExposure();
	status.taskState = TaskStateT::ABORTED;
	statusData->store(status);  // updates the variable safely
	return;
      }
    }

    // Get the image from the camera
    inCameraDevice->getImage(imageData);

    // Update status
    status.taskState = TaskStateT::FINISHED;
    status.progress = 100;
    status.secondsLeft = 0;
    statusData->store(status);  // updates the variable safely
    
    return;
  }


  // TODO... return resulting values (HFD, FWHM, ...) ..
  void
  calcStarValues(CImg<float> & inFrameImage, int * outDx = 0, int * outDy = 0, HfdT * outHfd = 0, FwhmT * outFwhmHorz = 0, FwhmT * outFwhmVert = 0) {
    // Post process image... we assume that the star did not move too far from the image center
    // NOTE: Boundaries of currSubImage are based on currImageFrameFF.
    PointT<float> assumedCenter((float) inFrameImage.width() / 2.0, (float) inFrameImage.height() / 2.0);
    FrameT<unsigned int> newSelectionFrameIF;
    bool insideBounds = StarFrameSelectorT::calc(inFrameImage, 0 /*bitPix - TODO / HACK: not needed */,
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
    CImg<float> subImg = inFrameImage.get_crop(newSelectionFrameIF.get<0>() /*x0*/,
					       newSelectionFrameIF.get<1>() /*y0*/,
					       newSelectionFrameIF.get<0>() + newSelectionFrameIF.get<2>() - 1/*x1=x0+w-1*/,
					       newSelectionFrameIF.get<1>() + newSelectionFrameIF.get<3>() - 1/*y1=y0+h-1*/);
    if (outHfd) {
      try {
	// TODO: HFD value INCREASES if coming to focus using the simulator... !!! Maybe a simulator problem?! --> need real test!!!
	outHfd->set(subImg); // NOTE: HfdT takes image center as centroid, it does not matter if image is bigger
      } catch(std::exception & exc) {
	LOG(warning) << "HFD calculation failed!"  << endl;
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
  }














  

  /******************************************************************************
   * FIND FOCUS TASK
   ******************************************************************************/
  // Aproximate parabel
  typedef CurveFitTmplT<ParabelFitTraitsT> ParabelMatcherT;

  struct FocusFindCntlDataT {
    float exposureTime;
    PointT<float> centerPosFF;
    BinningT binning;
    FocusFindCntlDataT() : exposureTime(0), binning(BinningT(1,1)), centerPosFF(PointT<float>(0,0)) {}
  };

  struct FocusFindStatusDataT {
    bool isRunning;
    int currAbsFocusPos;
    PointT<float> currCenterPosFF;
    int progress;
    float fwhmHorz;
    float fwhmVert;
    float hfd;
    FocusFindStatusDataT() : isRunning(false), currAbsFocusPos(0), progress(0), fwhmHorz(0), fwhmVert(0), hfd(0) {}
  };


  struct FocusFinderDataSetT {
    HfdT mHfd;
    FwhmT mFwhmHorz;
    FwhmT mFwhmVert;
    FocusFinderDataSetT() {};
    FocusFinderDataSetT(const HfdT & inHfd, const FwhmT & inFwhmHorz, const FwhmT & inFwhmVert) : mHfd(inHfd), mFwhmHorz(inFwhmHorz), mFwhmVert(inFwhmVert) {}
  };
  
  
  // TODO: Too many parameters... NEED NEW DESIGN...   maybe class / part of class with props as members...?!
  static PointT<float> mapToImgCoords(const PointT<float> & inCurveCoords, float inWidth, float inHeight, float inMinX, float inMaxX, float inMinY, float inMaxY) {
    const float facX = (inMaxX == inMinX ? 1.0f : inWidth / (inMaxX - inMinX)); 
    const float facY = (inMaxY == inMinY ? 1.0f : inHeight / (inMaxY - inMinY)); 
    
    float xMap = (inMaxX == inMinX ? 0.5f * inWidth : facX * (inCurveCoords.get<0>() - inMinX) );
    float yMap = (inMaxY == inMinY ? 0.5f * inHeight : inHeight - (facY * (inCurveCoords.get<1>() - inMinY)));

    return PointT<float>(xMap, yMap);
  }
  
  static void
  genCurve(const PointLstT<float> & inPoints, CImg<unsigned char> * outRgbImg, const LineT<float> * inLineL, const LineT<float> * inLineR,
	   const ParabelMatcherT::CurveParamsT::TypeT * inParabelParms, const unsigned char * inColor, size_t inCrossSize = 3) {

    // TODO: ASSERT outRgbImg...
    // Map position
    size_t width = outRgbImg->width();
    size_t height = outRgbImg->height();

    outRgbImg->fill(0);
    
    // TODO: Maybe put this code to a generic helper function...
    float minX = std::numeric_limits<float>::max();
    float maxX = std::numeric_limits<float>::min();
    float minY = std::numeric_limits<float>::max();
    float maxY = std::numeric_limits<float>::min();

    for (PointLstT<float>::const_iterator it = inPoints.begin(); it != inPoints.end(); ++it) {
      if (it->get<0>() < minX) { minX = it->get<0>(); }
      if (it->get<0>() > maxX) { maxX = it->get<0>(); }
      if (it->get<1>() < minY) { minY = it->get<1>(); }
      if (it->get<1>() > maxY) { maxY = it->get<1>(); }
    }

    // Draw points
    for (PointLstT<float>::const_iterator it = inPoints.begin(); it != inPoints.end(); ++it) {
      drawCross(outRgbImg, mapToImgCoords(*it, width, height, minX, maxX, minY, maxY), inColor, inCrossSize, 1.0);
    }

    // Draw lines
    if (inLineL) {
      for (int i = minX; i < maxX; ++i) {
	PointT<float> mapPoint = mapToImgCoords(PointT<float>(i, inLineL->f(i)), width, height, minX, maxX, minY, maxY);
	outRgbImg->draw_point(mapPoint.get<0>(), mapPoint.get<1>(), inColor, 1 /*opacity*/);
      }
    }

    if (inLineR) {
      for (int i = minX; i < maxX; ++i) {
	PointT<float> mapPoint = mapToImgCoords(PointT<float>(i, inLineR->f(i)), width, height, minX, maxX, minY, maxY);
	outRgbImg->draw_point(mapPoint.get<0>(), mapPoint.get<1>(), inColor, 1 /*opacity*/);
      }
    }

    if (inParabelParms) {
      for (int i = minX; i < maxX; ++i) {
	PointT<float> mapPoint = mapToImgCoords(PointT<float>(i, ParabelFitTraitsT::fx(i, *inParabelParms)), width, height, minX, maxX, minY, maxY);
	outRgbImg->draw_point(mapPoint.get<0>(), mapPoint.get<1>(), inColor, 1 /*opacity*/);
      }
    }

  }

  template<typename T>
  class PointLstAccessorT {
  public:
    typedef PointLstT<T> TypeT;
    static DataPointT getDataPoint(size_t inIdx, typename TypeT::const_iterator inIt) {
      DataPointT dp(inIt->get<0>(), inIt->get<1>());
      return dp;
    }
  };

  
  // IDEA: My focuser is quite fine grained! Hence the step-size is much bigger than the values in the paper! The "critical area" where I was trying to find the focus and where I was tring to record the V-Curve is just the very small "red" area in the graph. Hence, I need to choose the step-size much bigger than I thought or at least I need to move in a region I did not expect. Still, this requires to determine the initial slope / direction and this requires to determine the initial step-size (if it is not specified by the user).
  //
  // TODO:
  //
  // NP = new absolute position
  // OP = old absolute position
  // newHfd = desired new HFD (not measured?! - probably theoretical value - TODO: What is the theoretical best HFD?)
  // oldHfd is = measured HFD at old position
  // Slope = VCurve slope on the side we are operating (dy / dx = dHFD / dPos)
  //
  // NP = OP + (newHFD - oldHFD) / Slope
  // <=> NP - OP = (newHFD - oldHFD) / Slope
  // <=> Slope * (NP - OP) = newHFD - oldHFD
  // <=> Slope = (newHFD - oldHFD) / (NP - OP)   (= dy / dx = dHFD / dPos)
  //
  //
  // Given: OP, oldHFD (measure), Slope (from previous values -> VCurve -> Calc slope), newHFD is the HFD I want to achieve by moving to NP.
  // Needed: NP
  //
  // Question: Where do I get newHFD from???????????? -> Maybe specify desired number of steps?!
  //
  //
  // NOTE: Algorithm assumes that star is roughly in focus!!! (-> getAbsPos() is used as base)
  void focusFindTask(IndiCameraT * inCameraDevice, IndiFocuserT * inFocuserDevice, atomic<FocusFindCntlDataT> * cntl, atomic<FocusFindStatusDataT> * status) {
    map<int /*absPos*/, FocusFinderDataSetT> recordedStarData;
    
    FocusFindStatusDataT statusData;
    statusData.isRunning = true;
    status->store(statusData);  // updates the variable safely

    FocusFindCntlDataT cntlData = cntl->load();
    
    // 1. DETERMINE BOUNDARIES
    const size_t numSteps = 10;
    int focusStartPos = inFocuserDevice->getAbsPos();
    int deltaL = fabs(inFocuserDevice->getMinPos() - focusStartPos);
    int deltaR = fabs(inFocuserDevice->getMaxPos() - focusStartPos);
    int stepSize = std::min(deltaL, deltaR) / numSteps;

    cerr << "focusStartPos: " << focusStartPos << endl;
    cerr << "MaxFocusPos: " << inFocuserDevice->getMaxPos() << endl;
    cerr << "MinFocusPos: " << inFocuserDevice->getMinPos() << endl;
    cerr << "deltaL: " << deltaL << ", deltaR: " << deltaR << endl;
    cerr << "stepSize: " << stepSize << endl;

    PointT<float> currCenterPosFF = cntlData.centerPosFF;

    // Take a picture and determine current HFD and FWHM...
    CImg<float> currSubImage;
    inCameraDevice->takePicture(& currSubImage, cntlData.exposureTime, getImageFrame(currCenterPosFF),
				FrameTypeT::LIGHT, cntlData.binning, false /*compressed*/);
    
    HfdT hfd;
    FwhmT fwhmHorz, fwhmVert;
    int dx = 0, dy = 0;
    calcStarValues(currSubImage, & dx, & dy, & hfd, & fwhmHorz, & fwhmVert);

    if (true /*TODO: wantRecenter*/) {
      currCenterPosFF.get<0>() += dx;
      currCenterPosFF.get<1>() += dy;
    }

    // TODO: Save image data - status......
    // Update status...
    statusData.currAbsFocusPos = inFocuserDevice->getAbsPos();
    statusData.currCenterPosFF = currCenterPosFF;
    statusData.progress = 0;
    statusData.fwhmHorz = fwhmHorz.getValue();
    statusData.fwhmVert = fwhmVert.getValue();
    statusData.hfd = hfd.getValue();
    
    status->store(statusData);  // updates the variable safely

    // TODO: We may move StarDataT into the statusData directly...
    recordedStarData[inFocuserDevice->getAbsPos()] = FocusFinderDataSetT(hfd, fwhmHorz, fwhmVert);
      
    // NOTE: DISPLAY STIFF WILL BE MOVED OUT OF HERE
    CImgDisplay currImageDisp;// TODO: Should probably not be here...
    CImgDisplay currHfdCurveDisp, currFwhmHorzCurveDisp, currFwhmVertCurveDisp;// TODO: Should probably not be here...

    const size_t _width = 600, _height = 500;
    const unsigned char green[3] = { 0, 255, 0 }, red[3] = { 255, 0, 0 }, blue[3] = { 0, 0, 255 };
    const size_t cCrossSize = 5;
    CImg<unsigned char> hfdCurveRgbImg(_width, _height, 1 /*size_z*/, 3 /*3 channels - RGB*/, 1 /*interpolation_type*/);
    CImg<unsigned char> fwhmHorzCurveRgbImg(_width, _height, 1 /*size_z*/, 3 /*3 channels - RGB*/, 1 /*interpolation_type*/);
    CImg<unsigned char> fwhmVertCurveRgbImg(_width, _height, 1 /*size_z*/, 3 /*3 channels - RGB*/, 1 /*interpolation_type*/);
    

    
    // Do "numSteps" steps in both directions
    int currAbsFocusDestPos = focusStartPos;
    int sign[2] = { -1, 1 };
    PointLstT<float> hfdPoints[2];
    PointLstT<float> allHfdPoints, allFwhmHorzPoints, allFwhmVertPoints;

    for (size_t i = 0; i < 2; ++i) {
      for (size_t step = 0; step < numSteps; ++step) {
	// Move focus
	currAbsFocusDestPos += sign[i] * stepSize;
	cerr << "currAbsFocusDestPos: " << currAbsFocusDestPos << endl;
	inFocuserDevice->setAbsPos(currAbsFocusDestPos); // Wait for focus to be move - TODO!! Check if the function blocks by default!!!

	// TODO: CHECK if setAbsPos is BLOCKING!! Does not seem to be the case!!!!! -> WAIT in setAbsPos is commented out...
				 
	// Take picture etc.
	inCameraDevice->takePicture(& currSubImage, cntlData.exposureTime, getImageFrame(currCenterPosFF),
				    FrameTypeT::LIGHT, cntlData.binning, false /*compressed*/);
	calcStarValues(currSubImage, & dx, & dy, & hfd, & fwhmHorz, & fwhmVert);

	if (true /*TODO: wantRecenter*/) {
	  currCenterPosFF.get<0>() += dx;
	  currCenterPosFF.get<1>() += dy;
	}
	
	// Save data for further calculation (TODO: Code below might be simplified / removed...)
	// TODO: Add same for fwhms?
	PointT<float> currHfdPosition(inFocuserDevice->getAbsPos(), hfd.getValue());
	hfdPoints[i].push_back(currHfdPosition);
	allHfdPoints.push_back(currHfdPosition); // TODO: Required? Better solution?
	LOG(error) << "hfdPoints[" << i << "] = (" << inFocuserDevice->getAbsPos() << ", " << hfd.getValue() << ")" << endl;

	PointT<float> currFwhmHorzPosition(inFocuserDevice->getAbsPos(), fwhmHorz.getValue());
	allFwhmHorzPoints.push_back(currFwhmHorzPosition); // TODO: Required? Better solution?
	LOG(error) << "fwhmHorzPoints[" << i << "] = (" << inFocuserDevice->getAbsPos() << ", " << fwhmHorz.getValue() << ")" << endl;

	PointT<float> currFwhmVertPosition(inFocuserDevice->getAbsPos(), fwhmVert.getValue());
	allFwhmVertPoints.push_back(currFwhmVertPosition); // TODO: Required? Better solution?
	LOG(error) << "fwhmVertPoints[" << i << "] = (" << inFocuserDevice->getAbsPos() << ", " << fwhmVert.getValue() << ")" << endl;

	
	///////////////////////////////////////////////////////////////////////
	// DRAW START
	// DRAW - just TMP - will be moved out of algorithm and handled by/in update handler which will be called from here...
	genCurve(allHfdPoints, & hfdCurveRgbImg, 0, 0, 0, green, cCrossSize);
	currHfdCurveDisp.display(hfdCurveRgbImg);

	genCurve(allFwhmHorzPoints, & fwhmHorzCurveRgbImg, 0, 0, 0, red, cCrossSize);
	currFwhmHorzCurveDisp.display(fwhmHorzCurveRgbImg);

	genCurve(allFwhmVertPoints, & fwhmVertCurveRgbImg, 0, 0, 0, blue, cCrossSize);
	currFwhmVertCurveDisp.display(fwhmVertCurveRgbImg);
	// DRAW END
	////////////////////////////////////////////////////////////////////////
	
	
	// Update status...
	statusData.currAbsFocusPos = inFocuserDevice->getAbsPos();
	statusData.currCenterPosFF = currCenterPosFF;
	statusData.progress = 100.0 * ((float) step / (numSteps - 1)); // TODO: Need total progress as well...
	statusData.fwhmHorz = fwhmHorz.getValue();
	statusData.fwhmVert = fwhmVert.getValue();
	statusData.hfd = hfd.getValue();
    
	status->store(statusData);  // updates the variable safely

	recordedStarData[inFocuserDevice->getAbsPos()] = FocusFinderDataSetT(hfd, fwhmHorz, fwhmVert);

	
	// Display img... - TODO: Should probably not be here...
	CImg<unsigned char> rgbImg;
	genSelectionView(currSubImage, dx, dy, & rgbImg, inCameraDevice->getBitsPerPixel() /*BPP for normalization*/, 3.0 /*zoom*/);
	currImageDisp.display(rgbImg); // TODO - disable if no Wnds...
	
	try {
	  boost::this_thread::interruption_point();
	} catch(boost::thread_interrupted const& ) {
	  statusData.isRunning = false;
	  status->store(statusData);  // updates the variable safely
	  return;
	}
      }
      
      // Back to start pos...
      currAbsFocusDestPos = focusStartPos;
      inFocuserDevice->setAbsPos(focusStartPos);
    }

    // Finally go back to startPos ... TODO: Instead, calc optimal position... and then go there...
    inFocuserDevice->setAbsPos(focusStartPos);

    // DEBUG START - print values..
    for (map<int /*absPos*/, FocusFinderDataSetT>::const_iterator it = recordedStarData.begin(); it != recordedStarData.end(); it++) {
      int pos = it->first;
      const FocusFinderDataSetT & focusFinder = it->second;

      LOG(warning) << "pos: " << pos << ", hfd: " << focusFinder.mHfd.getValue()
		   << ", fwhmHorz: " << focusFinder.mFwhmHorz.getValue()
		   << ", fwhmVert: " << focusFinder.mFwhmVert.getValue() << endl;
    }
    // DEBUG END

    LineT<float> bestFitLineL(hfdPoints[0]);
    LineT<float> bestFitLineR(hfdPoints[1]);

    float corrL = correlation<float>(hfdPoints[0]);
    float corrR = correlation<float>(hfdPoints[1]);
    
    // DEBUG START
    // TODO: LOG does not convert ostream correctly... compile error...
    LOG(error) << " > bestFitLineL: " << endl;
    cerr << bestFitLineL << endl;
    LOG(error) << "CorrelationL: " << corrL << endl;
    
    LOG(error) << " > bestFitLineR: " << endl;
    // TODO: LOG does not convert ostream correctly... compile error...
    //LOG(error) << bestFitLineR << endl;
    cerr << bestFitLineR << endl;
    LOG(error) << "CorrelationR: " << corrR << endl;
    // DEBUG END

  
    // Do the LM fit, throws CurveFitExceptionT if error boundaries are specified and NOT satisfied
    ParabelMatcherT::CurveParamsT::TypeT parabelParms;
    ParabelMatcherT::fitGslLevenbergMarquart<PointLstAccessorT<float> >(allHfdPoints, & parabelParms, 1 /*TODO max abs err*/, 1 /* TODO max rel err*/, false /* no ex */);

    float a = parabelParms[ParabelMatcherT::CurveParamsT::A_IDX];
    float b = parabelParms[ParabelMatcherT::CurveParamsT::B_IDX];
    float c = parabelParms[ParabelMatcherT::CurveParamsT::C_IDX];
      
    LOG(error) << "Calculated parabel parms - a: " << a << ", b: " << b << ", c: " << c << endl;
    
    // Calculate xmin, ymin
    float xMin = - b / (2.0f * a);
    float yMin = - 0.25f * (b * b) / a + c;

    LOG(error) << "xMin: " << xMin << ", yMin: " << yMin << endl;
    
    // TODO: Draw parabel ...
    // Draw data points with lines
    genCurve(allHfdPoints, & hfdCurveRgbImg, & bestFitLineL, & bestFitLineR, & parabelParms, green, cCrossSize);
    currHfdCurveDisp.display(hfdCurveRgbImg);

    

    
    
    // Calculate intersection point
    try {
      PointT<float> sp = LineT<float>::calcIntersectionPoint(bestFitLineL, bestFitLineR);
      cerr << "SP: " << sp << endl;
      LOG(error) << "SP: " << sp << endl;
    } catch(LineIntersectionExceptionT & exc) {
      // TODO: Handle...
      LOG(error) << "Something went wrong... determined lines are parallel..." << endl;
      LOG(error) << " > bestFitLineL: " << endl;
      bestFitLineL.print(cerr);

      LOG(error) << " > bestFitLineR: " << endl;
      bestFitLineR.print(cerr);
    }

    
    // TODO - TMP - wait
    while (! currHfdCurveDisp.is_closed()) {
      currHfdCurveDisp.wait();
    }

    
    statusData.isRunning = false;
    status->store(statusData);  // updates the variable safely

    return;
  }


  void
  manualConsoleFocusCntl(const po::variables_map & inCmdLineMap, IndiCameraT * inCameraDevice, IndiFocuserT * inFocuserDevice, IndiFilterWheelT * inFilterWheelDevice, const FrameT<unsigned int> & inSelectionFrameFF, float inExposureTimeSec, BinningT inBinning, bool inFollowStar)  {

    static ConsoleDisplayT consoleDisplay;
    static std::vector<MenuEntryT *> menuEntries;
    static ConsoleMenuT consoleMenu(& consoleDisplay, menuEntries);
    static MenuSeparatorT sep;


    // "Tasks"
    thread exposureThread;
    atomic<ExposureStatusDataT> exposureStatus;
    atomic<ExposureCntlDataT> exposureCntlData;
    CImg<float> currSubImage;
    
    thread focusFindThread;
    atomic<FocusFindStatusDataT> focusFindStatus;
    atomic<FocusFindCntlDataT> focusFindCntl;
      
    // Windows - TODO: Not always available?
    CImgDisplay currImageDisp, currentHfdDisp, currentFwhmHorzDisp, currentFwhmVertDisp;

    
    // Set the initial image frame
    // NOTE:
    //   FF = Full Frame coordinates
    //   IF = Image Frame coordinates
    PointT<float> currCenterPosFF = frameToCenterPos(inSelectionFrameFF);

    
    // Build the menu
    int expTimeVal = inExposureTimeSec;
    MenuFieldT<int> expTimeMenuField(& expTimeVal, "Exposure time: ", 1 /* steps */, StepModeT::LINEAR,
				     inCameraDevice->getMinExposureTime() /* min */, inCameraDevice->getMaxExposureTime() /*max*/,
				     [](int * inValPtr) {
				       // Seconds to minutes and seconds
				       int sec = (*inValPtr) % 60;
				       int min = (int) ((float) (*inValPtr) / 60.0);
				       stringstream ss;
				       ss << *inValPtr << "s = (" << setfill('0') << setw(2) << min << "m" << setfill('0') << setw(2) << sec << "s)";
				       return ss.str();
				     });
    menuEntries.push_back(& expTimeMenuField);


    BinningT maxBinning = inCameraDevice->getMaxBinning();
    BinningT minBinning = inCameraDevice->getMinBinning();
    int binValXY = inBinning.get<0>(); // NOTE: Assume x=y
    MenuFieldT<int> binningMenuField(& binValXY, "Binning: ", 1 /* steps */, StepModeT::LINEAR,
				     minBinning.get<0>() /* min - NOTE: assuming that x=y */, maxBinning.get<0>() /*max - NOTE: assuming that x=y*/,
				     [](int * inValPtr) { return std::to_string(*inValPtr) +  "x" + std::to_string(*inValPtr); });
    menuEntries.push_back(& binningMenuField);


    int filterSelect = inFilterWheelDevice->getPos();
    MenuFieldT<int> filterSelectMenuField(& filterSelect, "Filter: ", 1 /* steps */, StepModeT::LINEAR,
					  inFilterWheelDevice->getMinPos() /* min */, inFilterWheelDevice->getMaxPos() /*max*/,
					  [](int * inValPtr) { return std::to_string(*inValPtr) +  " (TODO - query...)"; },
					  [&](int * inValPtr) {
					    inFilterWheelDevice->setPos(*inValPtr, 0 /* do not block */);
					  },
					  [&]() {
					    /*ABORT handler*/
					    // No abort supported by INDI Atik Wheel driver.
					  });

    // Check if there is actually a filter ...
    if (inFilterWheelDevice) {
      menuEntries.push_back(& filterSelectMenuField);
    } else {
      menuEntries.push_back(& sep);
    }


    // Camera cooler
    CoolerStateT::TypeE cameraCoolerState = (inCameraDevice->hasCooler() && inCameraDevice->isCoolerEnabled() ? CoolerStateT::ENABLED : CoolerStateT::DISABLED);
    int temperature = (inCameraDevice->hasCooler() ? inCameraDevice->getTemperature() : 0);
    
    MenuSelectT<CoolerStateT> coolerStateMenuSelect(& cameraCoolerState, "Cooler state: ",
						    [](CoolerStateT::TypeE * inValPtr) { return CoolerStateT::asStr(*inValPtr); },
						    [&](CoolerStateT::TypeE * inValPtr) {
						      if (*inValPtr == CoolerStateT::ENABLED) {
							// Enable / disable cooler - TODO: Handle busy state?
							// NOTE: setTemperature directly enables the cooler
							inCameraDevice->setTemperature(temperature, 0); // 0 = do not block
							inCameraDevice->setCoolerEnabled(true);
						      } else {
							inCameraDevice->setCoolerEnabled(false);
						      }
						    },
						    [&]() {
						      /*ABORT handler*/
						      inCameraDevice->setCoolerEnabled(false);
						    });


    MenuFieldT<int> coolerTemperatureMenuField(& temperature, "Temperature: ", 1 /* steps */, StepModeT::LINEAR,
					       inCameraDevice->getMinTemperature() /* min */, inCameraDevice->getMaxTemperature() /*max*/,
					       [](int * inValPtr) {
						 stringstream ssTemp;
						 ssTemp << *inValPtr << " degree celsius";
						 return ssTemp.str();
					       },
					       [&](int * inValPtr) {
						 // UPDATE handler
						 if (cameraCoolerState == CoolerStateT::ENABLED) {
						   inCameraDevice->setTemperature(temperature, 0); // 0 = do not block
						 }
					       },
					       [&]() {
						 // ABORT handler - ESC
						 // ...
					       });

    if (inCameraDevice->hasCooler()) {
      menuEntries.push_back(& coolerTemperatureMenuField);
      menuEntries.push_back(& coolerStateMenuSelect);
    }

  
    ExposureModeT::TypeE expMode = ExposureModeT::SINGLE;
    MenuSelectT<ExposureModeT> expModeMenuSelect(& expMode, "Exposure mode: ",
						 [](ExposureModeT::TypeE * inValPtr) { return ExposureModeT::asStr(*inValPtr); });
    menuEntries.push_back(& expModeMenuSelect);


    StartAbortT::TypeE expStartAbort = StartAbortT::START;
    MenuSelectT<StartAbortT> expStartAbortMenuSelect(& expStartAbort, "Exposure: ",
						     [](StartAbortT::TypeE * inValPtr) { return StartAbortT::asStr(*inValPtr); },
						     [&](StartAbortT::TypeE * inValPtr) {
						       // XXX
						     },
						     [&exposureThread, &exposureStatus]() {
						       /*ABORT handler*/
						       TaskStateT::TypeE taskState = exposureStatus.load().taskState;
						       if (taskState == TaskStateT::RUNNING) {
							 exposureThread.interrupt();
							 // TODO: Set
							 // filterSelect = INDI current value...
						       }
						     });
    menuEntries.push_back(& expStartAbortMenuSelect);

    menuEntries.push_back(& sep);


    int focusStepSize = 100;
    int absFocusPosDest = inFocuserDevice->getAbsPos();
    int focusDelta = 0;
    MenuFieldT<int> absFocusPosDestMenuField(& absFocusPosDest, "Abs focus pos: ", focusStepSize /* steps */, StepModeT::LINEAR,
					     inFocuserDevice->getMinPos() /* min */, inFocuserDevice->getMaxPos() /*max*/,
					     [](int * inValPtr) { return std::to_string(*inValPtr) + " steps"; },
					     [&](int * inValPtr) {
					       focusDelta = fabs(inFocuserDevice->getAbsPos() - *inValPtr);
					       inFocuserDevice->setAbsPos(*inValPtr, 0); // 0 = do not block
					     },
					     [&]() {
					       /*ABORT handler*/
					       inFocuserDevice->abortMotion();
					     });

    MenuFieldT<int> focusStepSizeMenuField(& focusStepSize, "Focus step size: ", 10 /* steps */, StepModeT::FACTOR,
					   inFocuserDevice->getMinPos() /* min */, inFocuserDevice->getMaxPos() / 10 /* max/10 */,
					   [](int * inValPtr) { return std::to_string(*inValPtr) + " steps"; },
					   [&absFocusPosDestMenuField, &focusStepSize](int * inValPtr) {
					     // CHANGE HANDLER
					     absFocusPosDestMenuField.setSteps(focusStepSize);
					   });
    menuEntries.push_back(& focusStepSizeMenuField);
    menuEntries.push_back(& absFocusPosDestMenuField);

    menuEntries.push_back(& sep);

    StartAbortT::TypeE focusFindStartAbort = StartAbortT::START;
    MenuSelectT<StartAbortT> focusFindStartAbortMenuSelect(& focusFindStartAbort, "Focus find: ",
							   [](StartAbortT::TypeE * inValPtr) { return StartAbortT::asStr(*inValPtr); },
							   [&](StartAbortT::TypeE * inValPtr) {
							     bool running = focusFindStatus.load().isRunning;
							   
							     if (*inValPtr == StartAbortT::ABORT) {
							       // Start thread if not yet active.
							       if (! running) {
								 FocusFindCntlDataT focusFindCntlData;
								 focusFindCntlData.binning = BinningT(binValXY, binValXY);
								 focusFindCntlData.exposureTime = expTimeVal;
								 focusFindCntlData.centerPosFF = currCenterPosFF;
								 focusFindCntl.store(focusFindCntlData);
								 
								 focusFindThread = thread(focusFindTask, inCameraDevice, inFocuserDevice, & focusFindCntl, & focusFindStatus);
							       }
							     } else {
							       // Abort - only interrupt if still running
							       if (running) {
								 focusFindThread.interrupt();
							       }
							     }
							   },
							   [&]() {
							     /*ABORT handler*/
							     bool running = focusFindStatus.load().isRunning;
							     if (running) {
							       focusFindThread.interrupt();
							     }
							   });
    menuEntries.push_back(& focusFindStartAbortMenuSelect);
  



    while (! consoleMenu.wantExit()) {

      // Handle menu
      consoleMenu.update();

      ////////////////////////////
      // Handle filter selction //
      ////////////////////////////
      if (inFilterWheelDevice) {
	stringstream ssFilterWheelStatus;
	if (inFilterWheelDevice->isMovementInProgess()) {
	  ssFilterWheelStatus << "moving... is: " << inFilterWheelDevice->getPos();
	} else {
	  ssFilterWheelStatus << "ready (pos=" << inFilterWheelDevice->getPos() << ").";
	}
	consoleDisplay.print(ConsoleMenuT::cLeftMenuBorder, 16, "Filter status: %s", ssFilterWheelStatus.str().c_str());
      }


      ////////////////////////////
      // Handle focus           //
      ////////////////////////////
      if (inFocuserDevice->isMovementInProgess()) {
	int progress = 100 * (1.0 - ((float)((int) inFocuserDevice->getAbsPos() - (int) absFocusPosDest) / focusDelta));
	consoleDisplay.print(ConsoleMenuT::cLeftMenuBorder, 15, "Moving focus... pos: %d/%d - progress: %d%\n",
			     inFocuserDevice->getAbsPos(), absFocusPosDest, progress);
      } else {
	consoleDisplay.print(ConsoleMenuT::cLeftMenuBorder, 15, "Focus ready - pos: %d\n", inFocuserDevice->getAbsPos());
      }

      
      /////////////////////////////////////
      // Handle focus temperature...     //
      /////////////////////////////////////
      // TODO!!!
      // if (inFocuserDevice->supportsTemperature()) {
      //     mv_print(infoColumn, focusStepSizePos + 2, "Focus temp: %d\0xf8 C", inFocuserDevice->getTemperature());
      // }

            
      //////////////////////////////////////
      // Handle camera temperature...     //
      //////////////////////////////////////
      if (inCameraDevice->hasCooler()) {
	// TODO: (inCameraDevice->isTemperatureReached() ? "yes" : "no") caused segfault?!
      	consoleDisplay.print(ConsoleMenuT::cLeftMenuBorder, 18, "Camera cooler state: %s - Dest temp: %d C, Is temp: %f C\n",
      			     CoolerStateT::asStr(cameraCoolerState), temperature, (float) inCameraDevice->getTemperature());
      }
      
      
      ////////////////////////////
      // Handle exposure...     //
      ////////////////////////////
      ExposureStatusDataT exposureStatusData = exposureStatus.load();
      
      switch(exposureStatusData.taskState) {
      case TaskStateT::READY:
	if (expStartAbort == StartAbortT::ABORT) {
	  ExposureCntlDataT cntlData;
	  cntlData.exposureTime = expTimeVal;
	  cntlData.centerPosFF = currCenterPosFF;
	  cntlData.binning = BinningT(binValXY, binValXY);
	  exposureCntlData.store(cntlData);

	  // Thread ONLY gets image - post processing does not take place in thread!
      	  exposureThread = thread(exposeTask, inCameraDevice, & exposureCntlData, & exposureStatus, & currSubImage);
	}
	break;
      
      case TaskStateT::RUNNING:
	// Abort - only interrupt if still running
	if (expStartAbort == StartAbortT::START) {
	  exposureThread.interrupt();
	} else {
	  consoleDisplay.print(ConsoleMenuT::cLeftMenuBorder, 13, "Exposure progress: %d%, seconds left: %d\n", exposureStatusData.progress, exposureStatusData.secondsLeft);
	}
	break;

      case TaskStateT::FINISHED:
      case TaskStateT::ABORTED:
	if (exposureStatusData.taskState == TaskStateT::FINISHED) {

	  int dx = 0, dy = 0;
	  HfdT hfd;
	  FwhmT fwhmHorz, fwhmVert;
	  calcStarValues(currSubImage, & dx, & dy, & hfd, & fwhmHorz, & fwhmVert);
	  
	  // Finally, generate and display the image
	  CImg<unsigned char> rgbImg;
	  genSelectionView(currSubImage, dx, dy, & rgbImg, inCameraDevice->getBitsPerPixel() /*BPP for normalization*/, 3.0 /*zoom*/);
	  currImageDisp.display(rgbImg); // TODO - disable if no Wnds...

	  consoleDisplay.print(ConsoleMenuT::cLeftMenuBorder, 19, "HFD: %f\n", hfd.getValue());
	  currentHfdDisp.display(hfd.genView());
	  	  
	  consoleDisplay.print(ConsoleMenuT::cLeftMenuBorder, 20, "FWHM(horz): %f\n", fwhmHorz.getValue());
	  currentFwhmHorzDisp.display(fwhmHorz.genView());
      	 
	  consoleDisplay.print(ConsoleMenuT::cLeftMenuBorder, 21, "FWHM(vert): %f\n", fwhmVert.getValue());
	  currentFwhmVertDisp.display(fwhmVert.genView());
	  
	  if (true /* TODO: wantRecenter */) {
	    currCenterPosFF.get<0>() += dx;
	    currCenterPosFF.get<1>() += dy;
	  }
	}

	if (expMode == ExposureModeT::SINGLE) {
	  expStartAbort = StartAbortT::START;
	}
	exposureStatusData.taskState = TaskStateT::READY;
	exposureStatus.store(exposureStatusData);
	break;

      default:
	break;
      }

      ////////////////////////////
      // Handle focus finder    //
      ////////////////////////////
      FocusFindStatusDataT focusFindStatusData = focusFindStatus.load();	
      if (focusFindStatusData.isRunning) {
	consoleDisplay.print(ConsoleMenuT::cLeftMenuBorder, 14, "Focus finder progress: %d, hfd: %f\n", focusFindStatusData.progress, focusFindStatusData.hfd);
      } else {
	focusFindStartAbort = StartAbortT::START;
      }

      
      
      // Wait a moment to keep processor down...
      chrono::milliseconds dura(10);
      this_thread::sleep_for(dura); // Sleeps for a bit
    } // end loop
  }
  




  








  
  //void
  //manualConsoleFocusCntl(const po::variables_map & inCmdLineMap, IndiCameraT * inCameraDevice, IndiFocuserT * inFocuserDevice, IndiFilterWheelT * inFilterWheelDevice, const FrameT<unsigned int> & inSelectionFrame, float inExposureTimeSec, BinningT inBinning, bool inFollowStar) // {
  //   const unsigned int statusPosX = 5;
  //   const unsigned int statusPosY = 27;
  //   const size_t infoColumn = 50;

  //   // TODO FIXME: getSelectionFrame() needs to take care of border!! Use borderFactor below...
  //   const float borderFactor = 3.0; // TODO: Pass as argument?!
  //   PointT<float> centerPos = frameToCenterPos(inSelectionFrame);
  //   FrameT<float> imageFrame = centerPosToFrame(centerPos, borderFactor * inSelectionFrame.get<2>(), borderFactor * inSelectionFrame.get<3>());

  //   bool haveFocalDistance = (inCmdLineMap.count("focal_distance") > 0);
  //   bool havePixelSize = (inCmdLineMap.count("pixel_size") > 0);
  //   unsigned int focalDistance = inCmdLineMap["focal_distance"].as<unsigned int>();
  //   const DimensionT<float> & pixelSize = inCmdLineMap["pixel_size"].as<DimensionT<float> >();
    
  //   LOG(trace) << "Initial selectionFrame(by click): " << inSelectionFrame
  // 	       << ", centerPos: " << centerPos << ", initial imageFrame: " << imageFrame << endl;
    
  //   CImgDisplay currentImageDisp, currentFwhmHorzDisp, currentFwhmVertDisp, currentHfdDisp, hfdHistoryDisp, fwhmHorzHistoryDisp, fwhmVertHistoryDisp;
  //   CImg<float> image;
  //   FocusFinderParabelFitImplT findFocusTask(inCameraDevice, inFocuserDevice); // TODO: Use pointer to TaskT<FocusFinderInfoT, MtTaskPolicyT> instead, later... so we can switch strategy at runtime!
    
  //   int key;

  //   int focusPos = inFocuserDevice->getAbsPos();
  //   float hfdValue = -1;
  //   float fwhmHorzValue = -1;
  //   float fwhmVertValue = -1;
  //   float maxPixelValue = -1;
    
  //   initscr();
  //   crmode();
  //   keypad(stdscr, TRUE);
  //   noecho();
  //   clear();
  //   move(7 /*y*/,5 /*x*/);
  //   refresh();

  //   timeout(10);
  //   key = getch();

  //   // Build the menu
  //   vector<MenuEntryT *> menuActions;
    
  //   SetFocusStepSizeT * focusStepSize = new SetFocusStepSizeT();
  //   ControlExposureT * controlExposure = new ControlExposureT(inCameraDevice, inExposureTimeSec);
  //   ControlBinningT * controlBinning = new ControlBinningT(inCameraDevice, inBinning);
  //   AutoFindFocusT * autoFindFocus = new AutoFindFocusT(inCameraDevice, inFocuserDevice, controlExposure, & findFocusTask);
  //   FilterSelectT * filterSelect = 0;
  //   ControlCameraTemperatureT * controlCameraTemperature = 0;
    
  //   addMenuItem(& menuActions, focusStepSize);
  //   addMenuItem(& menuActions, new MoveFocusT(inFocuserDevice, focusStepSize));
  //   addMenuItem(& menuActions, new SeparatorT());
  //   addMenuItem(& menuActions, new SeparatorT());

  //   addMenuItem(& menuActions, controlExposure);
  //   addMenuItem(& menuActions, controlBinning);

  //   if (inCameraDevice->hasCooler()) {
  //     controlCameraTemperature = new ControlCameraTemperatureT(inCameraDevice, inCameraDevice->getTemperature());
  //     addMenuItem(& menuActions, controlCameraTemperature);
  //   }

  //   if (inFilterWheelDevice) {
  //     filterSelect = new FilterSelectT(inFilterWheelDevice, inFilterWheelDevice->getPos());
  //     addMenuItem(& menuActions, new SeparatorT());
  //     addMenuItem(& menuActions, new SeparatorT());
  //     addMenuItem(& menuActions, filterSelect);
  //   }
    
  //   addMenuItem(& menuActions, new SeparatorT());
  //   addMenuItem(& menuActions, new SeparatorT());
  //   addMenuItem(& menuActions, autoFindFocus);

    
  //   int position = 0;
  //   int numEntries = menuActions.size();

  //   LimitedQueueT<float> limitedQueueHfd(50);
  //   LimitedQueueT<float> limitedQueueFwhmHorz(50);
  //   LimitedQueueT<float> limitedQueueFwhmVert(50);
  
  //   //bool exposureInProgress = false;
  //   //bool newImage = false;
    
  //   while(key != 'q') {

  //     // if (inCameraDevice->isExposureInProgress()) {
  //     // 	exposureInProgress = true;
  //     // 	newImage = false;
  //     // } else {
  //     // 	// Get previous image (if any)
  //     // 	inCameraDevice->getImage(& image);

  //     // 	LOG(debug) << "exposureInProgress: " << exposureInProgress << ", newImage: " << newImage << endl;
	
  //     // 	if (exposureInProgress) {
  //     // 	  exposureInProgress = false;
  //     // 	  LOG(debug) << "SET newImage to true!" << endl;
  //     // 	  newImage = true;
  //     // 	}

  //     // 	LOG(trace) << "Received image [subframe: " << imageFrame << "] from camera - size: " << image.width() << " x " << image.height() << endl;
  //     // 	LOG(trace) << "image.width(): " << image.width() << ", image.height(): " << image.height() << endl;
  //     // 	LOG(trace) << "controlBinning->getBinning().get<0>(): " << controlBinning->getBinning().get<0>()
  //     // 		   << ", controlBinning->getBinning().get<1>(): " << controlBinning->getBinning().get<1>() << endl;

  //     // 	if (image.width() == imageFrame.get<2>() && image.height() == imageFrame.get<3>()) {
  //     // 	  // Perform re-centroiding i.e. re-set selectionFrame in order to follow the star
  //     // 	  // ------------------------------------------------------------------------------------------------------------ //
  //     // 	  FrameT<unsigned int> newFrame;

  //     // 	  // IDEA: Pass current image, re-center using given method
  //     // 	  PointT<float> assumedCenter((float) image.width() / 2.0, (float) image.height() / 2.0); // We assume that the star did not move too far from the image center

  //     // 	  LOG(trace) << "image.width(): " << image.width() << ", image.height(): " << image.height()
  //     // 		     << ", --> assumed center: " << assumedCenter << endl;

  //     // 	  // TODO: Do not pass StarFrameSelectorT::StarRecognitionTypeT::PROXIMITY,CentroidT::CentroidTypeT::IWC...
  //     // 	  //       Instead pass what has been selected by parameters..
  //     // 	  bool insideBounds = StarFrameSelectorT::calc(image, 0 /*bitPix - TODO / HACK: not needed */,
  //     // 						       assumedCenter, & newFrame,
  //     // 						       StarFrameSelectorT::StarRecognitionTypeT::PROXIMITY,
  //     // 						       CentroidT::CentroidTypeT::IWC, inSelectionFrame.get<2>() /*frameSize*/);

  //     // 	  AT_ASSERT(StarFrameSelector, insideBounds, "Expected frame to be inside bounds.");
 
  //     // 	  // Calculate new imageFrame
  //     // 	  PointT<float> newLocalCenterPos = frameToCenterPos(newFrame); // In "imageFrame" coordinates
  //     // 	  float deltaX = ((float) imageFrame.get<2>() / 2.0) - newLocalCenterPos.get<0>();
  //     // 	  float deltaY = ((float) imageFrame.get<3>() / 2.0) - newLocalCenterPos.get<1>();
  //     // 	  PointT<float> oldCenterPos = frameToCenterPos(imageFrame); // In "full frame" coordinates
  //     // 	  PointT<float> newCenterPos(oldCenterPos.get<0>() - deltaX, oldCenterPos.get<1>() - deltaY); // In "full frame" coordinates
  //     // 	  FrameT<unsigned int> newImageFrame = centerPosToFrame(newCenterPos, imageFrame.get<2>(), imageFrame.get<3>());

  //     // 	  LOG(debug) << "Old imageFrame: " << imageFrame << ", oldCenterPos: " << oldCenterPos << endl;
  //     // 	  LOG(trace) << "frameSize used: " << inSelectionFrame.get<2>() << ", assumedCenter: " << assumedCenter
  //     // 		     << ", newFrame: " << newFrame << " --> newLocalCenterPos: " << newLocalCenterPos
  //     // 		     << ", --> deltaX: " << deltaX << ", deltaY: " << deltaY << endl;
  //     // 	  LOG(debug) << "New imageFrame: " << newImageFrame << ", newCenterPos: " << newCenterPos << endl;

  //     // 	  // TODO / FIXME: Check if it hits boundaries!!??
	  
  //     // 	  // Get sub-frame for analysis
  //     // 	  // ------------------------------------------------------------------------------------------------------------ //
  //     // 	  // Convert to imageFrame coordinates
  //     // 	  FrameT<float> selectionFrame(newFrame);

  //     // 	  CImg<float> subImg = image.get_crop(selectionFrame.get<0>() /*x0*/,
  //     // 	   				      selectionFrame.get<1>() /*y0*/,
  //     // 	   				      selectionFrame.get<0>() + selectionFrame.get<2>() - 1/*x1=x0+w-1*/,
  //     // 	   				      selectionFrame.get<1>() + selectionFrame.get<3>() - 1/*y1=y0+h-1*/);

  //     // 	  // Calculate star data
  //     // 	  // ------------------------------------------------------------------------------------------------------------ //
  //     // 	  // TODO: try catch?
  //     // 	  try {
  //     // 	    // TODO: HFD value INCREASES if coming to focus using the simulator... !!! Maybe a simulator problem?! --> need real test!!!
  //     // 	    HfdT hfd(subImg); // NOTE: HfdT takes image center as centroid, it does not matter if image is bigger
  //     // 	    hfdValue = hfd.getValue();
  //     // 	    currentHfdDisp.display(hfd.genView());
  //     // 	  } catch(std::exception & exc) {
  //     // 	    LOG(warning) << "HFD calculation failed!"  << endl;
  //     // 	  }
	  
  //     // 	  // Subtract median image
  //     // 	  double med = subImg.median();
  //     // 	  CImg<float> imageSubMed(subImg.width(), subImg.height());
  //     // 	  cimg_forXY(subImg, x, y) {
  //     // 	    imageSubMed(x, y) = (subImg(x, y) > med ? subImg(x, y) - med : 0);
  //     // 	  }
	  
  //     // 	  try {
  //     // 	    FwhmT fwhmHorz(extractLine<DirectionT::HORZ>(imageSubMed));
  //     // 	    fwhmHorzValue = fwhmHorz.getValue();
  //     // 	    currentFwhmHorzDisp.display(fwhmHorz.genView());
  //     // 	  } catch(std::exception & exc) {
  //     // 	    LOG(warning) << "FWHM(horz) calculation failed!"  << endl;
  //     // 	  }
	  
  //     // 	  try {
  //     // 	    FwhmT fwhmVert(extractLine<DirectionT::VERT>(imageSubMed));
  //     // 	    fwhmVertValue = fwhmVert.getValue();
  //     // 	    currentFwhmVertDisp.display(fwhmVert.genView());
  //     // 	  } catch(std::exception & exc) {
  //     // 	    LOG(warning) << "FWHM(horz) calculation failed!"  << endl;
  //     // 	  }
	  
  //     // 	  maxPixelValue = subImg.max();

	  
  //     // 	  // DEBUG START
  //     // 	  // CImg<unsigned char> tmpNormalizedImage(normalize(subImg, 65535.0, 5.0));
  //     // 	  // CImgDisplay thDsp(tmpNormalizedImage, "SUB-IMG...");
  //     // 	  // while (! thDsp.is_closed()) {
  //     // 	  //   thDsp.wait();
  //     // 	  // }
  //     // 	  // CImg<unsigned char> hfdView = hfd.genView();
  //     // 	  // CImgDisplay thDsp2(hfdView, "HFD-VIEW...");
  //     // 	  // while (! thDsp2.is_closed()) {
  //     // 	  //   thDsp2.wait();
  //     // 	  // }
  //     // 	  // DEBUG END
	  
	  
  //     // 	  if (inFollowStar && controlExposure->getExposureMode() != ControlExposureT::ExposureModeT::SINGLE) {
  //     // 	    // If star following is enabled, the camera sub-frame is re-centered after each image
  //     // 	    imageFrame = newImageFrame;
  //     // 	  }
  //     // 	} else {
  //     // 	  LOG(debug) << "Received something unexpected! Expected image size: " << imageFrame.get<2>() << " x " << imageFrame.get<3>()
  //     // 		     << " but received " << image.width() << " x " << image.height() << " -> ignoring." << endl;
  //     // 	}
	
  //     // 	// TODO: Should this be moved to the menu items?!
  //     // 	LOG(trace) << "setBinning(" << controlBinning->getBinning() << ")..." << endl;
  //     // 	inCameraDevice->setBinning(controlBinning->getBinning());

  //     // 	//TODO: setBinning after first selection of imageFrame causes CentroidException (Frame hit simage bounds...) -> need to recalc imageFrame(?) - based on old binning and given new binning!
  //     // 	LOG(trace) << "setBinnedFrame(" << imageFrame << ", " << controlBinning->getBinning() << ")..." << endl;
  //     // 	inCameraDevice->setBinnedFrame(imageFrame, controlBinning->getBinning());
	
  //     // 	LOG(trace) << "setFrameType(" << FrameTypeT::asStr(FrameTypeT::LIGHT) << ")..." << endl;
  //     // 	inCameraDevice->setFrameType(FrameTypeT::LIGHT);
  //     // 	LOG(trace) << "setCompressed(false)..." << endl;
  //     // 	inCameraDevice->setCompressed(false); // No compression when moving image to CImg

  //     // 	// TODO: getExposureDone?? TODO: or restart exposure if auto finder is enabled
  //     // 	if (! controlExposure->getExposureDone() || controlExposure->getExposureMode() == ControlExposureT::ExposureModeT::LOOP) {
  //     // 	  LOG(trace) << "startExposure(" << inExposureTimeSec << "s)..." << endl;
  //     // 	  inCameraDevice->startExposure(controlExposure->getExposureTime()); // Non-blocking call...!!
  //     // 	  controlExposure->setExposureDone(true);
  //     // 	}
  //     // } // end if - ! exposure in progress

      
  //     // if (newImage) {
  //     // 	newImage = false;
	
  //     // 	if (hfdValue > 0) {
  //     // 	  stringstream hfdArcSecSs;
  //     // 	  if (haveFocalDistance && havePixelSize) {
  //     // 	    hfdArcSecSs << " = " << FwhmT::pxToArcsec(hfdValue, focalDistance, pixelSize, controlBinning->getBinning()) << "\"";
  //     // 	  }
  //     // 	  mv_print(infoColumn, 15, "HFD: %fpx%s", hfdValue, hfdArcSecSs.str().c_str());
	
  //     // 	  // Add values to queue
  //     // 	  limitedQueueHfd.push(hfdValue);
  //     // 	} else {
  //     // 	  mv_print(infoColumn, 15, "HFD: n.a.");
  //     // 	}
  //     // 	hfdHistoryDisp.display(limitedQueueHfd.genView(400, 100));
	
  //     // 	if (fwhmHorzValue > 0) {
  //     // 	  stringstream fwhmHorzArcSecSs;

  //     // 	  if (haveFocalDistance && havePixelSize) {
  //     // 	    fwhmHorzArcSecSs << " = " << FwhmT::pxToArcsec(fwhmHorzValue, focalDistance, pixelSize, controlBinning->getBinning()) << "\"";
  //     // 	  }
  //     // 	  // Add values to queue
  //     // 	  limitedQueueFwhmHorz.push(fwhmHorzValue);

  //     // 	  mv_print(infoColumn, 16, "FWHM(horz): %fpx%s", fwhmHorzValue, fwhmHorzArcSecSs.str().c_str());
  //     // 	} else {
  //     // 	  mv_print(infoColumn, 16, "FWHM(horz): n.a.");
  //     // 	}
  //     // 	fwhmHorzHistoryDisp.display(limitedQueueFwhmHorz.genView(400, 100));

	
  //     // 	if (fwhmVertValue > 0) {
  //     // 	  stringstream fwhmVertArcSecSs;

  //     // 	  if (haveFocalDistance && havePixelSize) {
  //     // 	    fwhmVertArcSecSs << " = " << FwhmT::pxToArcsec(fwhmVertValue, focalDistance, pixelSize, controlBinning->getBinning()) << "\"";
  //     // 	  }
  //     // 	  // Add values to queue
  //     // 	  limitedQueueFwhmVert.push(fwhmVertValue);

  //     // 	  mv_print(infoColumn, 17, "FWHM(vert): %fpx%s", fwhmVertValue, fwhmVertArcSecSs.str().c_str());
  //     // 	} else {
  //     // 	  mv_print(infoColumn, 17, "FWHM(vert): n.a.");
  //     // 	}
  //     // 	fwhmVertHistoryDisp.display(limitedQueueFwhmVert.genView(400, 100));

  //     // 	if (maxPixelValue > 0) {
  //     // 	  mv_print(infoColumn, 18, "MAX: %f", maxPixelValue);
  //     // 	} else {
  //     // 	  mv_print(infoColumn, 18, "MAX: n.a.");
  //     // 	}
  //     // } // end if - ! inCameraDevice->isExposureInProgress() && newImage

      
  //     mv_print(5, 25, "Focus finder. Press 'q' to quit");

  //     switch(key) {
  //     case KEY_UP:
  // 	while (position > 0) {
  // 	  position--;
  // 	  bool isSeparator =  ! strcmp(menuActions[position]->getName(), "");
  // 	  if (! isSeparator) { break; }
  // 	}
  // 	break;
	
  //     case KEY_DOWN:
  // 	while (position < numEntries - 1) {
  // 	  position++;
  // 	  bool isSeparator =  ! strcmp(menuActions[position]->getName(), "");
  // 	  if (! isSeparator) { break; }
  // 	}
  // 	break;
	
  //     case 9: /* TAB */
  // 	bool isSeparator;
  // 	do {
  // 	  position++;
  // 	  position = position % numEntries;
	  
  // 	  isSeparator =  ! strcmp(menuActions[position]->getName(), "");
  // 	} while (isSeparator);
  // 	break;

  //     default: {
  // 	mv_print(10, 21, "%s", menuActions[position]->getHelpText());
  // 	menuActions[position]->execute(key);
  // 	break;
  //     }
  //     } /* switch */

  //     const int cTopMenuBorder = 1;
  //     const int cLeftMenuBorder = 10;
  //     const int cLeftMenuWidth = 20;

  //     // Highlight selected menu entry
  //     for (vector<MenuEntryT *>::iterator it = menuActions.begin(); it != menuActions.end(); ++it) {
  // 	const MenuEntryT * menuEntry = *it;
  // 	int pos = (*it)->getMenuPos();

  // 	if (pos == position) { attron(A_STANDOUT); }
  // 	mv_print(cLeftMenuBorder, cTopMenuBorder + pos, "%s", menuEntry->getName());
  // 	if (pos == position) { attroff(A_STANDOUT); }

  // 	mv_print(cLeftMenuBorder + cLeftMenuWidth, cTopMenuBorder + pos, "%s", menuEntry->getValueAsStr().c_str());
  //     }



  //     // Update focus data
  //     stringstream ssCameraStatus;
  //     if (inCameraDevice->isExposureInProgress()) {
  // 	ssCameraStatus << "exposing..." << inCameraDevice->getExposureTime() << "s";
  //     } else {
  // 	ssCameraStatus << "ready";
  //     }

  //     mv_print(infoColumn, cTopMenuBorder + controlExposure->getMenuPos(), "Camera status: %s", ssCameraStatus.str().c_str());

  //     if (inCameraDevice->hasCooler()) {
  // 	size_t coolerPos = cTopMenuBorder + controlCameraTemperature->getMenuPos();
  // 	mv_print(infoColumn, coolerPos, "Cooler status: ");
  // 	mv_print(infoColumn, coolerPos + 1, "Temperature: ");
  //     }

  //     if (inFilterWheelDevice) {
  // 	stringstream ssFilterWheelStatus;
  // 	if (inFilterWheelDevice->isMovementInProgess()) {
  // 	  ssFilterWheelStatus << "moving... is: " << inFilterWheelDevice->getPos();
  // 	} else {
  // 	  ssFilterWheelStatus << "ready (pos=" << inFilterWheelDevice->getPos() << ").";
  // 	}
  // 	mv_print(infoColumn, cTopMenuBorder + filterSelect->getMenuPos(), "Filter status: %s", ssFilterWheelStatus.str().c_str());
  //     }

  //     string focuserStatus = (inFocuserDevice->isMovementInProgess() ? "busy" : "ready"); 
  //     size_t focusStepSizePos = cTopMenuBorder + focusStepSize->getMenuPos();

  //     mv_print(infoColumn, focusStepSizePos, "Focus status: %s", focuserStatus.c_str());
  //     mv_print(infoColumn, focusStepSizePos + 1, "Focus pos (is): %d", inFocuserDevice->getAbsPos());
  //     if (inFocuserDevice->supportsTemperature()) {
  // 	mv_print(infoColumn, focusStepSizePos + 2, "Focus temp: %d\0xf8 C", inFocuserDevice->getTemperature());
  //     }

  //     // Update auto finder status
  //     size_t autoFindFocusPos = cTopMenuBorder + autoFindFocus->getMenuPos();
  //     mv_print(infoColumn, autoFindFocusPos, "Status: %s", FocusFinderInfoT::StateT::asStr(findFocusTask.getStatus().mState));
  //     mv_print(infoColumn, autoFindFocusPos + 1, "Progress: %d", findFocusTask.getStatus().mProgress);

  //     refresh();
  //     timeout(10);
  //     key = getch();
  //   } /* while */

  //   // Cleanup menu
  //   for (vector<MenuEntryT *>::iterator it = menuActions.begin(); it != menuActions.end(); ++it) {
  //     delete *it;
  //   }
  //   endwin();    
  // }
}; // end AT
