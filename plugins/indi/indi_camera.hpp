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

#ifndef _INDI_CAMERA_HPP_
#define _INDI_CAMERA_HPP_ _INDI_CAMERA_HPP_

// See: http://www.boost.org/doc/libs/1_36_0/libs/tuple/doc/tuple_users_guide.html
#include <boost/tuple/tuple.hpp>

#include "indi_device.hpp"
#include "at_exception.hpp"

using namespace boost;

DEF_Exception(IndiCamera);
DEF_Exception(IndiCameraNotConnected);
DEF_Exception(IndiCameraExposureInProgress);
DEF_Exception(IndiCameraNoImageAvailable);
DEF_Exception(IndiCameraTmpFile);

/**
 * Camera traits.
 *
 */

// See: http://en.wikipedia.org/wiki/Curiously_recurring_template_pattern
// See: http://en.wikipedia.org/wiki/Template_metaprogramming#Static_polymorphism
struct CameraTraitsT {
  static const DeviceTypeT::TypeE sDeviceType;


  struct VecPropsT {
    enum TypeE {
      CONNECTION,
      DRIVER_INFO,
      CONFIG_PROCESS,

      CCD_INFO,
      CCD_FRAME,
      CCD_BINNING,
      CCD_EXPOSURE,
      CCD_ABORT_EXPOSURE,
      CCD_TEMPERATURE,
      CCD_COMPRESSION,
      CCD1,
      CCD_FRAME_TYPE,
      CCD_RAPID_GUIDE,

      ACTIVE_DEVICES,

      COOLER_CONNECTION,

      SHUTTER_CONNECTION,

      _Count
    };

    // TODO: Maybe specifying the type to string translation is enough!
    static const char * asStr(const TypeE & inType) {
      switch (inType) {
      case CONNECTION: return "CONNECTION";
      case DRIVER_INFO: return "DRIVER_INFO";
      case CONFIG_PROCESS: return "CONFIG_PROCESS";

      case CCD_INFO: return "CCD_INFO";
      case CCD_FRAME: return "CCD_FRAME";
      case CCD_BINNING: return "CCD_BINNING";
      case CCD_EXPOSURE: return "CCD_EXPOSURE";
      case CCD_ABORT_EXPOSURE: return "CCD_ABORT_EXPOSURE";
      case CCD_TEMPERATURE: return "CCD_TEMPERATURE";
      case CCD_COMPRESSION: return "CCD_COMPRESSION";
      case CCD1: return "CCD1";
      case CCD_FRAME_TYPE: return "CCD_FRAME_TYPE";
      case CCD_RAPID_GUIDE: return "CCD_RAPID_GUIDE";

      case ACTIVE_DEVICES: return "ACTIVE_DEVICES";

      case COOLER_CONNECTION: return "COOLER_CONNECTION";

      case SHUTTER_CONNECTION: return "SHUTTER_CONNECTION";

      default: return "<?>";
      }
    }

    MAC_AS_TYPE(Type, E, _Count);

    static bool isOptional(const TypeE & inType) {
      switch (inType) {
      case COOLER_CONNECTION:
      case SHUTTER_CONNECTION:
      case CCD_TEMPERATURE:
    	return true;
      default:
    	return false;
      }
    }
  };


  struct PropsT {
    enum TypeE {
      CONNECT,
      DISCONNECT,
      PORT, // TODO: Does the Atik have this porp?!?!?!?!

      DRIVER_NAME,
      DRIVER_EXEC,
      DRIVER_VERSION,

      CCD_MAX_X,
      CCD_MAX_Y,
      CCD_PIXEL_SIZE,
      CCD_PIXEL_SIZE_X,
      CCD_PIXEL_SIZE_Y,
      CCD_BITSPERPIXEL,

      X,
      Y,
      WIDTH,
      HEIGHT,

      HOR_BIN,
      VER_BIN,

      CCD_EXPOSURE_VALUE,

      ABORT,

      CCD_TEMPERATURE_VALUE,

      CCD_COMPRESS,
      CCD_RAW,

      CCD1,

      FRAME_LIGHT,
      FRAME_BIAS,
      FRAME_DARK,
      FRAME_FLAT,

      ENABLE,
      DISABLE,

      ACTIVE_TELESCOPE,
      ACTIVE_FOCUSER,

      DISCONNECT_COOLER,
      CONNECT_COOLER,

      SHUTTER_ON,
      SHUTTER_OFF,

      _Count
    };

    
    static const char * asStr(const TypeE & inType) {
      switch (inType) {
      case CONNECT: return "CONNECT";
      case DISCONNECT: return "DISCONNECT";
      case PORT: return "PORT";

      case DRIVER_NAME: return "DRIVER_NAME";
      case DRIVER_EXEC: return "DRIVER_EXEC";
      case DRIVER_VERSION: return "DRIVER_VERSION";

      case CCD_MAX_X: return "CCD_MAX_X";
      case CCD_MAX_Y: return "CCD_MAX_Y";
      case CCD_PIXEL_SIZE: return "CCD_PIXEL_SIZE";
      case CCD_PIXEL_SIZE_X: return "CCD_PIXEL_SIZE_X";
      case CCD_PIXEL_SIZE_Y: return "CCD_PIXEL_SIZE_Y";
      case CCD_BITSPERPIXEL: return "CCD_BITSPERPIXEL";

      case X: return "X";
      case Y: return "Y";
      case WIDTH: return "WIDTH";
      case HEIGHT: return "HEIGHT";

      case HOR_BIN: return "HOR_BIN";
      case VER_BIN: return "VER_BIN";

      case CCD_EXPOSURE_VALUE: return "CCD_EXPOSURE_VALUE";

      case ABORT: return "ABORT";

      case CCD_TEMPERATURE_VALUE: return "CCD_TEMPERATURE_VALUE";

      case CCD_COMPRESS: return "CCD_COMPRESS";
      case CCD_RAW: return "CCD_RAW";

      case CCD1: return "CCD1";

      case FRAME_LIGHT: return "FRAME_LIGHT";
      case FRAME_BIAS: return "FRAME_BIAS";
      case FRAME_DARK: return "FRAME_DARK";
      case FRAME_FLAT: return "FRAME_FLAT";

      case ENABLE: return "ENABLE";
      case DISABLE: return "DISABLE";

      case ACTIVE_TELESCOPE: return "ACTIVE_TELESCOPE";
      case ACTIVE_FOCUSER: return "ACTIVE_FOCUSER";

      case DISCONNECT_COOLER: return "DISCONNECT_COOLER";
      case CONNECT_COOLER: return "CONNECT_COOLER";

      case SHUTTER_ON: return "SHUTTER_ON";
      case SHUTTER_OFF: return "SHUTTER_OFF";

      default: return "<?>";
      }
    }
    
    MAC_AS_TYPE(Type, E, _Count);

    static bool isOptional(const TypeE & inType) {
      switch (inType) {
      case PORT:
      case CCD_TEMPERATURE_VALUE:
      case DISCONNECT_COOLER:
      case CONNECT_COOLER:
      case SHUTTER_ON:
      case SHUTTER_OFF:
	return true;
      default:
    	return false;
      }
    }

  };
};




/**
 * Frame type.
 *
 */
struct FrameTypeT {
  enum TypeE {
    LIGHT,
    BIAS,
    DARK,
    FLAT,
    _Count
  };

  static const char * asStr(const TypeE & inType) {
    switch (inType) {
    case LIGHT: return "LIGHT";
    case BIAS: return "BIAS";
    case DARK: return "DARK";
    case FLAT: return "FLAT";
    default: return "<?>";
    }
  }

  MAC_AS_TYPE(Type, E, _Count);  
};


/**
 * Camera device.
 *
 * TODO: Need a way to check if an INDI device is really a camera!!! How? INDI API?
 *       Or just require certain properties to be there (or alt least mapped - btw..
 *       mapping should be done by DeviceT class already...)? For camera at least e.g.
 *       CCD1 or CCD2 have to be there...
 */
class IndiCameraT : public IndiDeviceT {
private:
  // We do not want device copies
  IndiCameraT(const IndiCameraT &);
  IndiCameraT & operator=(const IndiCameraT &); 

  // TODO: Handle this way? Or introduce a "condition" which can be passed in addition to the bind information?
  void internalNewBLOBHandler(IBLOB * inVec) {
    LOG(debug) << "internalNewBLOBHandler called..." << endl;

    // TODO: use enum types to compare?!
    if (! strcmp(inVec->name, "CCD1") || ! strcmp(inVec->name, "CCD2")) {
      LOG(debug) << "  --> ok, received a CCD1/CCD2 BLOB. Calling handler..." << endl;
      mNewImageReceivedListeners(inVec);
    } else {
      LOG(debug) << "  --> ok, received any other BLOB... but probably not an image... NOT calling handler..." << endl;
    }
  }

public:
  typedef CameraTraitsT::VecPropsT VecPropsT;
  typedef CameraTraitsT::PropsT PropsT;

  // TODO: Should not be allowed... otherwise... which device should be assigned?!
  IndiCameraT(IndiClientT * inIndiClient, BaseDevice * inBaseDevice) : IndiDeviceT(inIndiClient, inBaseDevice, IndiCameraT::getType()) {
    this->registerNewBLOBListener(boost::bind(& IndiCameraT::internalNewBLOBHandler, this, _1));    
  }
  static DeviceTypeT::TypeE getType() { return CameraTraitsT::sDeviceType; }

  // virtual void connect(bool inVerifyDeviceType = true) {
  //   IndiDeviceT::connect(); // Call base connect...

  //   if (inVerifyDeviceType) {
  //     DeviceTypeT::TypeE propDeviceType = getDeviceTypeFromProps();
      
  //     if (getType() != propDeviceType) {
  // 	stringstream exSs;
  // 	exSs << "Device type determined by props=" << DeviceTypeT::asStr(propDeviceType)
  // 	     << " does not match this device: " << DeviceTypeT::asStr(getType());
  // 	LOG(error) << exSs.str() << endl;
  // 	throw IndiDeviceTypeMismatchExceptionT(exSs.str().c_str());
  //     }
  //   }
  // }


  /**
   * Get camera chip info
   */
  inline DimensionT<unsigned int> getMaxResolution() const {
    INumberVectorProperty * vec = this->getNumberVec<CameraTraitsT>(VecPropsT::CCD_INFO);
    return boost::make_tuple(this->getNumberVal<CameraTraitsT>(vec, PropsT::CCD_MAX_X),
			     this->getNumberVal<CameraTraitsT>(vec, PropsT::CCD_MAX_Y));
  }

  inline DimensionT<float> getPixelSize() const {
    INumberVectorProperty * vec = this->getNumberVec<CameraTraitsT>(VecPropsT::CCD_INFO);
    return boost::make_tuple(this->getNumberVal<CameraTraitsT>(vec, PropsT::CCD_PIXEL_SIZE_X),
			     this->getNumberVal<CameraTraitsT>(vec, PropsT::CCD_PIXEL_SIZE_Y));
  }

  inline unsigned int getBitsPerPixel() const {
    return this->getNumberVal<CameraTraitsT>(VecPropsT::CCD_INFO, PropsT::CCD_BITSPERPIXEL);
  }

  /**
   * Frame
   */
  inline FrameT<int> getFrame() const {
    INumberVectorProperty * vec = this->getNumberVec<CameraTraitsT>(VecPropsT::CCD_FRAME);
    return boost::make_tuple(this->getNumberVal<CameraTraitsT>(vec, PropsT::X), 
			     this->getNumberVal<CameraTraitsT>(vec, PropsT::Y),
			     this->getNumberVal<CameraTraitsT>(vec, PropsT::WIDTH),
			     this->getNumberVal<CameraTraitsT>(vec, PropsT::HEIGHT));
  }

  inline void setFrame(int inFrameX, int inFrameY, int inFrameW, int inFrameH, int inTimeout = sDefaultTimeoutMs) {
    INumberVectorProperty * nVec = this->getNumberVec<CameraTraitsT>(VecPropsT::CCD_FRAME);
    this->updateNumberVal<CameraTraitsT>(nVec, PropsT::X, inFrameX);
    this->updateNumberVal<CameraTraitsT>(nVec, PropsT::Y, inFrameY);
    this->updateNumberVal<CameraTraitsT>(nVec, PropsT::WIDTH, inFrameW);
    this->updateNumberVal<CameraTraitsT>(nVec, PropsT::HEIGHT, inFrameH);
    this->sendNumberVec(nVec, inTimeout);
  }

  inline void setFrame(const FrameT<int> & inFrame, int inTimeout = sDefaultTimeoutMs) {
    this->setFrame(inFrame.get<0>() /*x*/, inFrame.get<1>() /*y*/, inFrame.get<2>() /*w*/, inFrame.get<3>() /*h*/, inTimeout);
  }

  inline void setBinnedFrame(const FrameT<int> & inUnbinnedFrame, const BinningT & inBinning, int inTimeout = sDefaultTimeoutMs) {
    FrameT<int> binnedFrame;
    binnedFrame.get<0>() /*x*/ = inUnbinnedFrame.get<0>() / inBinning.get<0>() /*bin_x*/;
    binnedFrame.get<1>() /*y*/ = inUnbinnedFrame.get<1>() / inBinning.get<1>() /*bin_y*/;
    binnedFrame.get<2>() /*w*/ = inUnbinnedFrame.get<2>() / inBinning.get<0>() /*bin_x*/;
    binnedFrame.get<3>() /*h*/ = inUnbinnedFrame.get<3>() / inBinning.get<1>() /*bin_y*/;
    
    this->setFrame(binnedFrame, inTimeout);
  }
  

  /**
   * Binning
   */
  inline BinningT getBinning() const {
    return boost::make_tuple(this->getNumberVal<CameraTraitsT>(VecPropsT::CCD_BINNING, PropsT::HOR_BIN), 
			     this->getNumberVal<CameraTraitsT>(VecPropsT::CCD_BINNING, PropsT::VER_BIN));
  }
  inline BinningT getMaxBinning() const {
    const INumber & nHor = this->getNumber<CameraTraitsT>(VecPropsT::CCD_BINNING, PropsT::HOR_BIN);
    const INumber & nVer = this->getNumber<CameraTraitsT>(VecPropsT::CCD_BINNING, PropsT::VER_BIN);
    return BinningT(nHor.max, nVer.max);
  }
  inline BinningT getMinBinning() const {
    const INumber & nHor = this->getNumber<CameraTraitsT>(VecPropsT::CCD_BINNING, PropsT::HOR_BIN);
    const INumber & nVer = this->getNumber<CameraTraitsT>(VecPropsT::CCD_BINNING, PropsT::VER_BIN);
    return BinningT(nHor.min, nVer.min);
  }
  
  inline void setBinning(unsigned int inHorzBinning, unsigned int inVertBinning, int inTimeout = sDefaultTimeoutMs) {
    INumberVectorProperty * nVec = this->getNumberVec<CameraTraitsT>(VecPropsT::CCD_BINNING);
    this->updateNumberVal<CameraTraitsT>(nVec, PropsT::HOR_BIN, inHorzBinning);
    this->updateNumberVal<CameraTraitsT>(nVec, PropsT::VER_BIN, inVertBinning);
    this->sendNumberVec(nVec, inTimeout);
  }
  inline void setBinning(const BinningT & inBinning, int inTimeout = sDefaultTimeoutMs) {
    this->setBinning(inBinning.get<0>(), inBinning.get<1>(), inTimeout);
  }


  /**
   * Compression
   */
  inline bool isCompressed() const { return this->getSwitchVal<CameraTraitsT>(VecPropsT::CCD_COMPRESSION, PropsT::CCD_COMPRESS);  }
  inline bool isRaw() const { return (! this->isCompressed()); }

  inline void setCompressed(bool inCompressed, int inTimeout = sDefaultTimeoutMs) {
    this->sendSwitchVal<CameraTraitsT>(VecPropsT::CCD_COMPRESSION, PropsT::CCD_COMPRESS, inCompressed, inTimeout);
  }


  /**
   * Frame type
   */
  FrameTypeT::TypeE getFrameType() const {
    ISwitchVectorProperty * sVec =  this->getSwitchVec<CameraTraitsT>(VecPropsT::CCD_FRAME_TYPE);
    if (this->getSwitchVal<CameraTraitsT>(sVec, PropsT::FRAME_LIGHT)) return FrameTypeT::LIGHT;
    else if (this->getSwitchVal<CameraTraitsT>(sVec, PropsT::FRAME_BIAS)) return FrameTypeT::BIAS;
    else if (this->getSwitchVal<CameraTraitsT>(sVec, PropsT::FRAME_DARK)) return FrameTypeT::DARK;
    else if (this->getSwitchVal<CameraTraitsT>(sVec, PropsT::FRAME_FLAT)) return FrameTypeT::FLAT;
    else return FrameTypeT::_Count;
  }

  inline void setFrameType(const FrameTypeT::TypeE & inFrameType, int inTimeout = sDefaultTimeoutMs) {
    string propName = string("FRAME_") + string(FrameTypeT::asStr(inFrameType));
    PropsT::TypeE frameTypeProp = PropsT::asType(propName.c_str());

    if (PropsT::_Count == frameTypeProp)
      throw IndiCameraExceptionT("Invalid frame type property (Maybe the property name has changed ?).");
    
    this->sendSwitchVal<CameraTraitsT>(VecPropsT::CCD_FRAME_TYPE, frameTypeProp, ISS_ON, inTimeout);
  }


  /**
   * Exposure
   *
   */

  /**
   * Returns the number of exposure seconds left.
   * Returns 0 if no exposure is in progress.
   */
  double getExposureTime() const { return this->getNumberVal<CameraTraitsT>(VecPropsT::CCD_EXPOSURE, PropsT::CCD_EXPOSURE_VALUE); }

  // Low level function
  void setExposureTime(double inExposureTimeSec, int inTimeout = sDefaultTimeoutMs) {
    // TODO: Check 1000 * inExposureTimeSec < inTimeout
    LOG(trace) << "setExposureTime - inExposureTimeSec: " << inExposureTimeSec << "[s], inTimeout: " << inTimeout << "[ms]..." << endl;
    this->sendNumberVal<CameraTraitsT>(VecPropsT::CCD_EXPOSURE, PropsT::CCD_EXPOSURE_VALUE, inExposureTimeSec, inTimeout);
    LOG(trace) << "setExposureTime - DONE." << endl;
  }

  bool isExposureInProgress() const {
    INumberVectorProperty * nVec = this->getNumberVec<CameraTraitsT>(VecPropsT::CCD_EXPOSURE);
    return (nVec->s == IPS_BUSY);
  }

  inline double getMaxExposureTime() const {
    const INumber & nTemp = this->getNumber<CameraTraitsT>(VecPropsT::CCD_EXPOSURE, PropsT::CCD_EXPOSURE_VALUE);
    return nTemp.max;
  }
  inline double getMinExposureTime() const {
    const INumber & nTemp = this->getNumber<CameraTraitsT>(VecPropsT::CCD_EXPOSURE, PropsT::CCD_EXPOSURE_VALUE);
    return nTemp.min;
  }


  
  /**
   * Blocks until eposure finished or throws TimeoutExceptionT if exposure
   * camera does not finish exposure as expected. Throws IndiCameraExposureInProgressExceptionT
   * if camera is currently exposing.
   */
  void expose(double inExposureTimeSec) {
    if (this->isExposureInProgress())
      throw IndiCameraExposureInProgressExceptionT();

    this->setExposureTime(inExposureTimeSec, 1000 * inExposureTimeSec + 20000 /* TODO: 20sec for transfer?!?!?! */);
  }

  /**
   * Just starts exposure - non-blocking. Throws IndiCameraExposureInProgressExceptionT
   * if camera is currently exposing.
   */
  // TODO: There are additional exposure settings for GUIDER! How to handle?
  void startExposure(double inExposureTimeSec) {
    LOG(debug) << "startExposure - Starting exposure..." << inExposureTimeSec << " sec." << endl;
    if (this->isExposureInProgress())
      throw IndiCameraExposureInProgressExceptionT();
    
    this->setExposureTime(inExposureTimeSec, 0 /* 0ms - Don't wait at all */);
    LOG(debug) << "startExposure - Exposure finished." << endl;
  }
  void abortExposure(int inTimeoutMs = sDefaultTimeoutMs) {
    ISwitchVectorProperty * vec = this->getSwitchVec<CameraTraitsT>(VecPropsT::CCD_ABORT_EXPOSURE);
    
    if (this->isExposureInProgress()) {
      LOG(debug) << "abortExposure - sending ABORT request..." << endl;
      this->sendSwitchVal<CameraTraitsT>(VecPropsT::CCD_ABORT_EXPOSURE, PropsT::ABORT, true, inTimeoutMs);
      LOG(debug) << "abortExposure - ABORT request sent..." << endl;
      
      // Since state of ABORT seems to leave busy state before exposure has been actually stopped, we wait here until it finally happens.
      ostringstream oss;
      oss << "abortExposure - Hit timeout (" << inTimeoutMs << "ms) setting while setting value.";
      WAIT_MAX_FOR(! this->isExposureInProgress(), inTimeoutMs, oss.str());

      LOG(debug) << "abortExposure - ABORTED." << endl;
    }
  }

  /**
   * Cooler
   */
  inline bool hasCooler() const { return this->hasVecProp<CameraTraitsT>(VecPropsT::COOLER_CONNECTION); }
  inline bool isCoolerEnabled() const { return this->getSwitchVal<CameraTraitsT>(VecPropsT::COOLER_CONNECTION, PropsT::CONNECT_COOLER); }
  inline void setCoolerEnabled(bool inEnableCooler, int inTimeout = sDefaultTimeoutMs) {
    this->sendSwitchVal<CameraTraitsT>(VecPropsT::COOLER_CONNECTION, PropsT::CONNECT_COOLER, inEnableCooler, inTimeout);
  }

  inline double getTemperature() const { return this->getNumberVal<CameraTraitsT>(VecPropsT::CCD_TEMPERATURE, PropsT::CCD_TEMPERATURE_VALUE); }
  inline void setTemperature(double inTemperature, int inTimeout = sDefaultTimeoutMs) {
    sendNumberVal<CameraTraitsT>(VecPropsT::CCD_TEMPERATURE, PropsT::CCD_TEMPERATURE_VALUE, inTemperature, inTimeout);
  }

  inline bool isTemperatureReached() const {
    INumberVectorProperty * nVec = this->getNumberVec<CameraTraitsT>(VecPropsT::CCD_TEMPERATURE);
    return (nVec->s != IPS_BUSY && nVec->s != IPS_ALERT);
  }

  inline double getMaxTemperature() const {
    const INumber & nTemp = this->getNumber<CameraTraitsT>(VecPropsT::CCD_TEMPERATURE, PropsT::CCD_TEMPERATURE_VALUE);
    return nTemp.max;
  }
  inline double getMinTemperature() const {
    const INumber & nTemp = this->getNumber<CameraTraitsT>(VecPropsT::CCD_TEMPERATURE, PropsT::CCD_TEMPERATURE_VALUE);
    return nTemp.min;
  }


  /**
   * Shutter control
   * TODO: How to handle state of vec? Still just return value?
   */
  bool isShutterOpen() const { return this->getSwitchVal<CameraTraitsT>(VecPropsT::SHUTTER_CONNECTION, PropsT::SHUTTER_ON); }
  void setShutterOpen(bool inShutterState, int inTimeout = sDefaultTimeoutMs) {
    this->sendNumberVal<CameraTraitsT>(VecPropsT::SHUTTER_CONNECTION, PropsT::SHUTTER_ON, inShutterState, inTimeout);
  }

  /**
   * Copies the latest image (BLOB) to the CImg pointe location.
   * Throws IndiCameraNoImageAvailableExceptionT if there is no image available (we may handle this differently...).
   * Throws IndiCameraExposureInProgressExceptionT if exposure is currently in progress.
   *
   * Note: Latest image remains on INDI device until next image has been captured. It is not deleted
   * directly after calling this function. That means multiple calls of this function deliver the same
   * result as long as there is a new image captured.
   *
   * TODO: Pass filename for simple image saving as well?
   * TODO: Add logging, better handle errors...
   * TODO: Who / what deletes the blob?
   * TODO: What about CCD2?
   * TODO: Add parameter - save image to ... (char *, use tmp. if 0, otherwise save to file).
   */
  void getImage(CImg<float> * outImg) {
    if (this->isExposureInProgress())
      throw IndiCameraExposureInProgressExceptionT();

    if (! outImg) {
      LOG(warning) << "No dest image pointer supplied..." << endl;
      return;
    }

    const IBLOB & blob = this->getBLOB<CameraTraitsT>(VecPropsT::CCD1, PropsT::CCD1);

    if (! blob.bloblen || ! blob.blob)
      throw IndiCameraNoImageAvailableExceptionT("BLOB empty! No image.");

    // Save to tmp - see http://cboard.cprogramming.com/c-programming/82961-example-mkstemp.html
    char sfn[15] = "";
    FILE * sfp;
    int fd = -1;
    
    strncpy(sfn, "/tmp/at.XXXXXX", sizeof sfn);
    if ((fd = mkstemp(sfn)) == -1 || (sfp = fdopen(fd, "w+")) == NULL) {
      LOG(error) << "Problem opening TMP file." << endl;
      if (fd != -1) {
	unlink(sfn);
	close(fd);
      }
      stringstream ssExc;
      ssExc << "Problem opening TMP file: " << sfn << ", msg: " << strerror(errno) << "." << endl;
      throw IndiCameraTmpFileExceptionT(ssExc.str());
    } else {
      LOG(debug) << "Successfully opened TMP file. Writing image to file: '" << sfn << "'..." << flush;      
      
      size_t bytesWritten = fwrite(static_cast<char *> (blob.blob), sizeof(char), blob.bloblen, sfp);
      
      LOG(debug) << "ok, bytesWritten: " << bytesWritten << endl;
      fclose(sfp);
      
      // Read file
      LOG(debug) << "Reading TMP file '" << sfn << "'..." << flush;
      readFile(*outImg, sfn);
      LOG(debug) << "DONE." << endl;
      
      // Removing tmp file
      if(remove(sfn)) {
	LOG(error) << "Error deleting TMP file '" << sfn << "'." << endl;
	stringstream ssExc;
	ssExc << "Error deleting TMP file: " << sfn << "." << endl;
	throw IndiCameraTmpFileExceptionT(ssExc.str());
      } else {
	LOG(debug) << "TMP file '" << sfn << "' successfully deleted." << endl;
      }
    }
  }
  
  /**
   * Convinience function for taking an image. Sync interface - function
   * blocks until image has been taken.
   *
   * TODO: What about image compression? Disabled compression works,
   *       enabled does not work. Maybe uncompress before required?
   *       Which algo? zip?
   * TODO: Additionally save image to a file... -> pass image path.
   */
  void takePicture(CImg<float> * outImg, double inExpTimeSec, const FrameT<int> & inFrame, FrameTypeT::TypeE inFrameType = FrameTypeT::LIGHT, const BinningT & inBinning = BinningT(1, 1), bool inCompressed = false) {

    if (! this->isConnected())
      throw IndiCameraNotConnectedExceptionT();

    if (this->isExposureInProgress())
      throw IndiCameraExposureInProgressExceptionT();

    LOG(trace) << "takePicture - Calling setBinning(" << inBinning << ")..." << endl;
    this->setBinning(inBinning);
    LOG(trace) << "takePicture - Calling setFrame(" << inFrame << ")..." << endl;
    this->setFrame(inFrame);
    LOG(trace) << "takePicture - Calling setFrameType(" << FrameTypeT::asStr(inFrameType) << ")..." << endl;
    this->setFrameType(inFrameType);
    LOG(trace) << "takePicture - setCompressed(" << (inCompressed ? "true" : "false") << ")..." << endl;
    this->setCompressed(inCompressed); // No compression when moving image to CImg
    LOG(trace) << "takePicture - expose(" << inExpTimeSec << "s)..." << endl;
    this->expose(inExpTimeSec); // Expose (blocking call).
    LOG(trace) << "takePicture - getImage(outImg=" << outImg << ")..." << endl;
    this->getImage(outImg);
    LOG(trace) << "takePicture - DONE." << endl;
  }  

  DEFINE_PROP_LISTENER(NewImageReceived, IBLOB*);
  // TODO: Register for further events...
  //   void (un)registerExposureFinishedHandler();
  //   void (un)registerExposureStartedHandler();
  //   void (un)registerExposureAbortHandler();
  // temperatureReached?!
};

#endif // _INDI_CAMERA_HPP_
