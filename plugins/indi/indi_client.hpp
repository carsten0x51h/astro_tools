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

#ifndef _INDI_CLIENT_H_
#define _INDI_CLIENT_H_ _INDI_CLIENT_H_

#include <iostream>
#include <exception>

#include <list>
#include <set>
#include <sstream>

// Serialization
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/split_member.hpp>
#include <fstream>

#include "indiapi.h"
#include "indidevapi.h"
#include "indicom.h"
#include "baseclient.h"
#include "basedevice.h"

#include "indi_listener.hpp"
#include "at_exception.hpp"
#include "io_util.hpp"
#include "at_logging.hpp"

#include "util.hpp"

DEF_Exception(IndiClient);
DEF_Exception(IndiClientDoesNotExist);
DEF_Exception(IndiClientAlreadyExists);
DEF_Exception(IndiClientConnectFailed);
DEF_Exception(IndiClientDisconnectFailed);
DEF_Exception(IndiDeviceNotFound);
DEF_Exception(IndiClientInvalidDelay);
DEF_Exception(UnknownIndiDeviceType);

using namespace std;
using namespace boost;
using namespace boost::serialization;
using namespace INDI;

class IndiDeviceT;
class IndiCameraT;
class IndiFilterWheelT;
class IndiFocuserT;


// TODO: Move DeviceTypeT def. elsewhere? Or leave here?
struct DeviceTypeT {
  enum TypeE {
    CAMERA,
    FOCUSER,
    FILTER_WHEEL,
    TELESCOPE, /*MOUNT?*/
    CUSTOM,
    _Count
  };

  static const char * asStr(const TypeE & inType) {
    switch (inType) {
    case CAMERA: return "CAMERA";
    case FOCUSER: return "FOCUSER";
    case FILTER_WHEEL: return "FILTER_WHEEL";
    case TELESCOPE: return "TELESCOPE";
    case CUSTOM: return "CUSTOM";
    default: return "<?>";
    }
  }

  MAC_AS_TYPE(Type, E, _Count);
};


// TODO: Move to class below?
typedef map<BaseDevice*, IndiDeviceT*> IndiDeviceMapT;

class IndiClientT : private INDI::BaseClient {

private:
  typedef map<string /* device name */, DeviceTypeT::TypeE> DeviceNameTypeMapT;

  friend class boost::serialization::access;
  friend class IndiDeviceT;

  static const char * sHostnameNvp;
  static const char * sPortNvp;
  static const char * sAutoConnectNvp;
  static const char * sDeviceNameMapNvp;

  static double sInitialDevicePopulationDelaySec;

  IndiDeviceMapT mDeviceMap;
  DeviceNameTypeMapT mDeviceNameMap;

  // TODO: We actually would like to move this stuff to DeviceT ?!
  DEFINE_PROP_LISTENER(NewSwitch, ISwitchVectorProperty*);
  DEFINE_PROP_LISTENER(NewNumber, INumberVectorProperty*);
  DEFINE_PROP_LISTENER(NewText, ITextVectorProperty*);
  DEFINE_PROP_LISTENER(NewLight, ILightVectorProperty*);
  DEFINE_PROP_LISTENER(NewBLOB, IBLOB*);

  DEFINE_INDI_SLOT(newSwitch, NewSwitch, ISwitchVectorProperty*); //!< Creates void newSwitch(ISwitchVectorProperty*)
  DEFINE_INDI_SLOT(newNumber, NewNumber, INumberVectorProperty*);
  DEFINE_INDI_SLOT(newText, NewText, ITextVectorProperty*);
  DEFINE_INDI_SLOT(newLight, NewLight, ILightVectorProperty*);
  DEFINE_INDI_SLOT(newBLOB, NewBLOB, IBLOB*);

  DEFINE_PROP_LISTENER(IndiServerConnectionStatus, bool);
  DEFINE_PROP_LISTENER(NewDevice, INDI::BaseDevice*);
  DEFINE_PROP_LISTENER(RemoveDevice, INDI::BaseDevice*);
  DEFINE_PROP_LISTENER(NewProp, INDI::Property*);
  //DEFINE_PROP_LISTENER(DeviceConnectionStatus, string);
  DEFINE_PROP_LISTENER(Message, const char*);

  IndiDeviceT * createDeviceInstance(INDI::BaseDevice * inBaseDevice, DeviceTypeT::TypeE inDeviceType);

public:
  static int sDefaultTimeoutMs;
  static const char * sDefaultIndiHostname;
  static const size_t sDefaultIndiPort;

  IndiClientT(const string & inHostname = sDefaultIndiHostname, int inPort = sDefaultIndiPort, bool inAutoConnect = false) : mIndiServerConnected(false), mServerExitCode(0), mAutoConnect(inAutoConnect), mAutoConnected(false) {
    this->setServer(inHostname.c_str(), inPort);
    //this->registerSwitchListener(boost::bind(& IndiClientT::internalSwitchListener, this, _1));

    if (mAutoConnect) {
      try {
	this->connect();
	mAutoConnected = true;
      } catch (IndiClientConnectFailedExceptionT & exc) {
	/* Ignore auto connection failed for now */
	LOG(info) << "Auto-connection to INDI server " << getHost() << ":" << getPort() << " failed." << endl;
      }
    }
  }

  ~IndiClientT();

  friend ostream & operator<<(ostream & os, const IndiClientT & inIndiClient);

  inline bool getAutoConnect() const { return mAutoConnect; }
  inline void setAutoConnect(bool inAutoConnect) { mAutoConnect = inAutoConnect; }
  inline bool getAutoConnected() const { return mAutoConnected; }

  // NOTE: Due to bad design of libindi interface we have to perform const_casts, here...
  inline const char * getHost() const { return const_cast<IndiClientT *>(this)->BaseClient::getHost(); }
  inline size_t getPort() const { return const_cast<IndiClientT *>(this)->BaseClient::getPort(); }
  inline void setServer(const char * inHost, size_t inPort) { this->BaseClient::setServer(inHost, inPort); } 

  static inline string getConnectionName(const string & inHostName, int inPort) {
    stringstream ss;
    ss << inHostName << ":" << inPort;
    return ss.str(); // Copy string
  }
  string getConnectionName() const { return getConnectionName(this->getHost(), this->getPort()); }


  /**
   * Wait some time until all currently available devices are populated to the client.
   * Currently we have no better way if we do not want to be sticked to an event-base design.
   */
  static void waitForInitialDevicePopulation() {
    int value = 1000000 * sInitialDevicePopulationDelaySec;
    usleep(value);
  }
  static void setInitialDevicePopulationDelaySec(double inDelay) {
    if (inDelay < 0)
      throw IndiClientInvalidDelayExceptionT("Delay must be positive.");
    sInitialDevicePopulationDelaySec = inDelay;
  }
  static double getInitialDevicePopulationDelaySec() { return sInitialDevicePopulationDelaySec; }


  /**
   * Blocking call until connected or hitting timeout. -> May be executed in a Qt thread.
   * Does nothing if already connected, throws IndiClientConnectFailedExceptionT if connectiong to INDI server failed.
   */
  void connect();


  /**
   * Blocks until disconnected from server.
   * TODO: What if server killed?
   */
  void disconnect();

  inline bool isConnected() const { return mIndiServerConnected; }

  // TODO: Should this map be accessable?!
  //const IndiDeviceMapT & getDeviceMap() { return mDeviceMap; }

  const vector<INDI::BaseDevice *> & getBaseDevices() const { return this->BaseClient::getDevices(); }
  
  IndiDeviceT * getDevice(const char * inDeviceName, DeviceTypeT::TypeE inDeviceType); // TODO: const?!
  // TODO:
  IndiCameraT * getCamera(const string & inDeviceName); // TODO: const?!
  IndiFilterWheelT * getFilterWheel(const string & inDeviceName); // TODO: const?!
  IndiFocuserT * getFocuser(const string & inDeviceName); // TODO: const?!


  /**
   * Object serialization using boost:
   * See: http://www.boost.org/doc/libs/1_43_0/libs/serialization/doc/serialization.html
   * See: http://stackoverflow.com/questions/3279037/serializing-a-map-of-objects-to-xml-using-boostserialization
   */
  template<class Archive>
  void save(Archive & ar, const unsigned int version) const
  {
    // NOTE: Due to bad design of libindi interface we have to perform an const_cast, here...
    string host(const_cast<IndiClientT *>(this)->getHost());
    int port = const_cast<IndiClientT *>(this)->getPort();

    ar << make_nvp(sHostnameNvp, host);
    ar << make_nvp(sPortNvp, port);
    ar << make_nvp(sAutoConnectNvp, mAutoConnect);
    ar << make_nvp(sDeviceNameMapNvp, mDeviceNameMap);
  }
  
  template<class Archive>
  void load(Archive & ar, const unsigned int version)
  {
    string host;
    int port;
    ar >> make_nvp(sHostnameNvp, host);
    ar >> make_nvp(sPortNvp, port);
    ar >> make_nvp(sAutoConnectNvp, mAutoConnect);
    ar >> make_nvp(sDeviceNameMapNvp, mDeviceNameMap);
    this->setServer(host.c_str(), port);
  }

  BOOST_SERIALIZATION_SPLIT_MEMBER();

  inline int getServerExitCode() const { return mServerExitCode; }

  ostream & print(ostream & os) const;

protected:

  /**
   * Implement the INDI interface.
   */
  virtual void newDevice(INDI::BaseDevice * inBaseDevice);
  virtual void removeDevice(INDI::BaseDevice * dp);
  virtual void newProperty(INDI::Property * pp) {
    LOG(trace) << "INDI CLIENT - New property received: " << pp->getDeviceName() << endl;
    mNewPropListeners(pp);
  }

  virtual void removeProperty(INDI::Property *) { /* ignore so far */ }
  virtual void newMessage(INDI::BaseDevice *dp, int messageID) {
    // TODO / FIXME: dp seems to be 0!?
    LOG(trace) << "INDI-CLIENT - new message received... dp: " << dp << endl;
    //LOG(debug) << "INDI-CLIENT - new message received..." << dp->messageQueue(messageID) << endl;
    // TODO / FIXME! Where is mMessageListeners defined??? No longer there?! 
    //mMessageListeners(dp->messageQueue(messageID));
  }

  virtual void serverConnected() {
    LOG(debug) << "INDI-CLIENT - server connected." << endl;
    mIndiServerConnectionStatusListeners(mIndiServerConnected);
  }

  virtual void serverDisconnected(int exitCode) {
    LOG(debug) << "INDI-CLIENT - server disconnected - exitCode: " << exitCode << endl;
    mIndiServerConnected = false;
    mServerExitCode = exitCode;
    mIndiServerConnectionStatusListeners(mIndiServerConnected);
  }

  /** 
   * ATTENTION!!!!! This function is executed from another thread!!!!!!! --> when CONNECTION Prop has been received!!!
   */
  // void internalSwitchListener(ISwitchVectorProperty * inSwitchVecPtr) {
    
  //   //cerr << ">>>>>>>>>>>>>>>>>>>> internalSwitchListener!!!" << endl;

  //   if (! inSwitchVecPtr)
  //     return;

  //   if (strcmp(inSwitchVecPtr->name, VecPropNameT::asStr(VecPropNameT::CONNECTION)))
  //     return;
    
  //   ISwitch * sw = IUFindSwitch(inSwitchVecPtr, PropNameT::asStr(PropNameT::CONNECT));
  //   if (! sw)
  //     return;

  //   //bool connected = (sw->s == ISS_ON && inSwitchVecPtr->s == IPS_OK);
  //   //mDeviceConnectionStatusListeners.sendUpdate(connected);
  //   mDeviceConnectionStatusListeners(inSwitchVecPtr->device);
  // } 

private:
  bool mIndiServerConnected; //!<Indi BaseClient does not allow access to its sConnected flag. Hence we need our own.
  int mServerExitCode;
  bool mAutoConnect;
  bool mAutoConnected;
};

typedef map<string /* connectionName */, IndiClientT *> IndiClientsT;
ostream & operator<<(ostream & os, const IndiClientT & inIndiClient);


#endif // _INDI_CLIENT_H_
