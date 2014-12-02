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

// NOTE / TODO: We may use CRTP here! pasing traits as tmpl param to classes, but tmpl. inherits from IndiDeviceT base class!

#ifndef _INDI_DEVICE_HPP_
#define _INDI_DEVICE_HPP_ _INDI_DEVICE_HPP_

#include <string>
#include <iostream>
#include <map>

#include "at_exception.hpp"
#include "at_logging.hpp"
#include "util.hpp"

#include "indi_listener.hpp"
#include "indi_utils.hpp"

#include "indiapi.h"
#include "indidevapi.h"
#include "indicom.h"
#include "baseclient.h"
#include "basedevice.h"

#include "indi_client.hpp"

// Serialization
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/split_member.hpp>
#include <fstream>


DEF_Exception(IndiDevice);
DEF_Exception(PropPermission);
DEF_Exception(SetPropValueFailed);
DEF_Exception(PropInvalidValue);
DEF_Exception(IndiDevicePropMapSerializationFailed);
DEF_Exception(IndiDeviceInvalidProp);
DEF_Exception(IndiDeviceConnectFailed);
DEF_Exception(IndiDeviceTypeMismatch);

class IndiClientT;

using namespace std;
using namespace INDI;
using namespace boost::serialization;


#define GET_PROP_VEC(__type__, __wocheck__)				\
  I##__type__##VectorProperty * get##__type__##Vec(const string & inVecPropName) const { \
    AT_ASSERT(IndiDevice, mBaseDevice, "Expected mBaseDevice to be set."); \
    I##__type__##VectorProperty * vec = mBaseDevice->get##__type__(inVecPropName.c_str()); \
      									\
    if (! vec) {							\
      const string exStr = "Vector property '" + inVecPropName  + "' not found."; \
      throw PropNotFoundExceptionT(exStr.c_str());			\
    }									\
    if (__wocheck__) {							\
      const string exStr = "Vector property '" + inVecPropName  + "' is write-only."; \
      throw PropPermissionExceptionT(exStr.c_str());			\
    }									\
    return vec;								\
  }									\
									\
  template<typename TraitsT>						\
  I##__type__##VectorProperty * get##__type__##Vec(typename TraitsT::VecPropsT::TypeE inVecProp) const { \
    return this->get##__type__##Vec(this->lookupVecPropName(TraitsT::VecPropsT::asStr(inVecProp))); \
  }									\


#define GET_PROP(__type__, __wocheck__)					\
  static inline const I##__type__ & get##__type__(I##__type__##VectorProperty * inVec, const string & inPropName) { \
    I##__type__ * p = IUFind##__type__(inVec, inPropName.c_str());	\
    if (! p) {							        \
      const string exStr = "Property '" + inPropName  + "' in vector '" + string(inVec->name) + "' not found."; \
       throw PropNotFoundExceptionT(exStr.c_str());			\
    }									\
    return *p;							        \
  }									\
									\
  template<typename TraitsT>						\
  inline const I##__type__ & get##__type__(I##__type__##VectorProperty * inVec, typename TraitsT::PropsT::TypeE inProp) const { \
    return this->get##__type__(inVec, this->lookupPropName(TraitsT::PropsT::asStr(inProp))); \
  }									\
									\
  inline const I##__type__ & get##__type__(const string & inVecPropName, const string & inPropName) const { \
    I##__type__##VectorProperty * vec = get##__type__##Vec(inVecPropName); \
    return this->get##__type__(vec, inPropName);		        \
  }									\
									\
  template<typename TraitsT>						\
  inline const I##__type__ & get##__type__(typename TraitsT::VecPropsT::TypeE inVecProp, typename TraitsT::PropsT::TypeE inProp) const { \
    return this->get##__type__(this->lookupVecPropName(TraitsT::VecPropsT::asStr(inVecProp)), this->lookupPropName(TraitsT::PropsT::asStr(inProp))); \
  }									\


#define GET_PROP_VAL(__type__, __rettype__, __assignment__)		\
  static inline __rettype__ get##__type__##Val(I##__type__##VectorProperty * inVec, const string & inPropName) { \
    AT_ASSERT(IndiUtils, inVec, "Expecting inVec to be != 0.");		\
    const I##__type__ & p = get##__type__(inVec, inPropName);		\
    return __assignment__;						\
  }									\
									\
  template<typename TraitsT>						\
  inline __rettype__ get##__type__##Val(I##__type__##VectorProperty * inVec, typename TraitsT::PropsT::TypeE inProp) const { \
    return this->get##__type__##Val(inVec, this->lookupPropName(TraitsT::PropsT::asStr(inProp))); \
  }									\
									\
  __rettype__ get##__type__##Val(const string & inVecPropName, const string & inPropName) const { \
    I##__type__##VectorProperty * vec = get##__type__##Vec(inVecPropName); \
      return this->get##__type__##Val(vec, inPropName);			\
  }									\
									\
  template<typename TraitsT>						\
  inline __rettype__ get##__type__##Val(typename TraitsT::VecPropsT::TypeE inVecProp, typename TraitsT::PropsT::TypeE inProp) const { \
    return this->get##__type__##Val(this->lookupVecPropName(TraitsT::VecPropsT::asStr(inVecProp)), this->lookupPropName(TraitsT::PropsT::asStr(inProp))); \
  }									\


#define SEND_PROP_VEC(__type__)						\
  void send##__type__##Vec(I##__type__##VectorProperty * inVec, int inTimeoutMs = sDefaultTimeoutMs) { \
    AT_ASSERT(IndiDevice, mBaseDevice, "Expected mBaseDevice to be set."); \
    if (! inVec)							\
      return;								\
    WAIT_MAX_FOR(inVec->s == IPS_OK || inVec->s == IPS_IDLE, inTimeoutMs, "Hit timeout setting while setting value."); \
    mIndiClient->sendNew##__type__(inVec);				\
    WAIT_MAX_FOR(inVec->s != IPS_BUSY, inTimeoutMs, "Hit timeout setting while setting value."); \
    if (IPS_ALERT == inVec->s) {					\
      const string exStr = "Problem setting vector '" + string(inVec->name) + "' (ALERT state)."; \
      throw SetPropValueFailedExceptionT(exStr.c_str());		\
    }									\
  }									\


#define SEND_PROP_VAL(__type__, __valtype__)				\
  inline void send##__type__##Val(I##__type__##VectorProperty * inVec, const string & inPropName, __valtype__ inValue, int inTimeoutMs = sDefaultTimeoutMs) { \
    this->update##__type__##Val(inVec, inPropName, inValue);		\
    send##__type__##Vec(inVec, inTimeoutMs);				\
  }									\
  									\
  template<typename TraitsT>						\
  inline void send##__type__##Val(I##__type__##VectorProperty * inVec, typename TraitsT::PropsT::TypeE inProp, __valtype__ inValue, int inTimeoutMs = sDefaultTimeoutMs) { \
    this->send##__type__##Val(inVec, this->lookupPropName(TraitsT::PropsT::asStr(inProp)), inValue, inTimeoutMs); \
  }									\
  									\
  inline void send##__type__##Val(const string & inVecPropName, const string & inPropName, __valtype__ inValue, int inTimeoutMs = sDefaultTimeoutMs) { \
    I##__type__##VectorProperty * vec = this->get##__type__##Vec(inVecPropName); \
    send##__type__##Val(vec, inPropName, inValue, inTimeoutMs);		\
  }									\
									\
  template<typename TraitsT>						\
  inline void send##__type__##Val(typename TraitsT::VecPropsT::TypeE inVecProp, typename TraitsT::PropsT::TypeE inProp, __valtype__ inValue, int inTimeoutMs = sDefaultTimeoutMs) { \
    this->send##__type__##Val(this->lookupVecPropName(TraitsT::VecPropsT::asStr(inVecProp)), this->lookupPropName(TraitsT::PropsT::asStr(inProp)), inValue, inTimeoutMs); \
  }									\


#define UPDATE_PROP_VAL(__type__, __valtype__)				\
  template<typename TraitsT>						\
  void update##__type__##Val(I##__type__##VectorProperty * inVec, typename TraitsT::PropsT::TypeE inProp, __valtype__ inValue) { \
    IndiDeviceT::update##__type__##Val(inVec, this->lookupPropName(TraitsT::PropsT::asStr(inProp)), inValue); \
  }									\
									\
  void update##__type__##Val(const string & inVecPropName, const string & inPropName, __valtype__ inValue) { \
  I##__type__##VectorProperty * vec = this->get##__type__##Vec(inVecPropName);	\
    update##__type__##Val(vec, inPropName, inValue);				\
  }									\
									\
  template<typename TraitsT>						\
  void update##__type__##Val(typename TraitsT::VecPropsT::TypeE inVecProp, typename TraitsT::PropsT::TypeE inProp, __valtype__ inValue) { \
    this->update##__type__##Val(this->lookupVecPropName(TraitsT::VecPropsT::asStr(inVecProp)), this->lookupPropName(TraitsT::PropsT::asStr(inProp)), inValue); \
  }									\


#define CASE_INSERT_PROPS(__type__, __valtype__, __numelem__, __elem__) \
  case INDI_##__type__: {						\
    I##__valtype__##VectorProperty * vp = static_cast<I##__valtype__##VectorProperty *>(indiProp->getProperty()); \
    AT_ASSERT(IndiDevice, vp, "VecProp ptr expected to be set.");	\
    for (size_t i = 0; i < vp->__numelem__; ++i)			\
      indiPropNames.insert(vp->__elem__[i].name);			\
    break;								\
  }									\



typedef vector<INDI::Property *> IndiPropVecT;

/**
// TODO: What about the indi client?! -> IndiClientMgr?
// NOTE: The IndiClientMgt stuff went to indi/IndiClientTmplT!!! ... Maybe that is a bad name?!
// TODO: What about configuring certain devices i.e. map their (vec) prop names to the AstroTools internal prop names?

// Above required?
// We could move an enum for VecPropsT and PropsT to DeviceT with all default names... and additional ones to CameraT, FocuserT etc... no traits....
*/

////// N O T E: IndiDeviceT will have a state! - the registered listener functions! Hence, it is not just a simple wrapper function any longer! We need to manage the instances and keep them in touch with the BaseDevice... Do we? Does inherritance solve this problem?! No. probably not.... static_cast may be a problem.... It is probably a cleaner solution to store an IndiDeviceT instance for each BaseDevice *... (MAP)... In this case the inherritance may no longer be required....?! --> IndiDeviceManager?
class IndiDeviceT : public INDI::BaseMediator {
public:
  typedef map<string, string> PropNameMapT;

  // Implementation of mediator interface
private:
  static const char * sVecPropNameMapNvp;
  static const char * sPropNameMapNvp;
  static double sInitialDevicePropertyPopulationDelaySec;

  DeviceTypeT::TypeE mType;

  PropNameMapT mVecPropNameMap;
  PropNameMapT mPropNameMap;


  // TODO: We actually would like to move this stuff to DeviceT ?!
  //DEFINE_PROP_LISTENER(IndiServerConnectionStatus, bool);
  DEFINE_PROP_LISTENER(NewSwitch, ISwitchVectorProperty*); //!< Creates (un)registerSwitchListener()
  DEFINE_PROP_LISTENER(NewNumber, INumberVectorProperty*);
  DEFINE_PROP_LISTENER(NewText, ITextVectorProperty*);
  DEFINE_PROP_LISTENER(NewLight, ILightVectorProperty*);
  DEFINE_PROP_LISTENER(NewBLOB, IBLOB*);
  DEFINE_PROP_LISTENER(NewDevice, BaseDevice*);
  DEFINE_PROP_LISTENER(NewProperty, Property*);
  DEFINE_PROP_LISTENER(RemoveProperty, Property*);
  DEFINE_PROP_LISTENER(NewMessage, const char*);

  // TODO: Is there a way to convert a char to upper case in a macro?!
  DEFINE_INDI_SLOT(newSwitch, NewSwitch, ISwitchVectorProperty*); //!< Creates void newSwitch(ISwitchVectorProperty*)
  DEFINE_INDI_SLOT(newNumber, NewNumber, INumberVectorProperty*);
  DEFINE_INDI_SLOT(newText, NewText, ITextVectorProperty*);
  DEFINE_INDI_SLOT(newLight, NewLight, ILightVectorProperty*);
  DEFINE_INDI_SLOT(newBLOB, NewBLOB, IBLOB*);
  DEFINE_INDI_SLOT(newDevice, NewDevice, BaseDevice*);
  DEFINE_INDI_SLOT(newProperty, NewProperty, Property*);
  DEFINE_INDI_SLOT(removeProperty, RemoveProperty, Property*);

  virtual void newMessage(BaseDevice *dp, int messageID) {
    // TODO: Pass this message to an output stream which has been set from "outside"! Don't pass it if no ostream was set. Same for below?!
    //cerr << ">>>>>>>>>> IndiDevice mediator - newMessage: " << dp->messageQueue(messageID) << endl;
  }
  virtual void serverConnected() {
    AT_ASSERT(IndiDevice, false, "IndiDevice::serverConnected() should never been hit.");
  }
  virtual void serverDisconnected(int exit_code) {
    // TODO: Is this always ok? What if server exits? -> Test...
    AT_ASSERT(IndiDevice, false, "IndiDevice::serverDisconnected() should never been hit.");
  }

  IndiClientT * mIndiClient;
  BaseDevice * mBaseDevice; /* OR shall we inherrit from INDI::Device instead ??? */

  // We do not want device copies
  IndiDeviceT(const IndiDeviceT &);
  IndiDeviceT & operator=(const IndiDeviceT &); 

  DeviceTypeT::TypeE getDeviceTypeFromProps();

  //   // Further device functions...
  //   // void addMessage(const char *msg);
  // Device property mgmt
  //   // void * getRawProperty(const char *name, INDI_TYPE type = INDI_UNKNOWN);
  //   // INDI::Property * getProperty(const char *name, INDI_TYPE type = INDI_UNKNOWN);
  //   cerr << ", Properties: " << device->getProperties()->size() << endl;

  // TODO: Set a device mapping directory? - static const in AstroToolsT?!
  static inline string genPropMapFileName(const string & inUniqueDeviceId) { return inUniqueDeviceId + ".xml"; }
  inline string genPropMapFileName() const { return IndiDeviceT::genPropMapFileName(this->getUniqueDeviceId()); }

  static void loadPropMaps(const string & inFilename, PropNameMapT * outVecPropNameMap, PropNameMapT * outPropNameMap) {
    AT_ASSERT(IndiDevice, outVecPropNameMap && outPropNameMap, "loadPropMaps needs valid map pointers.");

    std::ifstream ifs(inFilename.c_str());
    if (ifs.good()) {
      // Load mapping only if file exists, otherwise map remains empty
      boost::archive::xml_iarchive ia(ifs);
      ia >> make_nvp(sVecPropNameMapNvp, *outVecPropNameMap);
      ia >> make_nvp(sPropNameMapNvp, *outPropNameMap);

      LOG(info) << "Loaded prop map from file '" << inFilename << "'..." << endl;
    } else {
      LOG(info) << "No prop map with filename '" << inFilename << "'..." << endl;
    }
  }
  void loadPropMaps() { IndiDeviceT::loadPropMaps(this->genPropMapFileName(), & mVecPropNameMap, & mPropNameMap); }

  static void savePropMaps(const string & inFilename, const PropNameMapT & inVecPropNameMap, const PropNameMapT & inPropNameMap) {
    std::ofstream ofs(inFilename.c_str());
    if (! ofs.good()) {
      throw IndiDevicePropMapSerializationFailedExceptionT("Saving prop map failed.");
    }
    boost::archive::xml_oarchive oa(ofs);
    oa << make_nvp(sVecPropNameMapNvp, inVecPropNameMap);
    oa << make_nvp(sPropNameMapNvp, inPropNameMap);
    LOG(info) << "Saved prop map to file '" << inFilename << "'..." << endl;
  }
  void savePropMaps() { IndiDeviceT::savePropMaps(this->genPropMapFileName(), mVecPropNameMap, mPropNameMap); }


  static inline string lookupPropName(const PropNameMapT & inVecPropNameMap, const string & inDefaultName) {
    PropNameMapT::const_iterator it = inVecPropNameMap.find(inDefaultName);
    return (it != inVecPropNameMap.end() ? it->second : inDefaultName);
  }
  // TODO: Bad naming...
  string inline lookupVecPropName(const string & inDefaultName) const { return IndiDeviceT::lookupPropName(mVecPropNameMap, inDefaultName); }
  string inline lookupPropName(const string & inDefaultName) const { return IndiDeviceT::lookupPropName(mPropNameMap, inDefaultName); }


public:
  static int sDefaultTimeoutMs;

  /**
   * Wait some time until all currently available device properties are populated to the client.
   * Currently we have no better way if we do not want to be sticked to an event-base design.
   */
  static void waitForInitialDevicePropertyPopulation() { usleep(1000000 * sInitialDevicePropertyPopulationDelaySec); }
  static void setInitialDevicePropertyPopulationDelaySec(double inDelay) {
    if (inDelay < 0)
      throw IndiClientInvalidDelayExceptionT("Delay must be positive.");
    sInitialDevicePropertyPopulationDelaySec = inDelay;
  }
  static double getInitialDevicePropertyPopulationDelaySec() { return sInitialDevicePropertyPopulationDelaySec; }

  IndiDeviceT(IndiClientT * inIndiClient, BaseDevice * inBaseDevice, DeviceTypeT::TypeE inType = DeviceTypeT::CUSTOM) : mIndiClient(inIndiClient), mBaseDevice(inBaseDevice), mType(inType) {
    mBaseDevice->setMediator(this);

    // Load prop mapping
    this->loadPropMaps();

    // TODO: What should be the default? How far should users care about that?
    mIndiClient->setBLOBMode(B_ALSO, mBaseDevice->getDeviceName());
  }
  virtual ~IndiDeviceT() {
    // Save prop mapping
    this->savePropMaps();
  }

  friend ostream & operator<<(ostream & os, const IndiDeviceT & inIndiDevice);

  inline bool isConnected() const { return mBaseDevice->isConnected(); }

  void connect(int inTimeoutMs = sDefaultTimeoutMs, bool inVerifyDeviceType = true);
  void disconnect(int inTimeoutMs = sDefaultTimeoutMs);

  // TODO: Verify of this function works
  // TODO: Implement hasVecProp(string...)
  // TODO: also add for PropT
  template<typename TraitsT>
  bool hasVecProp(typename TraitsT::VecPropsT::TypeE inVecProp) const {
    return this->hasVecProp(TraitsT::VecPropsT::asStr(inVecProp));
  }

  bool hasVecProp(const string & inVecPropName) const {
    bool hasVecProp = false;
    PropNameMapT vecPropNameMap, propNameMap;
    IndiDeviceT::loadPropMaps(this->genPropMapFileName(), & vecPropNameMap, & propNameMap);
    const string vecPropTypeName(IndiDeviceT::lookupPropName(vecPropNameMap, inVecPropName));
    cerr << "vecPropTypeName: " << vecPropTypeName << endl;

    const IndiPropVecT * indiPropVec = getIndiProperties();
    for (typename IndiPropVecT::const_iterator it = indiPropVec->begin(); it != indiPropVec->end(); ++it) {
      Property * indiProp = *it;
      AT_ASSERT(IndiDevice, indiProp, "Expect indiProp to be valid.");
      
      // Check if vec prop exists in traits...
      if (! strcmp(vecPropTypeName.c_str(), indiProp->getName())) {
	hasVecProp = true;
	break;
      }
    } // end for - all indi props
    return hasVecProp;
  }


  // TODO: Pointers passed as const?!
  static inline string getUniqueDeviceId(IndiClientT * inIndiClient, BaseDevice * inBaseDevice) {
    return inIndiClient->getConnectionName() + string("_") + inBaseDevice->getDeviceName();
  }
  inline string getUniqueDeviceId() const { return IndiDeviceT::getUniqueDeviceId(mIndiClient, mBaseDevice); }

  inline const IndiPropVecT * getIndiProperties() const { return mBaseDevice->getProperties(); }


  inline PropNameMapT & getVecPropMap() { return mVecPropNameMap; }
  inline PropNameMapT & getPropMap() { return mPropNameMap; }


  /**
   * Light
   *
   */
  GET_PROP_VEC(Light, false);
  GET_PROP(Light, false);
  GET_PROP_VAL(Light, IPState, p.s);

  /**
   * BLOB
   *
   */
  GET_PROP_VEC(BLOB, IP_WO == vec->p);
  GET_PROP(BLOB, IP_WO == vec->p);
  GET_PROP_VAL(BLOB, void *, p.blob);
  // TODO: - is there any case where a client wants to send blob to a device? -> maybe firmware update?
  // void startBlob( const char *devName, const char *propName, const char *timestamp);
  // void sendOneBlob( const char *blobName, unsigned int blobSize, const char *blobFormat, void * blobBuffer);
  // void finishBlob();


  /**
   * Text
   *
   */
  GET_PROP_VEC(Text, IP_WO == vec->p);
  GET_PROP(Text, IP_WO == vec->p);
  GET_PROP_VAL(Text, const char *, p.text);

  UPDATE_PROP_VAL(Text, const char *);
  SEND_PROP_VEC(Text);
  SEND_PROP_VAL(Text, const char *);

  static void updateTextVal(ITextVectorProperty * inVec, const string & inPropName, const char * inValue) {
    AT_ASSERT(IndiUtils, inVec, "Expecting inVec to be != 0.");
    
    if (IP_RO == inVec->p) {
      const string exStr = "Vector property '" + string(inVec->name)  + "' is read-only.";
      throw PropPermissionExceptionT(exStr.c_str());
    }
    
    IText * p = IUFindText(inVec, inPropName.c_str());
    if (! p) {
      const string exStr = "Property '" + inPropName  + "' in vector '" + string(inVec->name) + "' not found.";
      throw PropNotFoundExceptionT(exStr.c_str());
    }
    if (! inValue)
      return;
    strcpy(p->text, inValue);
  }
  

  /**
   * Number
   *
   */
  GET_PROP_VEC(Number, IP_WO == vec->p);
  GET_PROP(Number, IP_WO == vec->p);
  GET_PROP_VAL(Number, double, p.value);
  UPDATE_PROP_VAL(Number, double);
  SEND_PROP_VEC(Number);
  SEND_PROP_VAL(Number, double);

  static void updateNumberVal(INumberVectorProperty * inVec, const string & inPropName, double inValue) {
    AT_ASSERT(IndiUtils, inVec, "Expecting inVec to be != 0.");

    if (IP_RO == inVec->p) {
      const string exStr = "Vector property '" + string(inVec->name)  + "' is read-only.";
      throw PropPermissionExceptionT(exStr.c_str());
    }

    INumber * p = IUFindNumber(inVec, inPropName.c_str());
    if (! p) {
      const string exStr = "Property '" + inPropName  + "' in vector '" + string(inVec->name) + "' not found.";
      throw PropNotFoundExceptionT(exStr.c_str());
    }

    if (inValue < p->min || inValue > p->max) {
       stringstream ss;
       ss << "Number '" + string(inVec->name) + "' is out of range. Is value: " << inValue << ", min: " << p->min << ", max: " << p->max << endl;
       throw PropInvalidValueExceptionT(ss.str().c_str());
    }
    p->value = inValue;
  }


  /**
   * Switch
   *
   */
  GET_PROP_VEC(Switch, IP_WO == vec->p);
  GET_PROP(Switch, IP_WO == vec->p);
  GET_PROP_VAL(Switch, bool, (p.s == ISS_ON));
  UPDATE_PROP_VAL(Switch, bool);
  SEND_PROP_VEC(Switch);
  SEND_PROP_VAL(Switch, bool);

  static void updateSwitchVal(ISwitchVectorProperty * inVec, const string & inPropName, double inValue) {
    if (IP_RO == inVec->p) {
      const string exStr = "Vector property '" + string(inVec->name) + "' is read-only.";
      throw PropPermissionExceptionT(exStr.c_str());
    }
    
    switch(inVec->r) {
    case ISR_1OFMANY: {
      // Only one can be active
      if (inValue) {
  	// Disable all and then enable the selected one...
  	for (size_t i=0; i < inVec->nsp; ++i) { inVec->sp[i].s = ISS_OFF; }
        // TODO: Code below we may put to a separate function (Find call including exception...)
        ISwitch * p = IUFindSwitch(inVec, inPropName.c_str());
        if (! p) {
  	  const string exStr = "Property '" + string(inVec->name) + "' in vector '" + string(inVec->name) + "' not found.";
  	  throw PropNotFoundExceptionT(exStr.c_str());
        }
        p->s = ISS_ON;
      } else {
  	// Disable a switch - additional checking required...
  	if (inVec->nsp <= 2) {
  	  for(size_t i=0; i < inVec->nsp; ++i) { inVec->sp[i].s = ISS_ON; }
          // TODO: Code below we may put to a separate function (Find call including exception...)
          ISwitch * p = IUFindSwitch(inVec, inPropName.c_str());
          if (! p) {
             const string exStr = "Property '" + inPropName  + "' in vector '" + string(inVec->name) + "' not found.";
             throw PropNotFoundExceptionT(exStr.c_str());
          }
          p->s = ISS_OFF;
  	} else {
  	  // Problem when setting just one switch to OFF because which one to enable when there are more than two switches?
  	  throw SetPropValueFailedExceptionT("Switch vector with rule ISR_1OFMANY has > 2 switches. Which one to enable?");
  	}
      }
      break;
    }
    case ISR_ATMOST1: {
      // There is only ONE switch.... - TODO: how to handle?! - on or off..?
      AT_ASSERT(IndiDevice, false, "Not yet implemented - no such case, yet.");
      break;
    }
    case ISR_NOFMANY: {
      // Any number of switches can be on (e.g. e.g. GUIDER_RAPID_GUIDE_SETUP->AUTO_LOOP, SEND_IMAGE, SHOW_MARKER)
      // TODO: Code below we may put to a separate function (Find call including exception...)
      ISwitch * p = IUFindSwitch(inVec, inPropName.c_str());
      if (! p) {
         const string exStr = "Property '" + inPropName  + "' in vector '" + string(inVec->name) + "' not found.";
         throw PropNotFoundExceptionT(exStr.c_str());
      }
      p->s = (inValue ? ISS_ON : ISS_OFF);
      break;
    }
    default: {
      AT_ASSERT(IndiDevice, false, "Invalid switch rule.");
    }
    } // end switch
  }
  
  template<typename TraitsT>
  static bool satisfiesTraits(const string inUniqueDeviceId, const IndiPropVecT * inIndiPropVec) {
    typedef typename TraitsT::VecPropsT VecPropsT;
    typedef typename TraitsT::PropsT PropsT;

    PropNameMapT vecPropNameMap, propNameMap;
    set<string> indiPropNames;

    // TODO: loaded map is empty.....?!
    IndiDeviceT::loadPropMaps(IndiDeviceT::genPropMapFileName(inUniqueDeviceId), & vecPropNameMap, & propNameMap);
      
    // VEC PROPS
    // Iterate all vec-props and see if it exists
    vector<typename VecPropsT::TypeE> missingVecProps;

    for (size_t i=0; i < VecPropsT::_Count; ++i) {
      typename VecPropsT::TypeE vecPropType = static_cast<typename VecPropsT::TypeE>(i);
      
      if (VecPropsT::isOptional(vecPropType))
	continue;

      bool hasVecProp = false;
      LOG(debug) << "Looking up vecPropTypeName: " << VecPropsT::asStr(vecPropType) << "..." << flush;
      const string vecPropTypeName (IndiDeviceT::lookupPropName(vecPropNameMap, VecPropsT::asStr(vecPropType)));
      LOG(debug) << "Result: " << vecPropTypeName << endl;

      // Collect all prop names and check if vec props exist in traits...
      for (typename IndiPropVecT::const_iterator it = inIndiPropVec->begin(); it != inIndiPropVec->end(); ++it) {
	Property * indiProp = *it;
	AT_ASSERT(IndiDevice, indiProp, "Expect indiProp to be valid.");

	// Collect all prop names to indiPropNames set... 
	switch(indiProp->getType()) {
	  CASE_INSERT_PROPS(NUMBER, Number, nnp, np);
	  CASE_INSERT_PROPS(SWITCH, Switch, nsp, sp);
	  CASE_INSERT_PROPS(TEXT, Text, ntp, tp);
	  CASE_INSERT_PROPS(LIGHT, Light, nlp, lp);
	  CASE_INSERT_PROPS(BLOB, BLOB, nbp, bp);
	default:
	  AT_ASSERT(IndiDevice, false, "Unsupported indi property type.");
	}
	
	// Check if vec prop exists in traits...
	if (! strcmp(vecPropTypeName.c_str(), indiProp->getName())) {
	  hasVecProp = true;
	  break;
	}
      } // end for - all indi props
      
      if (! hasVecProp)
	missingVecProps.push_back(vecPropType);

    } // end for - all VecPropsT


    // PROPS
    set<typename PropsT::TypeE> missingProps;
    
    // Iterate all props and see if it exists
    for (size_t i=0; i < PropsT::_Count; ++i) {
      typename PropsT::TypeE propType = static_cast<typename PropsT::TypeE>(i);
      
      if (PropsT::isOptional(propType))
	continue;

      bool hasProp = false;
      const char * propTypeName = IndiDeviceT::lookupPropName(propNameMap, PropsT::asStr(propType)).c_str();
      for (set<string>::const_iterator it = indiPropNames.begin(); it != indiPropNames.end(); ++it) {
	if (! strcmp(propTypeName, it->c_str())) {
	  LOG(debug) << "Expecting: " << propTypeName <<  " --> ok, has prop: " << it->c_str() << endl;
	  hasProp = true;
	  break;
	}
      } // end for
      
      if (! hasProp) {
	missingProps.insert(propType);
      }
    } // end for
    



    // TODO: logging...
    if (! missingVecProps.empty() || ! missingProps.empty()) {
      LOG(info) << inUniqueDeviceId << "' does not satisfy '" << DeviceTypeT::asStr(TraitsT::sDeviceType) << "' traits..." << endl;
    }
    if (! missingVecProps.empty()) {
      LOG(info) << missingVecProps.size() << " missing vec prop" << (missingVecProps.size() > 1 ? "s:" : ":") << endl;
      for (typename vector<typename VecPropsT::TypeE>::const_iterator it = missingVecProps.begin(); it != missingVecProps.end(); ++it) {
	LOG(info) << "   " << VecPropsT::asStr(*it) << endl;
      }
      LOG(info) << endl;
    }

    if (! missingProps.empty()) {
       LOG(info) << missingProps.size() << " missing prop" << (missingProps.size() > 1 ? "s:" : ":") << endl;
      for (typename set<typename PropsT::TypeE>::const_iterator it = missingProps.begin(); it != missingProps.end(); ++it) {
	LOG(info) << "   " << PropsT::asStr(*it) << endl;
      }
      LOG(info) << endl;
    }

     
    return (missingProps.empty() && missingVecProps.empty());
  }

  template<typename TraitsT>
  bool satisfiesTraits() const { return IndiDeviceT::satisfiesTraits<TraitsT>(this->getUniqueDeviceId(), this->mBaseDevice->getProperties()); }

  static DeviceTypeT::TypeE getType() { return DeviceTypeT::CUSTOM; }

  ostream & print(ostream & os) const;

};

ostream & operator<<(ostream & os, const IndiDeviceT & inIndiDevice);


#endif // _INDI_DEVICE_HPP_
