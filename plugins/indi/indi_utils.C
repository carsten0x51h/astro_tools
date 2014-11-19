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

#include <iostream>
#include <iomanip>
#include "indi_utils.hpp"

using namespace std;

size_t IndiUtilsT::sColNum = 0;
size_t IndiUtilsT::sColNumDiff = 4;

// TODO: Maybe define ostream which already does << setw(col) << " " in advance... i.e. which cares about the correct space...

#define COMMON_VECPROP_PRINTER(__os__, __propname__, __varname__)     \
  __os__ << setw(cCol1) << " " << setw(cCol2) << __propname__ << endl	\
  << setw(cCol1) << " " << setw(cCol2) << "> device: "    << __varname__.device << endl \
  << setw(cCol1) << " " << setw(cCol2) << "> group: "     << __varname__.group << endl \
  << setw(cCol1) << " " << setw(cCol2) << "> label: "     << __varname__.label << endl \
  << setw(cCol1) << " " << setw(cCol2) << "> name: "      << __varname__.name << endl \
  << setw(cCol1) << " " << setw(cCol2) << "> timestamp: " << __varname__.timestamp << endl \
  << setw(cCol1) << " " << setw(cCol2) << "> state: "     << IndiUtilsT::asStr(__varname__.s) << endl; \


#define COMMON_PROP_PRINTER(__os__, __proptype__, __varname__, __numname__, __propname__) \
  IndiUtilsT::sColNum += IndiUtilsT::sColNumDiff;			\
  for (size_t i=0; i < __varname__. __numname__; ++i) {			\
    __proptype__ & p = __varname__.__propname__[i];			\
    __os__ << p << endl;						\
  }									\
  IndiUtilsT::sColNum -= IndiUtilsT::sColNumDiff;			\
  

/**
 * Number ostream operator
 */
ostream & operator<<(ostream & os, const INumber & inNum) {
  const size_t cCol1 = IndiUtilsT::sColNum, cCol2 = 20;
  os << setw(cCol1) << " " << setw(cCol2) << "Number"     << left << endl
     << setw(cCol1) << " " << setw(cCol2) << "> name: "   << inNum.name << endl
     << setw(cCol1) << " " << setw(cCol2) << "> label: "  << inNum.label << endl
     << setw(cCol1) << " " << setw(cCol2) << "> value: "  << inNum.value << endl
     << setw(cCol1) << " " << setw(cCol2) << "> format: " << inNum.format << endl
     << setw(cCol1) << " " << setw(cCol2) << "> min: "    << inNum.min << endl
     << setw(cCol1) << " " << setw(cCol2) << "> max: "    << inNum.max << endl
     << setw(cCol1) << " " << setw(cCol2) << "> step: "   << inNum.step << endl;
  return os;
}

ostream & operator<<(ostream & os, const INumberVectorProperty & inNumVecProp) {
  const size_t cCol1 = IndiUtilsT::sColNum, cCol2 = 20;
  COMMON_VECPROP_PRINTER(os, "INumberVectorProperty", inNumVecProp);
  os << setw(cCol1) << " " << setw(cCol2) << "> timeout: "    << inNumVecProp.timeout << endl
     << setw(cCol1) << " " << setw(cCol2) << "> permission: " << IndiUtilsT::asStr(inNumVecProp.p) << endl;
  COMMON_PROP_PRINTER(os, INumber, inNumVecProp, nnp, np);  
  return os;
}


/**
 * Switch ostream operator
 */
ostream & operator<<(ostream & os, const ISwitch & inSwitch) {
  const size_t cCol1 = IndiUtilsT::sColNum, cCol2 = 20;
  os << setw(cCol1) << " " << setw(cCol2) << "Switch"    << left << endl
     << setw(cCol1) << " " << setw(cCol2) << "> name: "  << inSwitch.name << endl
     << setw(cCol1) << " " << setw(cCol2) << "> label: " << inSwitch.label << endl
     << setw(cCol1) << " " << setw(cCol2) << "> state: " << IndiUtilsT::asStr(inSwitch.s) << endl; 
  return os;
}

ostream & operator<<(ostream & os, const ISwitchVectorProperty & inSwitchVecProp) {
  const size_t cCol1 = IndiUtilsT::sColNum, cCol2 = 20;
  COMMON_VECPROP_PRINTER(os, "ISwitchVectorProperty", inSwitchVecProp);
  os << setw(cCol1) << " " << setw(cCol2) << "> timeout: "    << inSwitchVecProp.timeout << endl
     << setw(cCol1) << " " << setw(cCol2) << "> permission: " << IndiUtilsT::asStr(inSwitchVecProp.p) << endl
     << setw(cCol1) << " " << setw(cCol2) << "> rule: "       << IndiUtilsT::asStr(inSwitchVecProp.r) << endl;
  COMMON_PROP_PRINTER(os, ISwitch, inSwitchVecProp, nsp, sp);
  return os;
}


/**
 * Text ostream operator
 */
ostream & operator<<(ostream & os, const IText & inText) {
  const size_t cCol1 = IndiUtilsT::sColNum, cCol2 = 20;
  os << setw(cCol1) << " " << setw(cCol2) << "Text"      << left << endl
     << setw(cCol1) << " " << setw(cCol2) << "> name: "  << inText.name << endl
     << setw(cCol1) << " " << setw(cCol2) << "> label: " << inText.label << endl
     << setw(cCol1) << " " << setw(cCol2) << "> text: "  << inText.text << endl;
  return os;
}

ostream & operator<<(ostream & os, const ITextVectorProperty & inTextVecProp) {
  const size_t cCol1 = IndiUtilsT::sColNum, cCol2 = 20;
  COMMON_VECPROP_PRINTER(os, "ITextVectorProperty", inTextVecProp);
  os << setw(cCol1) << " " << setw(cCol2) << "> timeout: "    << inTextVecProp.timeout << endl
     << setw(cCol1) << " " << setw(cCol2) << "> permission: " << IndiUtilsT::asStr(inTextVecProp.p) << endl;
  COMMON_PROP_PRINTER(os, IText, inTextVecProp, ntp, tp);  
  return os;
}


/**
 * Light ostream operator
 */
ostream & operator<<(ostream & os, const ILight & inLight) {
  const size_t cCol1 = IndiUtilsT::sColNum, cCol2 = 20;
  os << setw(cCol1) << " " << setw(cCol2) << "Light"     << left << endl
     << setw(cCol1) << " " << setw(cCol2) << "> name: "  << inLight.name << endl
     << setw(cCol1) << " " << setw(cCol2) << "> label: " << inLight.label << endl
     << setw(cCol1) << " " << setw(cCol2) << "> light: " << IndiUtilsT::asStr(inLight.s) << endl;
  return os;
}

ostream & operator<<(ostream & os, const ILightVectorProperty & inLightVecProp) {
  const size_t cCol1 = IndiUtilsT::sColNum, cCol2 = 20;
  COMMON_VECPROP_PRINTER(os, "ILightVectorProperty", inLightVecProp);
  COMMON_PROP_PRINTER(os, ILight, inLightVecProp, nlp, lp);  
  return os;
}


/**
 * BLOB ostream operator
 */
ostream & operator<<(ostream & os, const IBLOB & inBLOB) {
  const size_t cCol1 = IndiUtilsT::sColNum, cCol2 = 20;
  os << setw(cCol1) << " " << setw(cCol2) << "BLOB"              << left << endl
     << setw(cCol1) << " " << setw(cCol2) << "> name: "          << inBLOB.name << endl
     << setw(cCol1) << " " << setw(cCol2) << "> label: "         << inBLOB.label << endl
     << setw(cCol1) << " " << setw(cCol2) << "> format: "        << inBLOB.format << endl
     << setw(cCol1) << " " << setw(cCol2) << "> #bytes: "        << inBLOB.bloblen << endl
     << setw(cCol1) << " " << setw(cCol2) << "> #uncompressed: " << inBLOB.size << endl
     << setw(cCol1) << " " << setw(cCol2) << "> blobPtr: "       << inBLOB.blob << endl;
  return os;
}

ostream & operator<<(ostream & os, const IBLOBVectorProperty & inBLOBVecProp) {
  const size_t cCol1 = IndiUtilsT::sColNum, cCol2 = 20;
  COMMON_VECPROP_PRINTER(os, "IBLOBVectorProperty", inBLOBVecProp);
  os << setw(cCol1) << " " << setw(cCol2) << "> timeout: "    << inBLOBVecProp.timeout << endl
     << setw(cCol1) << " " << setw(cCol2) << "> permission: " << IndiUtilsT::asStr(inBLOBVecProp.p) << endl;
  COMMON_PROP_PRINTER(os, IBLOB, inBLOBVecProp, nbp, bp);  
  return os;
}


#define CASE_INDI_PROP(__type__, __name__)				\
  case INDI_##__type__: {						\
    I##__name__##VectorProperty * vp = static_cast<I##__name__##VectorProperty *>(prop->getProperty());\
    if (vp)								\
      os << *vp;							\
    else								\
      os << "<##__name__##-empty>";					\
    break;								\
  }									\
  

// Overloading Indi-Property output operator 
ostream & operator<<(ostream & os, const INDI::Property & inProp) {
  const size_t cCol1 = IndiUtilsT::sColNum, cCol2 = 20;
  INDI::Property * prop = const_cast<INDI::Property*> (& inProp);

  os << setw(cCol1) << " " << setw(cCol2) << "Vector-Property"  << left << endl
     << setw(cCol1) << " " << setw(cCol2) << "> propName: "     << prop->getName() << endl
     << setw(cCol1) << " " << setw(cCol2) << "> propType: "     << IndiUtilsT::asStr(prop->getType()) << endl
     << setw(cCol1) << " " << setw(cCol2) << "> propState: "    << IndiUtilsT::asStr(prop->getState()) << endl
     << setw(cCol1) << " " << setw(cCol2) << "> propPerm: "     << IndiUtilsT::asStr(prop->getPermission()) << endl
     << setw(cCol1) << " " << setw(cCol2) << "> propLabel: "    << prop->getLabel() << endl
     << setw(cCol1) << " " << setw(cCol2) << "> groupLabel: "   << prop->getGroupName() << endl
     << setw(cCol1) << " " << setw(cCol2) << "> deviceName: "   << prop->getDeviceName() << endl
     << setw(cCol1) << " " << setw(cCol2) << "> isDynamic: "    << prop->isDynamic() << endl
     << setw(cCol1) << " " << setw(cCol2) << "> isRegistered: " << prop->getRegistered() << endl
     << setw(cCol1) << " " << setw(cCol2) << "> baseDevice: "   << prop->getBaseDevice() << endl
     << setw(cCol1) << " " << setw(cCol2) << "> propValues: "   << endl;

  // TODO: Maybe write a macro...
  IndiUtilsT::sColNum += IndiUtilsT::sColNumDiff;
  switch(prop->getType()) {
    CASE_INDI_PROP(NUMBER, Number);
    CASE_INDI_PROP(SWITCH, Switch);
    CASE_INDI_PROP(TEXT, Text);
    CASE_INDI_PROP(LIGHT, Light);
    CASE_INDI_PROP(BLOB, BLOB);
  default: {
    os << "<?>" << endl;
    break;
  }
  } // end switch
  IndiUtilsT::sColNum -= IndiUtilsT::sColNumDiff;

  return os;
}
