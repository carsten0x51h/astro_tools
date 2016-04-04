%module _libastrotoolspythonwrap
%include "std_string.i"
%{
   #include "indi/indi_client_tmpl.hpp"
   #include "indi/indi_camera_client.hpp"
%}

namespace cimg_library {} 
namespace CCfits {} 
namespace boost {} 


/* %include "indi/indi_client_tmpl.hpp" */
/* class IndiClientTmplT { */
/*  public: */
/*   IndiClientTmplT(); */
/*   IndiClientTmplT(const string & inHostname, int inPort); */
/*   inline void connectDevice(const string & inDeviceName, UpdatePolicyT::TypeE updPolicy = UpdatePolicyT::BLOCKING); */

/* }; */

%include "indi/indi_camera_client.hpp"

/* class IndiCameraClientT { */
/* public: */
/*   IndiCameraClientT(); */
/*   const string & getDeviceName() const; */
/*   double getTemperature() const; */
/* }; */


