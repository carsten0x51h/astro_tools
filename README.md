astro_tools
===========

Helpful astronomy tools.


What? Linux software to find focus
Why? Because FocusMax requires Maxim DL or CCD... software which is quite expensive.

Dependencies
-qt4 (take examples from prev. focuser project)
-qwt for displaying diagrams / charts (see http://qwt.sourceforge.net/)
-INDI (see indi_test and indi_checkout in workspace)
	-Atik CCD driver
	-Moonlight focuser
-gsl
	-Use for Non-Linear-Curve Fitting (see http://en.wikipedia.org/wiki/Levenberg%E2%80%93Marquardt_algorithm), get sigma parameter from pixel value
	-Also see http://irtfweb.ifa.hawaii.edu/Facility/xgfit/xgfit.pdf

-CCfits (use for loading and saving fits files)

-CIMG http://cimg.sourceforge.net/ (convlution as well as SNR calculation, also support very simple segmentation (see http://sourceforge.net/p/cimg/discussion/334630/thread/c9d78102/))
      -Calc of PSNR: http://en.wikipedia.org/wiki/Peak_signal-to-noise_ratio
      -Hough transformation: https://github.com/coupdair/CImg.Program/blob/master/hough/CImg.Hough_transform2d.cpp

-MAYBE: vigra image processing library (see vigra_smooth_test and libvigra git in workspace), for additional filtering of images, PSF convolution etc..
 


Process
A) Configuration and setup
-Configure camera and focuser (test connection, manual control of camera and focuser by UI, set exposure time) - TODO: What about filter wheel? At least remember user to set Filterwheel to Focus
-Take at least one big picture, only keep last one in memory
-Calculate infos for whole image (size, SNR, ...)
-Select a region which contains a suitable star (or better: select one automatically?)


A1) Automatic star recognition
    -For star recognition 1.) do a gaussian blur, then do a thresholding! (use histogram)... How to determine upper and lowe boundary?! Max is 65535, But min?
    -Use the CIMG Hough transform?

B) Find focus
-Take a picture of the selected region
-Calculate infos for region (SNR, min, max pixel value, ...)
-Try to recognize a star (convolution with PSF?)
-Calculate sigma / FWMH of the star
-Save FWMH in history
      -TODO: We may also consider calculating the Half-Flux Diameter: http://www.cyanogen.com/help/maximdl/Half-Flux.htm, http://www005.upp.so-net.ne.jp/k_miyash/occ02/halffluxdiameter/halffluxdiameter_en.html, http://users.bsdwebsolutions.com/~larryweber/ITSPaper.htm
-Modify focus accordingly (need good values for focus control - regulation) (we may do multiple iterations without modifying the focuser --> average FWMH...)
-Has minimum FWMH been reached with most accurate stepper control? -> Exit!
-Else: Continue with B)


C) Software architecture
-Qt application -> event based
-Software is INDI client -> also event based -> GUI updates only via Qt events --> not threadsafe

D) User interface design
   -First directive: Keep it simple! Do not overload with unimportant options and details! User wants to focus the camera - nothing else!
   -Hence the user needs:
   	  -Connection to camera and focuser + status info if connected
	  -Simple manual control of camera and focuser is required
	  -Select a star(?)
	  -Start focusing button
	  -User wants to see progress of automatic focus finding (FWMH / HFD over time) +  (V-curve /focus pos vs FWHM / HFD)
	  -User wants to get informed if best focus has been found, and of FWMH value, HFD-Value and MAX brightness value
	 





Focus process
-------------

   -Temperature           star-search-window size & pos
   -Exposure time            (previously determined)           
   -Binning                         (x,y,w,h)                   HFD r_out   star-window size
        |                               |                           |              |
        V                               V                           V              V
|---------------|             |------------------|              |----------------------|                |--------------------------------|         |-----------------|               |-------------|
|               |    Image    |                  |   Subimage   |                      | FWHM* & HFD*   |  Determine focus control data  |   dL    |                 | USB protocol  |             |
|    Camera     |------------>| Extract SubImage |------------->| Determine FWHM & HFD |--------------->|  (# of steps to go up or down) |<------->|   INDI Focus    |-------------->| Focus Motor |
|               |             |                  |              |                      |                |     dL = f(FWHM, HFD)          |         | Stepper Control |               |             |
|               |             |                  |              |                      |                |   Stop if minimum reached      |         |                 |               |             |
|---------------|             |------------------|              |----------------------|                |--------------------------------|         |-----------------|               |-------------|
       ^                                                                                                    |                 |                             |
       |                                                                                                    |                 V                             V
       |                                        Take picture (sub-frame)                                    |        Display HFD & FWHM,                 Position
       |----------------------------------------------------------------------------------------------------|         Progress, V-Curve


*) FWHM & HFD contain standard derivation, data points etc.


Determine focus control data
----------------------------

There are two approaches:

1.) Linear "drive to"

Measured values: FWHM, HFD (Is-Values)
Cntl values: focusPos (relative focus pos (up / down))
Goal: Minimize both
Determine dL = f(FWHM, HFD), #maxIter, take multiple stars into account?
Repeat a given point of time around minimum and stop if no significant imrpovements have beene made

2.) V-Curve Approximation
-Pre-Conditions
   -Camera connected and working
   -Focuser connected and working
   -Focus filter selected (if any)
   -Focus window selected
   -A "valid" star profile in the selected region (TODO: How to say there is a valid star profile? -> use HFD / FWHM values, other values?! Shape? Flux?)

-Determine initilal dL (initial direction)
   -Take picture
   -Save FWHM & HFD
   -Moving focus up N steps
   -Take picture
   -Compre new FWHM & HFD against saved values
   -If focus improved, "up" is the right direction
   -If focus gets worse, "down" is the right direction
   -Move foucs "down" N steps

-Find rough focus (in: FWMH & HFD at least to be reached, out: success/failure, FWMH & HFD obtained)
   -1. Take picture
   -2. Save FWHM & HFD
   -3. Move M steps into prev. direction
   -4. Take picture
   -5. Repeat at 1. until new FWHM & HFD are worse than saved
   -6. Move focus in oppsosite direction by M steps

-Determine rough relative focus boundaries (in: FWHM/HFD BOUNDARY, out; dFmin [steps], dFmax [steps])
   -Determine Fmin
      -1. Take picture
      -2. Determine FWHM & HFD
      -3. _Reduce_ focus pos. by K steps, accumulate K
      -4. Take picture
      -5. Repeat 1. as long as new FWHM & HFD are < FWHM/HFD BOUNDARY
      -6. Save "Fmin" focus position (and corresponding HWHM/HFD)
      -7. Move focus back to "center" position (increase focus by Sum(K))
   -Determine Fmax
      -1. Take picture
      -2. Determine FWHM & HFD
      -3. _Increase_focus pos. by K steps, accumulate K
      -4. Take picture
      -5. Repeat 1. as long as new FWHM & HFD are < FWHM/HFD BOUNDARY
      -6. Save "Fmax" focus position (and corresponding HWHM/HFD)
      -7. Move focus back to "center" position (decrease focus by Sum(K))

-Record V-Curve (in: Fmin, Fmax, J? (TODO: Or determine dynamically? J=f(HFD and/or FWHM)? ))
   -1. Move focus to Fmin (decrease by Fmin)
   -2. Take picture
   -3. Save FWHM & HFD and current focus position (and sub image?)
   -4. If Fmax reached, continue at 7.
   -5. Else:Increase focus by J steps
   -6. Continue at 2.
   -7. Bring focus back to start pos. (Decrease focus by f_rightFmax)

-Determine optimal focus position from V-Curve(s) (requires valid V-Curve(s)) (in: V-Curves (a V-Curve is a list of relative focusPos & FWHM/HFD pairs))
   -1. Build one average V-Curve (all V-Curves must be within same relative Fmin & Fmax) (Note: Then for each focusPos there is exactly one HFD & FWHM value per V-Curve)
      -For idx = Fmin to Fmax
         -For all V-Curves
            -Sum the idx value of all V-Curves
         -Divide accumulated idx value by the number of V-Curves

   -2. Separate V-Curve in decreasing data-points and increasing data-points, minimum is to be included in both sets (TODO: Really?))
      -TODO...

   -3. Determine optimal focus position 
      NOTE: Important condition: f_decreasing_fwhm&hfd(pos) = f_increasing_fwhm&hfd(pos) == FWHM == HFD == 0!! --> pos is optimal focus position
          --> Find a_1, b_1 & a_2, b_2 of both functions so that (MSE1 + MSE2) is minimized
          --> Maybe hard to formulate in gsl?! --> We may implement this manually....



-Move to optimal focus position (in: optimal focus position)
   -TODO: Does this involve any corrections (feedback?)


Minimize MSE for two functions under condition
----------------------------------------------
--> WE MAY USE GSL MULTIFIT functionality?! (i.e. define two functions and condition?!)

-Given datasets d1(x_i), d2(x_i), N1, N2 = #entries
-Minimize MSE for two functions f1(x) & f2(x) under condition f1(x_0) = f2(x_0) = 0
-MSE = SUM_i=1..N ( [d(x_i) - f(x=x_i)]² )
-Minimize MSE by modifying f(x)=a*x + b --> Find a, b so that MSE is minimal --> MSE' = 0

   -Find MSE'
      -d(x_i) and N are constants, given
      -f(x=x_i) = a*x_i + b --> MSE(a, b) = SUM_i=1..N ( [d(x_i) - (a * x_i + b)]² )

      LGS
      -dMSE(a, b) / da = 0 
      -dMSE(a, b) / db = 0 --> a, b


   -We have two of those problems related by one condition
      -f1(x=x_0)=a_1*x+b_1 = f2(x=x_0)=a_2*x+b_2 = 0 --> a_1*x_0+b_1 = a_2*x_0+b_2 = 0 --> x_0 = -b_1 / a_1 = -b_2 / a_2
      -MSE_1 = SUM_i=1..N ( [d_1(x_i) - f1(x=x_i)]² ) = SUM_i=1..N ( [d_1(x_i) - (a_1*x_i+b_1)]² )
      -MSE_2 = SUM_i=1..M ( [d_2(x_i) - f2(x=x_i)]² ) = SUM_i=1..M ( [d_2(x_i) - (a_2*x_i+b_2)]² )

      -Minimize (MSE_1 + MSE2)




--------------------------------------------------------------------------------------------------

ServerConnectionManager
-----------------------

Purpose
-Manage the connection to 1..n INDI servers
-Load / save configuration with server entries
-Connect / disconnect to INDI server
-Manage server exit / failure
-There is one INDI Client object per server connection
-Technically (INDI bug?) the creation of a new INDI Client is required after each disconnect

Use Cases
1.) Add / remove INDI server
2.) Get list of INDI servers
3.) Get certain INDI server connection
4.) Connect / disconnect to INDI servers
5.) Load / save server configuration

Data structure
map<string /* connection name */, IndiServerConnectionT> IndiServerConnectionsT;

class IndiServerConnectionT {
      string mName;
      string mDescription;
      string mHostName;
      int mPort;
      bool mAutoConnect;
      IndiClientTmplT * mIndiClient; // 0 if not connected, otherwise != 0
}

Non-UI Interface
UC 1.) & UC 2.)
IndiServerConnectionsT & getServerConnections(inConnecionStatus = CONNECTED, DISCONNECTED, ALL = CONNECTED | DISCONNECTED) const // Use default STL map interface to add / remoce / iterate connections

UC 3.)
IndiServerConnectionT * getServerConnection(const string & inServerConnectionName), returns 0 if connection does not exist

UC 4.)
void connect(const string & inServerConnectionName)
void disconnect(const string & inServerConnectionName)
void connectAll(inConnectPolicy = AUTOCONNECT_FLAG_ONLY | AUTOCONNECT_ALL)
void disconnectAll()
bool isConnected() const { return mIndiClient != 0); }

UC 5.)
void loadConfiguration(const string & inFileName) 
void saveConfiguration(const string & inFileName)


Idea: We put both - the ServerConnectionManagerT and the DeviceManagerT instances into one static program instance "AstroToolsT" (or a sigleton)


DeviceManager
-------------

Purpose
-Program wide organization and management of (INDI) device connection
-More specific, allow access to devices from all parts of the program
-Provides a list of available INDI devices (names + types -> TODO: Is there a way to get the INDI device type?)
-Handles connect and disconnect of devices (i.e. error handling when device disconnects or is not available)
-Maps certain INDI devices to a "roles"
-Load and save the role mapping in a configuration file


Aliases
-An alias is a kind of variable which provides access to a certain device under a given name
-An alias is for example "imaging camera", "guiding camera", "guiding focuser", "imaging focuser", "imaging filter wheel", "guiding filter wheel", "mount"...
-Technically, an alias consists of a name (like "imaging camera") and a device type which is accepted for this alias (e.g. camera, focuser, filterwheel, mount, ...)
-Each alias can be assigned 0..1 devices i.e. an alias can be empty
-There are n devices available and 0..m (m<=n) of them are coupled to aliases
-Each device can only be assigned to one alias at a time (?? Really ??)
-There are certain __default__ aliases defined which are coupled to __fixed__ names so that they can be used for example in python scripts but also hard coded in the C++ code
-In addition the user can define additional aliases (alias name and device type) which can then be used in additional python scripts


Use Cases
1.) Add / remove user alias
2.) Get device alias(es) class
3.) Get list of available devices (available does not say if device is connected or not)
4.) Get device for alias
5.) Couple / decouple alias and device
6.) Load / save configuration

Non-UI Interface

UC 1.)
void addUserDeviceAlias(string inDeviceAliasName, inAcceptDeviceType (asString?!?! or INDI enum? or own enum type?) ) throws DeviceAliasAlreadyExists
void removeUserDeviceAlias(string inDeviceAliasName) throws DeviceAliasDoesNotExist

UC 2.)
const DeviceAliasT * getDeviceAliasByName(string inDeviceAliasName) const, returns 0 if role does not exist
bool hasDeviceAlias(string inDeviceAliasName) const
bool hasDeviceAlias(const DeviceAliasT * inDeviceAlias) const
list<DeviceAliasT*> getDeviceAliases(roleType = DEFAULT, USER, ALL = DEFAULT | USER)

UC 3.)
list<INDI::Device*> getDevices(inIndiServerName = "" /*default from all servers*/, enum inDeviceType = ALL ??) const;

UC 4.)
INDI::Device * getDevice(string inAliasName) const, returns 0 if no device is assigned, throws DeviceAliasDoesNotExistT if alias does not exist

UC 5.)
void coupleDeviceToAlias(INDI::Device * ???, const RoleT * inDeviceAlias)
void decoupleDeviceAliasPair(INDI::Device * ???)
void decoupleDeviceAliasPair(const RoleT * inDeviceRole)

UC 6.)
void loadConfiguration(const string & inFilename) 
void saveConfiguration(const string & inFilename)







void connect(string inRoleName) throws
void disconnect(string inRoleName) throws ?



bool isConnected(string inRoleName)




map DeviceRoleT <-> DeviceTypeT --> then we do not need a role class?!
map DeviceRoleT <-> Device 





Major design change: Multiple INDI servers at once!!!
-----------------------------------------------------

           0..N Devices (connected / unconnected ------ Device name 0..N <-----------> 1 device-alias
                   / asyncronous new /remove)
                               |
                               |
                               |
     IndiServer <-------- IndiClientTmpl (Contains event register functions)----\
                                                                                 \
                                                                                  IndiClientManagerT (map)
                                                                                 /
     IndiServer <-------- IndiClientTmpl (Contains event register functions)----/
                               |
                               |
                               |
           0..M Devices (connected / unconnected ------ Device name 0..N <-----------> 1 device-alias
                   / asyncronous new /remove)

libindicpp
----------
-> for each device type (Focuser, Camera, ...) supported by the "C++-indi wrapper" there is one class which provides the access methods (and listeners?)
-> The class (or traits class) holds the set of the generally C++ supported properties and vector properties (which map exactly to those of INDI)
-> Furthermore it cares about the mapping to the libindi device properties and vector properties
   -> load and save mappings when starting/exiting program or when user wants, also save currently selected mapping-> serialization
   -> map configration to device by unique device key (combination of indi_server_host, port and device name -> required if there are multiple indi servers)
-> Need a default mapping

-> Additional use cases:
   -> Device may have additional properties or vector proerties which are not part of the "Default" indi properties
      -> User may want to access those proerties in his python scripts
   -> Device developers may replace default properties by their own property names (which are no default which do not exist in the default values)
      -> User wants to map the "custom" property to a default property name (and save it, and want it to be loaded later again)
   -> Device deveopers may for example invert X and Y properties
      -> User wants to map X to Y and Y to Y manually edit property mapping

---->  DeviceManager (Abstract devices from IndiServers (required?!), manage aliases)




Architecture

Non-GUI part (static instance or singleton) which exists once inside the process. Furthermore, there exists a GUI (qt4) part which can be used to control the DeviceManager via a UI. In addition, the DeviceManager can be controlled via the python interface. The DeviceManager holds one instance of the INDI client. However, the INDI client instance is not encapsulated because it is required to react on certain asyncronous events (register listener etc.) (TODO: Discuss?! Should the DeviceManagr actually hold the INDI client instance?? Maybe not, because the DeviceManager only manages devices... not the client itself?!). 















INDI Data Layers
----------------

Host:                                                          ----------- Computer ------------                                                                  Computer
                                                              /                                 \                                                                  /    \
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
                                                            /                                     \                                                              /        \
Server:                                           IndiServer : port                        IndiServer : port                                                  IndiServer : port
                                                /                  \                              |                                                          /                 \
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
                                              /                      \                            |                                                        /                     \
Device:                                    Device                   Device                      Device                                                 Device                    Device
                                          /      \              |                         /   \                                                  /    \                    /    \
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
                                        /          \                  |                       /       \                                              /        \                /        \    
                                       /            \              |                      /         \                                            /          \              /          \
                                      /              \               |                     /           \                                          /            \            /            \
VecProp:                          VecProp   ...    VecProp         VecProp                VecProp ... VecProp                                   VecProp ... VecProp      VecProp  ...   VecProp
(different type)                 /      \          /     \            |                  /       \       |                                      /     \        |         /    \         /     \
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
                                |       |         |      |            |                  |       |       |                                     |       |       |        |      |        |     |
Prop:                         Prop ... Prop    Prop ... Prop        Prop               Prop ... Prop   Prop                                  Prop ... Prop    Prop   Prop ... Prop   Prop ... Prop
(different types)


-> Generic UseCase: Read and write values to props (synchronous read & write + asynchronous read (notification))!
-> Register to property changes of devices (callback) (publish substrice / listener concept).
-> Vector properties / properties (?) have different types (Text, Switch, Light, Number, BLOB)
-> In order to uniquely get/set a property value, the following information is required:
    -hostname : port (one client per indi server and port) (e.g. localhost:7141)
    -device name (e.g. "Atik383L+")
    -vector property name (e.g. CONNECTION)
    -property name

-> Generic UseCase: In my app I want to use _fixed_ property names - fixed because they will be used in many different code parts.
-> This is especially important because: libindi may change, device driver may change, device driver may use no-standard names...
-> In order to have compiler type checking (and maybe to improve performance, better design), use enum types in addition to string representation of vecProp and prop names.
-> The indi (vec)prop names of _each indi device_ have to be mapped to the _fixed_ vecProp and prop names in my app.
-> The app should remember the users mapping for each device (and server!) - there could be two devices with the same name on two different servers)?!?!?





INDI Properties
---------------


basedevice.h (class INDI::BaseDevice) --> DefaultDevice...

    /** \brief Return a list of all properties in the device.
    */
    std::vector<INDI::Property *> * getProperties() { return &pAll; }


See indibase.h
/*! INDI property type */
typedef enum
{
    INDI_NUMBER, /*!< INumberVectorProperty. */
    INDI_SWITCH, /*!< ISwitchVectorProperty. */
    INDI_TEXT,   /*!< ITextVectorProperty. */
    INDI_LIGHT,  /*!< ILightVectorProperty. */
    INDI_BLOB,    /*!< IBLOBVectorProperty. */
    INDI_UNKNOWN
} INDI_TYPE;


baseclient.h
    enum { INDI_DEVICE_NOT_FOUND=-1, INDI_PROPERTY_INVALID=-2, INDI_PROPERTY_DUPLICATED = -3, INDI_DISPATCH_ERROR=-4 };


Devices - see baseclient.h
    /** \param deviceName Name of device to search for in the list of devices owned by INDI server,
         \returns If \e deviceName exists, it returns an instance of the device. Otherwise, it returns NULL.
    */
    INDI::BaseDevice * getDevice(const char * deviceName);

    /** \returns Returns a vector of all devices created in the client.
    */
    const vector<INDI::BaseDevice *> & getDevices() const { return cDevices; }

    /** \return Returns the device name */
    const char *getDeviceName();


 Send data to INDI server - also baseclient.h:
    /** \brief Send new Text command to server */
    void sendNewText (ITextVectorProperty *pp);
    /** \brief Send new Text command to server */
    void sendNewText (const char * deviceName, const char * propertyName, const char* elementName, const char *text);
    /** \brief Send new Number command to server */
    void sendNewNumber (INumberVectorProperty *pp);
    /** \brief Send new Number command to server */
    void sendNewNumber (const char * deviceName, const char *propertyName, const char* elementName, double value);
    /** \brief Send new Switch command to server */
    void sendNewSwitch (ISwitchVectorProperty *pp);
    /** \brief Send new Switch command to server */
    void sendNewSwitch (const char * deviceName, const char *propertyName, const char *elementName);

    /** \brief Send opening tag for BLOB command to server */
    void startBlob( const char *devName, const char *propName, const char *timestamp);
    /** \brief Send ONE blob content to server */
    void sendOneBlob( const char *blobName, unsigned int blobSize, const char *blobFormat, void * blobBuffer);
    /** \brief Send closing tag for BLOB command to server */
    void finishBlob();



Device allows access to property values - basedevice.h
    /** \return Return vector number property given its name */
    INumberVectorProperty * getNumber(const char *name);
    /** \return Return vector text property given its name */
    ITextVectorProperty * getText(const char *name);
    /** \return Return vector switch property given its name */
    ISwitchVectorProperty * getSwitch(const char *name);
    /** \return Return vector light property given its name */
    ILightVectorProperty * getLight(const char *name);
    /** \return Return vector BLOB property given its name */
    IBLOBVectorProperty * getBLOB(const char *name);




Events:
/**
 * \class INDI::BaseMediator
   \brief Meditates event notification as generated by driver and passed to clients.
*/
class INDI::BaseMediator
{
public:

    /** \brief Emmited when a new device is created from INDI server.
        \param dp Pointer to the base device instance
    */
    virtual void newDevice(INDI::BaseDevice *dp)  =0;

    /** \brief Emmited when a new property is created for an INDI driver.
        \param property Pointer to the Property Container

    */
    virtual void newProperty(INDI::Property *property)  =0;


    /** \brief Emmited when a property is deleted for an INDI driver.
        \param property Pointer to the Property Container to remove.

    */
    virtual void removeProperty(INDI::Property *property)  =0;


    /** \brief Emmited when a new BLOB value arrives from INDI server.
        \param bp Pointer to filled and process BLOB.
    */
    virtual void newBLOB(IBLOB *bp) =0;

    /** \brief Emmited when a new switch value arrives from INDI server.
        \param svp Pointer to a switch vector property.
    */
    virtual void newSwitch(ISwitchVectorProperty *svp) =0;

    /** \brief Emmited when a new number value arrives from INDI server.
        \param nvp Pointer to a number vector property.
    */
    virtual void newNumber(INumberVectorProperty *nvp) =0;

    /** \brief Emmited when a new text value arrives from INDI server.
        \param tvp Pointer to a text vector property.
    */
    virtual void newText(ITextVectorProperty *tvp) =0;

    /** \brief Emmited when a new light value arrives from INDI server.
        \param lvp Pointer to a light vector property.
    */
    virtual void newLight(ILightVectorProperty *lvp) =0;

    /** \brief Emmited when a new message arrives from INDI server.
        \param dp pointer to the INDI device the message is sent to.
        \param messageID ID of the message that can be used to retrieve the message from the device's messageQueue() function.
    */
    virtual void newMessage(INDI::BaseDevice *dp, int messageID) =0;

    /** \brief Emmited when the server is connected.
    */
    virtual void serverConnected() =0;

    /** \brief Emmited when the server gets disconnected.
        \param exit_code 0 if client was requested to disconnect from server. -1 if connection to server is terminated due to remote server disconnection.
    */
    virtual void serverDisconnected(int exit_code) =0;








-->>>>>
-> In baseclient I can use getDevices() to get a list of all devices (BaseDevice Ptr) or I can use I can use getDevice(name) to get a BaseDevice Ptr.
-> With this device ptr I can get the list of all properties supported by the device -> vector of Property Ptr. -> no event based design required
-> With the property ptr. I can get the property name and the property type (INDI_TYPE). 
-> With access functions of device (e.g. ISwitchVectorProperty * getSwitch(const char *name)) and property name I can get property value at any time (I don't have to wait for an INDI event). -> May avoid event based design.. or at least not require it!



Default device:


CONNECTION::CONNECT -> enum value
CONNECTION::DISCONNECT -> enum value
asStr(CONNECTION) -> "CONNECTION"
asStr(CONNECTION::CONNECT) -> "CONNECT"
asStr(CONNECTION::DISCONNECT) -> "DISCONNECT"


enum class CONNECTION { CONNECT, DISCONNECT } ;



struct DefaultDeviceTraitsT {

  struct VecPropNameT {
    enum TypeE {
      CONNECTION,
      DRIVER_INFO,
      DEBUG,
      SIMULATION,
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
    
    static TypeE asType(const char * typeName) {
      for (size_t i=0; i < _Count; ++i) {
    	TypeE type = static_cast<TypeE> (i);
    	if (! strcmp(typeName, asStr(type))) {
    	  return type;
    	}
      }
      return _Count;
    }

    // TODO: Maybe specifying the type to string translation is enough!
    static const char * asStr(const TypeE & inType) {
      switch (inType) {
      case CONNECTION: return "CONNECTION";
      case DRIVER_INFO: return "DRIVER_INFO";
      case DEBUG: return "DEBUG";
      case SIMULATION: return "SIMULATION";
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


  struct PropNameT {
    enum TypeE {
      CONNECT,
      DISCONNECT,

      NAME,
      EXEC,
      VERSION,

      ENABLE,
      DISABLE,

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

      COMPRESS,
      RAW,

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

    // TODO: We may put this into a macro...
    static TypeE asType(const char * typeName) {
      for (size_t i=0; i < _Count; ++i) {
    	TypeE type = static_cast<TypeE> (i);
    	if (! strcmp(typeName, asStr(type))) {
    	  return type;
    	}
      }
      return _Count;
    }
    
    static const char * asStr(const TypeE & inType) {
      switch (inType) {
      case CONNECT: return "CONNECT";
      case DISCONNECT: return "DISCONNECT";

      case NAME: return "NAME";
      case EXEC: return "EXEC";
      case VERSION: return "VERSION";

      case ENABLE: return "ENABLE";
      case DISABLE: return "DISABLE";

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

      case COMPRESS: return "COMPRESS";
      case RAW: return "RAW";

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
   

    // For crosschecking
    static VecPropNameT::TypeE getVecProp(const PropNameT::TypeE & inType) {
      switch(inType) {
      case CONNECT:
      case DISCONNECT:
	return VecPropNameT::CONNECTION;

      case NAME:
      case EXEC:
      case VERSION:
	return VecPropNameT::DRIVER_INFO;

      case ENABLE:
      case DISABLE:
        return VecPropNameT::DEBUG;

      case ENABLE:
      case DISABLE:
        return VecPropNameT::SIMULATION; --> PROBLEM!!

      case CCD_MAX_X:
      case CCD_MAX_Y:
      case CCD_PIXEL_SIZE:
      case CCD_PIXEL_SIZE_X:
      case CCD_PIXEL_SIZE_Y:
      case CCD_BITSPERPIXEL:
	return VecPropNameT::CCD_INFO;

      case X:
      case Y:
      case WIDTH:
      case HEIGHT:
	return VecPropNameT::CCD_FRAME;

      case HOR_BIN:
      case VER_BIN:
	return VecPropNameT::CCD_BINNING;

      case CCD_EXPOSURE_VALUE:
	return VecPropNameT::CCD_EXPOSURE;

      case ABORT:
	return VecPropNameT::CCD_ABORT_EXPOSURE;

      case CCD_TEMPERATURE_VALUE:
	return VecPropNameT::CCD_TEMPERATURE;

      case COMPRESS:
      case RAW:
	return VecPropNameT::CCD_COMPRESSION;

      case CCD1:
	return VecPropNameT::CCD1;

      case FRAME_LIGHT:
      case FRAME_BIAS:
      case FRAME_DARK:
      case FRAME_FLAT:
	return VecPropNameT::CCD_FRAME_TYPE;

      case ENABLE:
      case DISABLE:
	return VecPropNameT::CCD_RAPID_GUIDE;

      case ACTIVE_TELESCOPE:
      case ACTIVE_FOCUSER:
	return VecPropNameT::ACTIVE_DEVICES;

      case DISCONNECT_COOLER:
      case CONNECT_COOLER:
	return VecPropNameT::COOLER_CONNECTION;

      case SHUTTER_ON:
      case SHUTTER_OFF:
	return VecPropNameT::SHUTTER_CONNECTION;

      default:
	return VecPropNameT::_Count;
      }
    }
  }; // end struct
};



    IUFillSwitch(&SimulationS[0], "ENABLE", "Enable", ISS_OFF);
    IUFillSwitch(&SimulationS[1], "DISABLE", "Disable", ISS_ON);
    IUFillSwitchVector(&SimulationSP, SimulationS, NARRAY(SimulationS), getDeviceName(), "SIMULATION", "Simulation", "Options", IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

    IUFillSwitch(&ConfigProcessS[0], "CONFIG_LOAD", "Load", ISS_OFF);
    IUFillSwitch(&ConfigProcessS[1], "CONFIG_SAVE", "Save", ISS_OFF);
    IUFillSwitch(&ConfigProcessS[2], "CONFIG_DEFAULT", "Default", ISS_OFF);
    IUFillSwitchVector(&ConfigProcessSP, ConfigProcessS, NARRAY(ConfigProcessS), getDeviceName(), "CONFIG_PROCESS", "Configuration", "Options", IP_RW, ISR_1OFMANY, 0, IPS_IDLE);



CCD Camera
    DefaultDevice::initProperties();   //  let the base class flesh in what it wants

    // CCD Temperature
    IUFillNumber(&TemperatureN[0], "CCD_TEMPERATURE_VALUE", "Temperature (C)", "%5.2f", -50.0, 50.0, 0., 0.);
    IUFillNumberVector(&TemperatureNP, TemperatureN, 1, getDeviceName(), "CCD_TEMPERATURE", "Temperature", MAIN_CONTROL_TAB, IP_RW, 60, IPS_IDLE);

    // PRIMARY CCD Init

    IUFillNumber(&PrimaryCCD.ImageFrameN[0],"X","Left ","%4.0f",0,1392.0,0,0);
    IUFillNumber(&PrimaryCCD.ImageFrameN[1],"Y","Top","%4.0f",0,1040,0,0);
    IUFillNumber(&PrimaryCCD.ImageFrameN[2],"WIDTH","Width","%4.0f",0,1392.0,0,1392.0);
    IUFillNumber(&PrimaryCCD.ImageFrameN[3],"HEIGHT","Height","%4.0f",0,1392,0,1392.0);
    IUFillNumberVector(PrimaryCCD.ImageFrameNP,PrimaryCCD.ImageFrameN,4,getDeviceName(),"CCD_FRAME","Frame",IMAGE_SETTINGS_TAB,IP_RW,60,IPS_IDLE);

    IUFillSwitch(&PrimaryCCD.FrameTypeS[0],"FRAME_LIGHT","Light",ISS_ON);
    IUFillSwitch(&PrimaryCCD.FrameTypeS[1],"FRAME_BIAS","Bias",ISS_OFF);
    IUFillSwitch(&PrimaryCCD.FrameTypeS[2],"FRAME_DARK","Dark",ISS_OFF);
    IUFillSwitch(&PrimaryCCD.FrameTypeS[3],"FRAME_FLAT","Flat",ISS_OFF);
    IUFillSwitchVector(PrimaryCCD.FrameTypeSP,PrimaryCCD.FrameTypeS,4,getDeviceName(),"CCD_FRAME_TYPE","Frame Type",IMAGE_SETTINGS_TAB,IP_RW,ISR_1OFMANY,60,IPS_IDLE);

    IUFillNumber(&PrimaryCCD.ImageExposureN[0],"CCD_EXPOSURE_VALUE","Duration (s)","%5.2f",0,36000,0,1.0);
    IUFillNumberVector(PrimaryCCD.ImageExposureNP,PrimaryCCD.ImageExposureN,1,getDeviceName(),"CCD_EXPOSURE","Expose",MAIN_CONTROL_TAB,IP_RW,60,IPS_IDLE);

    IUFillSwitch(&PrimaryCCD.AbortExposureS[0],"ABORT","Abort",ISS_OFF);
    IUFillSwitchVector(PrimaryCCD.AbortExposureSP,PrimaryCCD.AbortExposureS,1,getDeviceName(),"CCD_ABORT_EXPOSURE","Expose Abort",MAIN_CONTROL_TAB,IP_RW,ISR_1OFMANY,60,IPS_IDLE);

    IUFillNumber(&PrimaryCCD.ImageBinN[0],"HOR_BIN","X","%2.0f",1,4,1,1);
    IUFillNumber(&PrimaryCCD.ImageBinN[1],"VER_BIN","Y","%2.0f",1,4,1,1);
    IUFillNumberVector(PrimaryCCD.ImageBinNP,PrimaryCCD.ImageBinN,2,getDeviceName(),"CCD_BINNING","Binning",IMAGE_SETTINGS_TAB,IP_RW,60,IPS_IDLE);

    IUFillNumber(&PrimaryCCD.ImagePixelSizeN[0],"CCD_MAX_X","Resolution x","%4.0f",1,16000,0,1392.0);
    IUFillNumber(&PrimaryCCD.ImagePixelSizeN[1],"CCD_MAX_Y","Resolution y","%4.0f",1,16000,0,1392.0);
    IUFillNumber(&PrimaryCCD.ImagePixelSizeN[2],"CCD_PIXEL_SIZE","Pixel size (um)","%5.2f",1,40,0,6.45);
    IUFillNumber(&PrimaryCCD.ImagePixelSizeN[3],"CCD_PIXEL_SIZE_X","Pixel size X","%5.2f",1,40,0,6.45);
    IUFillNumber(&PrimaryCCD.ImagePixelSizeN[4],"CCD_PIXEL_SIZE_Y","Pixel size Y","%5.2f",1,40,0,6.45);
    IUFillNumber(&PrimaryCCD.ImagePixelSizeN[5],"CCD_BITSPERPIXEL","Bits per pixel","%3.0f",8,64,0,8);
    IUFillNumberVector(PrimaryCCD.ImagePixelSizeNP,PrimaryCCD.ImagePixelSizeN,6,getDeviceName(),"CCD_INFO","CCD Information",IMAGE_INFO_TAB,IP_RO,60,IPS_IDLE);

    IUFillSwitch(&PrimaryCCD.CompressS[0],"COMPRESS","Compress",ISS_ON);
    IUFillSwitch(&PrimaryCCD.CompressS[1],"RAW","Raw",ISS_OFF);
    IUFillSwitchVector(PrimaryCCD.CompressSP,PrimaryCCD.CompressS,2,getDeviceName(),"CCD_COMPRESSION","Image",IMAGE_SETTINGS_TAB,IP_RW,ISR_1OFMANY,60,IPS_IDLE);
    PrimaryCCD.SendCompressed = true;

    IUFillBLOB(&PrimaryCCD.FitsB,"CCD1","Image","");
    IUFillBLOBVector(PrimaryCCD.FitsBP,&PrimaryCCD.FitsB,1,getDeviceName(),"CCD1","Image Data",IMAGE_INFO_TAB,IP_RO,60,IPS_IDLE);

    IUFillSwitch(&PrimaryCCD.RapidGuideS[0], "ENABLE", "Enable", ISS_OFF);
    IUFillSwitch(&PrimaryCCD.RapidGuideS[1], "DISABLE", "Disable", ISS_ON);
    IUFillSwitchVector(PrimaryCCD.RapidGuideSP, PrimaryCCD.RapidGuideS, 2, getDeviceName(), "CCD_RAPID_GUIDE", "Rapid Guide", OPTIONS_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

    IUFillSwitch(&PrimaryCCD.RapidGuideSetupS[0], "AUTO_LOOP", "Auto loop", ISS_ON);
    IUFillSwitch(&PrimaryCCD.RapidGuideSetupS[1], "SEND_IMAGE", "Send image", ISS_OFF);
    IUFillSwitch(&PrimaryCCD.RapidGuideSetupS[2], "SHOW_MARKER", "Show marker", ISS_OFF);
    IUFillSwitchVector(PrimaryCCD.RapidGuideSetupSP, PrimaryCCD.RapidGuideSetupS, 3, getDeviceName(), "CCD_RAPID_GUIDE_SETUP", "Rapid Guide Setup", RAPIDGUIDE_TAB, IP_RW, ISR_NOFMANY, 0, IPS_IDLE);

    IUFillNumber(&PrimaryCCD.RapidGuideDataN[0],"GUIDESTAR_X","Guide star position X","%5.2f",0,1024,0,0);
    IUFillNumber(&PrimaryCCD.RapidGuideDataN[1],"GUIDESTAR_Y","Guide star position Y","%5.2f",0,1024,0,0);
    IUFillNumber(&PrimaryCCD.RapidGuideDataN[2],"GUIDESTAR_FIT","Guide star fit","%5.2f",0,1024,0,0);
    IUFillNumberVector(PrimaryCCD.RapidGuideDataNP,PrimaryCCD.RapidGuideDataN,3,getDeviceName(),"CCD_RAPID_GUIDE_DATA","Rapid Guide Data",RAPIDGUIDE_TAB,IP_RO,60,IPS_IDLE);

    // GUIDER CCD Init

    IUFillNumber(&GuideCCD.ImageFrameN[0],"X","Left ","%4.0f",0,1392.0,0,0);
    IUFillNumber(&GuideCCD.ImageFrameN[1],"Y","Top","%4.0f",0,1040,0,0);
    IUFillNumber(&GuideCCD.ImageFrameN[2],"WIDTH","Width","%4.0f",0,1392.0,0,1392.0);
    IUFillNumber(&GuideCCD.ImageFrameN[3],"HEIGHT","Height","%4.0f",0,1040,0,1040);
    IUFillNumberVector(GuideCCD.ImageFrameNP,GuideCCD.ImageFrameN,4,getDeviceName(),"GUIDER_FRAME","Frame",GUIDE_HEAD_TAB,IP_RW,60,IPS_IDLE);

    IUFillNumber(&GuideCCD.ImageBinN[0],"HOR_BIN","X","%2.0f",1,4,1,1);
    IUFillNumber(&GuideCCD.ImageBinN[1],"VER_BIN","Y","%2.0f",1,4,1,1);
    IUFillNumberVector(GuideCCD.ImageBinNP,GuideCCD.ImageBinN,2,getDeviceName(),"GUIDER_BINNING","Binning",GUIDE_HEAD_TAB,IP_RW,60,IPS_IDLE);

    IUFillNumber(&GuideCCD.ImagePixelSizeN[0],"CCD_MAX_X","Resolution x","%4.0f",1,16000,0,1392.0);
    IUFillNumber(&GuideCCD.ImagePixelSizeN[1],"CCD_MAX_Y","Resolution y","%4.0f",1,16000,0,1392.0);
    IUFillNumber(&GuideCCD.ImagePixelSizeN[2],"CCD_PIXEL_SIZE","Pixel size (um)","%5.2f",1,40,0,6.45);
    IUFillNumber(&GuideCCD.ImagePixelSizeN[3],"CCD_PIXEL_SIZE_X","Pixel size X","%5.2f",1,40,0,6.45);
    IUFillNumber(&GuideCCD.ImagePixelSizeN[4],"CCD_PIXEL_SIZE_Y","Pixel size Y","%5.2f",1,40,0,6.45);
    IUFillNumber(&GuideCCD.ImagePixelSizeN[5],"CCD_BITSPERPIXEL","Bits per pixel","%3.0f",8,64,0,8);
    IUFillNumberVector(GuideCCD.ImagePixelSizeNP,GuideCCD.ImagePixelSizeN,6,getDeviceName(),"GUIDER_INFO", "Guide Info",IMAGE_INFO_TAB,IP_RO,60,IPS_IDLE);

    IUFillSwitch(&GuideCCD.FrameTypeS[0],"FRAME_LIGHT","Light",ISS_ON);
    IUFillSwitch(&GuideCCD.FrameTypeS[1],"FRAME_BIAS","Bias",ISS_OFF);
    IUFillSwitch(&GuideCCD.FrameTypeS[2],"FRAME_DARK","Dark",ISS_OFF);
    IUFillSwitch(&GuideCCD.FrameTypeS[3],"FRAME_FLAT","Flat",ISS_OFF);
    IUFillSwitchVector(GuideCCD.FrameTypeSP,GuideCCD.FrameTypeS,4,getDeviceName(),"GUIDER_FRAME_TYPE","Frame Type",GUIDE_HEAD_TAB,IP_RW,ISR_1OFMANY,60,IPS_IDLE);

    IUFillNumber(&GuideCCD.ImageExposureN[0],"GUIDER_EXPOSURE_VALUE","Duration (s)","%5.2f",0,36000,0,1.0);
    IUFillNumberVector(GuideCCD.ImageExposureNP,GuideCCD.ImageExposureN,1,getDeviceName(),"GUIDER_EXPOSURE","Guide Head",MAIN_CONTROL_TAB,IP_RW,60,IPS_IDLE);

    IUFillSwitch(&GuideCCD.AbortExposureS[0],"ABORT","Abort",ISS_OFF);
    IUFillSwitchVector(GuideCCD.AbortExposureSP,GuideCCD.AbortExposureS,1,getDeviceName(),"GUIDER_ABORT_EXPOSURE","Guide Abort",MAIN_CONTROL_TAB,IP_RW,ISR_1OFMANY,60,IPS_IDLE);

    IUFillSwitch(&GuideCCD.CompressS[0],"GCOMPRESS","Compress",ISS_ON);
    IUFillSwitch(&GuideCCD.CompressS[1],"GRAW","Raw",ISS_OFF);
    IUFillSwitchVector(GuideCCD.CompressSP,GuideCCD.CompressS,2,getDeviceName(),"GUIDER_COMPRESSION","Image",GUIDE_HEAD_TAB,IP_RW,ISR_1OFMANY,60,IPS_IDLE);
    GuideCCD.SendCompressed = true;

    IUFillBLOB(&GuideCCD.FitsB,"CCD2","Guider Image","");
    IUFillBLOBVector(GuideCCD.FitsBP,&GuideCCD.FitsB,1,getDeviceName(),"CCD2","Image Data",IMAGE_INFO_TAB,IP_RO,60,IPS_IDLE);

    IUFillSwitch(&GuideCCD.RapidGuideS[0], "ENABLE", "Enable", ISS_OFF);
    IUFillSwitch(&GuideCCD.RapidGuideS[1], "DISABLE", "Disable", ISS_ON);
    IUFillSwitchVector(GuideCCD.RapidGuideSP, GuideCCD.RapidGuideS, 2, getDeviceName(), "GUIDER_RAPID_GUIDE", "Guider Head Rapid Guide", OPTIONS_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

    IUFillSwitch(&GuideCCD.RapidGuideSetupS[0], "AUTO_LOOP", "Auto loop", ISS_ON);
    IUFillSwitch(&GuideCCD.RapidGuideSetupS[1], "SEND_IMAGE", "Send image", ISS_OFF);
    IUFillSwitch(&GuideCCD.RapidGuideSetupS[2], "SHOW_MARKER", "Show marker", ISS_OFF);
    IUFillSwitchVector(GuideCCD.RapidGuideSetupSP, GuideCCD.RapidGuideSetupS, 3, getDeviceName(), "GUIDER_RAPID_GUIDE_SETUP", "Rapid Guide Setup", RAPIDGUIDE_TAB, IP_RW, ISR_NOFMANY, 0, IPS_IDLE);

    IUFillNumber(&GuideCCD.RapidGuideDataN[0],"GUIDESTAR_X","Guide star position X","%5.2f",0,1024,0,0);
    IUFillNumber(&GuideCCD.RapidGuideDataN[1],"GUIDESTAR_Y","Guide star position Y","%5.2f",0,1024,0,0);
    IUFillNumber(&GuideCCD.RapidGuideDataN[2],"GUIDESTAR_FIT","Guide star fit","%5.2f",0,1024,0,0);
    IUFillNumberVector(GuideCCD.RapidGuideDataNP,GuideCCD.RapidGuideDataN,3,getDeviceName(),"GUIDER_RAPID_GUIDE_DATA","Rapid Guide Data",RAPIDGUIDE_TAB,IP_RO,60,IPS_IDLE);

    // CCD Class Init

    IUFillSwitch(&UploadS[0], "UPLOAD_CLIENT", "Client", ISS_ON);
    IUFillSwitch(&UploadS[1], "UPLOAD_LOCAL", "Local", ISS_OFF);
    IUFillSwitch(&UploadS[2], "UPLOAD_BOTH", "Both", ISS_OFF);
    IUFillSwitchVector(&UploadSP, UploadS, 3, getDeviceName(), "UPLOAD_MODE", "Upload", OPTIONS_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

    IUFillText(&UploadSettingsT[0],"UPLOAD_DIR","Dir","");
    IUFillText(&UploadSettingsT[1],"UPLOAD_PREFIX","Prefix","IMAGE_XX");
    IUFillTextVector(&UploadSettingsTP,UploadSettingsT,2,getDeviceName(),"UPLOAD_SETTINGS","Upload Settings",OPTIONS_TAB,IP_RW,60,IPS_IDLE);

    IUFillText(&ActiveDeviceT[0],"ACTIVE_TELESCOPE","Telescope","Telescope Simulator");
    IUFillText(&ActiveDeviceT[1],"ACTIVE_FOCUSER","Focuser","Focuser Simulator");
    IUFillTextVector(ActiveDeviceTP,ActiveDeviceT,2,getDeviceName(),"ACTIVE_DEVICES","Snoop devices",OPTIONS_TAB,IP_RW,60,IPS_IDLE);

    IUFillNumber(&EqN[0],"RA","Ra (hh:mm:ss)","%010.6m",0,24,0,0);
    IUFillNumber(&EqN[1],"DEC","Dec (dd:mm:ss)","%010.6m",-90,90,0,0);
    IUFillNumberVector(&EqNP,EqN,2,ActiveDeviceT[0].text,"EQUATORIAL_EOD_COORD","EQ Coord","Main Control",IP_RW,60,IPS_IDLE);

    IDSnoopDevice(ActiveDeviceT[0].text,"EQUATORIAL_EOD_COORD");


    // Guider Interface
    initGuiderProperties(getDeviceName(), GUIDE_CONTROL_TAB);


Guider
    IUFillNumber(&GuideNSN[0],"TIMED_GUIDE_N","North (msec)","%g",0,60000,10,0);
    IUFillNumber(&GuideNSN[1],"TIMED_GUIDE_S","South (msec)","%g",0,60000,10,0);
    IUFillNumberVector(&GuideNSNP,GuideNSN,2,deviceName,"TELESCOPE_TIMED_GUIDE_NS","Guide North/South",groupName,IP_RW,60,IPS_IDLE);

    IUFillNumber(&GuideWEN[0],"TIMED_GUIDE_E","East (msec)","%g",0,60000,10,0);
    IUFillNumber(&GuideWEN[1],"TIMED_GUIDE_W","West (msec)","%g",0,60000,10,0);
    IUFillNumberVector(&GuideWENP,GuideWEN,2,deviceName,"TELESCOPE_TIMED_GUIDE_WE","Guide East/West",groupName,IP_RW,60,IPS_IDLE);


Controller
    IUFillSwitch(&UseJoystickS[0], "ENABLE", "Enable", ISS_OFF);
    IUFillSwitch(&UseJoystickS[1], "DISABLE", "Disable", ISS_ON);
    IUFillSwitchVector(&UseJoystickSP, UseJoystickS, 2, device->getDeviceName(), "USEJOYSTICK", "Joystick", OPTIONS_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

    IUFillTextVector(&JoystickSettingTP, JoystickSettingT, JoystickSettingTP.ntp, device->getDeviceName(), "JOYSTICKSETTINGS", "Settings", "Joystick", IP_RW, 0, IPS_IDLE);




//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
// ASTRO TOOLS ARCHITECTURE, INTERFACES & USAGE
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////

// We try to make a BOTTOM-UP approach here!!!!!! -> Conde improvements and abstractions can be applied iteratively! 

// ---------------------------- CLIENTS  ---------------------------- 

// -------------------------------- IndiClientT  --------------------
class IndiClientT {
   // TODO: DISCUSS - Inherit from INDI::BaseClient - protected? private? Or encapsulate and create an "inner class" which implements Mediator IF?

   // Indi server connection
   void connectServer();
   void disconnectServer();
   bool isServerConnected() const;

   get / set / is AutoConnect
   getName() /*ClientName (composed of hostname and port), used to uniquely identify INDI client - was: getConnectioName() */

   // Serialization / serialize...
   void saveConfig(Archive & ar, const unsigned int version) const;
   void loadConfig(Archive & ar, const unsigned int version) const;
      
   // TODO: DISCUSS! 
   DISCUSS: getDevice?!?! connectDevice?! --> move elsewhere?!
   use of getDevices() ... use of getProperties() in device?!
   DISCUSS DISCUSS DISCUSS
   // TOOD: Use  client function const vector<INDI::BaseDevice *> & devices = myIndiClient->getDevices(); to lookup all devices...

};

typedef map<string /* connectionName */, IndiClientTmplT *> IndiClientsT;


// -------------------------------- IndiClientManagerT  --------------------
class IndiClientManagerT {
private:
   IndiClientsT mClients;

public:
   IndiClientT * createClient(const string & inHostName, int inPort, bool inAutoConnect);
   void destroyClient(const string & inConnName);
   void destroyClient(IndiClientT * inIndiClient); // optional... required?

   IndiClientsT & getClients();
   IndiClientT * getClient(inConnName);
   bool connectAll();
   bool disconnectAll();
   void loadConfig(const char * inFileName);
   void saveConfig(const char * inFileName);

};


// ---------------------------- DEVICES  ---------------------------- 


// TODO: Are the upper traits actually required in this way?

// TODO: What about the indi client?! -> IndiClientMgr?
// NOTE: The IndiClientMgt stuff went to indi/IndiClientTmplT!!! ... Maybe that is a bad name?!
// TODO: What about configuring certain devices i.e. map their (vec) prop names to the AstroTools internal prop names?


// Above required?
// We could move an enum for VecPropsT and PropsT to DeviceT with all default names... and additional ones to CameraT, FocuserT etc... no traits....

////////////////////////////////////////////////////////////////////////////////////////////


class DeviceT /* OR is this just INDI::Device ??? */ {

  // NOTE: Just see those enmus as "variable" to not always use the same strings... like "CONNECTION", ...
  // In order to check if a device has a certain property, just request it directly using the INDI::Device property (TODO: how? IF? -> look across getProperties() result...) 
  struct VecPropsT {
    enum {
      CONNECTION,
      DEBUG,
      _Count
    };
  };

  struct PropsT {
    enum {
      CONNECT,
      DISCONNECT,
      _Count
    };
  };

  // Implement generic property MAPPING (load & save) in this DeviceT class...!!!

private:
   INDI::Device * mIndiDevice; /* OR shall we inherrit from INDI::Device instead ???*/
     -> should hold device name, -> and pointer to INDI client?!, allows getting all the device properties!

public:
   bool isConnected() { }
   void connect();
   void disconnect();

   void setProp<bool /*SWITCH / INDI PROP TYPE*/>("CONNECTION", "CONNECT", true /*SWITCH_TYPE?!*/, BLOCKING | NON_BLOCKING) {
     // Use the INDI::Device interface...
     ISwitchVectorProperty *pp; // Create property vector with properties to be set...
     mIndiDevice->sendNewNumber(pp);
     // TODO: Think about detailed IMPL...
   }
};

class WheelT : public DeviceT {
   
};

// C++ IF classes to directly access devices from C++ or python.
class Focuser : public DeviceT /*inherritance required? good idea?*/ {
  void setAbsFocusPos() {
     // Use PROP-IF of DeviceT!
     this->setProp<float / NUMBER>(ABS_POS, ABS_POS_PROP, 1234 / NUMBER, BLOCKING | NON_BLOCKING)
     // TODO: Blocking?! Return value? Handle exceptions? return value?...
  }
};

class CameraT : public DeviceT {

  // TODO: Need a way to check if an INDI device is really a camera!!! How? INDI API? Or just require certain properties to be there (or alt least mapped - btw.. mapping should be done by DeviceT class already...)? For camera at least e.g. CCD1 or CCD2 have to be there...

  void startExposure(); // Async
  void abortExposure();
  int exposureTimeLeft() const;
  bool isExposureInProgress() const;
  getImage(); // Receive the new image from the camera

  // Convinience function
  void takePicture() // Sync IF - wait until image has been taken

  // TODO: Register for "image receive" event...
  void (un)registerExposureFinishedHandler();
  void (un)registerExposureStartedHandler();
  void (un)registerExposureAbortHandler();
};


// -------------------------------- Device "manager" --------------------
// Manage devices across all INDI clients...
// Manage assignment of devices to alias names...
class DeviceManagerT {

// TODO: Alias mgmt - load configuration - map alias TO device name + hostname

// Requires dynamic cast (possible in python?)
// TODO: How to handle multiple devices found? e.g. two servers with Atik383L+? For now, just return first device...
DeviceT * lookupDevice(inDeviceName, host_port /* Allow PCRE?! */) {
   // Loop across all indi clients
   IndiClientsT & indiClients = AT::getClientManager().getClients();

   for_each(indiClients) {
      // If no host_port filter or if filterStr matches current name... have a closer look at the devices
      if (host_port.empty() || currentClient.getName() == host_port) {
         return currentClient.getDevice(inDeviceName);
      }
   }
   return 0; // no device found
}

CameraT * lookupCamera(deviceName, host+port); // static / dynamic_cast of lookupDevice, check if valid camera...
CameraT * getImageCamera();
};



// -------------------------------- AstroToolsT --------------------
// static 
class AstroToolsT {
private:
  AstroToolsT(); // private constructor

  IndiClientManagerT sIndiClientManager;
  DeviceManagerT sDeviceManager;

public:
   void init();
   void destroy();

   IndiClientManagerT & getClientManager() { return sIndiClientManager; }
   IndiClientManagerT & getDeviceManager() { return sDeviceManager; }
};



// ConfigMgr?? Or locally? --> also set from cmd line switch parser etc...
// TODO: PluginMgrT -> dlopen()

// ------------------------------- Algorithm in AstroTools / Plugin / Other code (loaded and executed from framework / python IF) -------------------------
class FocusFinderPluginT : public PluginT /* required ?*/ {
  // GUI hooks?
  // Plugin IF?

  void execute() {
    /////////////////////////////////
    // Abstract, generic device IF //
    /////////////////////////////////
    // 1. Requesting multiple devices (allow PCRE?)
    typedef vector<DeviceT*> DeviceVecT;
    DeviceVecT devices = AstroToolsT::getDeviceMgr().lookupDevices("Atik383L+");

    DeviceT * myDevice = AstroToolsT::getDeviceMgr().lookupDevice("Atik383L+", "localhost:1234" /* optional, leave empty */); // Requires dynamic cast (possible in python?), throws AbigliousDeviceException...?! Or better just return first device found which matches?

    // 2. Get camera device by name - TODO: How to check if "myCameraName" is really a camera?, throws DeviceDoesNotExistT, NotAValidCameraExceptionT?
    CameraT * myCamera = AstroToolsT::getDeviceMgr().lookupCamera("myCameraName", "localhost:1234" /* optional */);
    myCamera->isConnected(); // inherrited from DeviceT
    myCamera->connect();
    myCamera->takePicture();

    // 3. "Functional" (role) device interface (mapping what image camera is, configured)
    CameraT * imgCamera = AstroToolsT::getDeviceMgr().getImageCamera(); // returns 0 if there is no camera...
    imgCamera->takePicture(); // blocking? result? can this be exported to python?






    ////////////////////////
    // DeviceT property interface
    // ///////////////////// 
    // NOTE: If template does not work, just create methods for NUMBER, TEXT, BLOB, LIGHT and SWITCH... !!!! (or mapped data types)...
    // 1. By (vec) prop name (string) for flexible usage and for non default (vec) prop names. 
    myDevice->setProp<bool /*SWITCH / INDI PROP TYPE*/>("CONNECTION", "CONNECT", true /*SWITCH_TYPE?!*/, BLOCKING | NON_BLOCKING); // tmpl required? throws if prop does not exist
    bool res = myDevice->getProp<bool /*SWITCH / INDI PROP TYPE*/>("CONNECTION", "CONNECT"); // tmpl required? throws if prop does not exist...
    bool has1 = myDevice->hasProp("CCD_TEMPERATURE", "TEMPERATURE");
    myDevice->waitForPropEqualsValue("CONNECTION", "CONNECT", value, timeoutMs);


    // 2. By enum type for default (vec) prop names
    myDevice->setProp<bool /*SWITCH / INDI PROP TYPE*/>(CONNECTION /*CONNECTION is enum of myDevice?! or static const char*? */, CONNECT, true /*SWITCH_TYPE?!*/, BLOCKING | NON_BLOCKING); // tmpl required?
    bool res = myDevice->getProp<bool /*SWITCH / INDI PROP TYPE*/>(CONNECTION, CONNECT); // tmpl required?
    float fres = myDevice->getProp<float /*NUMBER / INDI PROP TYPE*/>(CCD_TEMPERATURE, TEMPERATURE); // tmpl required? TODO: How to handle?!!!!!!!!! Possible?! Yes. throw if prop does not exist! DeviceT is generic IF, accepting all combinations. Device has internal list of available pros (getProperties from INDI)... 
    bool has1 = myDevice->hasProp(CCD_TEMPERATURE, TEMPERATURE);
    myDevice->waitForPropEqualsValue(CONNECTION, CONNECT, value, timeoutMs);




    ///////////////////////////////////////////////
    // Device vec-prop change interface (listener) - calls come asynchronous!!!
    ///////////////////////////////////////////////
    // 1. Register / unregister by (vec)prop name (string) - (is there a template-solution?)

    // 1.a. Register for change of all (vec) props of a device of one type
    myCamera->registerSwitch(& MyClassT::callback..., "CONNECTION" /* Allow PCRE as well ?! Required??*/ ); // See IndiClientT... boost::signal2...
    myCamera->registerSwitch(& MyClassT::callback..., "DEBUG");
    myCamera->registerNumber(& MyClassT::callback, "CCD_TEMPERATURE"); // throws if no ProVec with given name exists (of given type)
    myCamera->registerNumber(& callbackAllCamNumbers /* leave vecProp name empty to get all... */); // throws if no ProVec with given name exists (of given type)

    // 1.b. unregister
    myCamera->unregisterNumber(& MyClassT::callback, "CONNECTION");
    myCamera->unregisterNumber(& MyClassT::callback); // Unregisters all uses of MyClassT::callback - for CONNECTION and CCD_TEMPERATURE. If specifying CCD_TEMPERATURE, only MyClassT::callback for CCD_TEMPERATURE will be removed. Will this technically work? - at max. a technical problem.... guess boost has a solution...
    myCamera->unregisterNumber(& callbackAllCamNumbers); // Unregisters



    // 2.a. For default (vec) prop names, register by enum / struct / variable?... 
    myCamera->registerSwitch(& MyClassT::callback..., CONNECTION); // See IndiClientT... boost::signal2...
    myCamera->registerSwitch(& MyClassT::callback..., DEBUG);
    myCamera->registerNumber(& callback, CCD_TEMPERATURE); // throws if no ProVec with given name exists (of given type)
    myCamera->registerNumber(& callback /* leave vecProp name empty to get all... */); // throws if no ProVec with given name exists (of given type)

    // 2.b. Unregister
    myCamera->unregisterNumber(& MyClassT::callback, CONNECTION);
    myCamera->unregisterNumber(& MyClassT::callback); // Unregisters all uses of MyClassT::callback - for CONNECTION and CCD_TEMPERATURE. If specifying CCD_TEMPERATURE, only MyClassT::callback for CCD_TEMPERATURE will be removed. Will this technically work? - at max. a technical problem.... guess boost has a solution...
    myCamera->unregisterNumber(& callbackAllCamNumbers); // Unregisters
    

  }
};



int main(void) {
  cerr << "Dynamic Enumeration Test" << endl;

  return 0;
}




////////////////////////////////////////////////////////////////
// PLUGIN MECHANISM

console app vs gui app
----------------------

a. Which use cases exist for a pure console APP?

-User wants to execute a functionality without performing any GUI interaction (e.g. scripting / automation)
 This requires that the functionality is fully reachable and configurable by the command line. The functionality may either be built into the
 astro_tools framework or may be part of a plugin.

 Examples
 -The astro_tools framework could print the list of all available plugins
 -An astro-time plugin prints the current star time
 -A fully automatically running focus finder algorithm (which requires no GUI interaction) finds focus
 -Take a picture with the specified camera + options (TODO: Is INDI stuff a plugin? or built-in?).

-User wants to execute a (non-GUI) python script (TODO: Is python stuff plugin or built-in?)




b. Separation of console vs GUI on two levels possible: Compile time and / or run time

1. Compile time
   Leaving out GUI (QT) dependecies when compiling astro_tools more or less only impacts development (dependencies, compile time).
   That means the APP only runs in console mode and is not able to open any window. Plugin GUI menu hooks are just not registered.
 
   Solution?! offer two executables - one astro_tools and castro_tools?! Inherrit from AstroToolsT to create AstroToolsGuiT?! Or two very different executables?!


2. Runtime
   GUI stuff is part of executable (linked and required by APP). The user just may disable showing the GUI for certain tasks (e.g. by command line switch). 



Plugin loading mechanism
------------------------
-Which plugins are there? --> scan list of directories for libat* with "instance" method and "version"...?! recursive?
-Which plugins should be loaded? -> plugin load list vs symlinks to plugins which should be loaded?? Platform dependency?
-What if plugin which should be loaded could not be loaded (not found / not valid / not compatible)? -> print error message, leave it out and continue loading others...
-What if a plugin with GUI part is loaded if in console mode? --> warning msg, only hook non-GUI stuff?!
-How to handle plugin dependcies (i.e. if one plugin needs another?) - loading order? Plugins should know wha they need? TODO: Or should this be covered by e.g. apt version control? --> Solve by version control. If something is missing during runtime, just print error message.
-How to handle duplicated commands? -> Waring / error message, first come first server. The second one will not be registered, the first one remains in place



Results
-------
-astro_tools is console application
  -Usage: astro_tools <cmd> [options]
  -Cmds are:
     -either built into the binary (part of the framework)
     -added by C++ plugins
     -Just an idea: added by python scripts which are loaded automatically... (?)

-astro_tools_gui is GUI application

Both applications may or may not share certain arguments and / or code. This will become visible during development.
First we are going to develop astro_tools - the console application. Based on that we can then develop the GUI part.
Note: Even if we do not have a the gui app, a plugin which is linked against Qt, may still be able to display windows. Those are just not hooked into the geaphical menus.


Software-Design
---------------

4 Ways:

1.) 2 sources - one for console app, one for gui app -> No reuse
2.) inherritance - put common code parts into base class and inherrit for each App
3.) policy - write two "app policies" - one for console app and one for gui app, the common part is implemented as template
4.) 2 sources - one for console app and one for gui, put common part into App class and call its methods accordingly

Question:
-Requirements, data structure and functionality of console app
-Requirements, data structure and functionality of GUI app
-> What is common between those two?
   -Logging
   -Certain program options
   -plugin loading mechanism (partly - what about registering gui hooks?)





TODO: What if multiple commands are required in a row (e.g. connect camera, take picture)? Or can't this happen? --> Avoid! take_picture implies connection and does not have to be executed in a row!


TODO: Fixtures in boost test framework. See indi_camera_test.C -> Shutter test...




TODO: New indi structure...

Situation:
-Need to set a device property _before_ connecting to the device (e.g. port)
-In order to determine device type (by props) a connection to the device is required
-BUT: The device type is required in order to create a device class instance which is required to set the device property

-> Or: Create device instance not in client connect() but in getDevice() method...
   -> allow creation of device passing a BaseDevice (or a name)...
   -> verification if device is a e.g. valid camera happens at device connect! - not before. This allows setting of certain parameters using the actual device class API. Verification if device fulfills all props can be disabled... by default it is enabled.
   -> developer / user decides which type of device he is creating. Devices are created by a function called createDevice(name, type) / or getDevice(name, type)
   -> getDevice fails if device does not exist (throw)
   -> device instance is created with new only the first time, pointer is saved in an internal map hold by the client.
   -> all instances are deleted when the client is destroyed.
   -> the device auto detection feature is being removed
   -> if getDevice() is called a second time for the same device name but for a different device type, the old instance is deleted and a new on eis created.
