#ifndef _ARDRONE_TOOL_H_
#define _ARDRONE_TOOL_H_

#include <ardrone_api.h>
#include <VP_Os/vp_os_types.h>
#include <config.h>

#define ARDRONE_REFRESH_MS        20

#define MAX_NAME_LENGTH           255
#define MAX_NUM_DEVICES           10
#define MAX_NUM_INPUT_EVENTS      16

#ifdef ND_WRITE_TO_FILE
extern uint32_t num_picture_decoded;
extern uint32_t wiimote_enable;
extern float32_t wiimote_ax, wiimote_ay, wiimote_az;
#endif

extern char wifi_ardrone_ip[];
extern char app_id[];
extern char app_name[];
extern char usr_id[];
extern char usr_name[];
extern char ses_id[];
extern char ses_name[];

typedef struct _ardrone_tool_configure_data_t {
  char* var;
  char* value;
} ardrone_tool_configure_data_t;

///
/// Required api each tool can implement
///
extern ardrone_tool_configure_data_t configure_data[] WEAK;

extern C_RESULT ardrone_tool_init_custom(int argc, char **argv) WEAK;
extern C_RESULT ardrone_tool_update_custom( void ) WEAK;
extern C_RESULT ardrone_tool_display_custom( void ) WEAK;
extern C_RESULT ardrone_tool_shutdown_custom( void ) WEAK;
extern bool_t   ardrone_tool_exit( void ) WEAK;

// cmd line parsing
extern C_RESULT ardrone_tool_check_argc_custom( int32_t argc) WEAK;
extern void ardrone_tool_display_cmd_line_custom( void ) WEAK;
extern bool_t ardrone_tool_parse_cmd_line_custom( const char* cmd ) WEAK;

// Tricky ...
extern int custom_main(int argc, char **argv) WEAK;

// This is implemented by the library
#ifdef NO_ARDRONE_MAINLOOP
C_RESULT ardrone_tool_init( const char* ardrone_ip, size_t n, AT_CODEC_FUNCTIONS_PTRS *ptrs, const char *appname, const char *usrname);
#else
C_RESULT ardrone_tool_init(int argc, char **argv);
#endif
C_RESULT ardrone_tool_set_refresh_time(int refresh_time_in_ms);
C_RESULT ardrone_tool_pause( void );
C_RESULT ardrone_tool_resume( void );
C_RESULT ardrone_tool_setup_com( const char* ssid );
C_RESULT ardrone_tool_update(void);
C_RESULT ardrone_tool_shutdown(void);


void ardrone_tool_send_com_watchdog(void);  // To send it only once
int main();

// There because not defined in embedded
void api_configuration_get_ctrl_mode(void);
void api_configuration_ack_ctrl_mode(void);

/*! \page page2
 * @defgroup ARDrone_Tool ARDrone_Tool

<p>
<hr>

<center><h2> General Overview </h2></center>

\par

ARDrone Tool is an attempt to create a common base code for all tools that must connect to ARDrone. In this document we call client any tools that link with ARDrone Tool. ARDrone Tool

   <ul>
   <li> already implements main and do various initialization including com layer
   <li> implements stub for ATcodec. All functions declared in ardrone_api.h are implemented.
   <li> receives, parses Navdata and calls user defined handlers
   <li> provides vp_sdk's stages to record video, ...
   <li> provides a framework to handle user inputs (keyboard, gamepad, wiimote)
   </ul>

\par

Usually a client will create a user interface
User inputs are refreshed every 20 ms (see ARDRONE_REFRESH_MS in ardrone_tool.h)

\par WEAK functions

ARDrone Tool uses a gcc specific extension : WEAK. This extension allows ARDrone Tool to give a default implementation to functions that can be overriden by clients. Overriding is not mandatory so a client can choose to keep a default implementation for these functions.

\note
Some version of mingw32 doesn't support WEAK function.

<p>
<hr>

<center><h2> Code implementation </h2></center>

\section Navdata

Navdata works with a DHCP's option like system. This text is inspired by the rfc2132.

\par
Navdata items are carried in tagged data items that are stored in the options field of the navdata packet. The data items are also called "options". Basically an option is a declaration respecting the following format:

\code
  typedef struct _navdata_option_t {
    // Common part
    uint16_t  tag;
    uint16_t  size;

    // Opaque declaration
    uint8_t   data[];
  } navdata_option_t;
\endcode

For example :

\code
  typedef struct _navdata_demo_t {
    // Common part
    uint16_t    tag;
    uint16_t    size;

    // Specialize part
    uint32_t    ctrl_state;
    uint32_t    vbat_flying_percentage;

    float32_t   theta;
    float32_t   phi;
    float32_t   psi;

    int32_t     altitude;

    float32_t   vx;
    float32_t   vy;
  } navdata_demo_t;
\endcode

A navdata packet follow the following prototype:

\code
  typedef struct _navdata_t {
    uint32_t    header;
    uint32_t    ardrone_state;
    uint32_t    sequence;
    bool_t      vision_defined;

    navdata_option_t  options[1];
  } navdata_t;
\endcode

\par
At the moment of we write this document, developper can choose to send all navdata or only a prefdefined subset called Navdata Demo by using a ardrone config variable called navdata_demo. Navdata demo defines minimum data ARDrone must sent to a remote host

\subsection navdata_list List of navdata options.

In the following subsection we described all the currently available navdata options and their meanings.

<TABLE>
<TR><TH>Option</TH><TH>Description</TH></TR>
<TR><TH>NAVDATA_DEMO</TH><TH>Minimum data needed</TH></TR>
<TR><TH>NAVDATA_TIME</TH><TH>ARDrone current time</TH></TR>
<TR><TH>NAVDATA_RAW_MEASURES</TH><TH>Raw measures (acceleros & gyros) coming from PIC</TH></TR>
<TR><TH>NAVDATA_PHYS_MEASURES</TH><TH>Filtered values after control processing</TH></TR>
<TR><TH>NAVDATA_GYROS_OFFSETS</TH><TH>Gyros offsets</TH></TR>
<TR><TH>NAVDATA_EULER_ANGLES</TH><TH>Fused euler angles</TH></TR>
<TR><TH>NAVDATA_REFERENCES</TH><TH></TH></TR>
<TR><TH>NAVDATA_TRIMS</TH><TH></TH></TR>
<TR><TH>NAVDATA_RC_REFERENCES</TH><TH></TH></TR>
<TR><TH>NAVDATA_PWM</TH><TH>Data used to control motors</TH></TR>
<TR><TH>NAVDATA_ALTITUDE</TH><TH>Estimated values with a relation to altitude</TH></TR>
<TR><TH>NAVDATA_VISION_RAW</TH><TH>Vision's estimated velocities</TH></TR>
<TR><TH>NAVDATA_VISION</TH><TH>Data used when computing vision</TH></TR>
<TR><TH>NAVDATA_VISION_PERF</TH><TH>Performance data collected when profiling vision code</TH></TR>
<TR><TH>NAVDATA_TRACKERS_SEND</TH><TH>Position of all trackers computed by vision</TH></TR>
<TR><TH>NAVDATA_VISION_DETECT</TH><TH>Position of the chemney detected by vision</TH></TR>
<TR><TH>NAVDATA_WATCHDOG</TH><TH>Tells if there was an anormal delay between two navdata packets</TH></TR>
<TR><TH>NAVDATA_IPHONE_ANGLES</TH><TH>Used to send back to iPhone its attitude (was an attempt to compute latency between ardrone & iPhone)</TH></TR>
<TR><TH>NAVDATA_ADC_DATA_FRAME</TH><TH>Used in remote control. Sends data frame coming from PIC</TH></TR>
<TR><TH>NAVDATA_CKS</TH><TH>Description</TH></TR>

\subsection navdata_new Adding/Customizing navdata options

\par

When updating a navdata option or adding a navdata option one must take care to update the following files too:

   <ul>
   <li> navdata_server.h and navdata_server.c in \ref Toy
   <li> navdata.h and navdata.c in Soft/Lib/Control
   <li> any navdata handler (in particular ardrone_navdata_file.c in \ref ARDrone_Tool )
   </ul>

\note
There's a way to ease this process by defining an header file like config_keys.h (TODO list ;-))

\subsection navdata_handling Handling navdata options

\par
ARDrone Tools provides facility to handle navdata options. First ARDrone Tool will established a connection, as a client, on port 5554, when application starts (it handles timeout and reconnections). Then it will listen to navdata's udp packets to parse navdata options found inside them. We called this functionnality unpacking and it is implemented in Control library (Soft/Lib/Control/navdata.c).

\par
When all options are parsed, navdata handlers are called to allow user to manipulate navdata options. Some handlers are predefined. The most important one is ardrone_navdata_file that registers all incomming navdata in local storage.

\par
To add a new handler, a developper have to implement three functions:

   <ul>
   <li> an init function
   <li> a process function
   <li> a release function
   </ul>

The init and release functions are called only once and the process function is called whenever a new navdata packet is received. The init function can receive any data as a parameter (see ardrone_navdataf_file.c or navdata_ihm.c if you want examples).

\subsection navdata_control Relationship between flashing/updating and configuration by wifi and navdata

\par
Navdata are also used to regulate data we send to ARDrone. We find out there was problem to send big amount of data (some packets were lost). We decided to split large amount of data in smaller packets and to use navdata to delay their sending.

\par

This approach was generalized to send all files to ARDrone:

   <ul>
   <li> Update file for P5P software [ARDRONE_UPDATE_CONTROL_MODE]
   <li> Update file for ADC software [PIC_UPDATE_CONTROL_MODE]
   </ul>

to ask for files containing:

   <ul>
   <li> Configuration data (ini file) [CFG_GET_CONTROL_MODE]
   <li> Log of previous flies [LOGS_GET_CONTROL_MODE]
   </ul>

and to know when some commands sent over UDP (for example AT_MSG_ATCMD_CONFIG_EXE) are received by setting a flag in navdata's ardrone_state (ARDRONE_COMMAND_MASK).

\note
CONTROL is perhaps a badly choosen name.

 */

#endif // _ARDRONE_TOOL_H_
