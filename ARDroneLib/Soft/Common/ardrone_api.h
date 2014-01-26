/******************************************************************************
 *                        COPYRIGHT PARROT 2010
 ******************************************************************************
 * PARROT A.R.Drone SDK
 *---------------------------------------------------------------------------*/
/**
 * @file   ardrone_api.h
 * @brief  Data types and functions to communicate with the drone.
 *
 ******************************************************************************/

#ifndef _ARDRONE_API_H_
#define _ARDRONE_API_H_

/*--- Libraries --------------------------------------------------------------*/

#include <sys/time.h>

#include <ardrone_common_config.h>
#include <ATcodec/ATcodec_api.h>
#include <VP_Os/vp_os_malloc.h>
#include <navdata_common.h>
#include <vision_common.h>
#include <Maths/quaternions.h>

/** @def API_WEAK
 * @brief Defines the API_WEAK attribute.
 * It is used to define a function which
 * can be redefined by the SDK user with a custom one, without generating
 * a 'multiple definition' compilation error. This is a GCC specific feature.
 **/
#if defined(USE_LINUX) && defined(_EMBEDDED) && !defined(USE_MINGW32)
#define API_WEAK WEAK
#else
#define API_WEAK
#endif

/** @def ARDRONE_CONFIGURATION_SET
 *  @brief Sets a drone configuration value.
 *  This is the main method that SDK users should use to change a setting on
 *  the drone.
 *  @def ARDRONE_CONFIGURATION_SET_FUNCTION
 *  @brief Used internally by ARDroneLib to build the name of a function
 *  changing a particular drone configuration value.
 *  @def ARDRONE_CONFIGURATION_PROTOTYPE
 *  @brief Used internally by ARDroneLib to define a function changing a
 *  particular drone configuration value.
 * **/
//Deprecated - no ack method - #define ARDRONE_CONFIGURATION_SET(NAME, VALUE) 		     ARDRONE_CONFIGURATION_SET_FUNCTION(NAME)(VALUE)
#define ARDRONE_CONFIGURATION_SET_FUNCTION(NAME)	ardrone_at_configuration_set_##NAME
#define ARDRONE_CONFIGURATION_PROTOTYPE(NAME) 		C_RESULT ARDRONE_CONFIGURATION_SET_FUNCTION(NAME)(void* value, char* ses_id, char* usr_id, char* app_id)

/**
 * @brief Used internally by ARDroneTool as an argument to the creation of
 * a control event, when changing a configuration value.
 * SDK users should not deal with this; see ARDRONE_CONFIGURATION_SET to change
 * a drone setting.
 */

typedef C_RESULT (*ardrone_at_configuration_set)(void* value, char* ses_id, char* usr_id, char* app_id);

/**
 * @brief Used internally by ARDroneTool as argument to the miscellaneous
 * settings (see AT*MISC command).
 */
enum
{
  NO_CONTROL                           = 0,
  ALTITUDE_CONTROL                     = 1,
  ALTITUDE_VISION_CONTROL              = 2,
  ALTITUDE_VISION_CONTROL_TAKEOFF_TRIM = 3,
  MULTICONFIGURATION                   = 1024
};


/**
 * @brief Possible states of the drone 'control' thread.
  */
typedef enum
{
  NO_CONTROL_MODE = 0,          /*<! Doing nothing */
  ARDRONE_UPDATE_CONTROL_MODE,  /*<! Not used */
  PIC_UPDATE_CONTROL_MODE,      /*<! Not used */
  LOGS_GET_CONTROL_MODE,        /*<! Not used */
  CFG_GET_CONTROL_MODE,         /*<! Send active configuration file to a client through the 'control' socket UDP 5559 */
  ACK_CONTROL_MODE,             /*<! Reset command mask in navdata */
  CUSTOM_CFG_GET_CONTROL_MODE   /*<! Requests the list of custom configuration IDs */
} ARDRONE_CONTROL_MODE;

/**
 * @brief Drone video channels selection values.
 * A client can choose to receive video data from the horizontal camera,
 * the vertical one, or both (in a Picture-In-Picture way).
  */
typedef enum
{
  ZAP_CHANNEL_FIRST = 0,                                /*<! First element */
  ZAP_CHANNEL_HORI = ZAP_CHANNEL_FIRST,                 /*<! Selects the horizontal camera */
  ZAP_CHANNEL_VERT,                                     /*<! Selects the vertical camera */
  ZAP_CHANNEL_LARGE_HORI_SMALL_VERT,                    /*<! Selects the horizontal camera with vertical camera picture inserted in the left-top corner */
  ZAP_CHANNEL_LARGE_VERT_SMALL_HORI,                    /*<! Selects the vertical camera with horizontal camera picture inserted in the left-top corner */
  ZAP_CHANNEL_LAST = ZAP_CHANNEL_LARGE_VERT_SMALL_HORI, /*<! Last element */
  ZAP_CHANNEL_NEXT,                                     /*<! Selects the next available format among those above. */
} ZAP_VIDEO_CHANNEL;

/**
 * @brief Values for the detection type on drone cameras.
 */
typedef enum
{
  CAD_TYPE_HORIZONTAL = 0,           /*<! Deprecated */
  CAD_TYPE_VERTICAL,                 /*<! Deprecated */
  CAD_TYPE_VISION,                   /*<! Detection of 2D horizontal tags on drone shells */
  CAD_TYPE_NONE,                     /*<! Detection disabled */
  CAD_TYPE_COCARDE,                  /*<! Detects a roundel under the drone */
  CAD_TYPE_ORIENTED_COCARDE,         /*<! Detects an oriented roundel under the drone */
  CAD_TYPE_STRIPE,                   /*<! Detects a uniform stripe on the ground */
  CAD_TYPE_H_COCARDE,                /*<! Detects a roundel in front of the drone */
  CAD_TYPE_H_ORIENTED_COCARDE,       /*<! Detects an oriented roundel in front of the drone */
  CAD_TYPE_STRIPE_V,
  CAD_TYPE_MULTIPLE_DETECTION_MODE,  /* The drone uses several detections at the same time */
  CAD_TYPE_CAP,                      /*<! Detects a Cap orange and green in front of the drone */
  CAD_TYPE_ORIENTED_COCARDE_BW,      /*<! Detects the black and white roundel */
  CAD_TYPE_VISION_V2,                /*<! Detects 2nd version of shell/tag in front of the drone */
  CAD_TYPE_TOWER_SIDE,               /*<! Detect a tower side with the front camera */
  CAD_TYPE_NUM,                      /*<! Number of possible values for CAD_TYPE */
} CAD_TYPE;

/* Type of tag to detect
 * This enum deprecates 'CAD_TYPE' which did not allow to mix tag types and cameras */
typedef enum
{
  TAG_TYPE_NONE             = 0,
  TAG_TYPE_SHELL_TAG        ,
  TAG_TYPE_ROUNDEL          ,
  TAG_TYPE_ORIENTED_ROUNDEL ,
  TAG_TYPE_STRIPE           ,
  TAG_TYPE_CAP              ,
  TAG_TYPE_SHELL_TAG_V2     ,
  TAG_TYPE_TOWER_SIDE       ,
  TAG_TYPE_BLACK_ROUNDEL    ,
  TAG_TYPE_NUM
} TAG_TYPE;

#define TAG_TYPE_MASK(tagtype) (  ((tagtype)==0)? 0 : (1<<((tagtype)-1)) )

typedef enum
{
  DETECTION_SOURCE_CAMERA_HORIZONTAL=0,   /*<! Tag was detected on the front camera picture */
  DETECTION_SOURCE_CAMERA_VERTICAL,       /*<! Tag was detected on the vertical camera picture at full speed */
  DETECTION_SOURCE_CAMERA_VERTICAL_HSYNC, /*<! Tag was detected on the vertical camera picture inside the horizontal pipeline */
  DETECTION_SOURCE_CAMERA_NUM,
} DETECTION_SOURCE_CAMERA;

#define DETECTION_MAKE_TYPE(source,tag) ( ((source)<<16) | (tag) )
#define DETECTION_EXTRACT_SOURCE(type)  ( ((type)>>16) & 0x0FF )
#define DETECTION_EXTRACT_TAG(type)     ( (type) & 0x0FF )

/**
 * @brief Video bitrate control mode.
 */
typedef enum
{
  VBC_MODE_DISABLED = 0,  /*<! no video bitrate control */
  VBC_MODE_DYNAMIC,       /*<! video bitrate control active */
  VBC_MANUAL              /*<! video bitrate control active */
} VIDEO_BITRATE_CONTROL_MODE;

/**
 * @brief 2D-tag color values, used by the detection algorithms.
 */
typedef enum
{
  ARDRONE_DETECTION_COLOR_ORANGE_GREEN = 1,    /*!< Cameras detect orange-green-orange tags */
  ARDRONE_DETECTION_COLOR_ORANGE_YELLOW,       /*!< Cameras detect orange-yellow-orange tags*/
  ARDRONE_DETECTION_COLOR_ORANGE_BLUE,          /*!< Cameras detect orange-blue-orange tags */
  ARDRONE_DETECTION_COLOR_ARRACE_FINISH_LINE=0x10,
  ARDRONE_DETECTION_COLOR_ARRACE_DONUT=0x11
} COLORS_DETECTION_TYPE, ENEMY_COLORS_TYPE;

/**
 * @brief User-control mode setting in FreeFlight.
 * @description Bit mask showing how a user of the FreeFlight app. controls their drone.
 */
typedef enum
{
  // This is a bit to shift
  // CONTROL_LEVEL_NOT_USED   = 0,
  CONTROL_LEVEL_COMBINED_YAW  = 1,    /*!< =1 : drone tilts sideward and turns simultaneously */
  // CONTROL_LEVEL_NOT_USED   = 2,
  //4 used for CONTROL_LEVEL_CONTROL_MODE_BIT
} CONTROL_LEVEL;

/**
 * @enum LED_ANIMATION_IDS
 * @brief Led animation values.
 * See ardrone_at_set_led_animation function.
 */
//
typedef enum LED_ANIMATION_IDS_
{
  #define LED_ANIMATION(NAME, ... ) NAME ,
  #include <led_animation.h>
  #undef LED_ANIMATION
  NUM_LED_ANIMATION
} LED_ANIMATION_IDS;

typedef enum
{
	USERBOX_CMD_STOP = 0,
	USERBOX_CMD_START,
	USERBOX_CMD_SCREENSHOT,
	USERBOX_CMD_CANCEL,
	USERBOX_CMD_MAX,
} USERBOX_CMD;

typedef enum 
{
	FLYING_STATE_LANDED = 0,
	FLYING_STATE_FLYING,
	FLYING_STATE_TAKING_OFF,
	FLYING_STATE_LANDING,
} FLYING_STATE;

typedef enum ARDRONE_PROGRESSIVE_CMD_FLAG_
{
  ARDRONE_PROGRESSIVE_CMD_ENABLE,              // 1: use progressive commands - 0: try hovering
  ARDRONE_PROGRESSIVE_CMD_COMBINED_YAW_ACTIVE, // 1: activate combined yaw - 0: Deactivate combined yaw
  ARDRONE_MAGNETO_CMD_ENABLE,				   // 1: activate the magneto piloting mode - 0: desactivate the mode
  ARDRONE_PROGRESSIVE_CMD_MAX
} ARDRONE_PROGRESSIVE_CMD_FLAG;

/**
 * @struct _euler_angles_t
 * @brief  Euler angles in float32_t format expressed in radians.
 */
typedef struct _euler_angles_t
{
  float32_t theta;   /*<! Drone front-back angle (pitch) */
  float32_t phi;     /*<! Drone left-right angle (roll) */
  float32_t psi;     /*<! Drone orientation (yaw) */
} euler_angles_t;

/**
 * @struct _euler_angles_t
 * @brief  Euler angles in int32_t format expressed in radians.
 */
typedef struct _iEuler_angles_t
{
  int32_t theta;     /*<! Drone front-back angle (pitch) */
  int32_t phi;       /*<! Drone left-right angle (roll) */
  int32_t psi;       /*<! Drone orientation (yaw) */
} iEuler_angles_t;

/**
 * @struct _angular_rates_t
 * @brief Angular rates in float32_t format
 */
typedef struct _angular_rates_t
{
  float32_t p;       /*<! Drone front-back angular rate (pitch) */
  float32_t q;       /*<! Drone left-right angular rate (roll) */
  float32_t r;       /*<! Drone orientation angular rate (yaw) */
} angular_rates_t;

/**
 * \struct _velocities_t
 * \brief Velocities in float32_t format
 */
/*
typedef struct _velocities_t {
  float32_t x;
  float32_t y;
  float32_t z;
} velocities_t;
*/

/**
 * @struct _acq_vision_t
 * @brief Deprecated - used internally by the drone - Vision params in float32_t
 */
#ifndef DISABLE_DEPRECATED_CODE
  typedef struct _acq_vision_t
  {
    float32_t tx;
    float32_t ty;
    float32_t tz;

    int32_t turn_angle;
    int32_t height;
    int32_t turn_finished;

    bool_t flag_25Hz;
  } acq_vision_t;
#endif

/**
 * @struct _polaris_data_t
 * @brief Used by Parrot only - drone externally-measured position in 3D-space
 */
typedef struct _polaris_data_t
{
  float32_t x;
  float32_t y;
  float32_t psi;
  bool_t    defined;
  int32_t   time_us;
} polaris_data_t;

/**
 * @struct api_control_gains_t
 * @brief Drone control loop PID settings
*/
typedef struct _api_control_gains_t
{
  int32_t pq_kp;            /**<    Gain for proportional correction in pitch (p) and roll (q) angular rate control */
  int32_t r_kp;             /**<    Gain for proportional correction in yaw (r) angular rate control */
  int32_t r_ki;             /**<    Gain for integral correction in yaw (r) angular rate control */
  int32_t ea_kp;            /**<    Gain for proportional correction in Euler angle control */
  int32_t ea_ki;            /**<    Gain for integral correction in Euler angle control */
  int32_t alt_kp;           /**<    Gain for proportional correction in Altitude control */
  int32_t alt_ki;           /**<    Gain for integral correction in Altitude control */
  int32_t vz_kp;            /**<    Gain for proportional correction in Vz control */
  int32_t vz_ki;            /**<    Gain for integral correction in Vz control */
  int32_t hovering_kp;      /**<    Gain for proportional correction in hovering control */
  int32_t hovering_ki;      /**<    Gain for integral correction in hovering control */
  int32_t hovering_b_kp;    /**<    Gain for proportional correction in hovering beacon control */
  int32_t hovering_b_ki;    /**<    Gain for integral correction in hovering beacon control */
  int32_t hovering_b_kp2 ;
  int32_t hovering_b_ki2 ;
  int32_t hovering_b_kd2 ;
} api_control_gains_t;

/**
 * @struct _api_vision_tracker_params_t
 * @brief Computer vision tracking settings
*/
typedef struct _api_vision_tracker_params_t
{
  int32_t coarse_scale;         /**<    scale of current picture with respect to original picture */
  int32_t nb_pair;              /**<    number of searched pairs in each direction */
  int32_t loss_per;             /**<    authorized lost pairs percentage for tracking */
  int32_t nb_tracker_width;     /**<    number of trackers in width of current picture */
  int32_t nb_tracker_height;    /**<    number of trackers in height of current picture */
  int32_t scale;                /**<    distance between two pixels in a pair */
  int32_t trans_max;            /**<    largest value of trackers translation between two adjacent pictures */
  int32_t max_pair_dist;        /**<    largest distance of pairs research from tracker location */
  int32_t noise;                /**<    threshold of meaningful contrast */
} api_vision_tracker_params_t;

/* Stephane
 * @enum FLYING_MODE
 * @brief Values for the CONTROL:flying_mode configuration.
 */
typedef enum
{
  FLYING_MODE_FREE_FLIGHT = 0,            			/**< Normal mode, commands are enabled */
  FLYING_MODE_HOVER_ON_TOP_OF_ROUNDEL = 1 << 0,  	/**< Commands are disabled, drones hovers on top of a roundel - roundel detection MUST be activated by the user with 'detect_type' configuration. */
  FLYING_MODE_HOVER_ON_TOP_OF_ORIENTED_ROUNDEL = 1 << 1, /**< Commands are disabled, drones hovers on top of an oriented roundel - oriented roundel detection MUST be activated by the user with 'detect_type' configuration. */
} FLYING_MODE;

/**
 * @enum WIFI_MODE
 * @brief Values for the NETWORK:wifi_mode configuration, who set the wifi mode when drone start
 */
typedef enum
{
  WIFI_MODE_INFRA = 0,        /**<    Access point mode  */
  WIFI_MODE_ADHOC,            /**<    Ad-Hoc mode        */ 
  WIFI_MODE_MANAGED,          /**<    Managed mode       */
  WIFI_MODE_NUM
} WIFI_MODE;

/********************************************************************
*                        NAVDATA FUNCTIONS
********************************************************************/

/**
 * @struct _navdata_unpacked_t
 * @brief Decoded navigation data.
*/
typedef struct _navdata_unpacked_t
{
  uint32_t  nd_seq;
  uint32_t  ardrone_state;
  bool_t    vision_defined;
  uint32_t  last_navdata_refresh;  /*! mask showing which block was refreshed when receiving navdata */

#define NAVDATA_OPTION_DEMO(STRUCTURE,NAME,TAG)  STRUCTURE NAME ;
#define NAVDATA_OPTION(STRUCTURE,NAME,TAG)       STRUCTURE NAME ;
#define NAVDATA_OPTION_CKS(STRUCTURE,NAME,TAG)
	#include <navdata_keys.h>

} navdata_unpacked_t;

/**
 * @def ardrone_navdata_pack
 * @brief Add an 'option' to the navdata network packet to be sent to a client.
 * Used by the drone 'navdata' thread.
*/
#define ardrone_navdata_pack( navdata_ptr, option ) (navdata_option_t*) navdata_pack_option( (uint8_t*) navdata_ptr, \
                                                                                     (uint8_t*) &option,             \
                                                                                     option.size )

/**
 * @def ardrone_navdata_unpack
 * @brief Extract an'option' from the navdata network packet sent by the drone.
 * Used by the client 'navdata' thread inside ARDroneTool.
*/
#define ardrone_navdata_unpack( navdata_ptr, option ) (navdata_option_t*) navdata_unpack_option( (uint8_t*) navdata_ptr, \
                                                                                         navdata_ptr->size,              \
                                                                                         (uint8_t*) &option,             \
                                                                                         sizeof (option) )

/**
 * @brief Add an 'option' to the navdata network packet to be sent to a client.
 * Used by the drone 'navdata' thread.
 * @param navdata_ptr Pointer to the buffer containing packed navdata to be sent.
*/
static INLINE uint8_t* navdata_pack_option( uint8_t* navdata_ptr, uint8_t* data, uint32_t size )
{
  vp_os_memcpy(navdata_ptr, data, size);
  return (navdata_ptr + size);
}

/**
 * @fn navdata_unpack_option
 * @brief Extract an 'option' from the navdata network packet sent by the drone.
 * Used by the client 'navdata' thread inside ARDroneTool.
*/
static INLINE uint8_t* navdata_unpack_option( uint8_t* navdata_ptr, uint32_t ptrsize, uint8_t* data, uint32_t datasize )
{
  uint32_t minSize = (ptrsize < datasize) ? ptrsize : datasize;
  vp_os_memcpy(data, navdata_ptr, minSize);
  return (navdata_ptr + ptrsize);
}

/**
 * @fn navdata_next_option
 * @brief Jumps to the next 'option' inside the packed navdata.
 * Used by the client 'navdata' thread inside ARDroneTool.
*/
static INLINE navdata_option_t* navdata_next_option( navdata_option_t* navdata_options_ptr )
{
  uint8_t* ptr;

  ptr  = (uint8_t*) navdata_options_ptr;
  ptr += navdata_options_ptr->size;

  return (navdata_option_t*) ptr;
}

/**
 * @brief Creates a checksum from a packed navdata packet.
 * @param nv Data to calculate the checksum.
 * @param size Size of data calculate as follow : size-sizeof(navdata_cks_t).
 * @return Retrieves the checksum from the navdata nv.
 */
uint32_t ardrone_navdata_compute_cks( uint8_t* nv, int32_t size ) API_WEAK;

/**
 * @param navdata_unpacked  navdata_unpacked in which to store the navdata.
 * @param navdata One packet read from the port NAVDATA.
 * @param Checksum of navdata
 * @brief Disassembles a buffer of received navdata, and dispatches it inside 'navdata_unpacked' structure.
 */
C_RESULT ardrone_navdata_unpack_all(navdata_unpacked_t* navdata_unpacked, navdata_t* navdata, uint32_t* cks) API_WEAK;

/***
 * @param navdata_options_ptr
 * @param Tag ID of the bloc to search for.
 * @brief Jumps to a specified 'option' (block of navdata) inside a navdata packed buffer.
 */
navdata_option_t* ardrone_navdata_search_option( navdata_option_t* navdata_options_ptr, navdata_tag_t tag ) API_WEAK;

/********************************************************************
*                        AT FUNCTIONS
********************************************************************/

/**
 * @brief Initializes the AT Command managing thread of ARDroneTool.
 * The AT Codec is initialized.
 */
void ardrone_at_init( const char* ip, size_t ip_len ) API_WEAK;

/**
 * @brief Initializes the AT Command managing thread of ARDroneTool, by
 * providing it with custom functions for I/O from/to the drone.
 */
void ardrone_at_init_with_funcs ( const char* ip, size_t ip_len, AT_CODEC_FUNCTIONS_PTRS *funcs) API_WEAK;

/**
 * @fn ardrone_at_open
 * @brief Opens the AT command socket.
 */
ATCODEC_RET ardrone_at_open ( void ) API_WEAK;

/**
 * @brief Makes ARDroneToll send all queued AT commands to the drone.
 * ARDroneTool locally buffers AT commands sent by the client program
 * until this function is called.
 * Usually the client program GUI calls ardrone_at_xxx commands to
 * pilot the drone; these commands are then queued by ARDroneTool.
 * The ARDroneTool main thread calls this functions periodically
 * (every 30ms for example) to actually send to queued commands to
 * the drone.
 */
ATCODEC_RET ardrone_at_send ( void ) API_WEAK;

/*
 * @brief  Sends input sequence number to avoid reception of old data
 * @param  value : sequence number
 * @deprecated
 * @return void
 * Used on prototypes.
 */
//void ardrone_at_set_sequence( uint32_t sequence ) API_WEAK;

/**
 * @brief  Set the flight status command.
 * @param  value  Bit mask containing the desired commmand.
 * The bit mask contains the bit controlling take-off/landing
 * and the bit signaling /resuming from an emergency.
 * This mask is periodically sent to the drone by ARDroneTool
 * inside an AT*REF command.
 */
void ardrone_at_set_ui_pad_value( uint32_t value ) API_WEAK;

/**
 * @brief  Used internally by Parrot - send misc. config. data.
 * @param  data used to configure control
 * @return void
 */
void ardrone_at_set_pmode( int32_t pmode ) API_WEAK;

#ifndef DISABLE_DEPRECATED_CODE
  /**
   * @fn     Tell to keep trim result
   * @param  yes or no
   * @return C_RESULT
   */
  void ardrone_at_keep_trim( bool_t keep ) API_WEAK;

  /**
   * @fn     Reset trim/misc0 related ack's
   * @return void
   */
  void ardrone_at_trim_ack_reset( void ) API_WEAK;
#endif

/**
 * @brief  Used by Parrot - send misc. config. data for drone debugging.
 * @param  data are used to configure control (for instance)
 * @return void
 *  This should not be used by SDK users. Parameters value do not have any fixed meaning.
 */
void ardrone_at_set_ui_misc( int32_t m1, int32_t m2, int32_t m3, int32_t m4 ) API_WEAK;

/**
 * @brief  Makes the drone play a predefined movement
 * @param  type type of animation
 * @param  duration duration of the animation in seconds
 */
void ardrone_at_set_anim( anim_mayday_t type, int32_t duration ) API_WEAK;

/**
 * @brief  Instructs the drone to use its current position
 * as a reference for the horizontal plane.
 */
void ardrone_at_set_flat_trim( void ) API_WEAK;

/**
 * @brief Sets a manual trim (offset) on the drone commands
 */
void ardrone_at_set_manual_trims( float32_t trim_pitch, float32_t trim_roll, float32_t trim_yaw ) API_WEAK;

/**
 * @brief  Changes PID gains of the drone control loop
 * @param  user_ctrl_gains gains to be set
 */
void ardrone_at_set_control_gains( api_control_gains_t* user_ctrl_gains ) API_WEAK;

/**
 * @brief  Changes the tracking parameters (only in UE_IHM_PO mode)
 * @param  params : new params
 */
void ardrone_at_set_vision_track_params( api_vision_tracker_params_t* params ) API_WEAK;

/**
 * @brief  Start/stop raw capture on the drone.
 * @param  active : 1 for start capture, 0 to stop
 */
void ardrone_at_raw_capture( uint8_t active ) API_WEAK;

/**
 * @brief  Capture a frame in raw format on the drone.
 */
void ardrone_at_raw_picture(void) API_WEAK;

/**
 * @brief Changes the type of object to detect
 */
void ardrone_at_cad( CAD_TYPE type, float32_t tag_size ) API_WEAK;

/**
 * @brief Change the broadcasted video channel
 */
void ardrone_at_zap( ZAP_VIDEO_CHANNEL channel ) API_WEAK;

/**
 * @brief    Plays a led animation
 */
void ardrone_at_set_led_animation ( LED_ANIMATION_IDS anim_id, float32_t freq, uint32_t duration_sec ) API_WEAK;

/**
 * @fn     Set vision update options (only in UE_IHM_PO mode)
 * @param  user_vision_option : new option
 */
void ardrone_at_set_vision_update_options( int32_t user_vision_option ) API_WEAK;

/**
 * @brief  Used internally at Parrot - sets the drones position as seen by polaris
 * @param  x_polaris : x of ardrone position seen by polaris
 * @param  y_polaris : y of ardrone position seen by polaris
 * @param  defined : tells if polaris data are valid or not
 * @param  time_us : time in us
 */
void ardrone_at_set_polaris_pos( float32_t x_polaris, float32_t y_polaris, float32_t psi_polaris, bool_t defined, int32_t time_us ) API_WEAK;

/**
 * @brief  Used internally at Parrot - sets the drones position as seen by vicon
 * @param  time : vicon timestamp
 * @param  frame_number : vicon frame_number
 * @param  latency : vicon latency
 * @param  global_translation : vicon global translation
 * @param  global_rotation_euler : vicon global rotation (euler angles)
 */
void ardrone_at_set_vicon_data(struct timeval time, int32_t frame_number, float32_t latency, vector31_t global_translation, vector31_t global_rotation_euler);

/**
 * @brief     Send to drone a identified configuration
 * @param  param : parameter to set or update
 * @param  ses_id : session id
 * @param  usr_id : user id
 * @param  app_id : application id
 * @param  value : value to apply to the parameter
 */
void ardrone_at_set_toy_configuration_ids(const char* param,char* ses_id, char* usr_id, char* app_id,  const char* value) API_WEAK;

/**
 * @brief Asks the drone to reset the communication watchdog.
 * If no command is received by the drone
 */
void ardrone_at_reset_com_watchdog( void ) API_WEAK;

/**
 * @brief Asks the drone to purge log files
 */
void ardrone_at_reset_logs( void ) API_WEAK;

/**
 * @brief Asks the drone we receive ackknowledges
 */
void ardrone_at_update_control_mode(ARDRONE_CONTROL_MODE control_mode) API_WEAK;

/**
 * @brief Asks the drone to send control mode
 */
void ardrone_at_configuration_get_ctrl_mode( void ) API_WEAK;
void ardrone_at_custom_configuration_get_ctrl_mode(void) API_WEAK;

/**
 * @brief Tells the drone we received the control mode
 */
void ardrone_at_configuration_ack_ctrl_mode( void ) API_WEAK;

void ardrone_at_set_autonomous_flight( int32_t isActive );

typedef enum
  {
    ARDRONE_CALIBRATION_DEVICE_MAGNETOMETER = 0,
    ARDRONE_CALIBRATION_DEVICE_NUMBER,
  } ardrone_calibration_device_t;

/**
 * @brief Start the calibration of the given device
 */
void ardrone_at_set_calibration (int32_t deviceId);

/**
 * @brief Sets the drone motor command directly
 * This is enables only on prototypes drones.
 */
void ardrone_at_set_pwm( int32_t p1, int32_t p2, int32_t p3, int32_t p4 ) API_WEAK;

/**
 * @fn ardrone_at_set_progress_cmd
 * @brief Sends the drone progressive commands
 * @param flag Use 1 << value of ARDRONE_PROGRESSIVE_CMD_FLAG_XXX to use a flag
 * @param phi Left/right angle between -1 to +1 - negative values bend leftward.
 * @param roll Front/back angle between -1 to +1 - negative values bend forward.
 * @param gaz Vertical speed - negative values make the drone go down.
 * @param yaw Angular speed - negative values make the drone spin left.
 * @param magneto_psi floating value between -1 to +1.
 * @param magneto_psi_accuracy floating value between -1 to +1
 * This function allows the client program to control the drone by giving it a front/back
 * and left/right bending order, a vertical speed order, and a rotation order.
 * All values are given as a percentage of the maximum bending angles (in degrees),
 * vertical speed (in millimeters per second) and angular speed (in degrees per second).
 */
void ardrone_at_set_progress_cmd_with_magneto( int32_t flag, float32_t phi, float32_t theta, float32_t gaz, float32_t yaw,  float32_t magneto_psi, float32_t magneto_psi_accuracy );
void ardrone_at_set_progress_cmd( int32_t flag, float32_t phi, float32_t theta, float32_t gaz, float32_t yaw);

/*****************************************************************
*                       AT CONFIG FUNCTIONS
*****************************************************************/
#undef ARDRONE_CONFIG_KEY_IMM
#undef ARDRONE_CONFIG_KEY_REF
#undef ARDRONE_CONFIG_KEY_STR
#define ARDRONE_CONFIG_KEY_IMM(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, DEFAULT, CALLBACK)
#define ARDRONE_CONFIG_KEY_REF(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, DEFAULT, CALLBACK)
#define ARDRONE_CONFIG_KEY_STR(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, DEFAULT, CALLBACK)
#include <config_keys.h> // must be included before to have types

#undef ARDRONE_CONFIG_KEY_IMM
#undef ARDRONE_CONFIG_KEY_REF
#undef ARDRONE_CONFIG_KEY_STR
#define ARDRONE_CONFIG_KEY_IMM(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, DEFAULT, CALLBACK) ARDRONE_CONFIGURATION_PROTOTYPE(NAME);
#define ARDRONE_CONFIG_KEY_REF(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, DEFAULT, CALLBACK)
#define ARDRONE_CONFIG_KEY_STR(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, DEFAULT, CALLBACK) ARDRONE_CONFIGURATION_PROTOTYPE(NAME);
#include <config_keys.h> // must be included before to have types

/********************************************************************
*                        CONFIG FUNCTIONS
********************************************************************/
#undef ARDRONE_CONFIG_KEY_IMM
#undef ARDRONE_CONFIG_KEY_REF
#undef ARDRONE_CONFIG_KEY_STR
#define ARDRONE_CONFIG_KEY_IMM(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, DEFAULT, CALLBACK)
#define ARDRONE_CONFIG_KEY_REF(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, DEFAULT, CALLBACK)
#define ARDRONE_CONFIG_KEY_STR(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, DEFAULT, CALLBACK)
#include <config_keys.h> // must be included before to have types

#undef ARDRONE_CONFIG_KEY_IMM
#undef ARDRONE_CONFIG_KEY_REF
#undef ARDRONE_CONFIG_KEY_STR
#define ARDRONE_CONFIG_KEY_IMM(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, DEFAULT, CALLBACK) \
  C_TYPE get_##NAME(void) API_WEAK; \
  C_RESULT set_##NAME(C_TYPE val) API_WEAK;
#define ARDRONE_CONFIG_KEY_REF(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, DEFAULT, CALLBACK) \
  C_TYPE_PTR get_##NAME(void) API_WEAK; \
  C_RESULT set_##NAME(C_TYPE_PTR val) API_WEAK;
#define ARDRONE_CONFIG_KEY_STR(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, DEFAULT, CALLBACK) \
  C_TYPE_PTR get_##NAME(void) API_WEAK; \
  C_RESULT set_##NAME(C_TYPE_PTR val) API_WEAK;
// Generate all accessors functions prototypes
#include <config_keys.h>


#undef ARDRONE_CONFIG_KEY_IMM
#undef ARDRONE_CONFIG_KEY_REF
#undef ARDRONE_CONFIG_KEY_STR
#define ARDRONE_CONFIG_KEY_IMM(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, DEFAULT, CALLBACK)
#define ARDRONE_CONFIG_KEY_REF(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, DEFAULT, CALLBACK)
#define ARDRONE_CONFIG_KEY_STR(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, DEFAULT, CALLBACK)
#include <config_keys.h> // must be included before to have types

#undef ARDRONE_CONFIG_KEY_IMM
#undef ARDRONE_CONFIG_KEY_REF
#undef ARDRONE_CONFIG_KEY_STR
#define ARDRONE_CONFIG_KEY_IMM(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, DEFAULT, CALLBACK) C_TYPE NAME;
#define ARDRONE_CONFIG_KEY_REF(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, DEFAULT, CALLBACK) C_TYPE NAME;
#define ARDRONE_CONFIG_KEY_STR(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, DEFAULT, CALLBACK) C_TYPE NAME;

// Fill structure fields types
typedef struct _ardrone_config_t
{
#include <config_keys.h>
}
_ATTRIBUTE_PACKED_ ardrone_config_t;

void reset_ardrone_config(void);

/********************************************************************
 *                        USER FUNCTIONS
 ********************************************************************/
#define MULTICONFIG_ID_SIZE 9
#define SESSION_NAME_SIZE 1024
#define USER_NAME_SIZE 1024
#define APPLI_NAME_SIZE 1024
#define ROOT_NAME_SIZE 256
#define FLIGHT_NAME_SIZE 256

typedef struct _ardrone_user_t
{
  char ident[MULTICONFIG_ID_SIZE];
  char description[USER_NAME_SIZE];
} ardrone_user_t;

typedef struct _ardrone_users_t
{
  int userCount;
  ardrone_user_t *userList;
} ardrone_users_t;

void ardrone_refresh_user_list(void); // Ask for a userlist refresh
void ardrone_switch_to_user(const char *new_user); // Can be used to create user
void ardrone_switch_to_user_id(const char *new_user_id); // Must be used only with existing users
ardrone_users_t *ardrone_get_user_list(void); // Get a list of users (MUST BE FREED BY A ardrone_free_user_list CALL)
void ardrone_free_user_list (ardrone_users_t **users); // Free an ardrone_users_t list allocated by ardrone_get_user_list


#endif // _ARDRONE_API_H_
