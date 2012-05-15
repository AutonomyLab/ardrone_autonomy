/******************************************************************************
 *                        COPYRIGHT PARROT 2010
 ******************************************************************************
 * PARROT A.R.Drone SDK
 *---------------------------------------------------------------------------*/
/**
 * @file   config_keys.h
 * @brief  Definition of all the configuration values for the drone.
 *
 * This file mainly consists in a list of ARDRONE_CONFIG_KEY_xxx macros
 * whose arguments describe the available configuration values.
 * By redefining those macros and including this file anywhere else in the
 * project, it is possible to build a set of C variables, functions or macros
 * based on the configuration descriptions.
 *
 ******************************************************************************/


#ifndef CFG_STRINGIFY
	#define CFG_STRINGIFY(x) #x
#endif



#ifndef CONFIG_KEYS_STRING_TYPE_DEFINED
#define CONFIG_KEYS_STRING_TYPE_DEFINED
  #define STRING_T_SIZE 128
  typedef char string_t[STRING_T_SIZE+1];
#endif // ! CONFIG_KEYS_STRING_TYPE_DEFINED
#include <Maths/maths.h>
#include <VLIB/video_codec.h>

#ifndef CONFIG_KEYS_RW_ENUM_DEFINED
#define CONFIG_KEYS_RW_ENUM_DEFINED

/**
 * @brief Describes the behaviour of a drone configuration variable.
 */
  enum {
    K_READ    = 1,     /*!< Value can be read by a remote client */
    K_WRITE   = 1<<1,  /*!< Value can be written by a remote client */
    K_NOBIND  = 1<<2,  /*!< Data are stored to the config.ini file, but not read from this file at startup.*/
    K_SHALLOW = 1<<3,  /*!< Data will no be stored to the config.ini file, nor read from this file at startup.*/
  };

  enum {
	  CAT_COMMON = 0,
	  CAT_APPLI  ,
	  CAT_USER   ,
	  CAT_SESSION,
	  NB_CONFIG_CATEGORIES
  };

  /* Stephane - multiconfiguration support */

  	  /* Size of the hexadecimal ID representing a custom configuration - example : 1234abcd */
		#define CUSTOM_CONFIGURATION_ID_LENGTH 8

		typedef struct{
			char id[CUSTOM_CONFIGURATION_ID_LENGTH+1];
			char description[1024];
		}custom_configuration_t;

		typedef struct
		{
			custom_configuration_t* list;
			int nb_configurations;
		}custom_configuration_list_t;

		extern const char * configuration_switching_commands[NB_CONFIG_CATEGORIES+1];
		extern const char * custom_configuration_headers[NB_CONFIG_CATEGORIES+1];
		extern const char * custom_configuration_id_keys[NB_CONFIG_CATEGORIES+1];
		extern custom_configuration_list_t available_configurations[NB_CONFIG_CATEGORIES];

		/**
		 * \brief Checks if a character is valid in a custom configuration identifier.
		 * Currently it checks if the character is an hexadecimal digit.
		 */
		C_RESULT configuration_check_config_id_char(const char session_id_char);
		/**
		 * \brief Checks if a string is a valid custom configuration identifier.
		 * Currently it checks if the string is made of 8 hexa. digits.
		 */
		C_RESULT configuration_check_config_id(const char * session_id);


  /* Stephane - multiconfiguration support */


#endif // ! CONFIG_KEYS_RW_ENUM_DEFINED

#ifndef CONFIG_KEYS_DEFINES_DEFINED

# ifdef INSIDE_FLIGHT
#   define MAX_EULER_ANGLES_REF (12000.0f * MDEG_TO_RAD)     /* EA control, maximum reference [rad] */
#   define MAX_OUTDOOR_EULER_ANGLES_REF (20000.0f * MDEG_TO_RAD)     /* EA control, maximum reference [rad] */
# else
#   define MAX_EULER_ANGLES_REF (12000.0f * MDEG_TO_RAD)    /* EA control, maximum reference [rad] */
#   define MAX_OUTDOOR_EULER_ANGLES_REF (20000.0f * MDEG_TO_RAD)     /* EA control, maximum reference [rad] */
# endif

# define CONFIG_KEYS_DEFINES_DEFINED
//Calibration renvoye par le PIC dans le cas ou il n'en a pas recut
#define DEFAULT_PWM_REF_GYRO                500
#define DEFAULT_GYRO_OFFSET_THR_X           4.0
#define DEFAULT_GYRO_OFFSET_THR_Y           4.0
#define DEFAULT_GYRO_OFFSET_THR_Z           0.5
//#define default_accs_offset          {{{ -2048.0f, 2048.0f, 2048.0f}}}
//#define default_accs_gain            {1.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f, 0.0f, -1.0f }
//#define default_gyros_offset         {{{ 1662.5f, 1662.5f, 1662.5f}}}
//#define default_gyros_gains          {{{ 395.0f * MDEG_TO_RAD, -395.0f * MDEG_TO_RAD, -207.5f * MDEG_TO_RAD }}}
//#define default_gyros110_offset      {{ 1662.5f, 1662.5f}}
//#define default_gyros110_gains       {{ 87.5f * MDEG_TO_RAD, -87.5f * MDEG_TO_RAD }}
#define default_motor_version         "0.0"
#define NULL_MAC					  "00:00:00:00:00:00"

/* Selection of navdata blocks sent when starting the 'demo' mode */
#define default_navdata_options ( NAVDATA_OPTION_MASK(NAVDATA_DEMO_TAG)|NAVDATA_OPTION_MASK(NAVDATA_VISION_DETECT_TAG) )

#define COMPILE_TIME ( (2010-1970)*(365+(2010-1970)/4)*(24)*(3600) )

extern const vector31_t default_accs_offset;
extern const matrix33_t default_accs_gain;
extern const vector31_t default_gyros_offset;
extern const vector31_t default_gyros_gains;
extern const vector21_t default_gyros110_offset;
extern const vector21_t default_gyros110_gains;

#define default_pwm_ref_gyro          DEFAULT_PWM_REF_GYRO
#define default_gyro_offset_thr_x     DEFAULT_GYRO_OFFSET_THR_X
#define default_gyro_offset_thr_y     DEFAULT_GYRO_OFFSET_THR_Y
#define default_gyro_offset_thr_z     DEFAULT_GYRO_OFFSET_THR_Z

#define default_euler_angle_ref_max    		MAX_EULER_ANGLES_REF
#define default_outdoor_euler_angle_ref_max	MAX_OUTDOOR_EULER_ANGLES_REF
# define default_altitude_max  			(3000)
# define default_altitude_min  			(50)
# define default_control_trim_z  		(0.0f * MDEG_TO_RAD)
# define default_control_iphone_tilt 	(20000.0f * MDEG_TO_RAD)
# define default_control_vz_max			(700.0f)
# define default_outdoor_control_vz_max	(1000.0f)
# define default_control_yaw			(100000.0f * MDEG_TO_RAD)
# define default_outdoor_control_yaw	(200000.0f * MDEG_TO_RAD)

#define default_enemy_colors			(ARDRONE_DETECTION_COLOR_ORANGE_GREEN)
#define default_groundstripe_colors     (ARDRONE_DETECTION_COLOR_ARRACE_FINISH_LINE)
#define default_detect_type				CAD_TYPE_NONE

#define DEFAULT_APPLICATION_DESC "Default application configuration"
#define DEFAULT_PROFILE_DESC "Default profile configuration"
#define DEFAULT_SESSION_DESC "Default session configuration"

#define CUSTOM_CONFIGURATION_DELETE_ALL_CMD "all"

#ifndef CARD_VERSION
#define CARD_VERSION 0x00
#endif

#endif // ! CONFIG_KEYS_DEFINES_DEFINED

/* ---- List of configuration properties - see the Developer Guide for a comprehensive description ---- */

/* Parameters attributes are : (KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, RW_CUSTOM, DEFAULT, CALLBACK)
 * Key : category of parameter, used by the ini file parser ; corresponds to a section inside the config.ini file.
 * Name : name of the parameter as found in the AT*CONFIG command, and as expected by ARDRONE_TOOL_CONFIGURATION_ADDEVENT

 */
#ifndef ARDRONE_CONFIG_KEY_IMM
	#define ARDRONE_CONFIG_KEY_IMM(a,b,c,d,e,f,g,h)
#endif
#ifndef ARDRONE_CONFIG_KEY_STR
	#define ARDRONE_CONFIG_KEY_STR(a,b,c,d,e,f,g,h)
#endif
#ifndef ARDRONE_CONFIG_KEY_REF
	#define ARDRONE_CONFIG_KEY_REF(a,b,c,d,e,f,g,h)
#endif

#ifndef ARDRONE_CONFIG_KEY_IMM_a10
#define ARDRONE_CONFIG_KEY_IMM_a10(a,b,c,d,e,f,g,h,i,j) ARDRONE_CONFIG_KEY_IMM(a,b,c,d,e,f,h,i)
#endif
#ifndef ARDRONE_CONFIG_KEY_STR_a10
#define ARDRONE_CONFIG_KEY_STR_a10(a,b,c,d,e,f,g,h,i,j) ARDRONE_CONFIG_KEY_STR(a,b,c,d,e,f,h,i)
#endif
#ifndef ARDRONE_CONFIG_KEY_REF_a10
#define ARDRONE_CONFIG_KEY_REF_a10(a,b,c,d,e,f,g,h,i,j) ARDRONE_CONFIG_KEY_REF(a,b,c,d,e,f,h,i)
#endif

ARDRONE_CONFIG_KEY_IMM_a10("general", num_version_config,  INI_INT,      int32_t,    int32_t*,     (K_READ|K_NOBIND), 0, CURRENT_NUM_VERSION_CONFIG,      default_config_callback,CAT_COMMON)
ARDRONE_CONFIG_KEY_IMM_a10("general", num_version_mb,      INI_INT,      int32_t,    int32_t*,     (K_READ|K_NOBIND), 0, CARD_VERSION,        			default_config_callback,CAT_COMMON)
ARDRONE_CONFIG_KEY_STR_a10("general", num_version_soft,    INI_STRING,   string_t,   char*,    	   (K_READ|K_NOBIND), 0, CURRENT_NUM_VERSION_SOFT,        default_config_callback,CAT_COMMON)
ARDRONE_CONFIG_KEY_STR_a10("general", soft_build_date,  	  INI_STRING,   string_t,   char*, (K_READ|K_NOBIND), 0, CURRENT_BUILD_DATE,              default_config_callback,CAT_COMMON)
ARDRONE_CONFIG_KEY_STR_a10("general", motor1_soft,    	  INI_STRING,   string_t,   char*,    	   (K_READ|K_NOBIND), 0, default_motor_version,           default_config_callback,CAT_COMMON)
ARDRONE_CONFIG_KEY_STR_a10("general", motor1_hard,    	  INI_STRING,   string_t,   char*,    	   (K_READ|K_NOBIND), 0, default_motor_version,           default_config_callback,CAT_COMMON)
ARDRONE_CONFIG_KEY_STR_a10("general", motor1_supplier, 	  INI_STRING,   string_t,   char*,    	   (K_READ|K_NOBIND), 0, default_motor_version,           default_config_callback,CAT_COMMON)
ARDRONE_CONFIG_KEY_STR_a10("general", motor2_soft,    	  INI_STRING,   string_t,   char*,         (K_READ|K_NOBIND), 0, default_motor_version,           default_config_callback,CAT_COMMON)
ARDRONE_CONFIG_KEY_STR_a10("general", motor2_hard,    	  INI_STRING,   string_t,   char*,    	   (K_READ|K_NOBIND), 0, default_motor_version,           default_config_callback,CAT_COMMON)
ARDRONE_CONFIG_KEY_STR_a10("general", motor2_supplier, 	  INI_STRING,   string_t,   char*,    	   (K_READ|K_NOBIND), 0, default_motor_version,           default_config_callback,CAT_COMMON)
ARDRONE_CONFIG_KEY_STR_a10("general", motor3_soft,    	  INI_STRING,   string_t,   char*,    	   (K_READ|K_NOBIND), 0, default_motor_version,           default_config_callback,CAT_COMMON)
ARDRONE_CONFIG_KEY_STR_a10("general", motor3_hard,    	  INI_STRING,   string_t,   char*,    	   (K_READ|K_NOBIND), 0, default_motor_version,           default_config_callback,CAT_COMMON)
ARDRONE_CONFIG_KEY_STR_a10("general", motor3_supplier,     INI_STRING,   string_t,   char*,    	   (K_READ|K_NOBIND), 0, default_motor_version,           default_config_callback,CAT_COMMON)
ARDRONE_CONFIG_KEY_STR_a10("general", motor4_soft,    	  INI_STRING,   string_t,   char*,         (K_READ|K_NOBIND), 0, default_motor_version,           default_config_callback,CAT_COMMON)
ARDRONE_CONFIG_KEY_STR_a10("general", motor4_hard,    	  INI_STRING,   string_t,   char*,    	   (K_READ|K_NOBIND), 0, default_motor_version,           default_config_callback,CAT_COMMON)
ARDRONE_CONFIG_KEY_STR_a10("general", motor4_supplier, 	  INI_STRING,   string_t,   char*,    	   (K_READ|K_NOBIND), 0, default_motor_version,           default_config_callback,CAT_COMMON)
ARDRONE_CONFIG_KEY_STR_a10("general", ardrone_name,        INI_STRING,   string_t,   char*,        (K_READ|K_WRITE ), 0, "My ARDrone",                    default_config_callback,CAT_COMMON)
ARDRONE_CONFIG_KEY_IMM_a10("general", flying_time,         INI_INT,      uint32_t,   uint32_t*,    (K_READ)         , 0, 0,                               default_config_callback,CAT_COMMON)
ARDRONE_CONFIG_KEY_IMM_a10("general", navdata_demo,        INI_BOOLEAN,  bool_t,     bool_t*,      (K_READ|K_WRITE) , 0, FALSE,                           navdata_demo_config_callback,CAT_COMMON)
ARDRONE_CONFIG_KEY_IMM_a10("general", navdata_options,     INI_INT,      int32_t,    int32_t*,     (K_READ|K_WRITE|K_NOBIND|K_SHALLOW), (K_READ|K_WRITE|K_NOBIND|K_SHALLOW), default_navdata_options, navdata_options_config_callback, CAT_APPLI)
ARDRONE_CONFIG_KEY_IMM_a10("general", com_watchdog,        INI_INT,      int32_t,    int32_t*,     (K_READ|K_WRITE) , 0, COM_INPUT_LANDING_TIME,          default_config_callback,CAT_COMMON)
ARDRONE_CONFIG_KEY_IMM_a10("general", video_enable,        INI_BOOLEAN,  bool_t,     bool_t*,      (K_READ|K_WRITE) , 0, TRUE,                            default_config_callback,CAT_COMMON)
ARDRONE_CONFIG_KEY_IMM_a10("general", vision_enable,       INI_BOOLEAN,  bool_t,     bool_t*,      (K_READ|K_WRITE) , 0, TRUE,                            default_config_callback,CAT_COMMON)
ARDRONE_CONFIG_KEY_IMM_a10("general", vbat_min,            INI_INT,      int32_t,    int32_t*,     (K_READ|K_NOBIND), 0, VBAT_POWERING_OFF,               default_config_callback,CAT_COMMON)

ARDRONE_CONFIG_KEY_REF_a10("control", accs_offset,         INI_VECTOR,   vector31_t, vector31_t*, (K_READ|K_NOBIND), 0, default_accs_offset,              default_config_callback,CAT_COMMON)
ARDRONE_CONFIG_KEY_REF_a10("control", accs_gains,          INI_MATRIX,   matrix33_t, matrix33_t*, (K_READ|K_NOBIND), 0, default_accs_gain,              default_config_callback,CAT_COMMON)
ARDRONE_CONFIG_KEY_REF_a10("control", gyros_offset,        INI_VECTOR,   vector31_t, vector31_t*, (K_READ|K_NOBIND), 0, default_gyros_offset,           default_config_callback,CAT_COMMON)
ARDRONE_CONFIG_KEY_REF_a10("control", gyros_gains,         INI_VECTOR,   vector31_t, vector31_t*, (K_READ|K_NOBIND), 0, default_gyros_gains,            default_config_callback,CAT_COMMON)
ARDRONE_CONFIG_KEY_REF_a10("control", gyros110_offset,     INI_VECTOR21, vector21_t, vector21_t*, (K_READ|K_NOBIND), 0, default_gyros110_offset,        default_config_callback,CAT_COMMON)
ARDRONE_CONFIG_KEY_REF_a10("control", gyros110_gains,      INI_VECTOR21, vector21_t, vector21_t*, (K_READ|K_NOBIND), 0, default_gyros110_gains,         default_config_callback,CAT_COMMON)
ARDRONE_CONFIG_KEY_IMM_a10("control", gyro_offset_thr_x,   INI_FLOAT,    float32_t,  float32_t*,  (K_READ|K_NOBIND), 0, default_gyro_offset_thr_x,      default_config_callback,CAT_COMMON)
ARDRONE_CONFIG_KEY_IMM_a10("control", gyro_offset_thr_y,   INI_FLOAT,    float32_t,  float32_t*,  (K_READ|K_NOBIND), 0, default_gyro_offset_thr_y,      default_config_callback,CAT_COMMON)
ARDRONE_CONFIG_KEY_IMM_a10("control", gyro_offset_thr_z,   INI_FLOAT,    float32_t,  float32_t*,  (K_READ|K_NOBIND), 0, default_gyro_offset_thr_z,      default_config_callback,CAT_COMMON)
ARDRONE_CONFIG_KEY_IMM_a10("control", pwm_ref_gyros,       INI_INT,      int32_t,    int32_t*,    (K_READ|K_NOBIND), 0, default_pwm_ref_gyro,           default_config_callback,CAT_COMMON)
ARDRONE_CONFIG_KEY_IMM_a10("control", control_level,       INI_INT,      int32_t,    int32_t*,    (K_READ|K_WRITE) , (K_READ|K_WRITE), 0,		default_config_callback,CAT_APPLI)
ARDRONE_CONFIG_KEY_IMM_a10("control", shield_enable,       INI_INT,      int32_t,    int32_t*,    (K_READ|K_WRITE) , 0, 1,                                 default_config_callback,CAT_COMMON)
ARDRONE_CONFIG_KEY_IMM_a10("control", euler_angle_max,     INI_FLOAT,    float32_t,  float32_t*,  (K_READ|K_WRITE) , (K_READ|K_WRITE), default_euler_angle_ref_max,       control_changed_config_callback,CAT_USER)
ARDRONE_CONFIG_KEY_IMM_a10("control", altitude_max,        INI_INT,    	int32_t,    int32_t*,     (K_READ|K_WRITE) , 0, default_altitude_max,              default_config_callback,CAT_COMMON)
ARDRONE_CONFIG_KEY_IMM_a10("control", altitude_min,        INI_INT,    	int32_t,    int32_t*,     (K_READ|K_WRITE) , 0, default_altitude_min,              default_config_callback,CAT_COMMON)
ARDRONE_CONFIG_KEY_IMM_a10("control", control_trim_z,      INI_FLOAT,    float32_t,  float32_t*,  (K_READ|K_WRITE) , (K_READ|K_WRITE), default_control_trim_z,            default_config_callback,CAT_USER)
ARDRONE_CONFIG_KEY_IMM_a10("control", control_iphone_tilt, INI_FLOAT,    float32_t,  float32_t*,  (K_READ|K_WRITE) , (K_READ|K_WRITE), default_control_iphone_tilt,       default_config_callback,CAT_USER)
ARDRONE_CONFIG_KEY_IMM_a10("control", control_vz_max,      INI_FLOAT,    float32_t,  float32_t*,  (K_READ|K_WRITE) , (K_READ|K_WRITE), default_control_vz_max,        	 control_changed_config_callback,CAT_USER)
ARDRONE_CONFIG_KEY_IMM_a10("control", control_yaw,         INI_FLOAT,    float32_t,  float32_t*,  (K_READ|K_WRITE) , (K_READ|K_WRITE), default_control_yaw,               control_changed_config_callback,CAT_USER)
ARDRONE_CONFIG_KEY_IMM_a10("control", outdoor,	       	  INI_BOOLEAN,  bool_t,     bool_t*,      (K_READ|K_WRITE) , 0, FALSE,				        	 control_config_callback,CAT_COMMON)
ARDRONE_CONFIG_KEY_IMM_a10("control", flight_without_shell,INI_BOOLEAN,  bool_t,     bool_t*,     (K_READ|K_WRITE) , 0, FALSE,				             default_config_callback,CAT_COMMON)
ARDRONE_CONFIG_KEY_IMM_a10("control", brushless,	          INI_BOOLEAN,  bool_t, bool_t*,  (K_READ|K_WRITE) , 0, TRUE,				             	 default_config_callback,CAT_COMMON)
ARDRONE_CONFIG_KEY_IMM_a10("control", autonomous_flight,   INI_BOOLEAN,  bool_t,     bool_t*,     (K_READ|K_WRITE) , 0, FALSE,				             default_config_callback,CAT_COMMON)
ARDRONE_CONFIG_KEY_IMM_a10("control", manual_trim,		  INI_BOOLEAN,  bool_t,     bool_t*,     (K_READ|K_WRITE), (K_READ|K_WRITE), FALSE,				             default_config_callback,CAT_USER)

ARDRONE_CONFIG_KEY_IMM_a10("control", indoor_euler_angle_max,     INI_FLOAT,    float32_t,  float32_t*,  (K_READ|K_WRITE), (K_READ|K_WRITE), default_euler_angle_ref_max,default_config_callback,CAT_USER)
ARDRONE_CONFIG_KEY_IMM_a10("control", indoor_control_vz_max,      INI_FLOAT,    float32_t,  float32_t*,  (K_READ|K_WRITE), (K_READ|K_WRITE), default_control_vz_max,     default_config_callback,CAT_USER)
ARDRONE_CONFIG_KEY_IMM_a10("control", indoor_control_yaw,         INI_FLOAT,    float32_t,  float32_t*,  (K_READ|K_WRITE), (K_READ|K_WRITE), default_control_yaw,        default_config_callback,CAT_USER)
ARDRONE_CONFIG_KEY_IMM_a10("control", outdoor_euler_angle_max,    INI_FLOAT,    float32_t,  float32_t*,  (K_READ|K_WRITE), (K_READ|K_WRITE), default_outdoor_euler_angle_ref_max, default_config_callback,CAT_USER)
ARDRONE_CONFIG_KEY_IMM_a10("control", outdoor_control_vz_max,     INI_FLOAT,    float32_t,  float32_t*,  (K_READ|K_WRITE), (K_READ|K_WRITE), default_outdoor_control_vz_max,      default_config_callback,CAT_USER)
ARDRONE_CONFIG_KEY_IMM_a10("control", outdoor_control_yaw,        INI_FLOAT,    float32_t,  float32_t*,  (K_READ|K_WRITE), (K_READ|K_WRITE), default_outdoor_control_yaw,         default_config_callback,CAT_USER)

ARDRONE_CONFIG_KEY_IMM_a10("control", flying_mode,        INI_INT,    int32_t,  int32_t*,   (K_READ|K_WRITE|K_NOBIND|K_SHALLOW), (K_READ|K_WRITE), 0,    		flying_mode_config_callback,CAT_SESSION)
ARDRONE_CONFIG_KEY_STR_a10("control", flight_anim,        INI_STRING,    string_t,  char*,  (K_READ|K_WRITE|K_NOBIND|K_SHALLOW), 0, "0,0",         flight_animation_selection_callback,CAT_COMMON)

ARDRONE_CONFIG_KEY_STR_a10("network", ssid_single_player,  INI_STRING,   string_t,   char*,       (K_READ|K_WRITE), 0, WIFI_NETWORK_NAME,               default_config_callback, CAT_COMMON)
ARDRONE_CONFIG_KEY_STR_a10("network", ssid_multi_player,   INI_STRING,   string_t,   char*,       (K_READ|K_WRITE), 0,  WIFI_NETWORK_NAME,               default_config_callback, CAT_COMMON)
ARDRONE_CONFIG_KEY_IMM_a10("network", wifi_mode,           INI_INT,      int32_t,    int32_t*,    (K_READ|K_WRITE), 0,  WIFI_MODE_INFRA,                               default_config_callback, CAT_COMMON)
ARDRONE_CONFIG_KEY_IMM_a10("network", secure,              INI_BOOLEAN,  bool_t,     bool_t*,     (K_READ|K_WRITE), 0,  0,                               default_config_callback, CAT_COMMON)
ARDRONE_CONFIG_KEY_STR_a10("network", passkey,             INI_STRING,   string_t,   char*,       (K_READ|K_WRITE), 0,  "",                              default_config_callback, CAT_COMMON)
ARDRONE_CONFIG_KEY_IMM_a10("network", navdata_port,        INI_INT,      int32_t,    int32_t*,    (K_READ|K_NOBIND), 0, NAVDATA_PORT,                    default_config_callback, CAT_COMMON)
ARDRONE_CONFIG_KEY_IMM_a10("network", video_port,          INI_INT,      int32_t,    int32_t*,    (K_READ|K_NOBIND), 0, VIDEO_PORT,                      default_config_callback, CAT_COMMON)
ARDRONE_CONFIG_KEY_IMM_a10("network", at_port,             INI_INT,      int32_t,    int32_t*,    (K_READ|K_NOBIND), 0, AT_PORT,                         default_config_callback, CAT_COMMON)
ARDRONE_CONFIG_KEY_IMM_a10("network", cmd_port,            INI_INT,      int32_t,    int32_t*,    (K_READ|K_NOBIND), 0, CONTROL_PORT,                    default_config_callback, CAT_COMMON)
ARDRONE_CONFIG_KEY_STR_a10("network", owner_mac,           INI_STRING,   string_t,   char*,       (K_READ|K_WRITE), 0, NULL_MAC,                        owner_mac_callback,      CAT_COMMON)
ARDRONE_CONFIG_KEY_IMM_a10("network", owner_ip_address,    INI_INT,      uint32_t,   uint32_t*,   (K_READ|K_WRITE), 0, 0,                               default_config_callback, CAT_COMMON)
ARDRONE_CONFIG_KEY_IMM_a10("network", local_ip_address,    INI_INT,      uint32_t,   uint32_t*,   (K_READ|K_WRITE), 0, 0,                               default_config_callback, CAT_COMMON)
ARDRONE_CONFIG_KEY_IMM_a10("network", broadcast_address,   INI_INT,      uint32_t,   uint32_t*,   (K_READ|K_WRITE), 0, 0,                               default_config_callback, CAT_COMMON)

ARDRONE_CONFIG_KEY_IMM_a10("pic",     ultrasound_freq,     INI_INT,      int32_t,    int32_t*,    (K_READ|K_WRITE), 0, ADC_CMD_SELECT_ULTRASOUND_25Hz,    ultrasound_freq_callback,CAT_COMMON)
ARDRONE_CONFIG_KEY_IMM_a10("pic",     ultrasound_watchdog, INI_INT,      int32_t,    int32_t*,    (K_READ|K_WRITE), 0, 3,                                 default_config_callback,CAT_COMMON)
ARDRONE_CONFIG_KEY_IMM_a10("pic",     pic_version        , INI_INT,      int32_t,    int32_t*,    (K_READ|K_NOBIND), 0, 0x00040030,              default_config_callback,CAT_COMMON)

ARDRONE_CONFIG_KEY_IMM_a10("video",   camif_fps,           INI_INT,      int32_t,    int32_t*,    (K_READ|K_NOBIND), 0, 15,                     default_config_callback,CAT_COMMON)
ARDRONE_CONFIG_KEY_IMM_a10("video",   camif_buffers,       INI_INT,      int32_t,    int32_t*,    (K_READ|K_NOBIND), 0, CAMIF_NUM_BUFFERS,      default_config_callback,CAT_COMMON)
ARDRONE_CONFIG_KEY_IMM_a10("video",   num_trackers,        INI_INT,      int32_t,    int32_t*,    (K_READ|K_NOBIND), 0, 12,                     default_config_callback,CAT_COMMON)
ARDRONE_CONFIG_KEY_IMM_a10("video",   bitrate,             INI_INT,      int32_t,    int32_t*,	  (K_READ|K_SHALLOW), (K_READ|K_WRITE), 0,               default_config_callback,CAT_APPLI)
ARDRONE_CONFIG_KEY_IMM_a10("video",   bitrate_ctrl_mode,   INI_INT,      int32_t,    int32_t*,    (K_READ|K_SHALLOW), (K_READ|K_WRITE), 0,               default_config_callback,CAT_APPLI)
ARDRONE_CONFIG_KEY_IMM_a10("video",   video_codec,               INI_INT,      int32_t,    int32_t*,    (K_READ|K_SHALLOW), (K_READ|K_WRITE), UVLC_CODEC,                codec_config_callback, CAT_APPLI)
ARDRONE_CONFIG_KEY_IMM_a10("video",   video_channel,       INI_INT,      int32_t,    int32_t*,    (K_READ|K_WRITE|K_NOBIND|K_SHALLOW), (K_READ|K_WRITE), 0,   video_channel_selection_callback,CAT_SESSION)

ARDRONE_CONFIG_KEY_STR_a10("leds",    leds_anim,           INI_STRING,   string_t,    char*,      (K_READ|K_WRITE|K_NOBIND|K_SHALLOW), 0, "0,0,0",           leds_animation_selection_callback,CAT_COMMON)

ARDRONE_CONFIG_KEY_IMM_a10("detect",  enemy_colors, 	  INI_INT,      int32_t,    int32_t*,    (K_READ|K_WRITE), 0, default_enemy_colors,      		 default_config_callback,CAT_COMMON)
ARDRONE_CONFIG_KEY_IMM_a10("detect",  groundstripe_colors, INI_INT,      int32_t,    int32_t*,    (K_READ|K_WRITE), (K_READ|K_WRITE), default_groundstripe_colors,   	 default_config_callback,CAT_SESSION)
ARDRONE_CONFIG_KEY_IMM_a10("detect",  enemy_without_shell, INI_INT,      int32_t,    int32_t*,    (K_READ|K_WRITE), 0, 0,      		 					 default_config_callback,CAT_COMMON)
ARDRONE_CONFIG_KEY_IMM_a10("detect",  detect_type, 		  INI_INT,      int32_t,    int32_t*,    (K_READ|K_WRITE|K_NOBIND), (K_READ|K_WRITE|K_NOBIND), default_detect_type,      detect_type_callback,CAT_SESSION)
ARDRONE_CONFIG_KEY_IMM_a10("detect",  detections_select_h, 		  INI_INT,      int32_t,    int32_t*,    (K_READ|K_WRITE|K_NOBIND), (K_READ|K_WRITE), 0,      detections_select_callback,CAT_SESSION)
ARDRONE_CONFIG_KEY_IMM_a10("detect",  detections_select_v_hsync,  INI_INT,      int32_t,    int32_t*,    (K_READ|K_WRITE|K_NOBIND), (K_READ|K_WRITE), 0,      detections_select_callback,CAT_SESSION)
ARDRONE_CONFIG_KEY_IMM_a10("detect",  detections_select_v, 		  INI_INT,      int32_t,    int32_t*,    (K_READ|K_WRITE|K_NOBIND), (K_READ|K_WRITE), 0,      detections_select_callback,CAT_SESSION)

ARDRONE_CONFIG_KEY_IMM_a10("syslog",  output,              INI_INT,      int32_t,    int32_t*,    (K_READ|K_WRITE), 0, (UART_PRINT|WIFI_PRINT|FLASH_PRINT), default_config_callback,CAT_COMMON)
ARDRONE_CONFIG_KEY_IMM_a10("syslog",  max_size,            INI_INT,      int32_t,    int32_t*,    (K_READ|K_WRITE), 0, (100*1024),                          default_config_callback,CAT_COMMON)
ARDRONE_CONFIG_KEY_IMM_a10("syslog",  nb_files,            INI_INT,      int32_t,    int32_t*,    (K_READ|K_WRITE), 0, 5,                                 default_config_callback,CAT_COMMON)

/*- Multi configuration management -*/
ARDRONE_CONFIG_KEY_STR_a10("custom",  application_id,      INI_STRING,   string_t,    char*,    (K_READ|K_WRITE|K_NOBIND|K_SHALLOW), (K_READ|K_WRITE), "00000000",     default_config_callback,CAT_SESSION)
ARDRONE_CONFIG_KEY_STR_a10("custom",  application_desc,    INI_STRING,   string_t,    char*,    (K_READ|K_WRITE), (K_READ|K_WRITE), DEFAULT_APPLICATION_DESC,          application_desc_callback,CAT_APPLI)

ARDRONE_CONFIG_KEY_STR_a10("custom",  profile_id,          INI_STRING,   string_t,    char*,    (K_READ|K_WRITE|K_NOBIND|K_SHALLOW), (K_READ|K_WRITE), "00000000",     default_config_callback,CAT_SESSION)
ARDRONE_CONFIG_KEY_STR_a10("custom",  profile_desc,        INI_STRING,   string_t,    char*,    (K_READ|K_WRITE),  (K_READ|K_WRITE),                                  DEFAULT_PROFILE_DESC,              profile_desc_callback,CAT_USER)

ARDRONE_CONFIG_KEY_STR_a10("custom",  session_id,          INI_STRING,   string_t,    char*,    (K_READ|K_WRITE|K_NOBIND|K_SHALLOW), (K_READ|K_WRITE), "00000000",     default_config_callback,CAT_SESSION)
ARDRONE_CONFIG_KEY_STR_a10("custom",  session_desc,        INI_STRING,   string_t,    char*,    (K_READ|K_WRITE),  (K_READ|K_WRITE),                                  DEFAULT_SESSION_DESC,              session_desc_callback,CAT_SESSION)

/* Prevents a further inclusion of config_keys from generating garbage code */
#undef ARDRONE_CONFIG_KEY_IMM_a10
#undef ARDRONE_CONFIG_KEY_REF_a10
#undef ARDRONE_CONFIG_KEY_STR_a10
#undef ARDRONE_CONFIG_KEY_IMM
#undef ARDRONE_CONFIG_KEY_REF
#undef ARDRONE_CONFIG_KEY_STR
