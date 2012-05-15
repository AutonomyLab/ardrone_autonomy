/**
 *  \file     navdata_common.h
 *  \brief    Common navdata configuration
 *  \author   Sylvain Gaeremynck <sylvain.gaeremynck@parrot.com>
 */

#ifndef _NAVDATA_COMMON_H_
#define _NAVDATA_COMMON_H_

/*------------------------------------------ NAVDATA STRUCTURES DECLARATIONS ---------------------------------------------------------------*/

#include <config.h>
#include <vision_common.h>

#include <VP_Os/vp_os_types.h>
#include <Maths/maths.h>
#include <Maths/matrices.h>

#if defined(_MSC_VER)
	#define _ATTRIBUTE_PACKED_
	/* Asks Visual C++ to pack structures from now on*/
	#pragma pack(1)
#else
	#define _ATTRIBUTE_PACKED_  __attribute__ ((packed))
#endif

// Define constants for gyrometers handling
typedef enum {
  GYRO_X    = 0,
  GYRO_Y    = 1,
  GYRO_Z    = 2,
  NB_GYROS  = 3
} def_gyro_t;


// Define constants for accelerometers handling
typedef enum {
  ACC_X   = 0,
  ACC_Y   = 1,
  ACC_Z   = 2,
  NB_ACCS = 3
} def_acc_t;

/**
 * \struct _velocities_t
 * \brief Velocities in float32_t format
 */
typedef struct _velocities_t {
  float32_t x;
  float32_t y;
  float32_t z;
} velocities_t;

// Default control loops gains TODO Put these values in flash memory
// To avoid divisions in embedded software, gains are defined as ratios where
//  - the numerator is an integer
//  - the denominator is an integer

/**
 *  \var     CTRL_DEFAULT_NUM_PQ_KP
 *  \brief   Numerator of default proportionnal gain for pitch (p) and roll (q) angular rate control loops
 */
/**
 *  \var     CTRL_DEFAULT_NUM_EA_KP
 *  \brief   Numerator of default proportionnal gain for Euler Angle control loops
 */
/**
 *  \var     CTRL_DEFAULT_NUM_EA_KI
 *  \brief   Numerator of default integral gain for Euler Angle control loops
 */

#define CTRL_DEFAULT_NUM_PQ_KP_NO_SHELL /*30000  26000 */ 40000
#define CTRL_DEFAULT_NUM_EA_KP_NO_SHELL /*7000  9000  7000 */ 8000
#define CTRL_DEFAULT_NUM_EA_KI_NO_SHELL /*4000  7000 6000 */ 7000

#define CTRL_DEFAULT_NUM_PQ_KP_SHELL /*30000 23000*/ 40000
#define CTRL_DEFAULT_NUM_EA_KP_SHELL /*9000 10000*/  9000
#define CTRL_DEFAULT_NUM_EA_KI_SHELL /*5000 9000*/   8000

/**
 *  \var     CTRL_DEFAULT_NUM_H_R
 *  \brief   Numerator of default proportionnal gain for yaw (r) angular rate control loop
 */
#define CTRL_DEFAULT_NUM_R_KP 200000

/**
 *  \var     CTRL_DEFAULT_NUM_R_KI
 *  \brief   Numerator of default integral gain for yaw control loops
 */
#define CTRL_DEFAULT_NUM_R_KI 10000

/**
 *  \var     CTRL_DEFAULT_DEN_W
 *  \brief   Denominator of default proportionnal gain of pitch (p) roll (q) and yaw (r) angular rate control loops
 */
#define CTRL_DEFAULT_DEN_W 1024.0 //2^10

/**
 *  \var     CTRL_DEFAULT_DEN_EA
 *  \brief   Denominator of default gains for Euler Angle control loops
 */
#define CTRL_DEFAULT_DEN_EA 1024.0 //2^10

/**
 *  \var     CTRL_DEFAULT_NUM_ALT_KP
 *  \brief   Numerator of default proportionnal gain for Altitude control loop
 */
#ifdef NEW_ALT_FUSION
#define CTRL_DEFAULT_NUM_ALT_KP 4000
#else
#define CTRL_DEFAULT_NUM_ALT_KP 2000
#endif
/**
 *  \var     CTRL_DEFAULT_NUM_ALT_KI
 *  \brief   Numerator of default integral gain for Altitude control loop
 */
#ifdef NEW_ALT_FUSION
	#define CTRL_DEFAULT_NUM_ALT_KI 2000
#else
	#define CTRL_DEFAULT_NUM_ALT_KI 400
#endif
/**
 *  \var     CTRL_DEFAULT_NUM_ALT_KD
 *  \brief   Numerator of default derivative gain for Altitude control loop
 */
#define CTRL_DEFAULT_NUM_VZ_KP 200

/**
 *  \var     CTRL_DEFAULT_NUM_ALT_TD
 *  \brief   Numerator of default derivative time constant gain for Altitude control loop
 */
#define CTRL_DEFAULT_NUM_VZ_KI 100

/**
 *  \var     CTRL_DEFAULT_DEN_ALT
 *  \brief   Denominator of default gains for Altitude control loop
 */
#define CTRL_DEFAULT_DEN_ALT 1024.0

/**
 *  \var     CTRL_DEFAULT_NUM_HOVER_KP
 *  \brief   Numerator of default proportionnal gain for hovering control loop
 */
#define CTRL_DEFAULT_NUM_HOVER_KP_SHELL /*5000* 8000*/ 8000
#define CTRL_DEFAULT_NUM_HOVER_KP_NO_SHELL /*6000 12000 5000*/ 7000

/**
 *  \var     CTRL_DEFAULT_NUM_HOVER_KP
 *  \brief   Numerator of default proportionnal gain for hovering beacon control loop
 */
#define CTRL_DEFAULT_NUM_HOVER_B_KP_SHELL 1200
#define CTRL_DEFAULT_NUM_HOVER_B_KP_NO_SHELL 1200

/**
 *  \var     CTRL_DEFAULT_NUM_HOVER_KI
 *  \brief   Numerator of default integral gain for hovering control loop
 */
#define CTRL_DEFAULT_NUM_HOVER_KI_SHELL /*3000 10000*/ 8000
#define CTRL_DEFAULT_NUM_HOVER_KI_NO_SHELL /*3000 8000 5000*/ 6000

/**
 *  \var     CTRL_DEFAULT_NUM_HOVER_KI
 *  \brief   Numerator of default integral gain for hovering beacon control loop
 */
#define CTRL_DEFAULT_NUM_HOVER_B_KI_SHELL 500
#define CTRL_DEFAULT_NUM_HOVER_B_KI_NO_SHELL 500

/*
 *  \var     CTRL_DEFAULT_DEN_HOVER
 *  \brief   Numerator of default proportionnal gain for hovering control loop
 */
#define CTRL_DEFAULT_DEN_HOVER 32768.0 //2^15


/* Timeout for mayday maneuvers*/
static const int32_t MAYDAY_TIMEOUT[ARDRONE_NB_ANIM_MAYDAY] = {
    1000,  // ARDRONE_ANIM_PHI_M30_DEG
    1000,  // ARDRONE_ANIM_PHI_30_DEG
    1000,  // ARDRONE_ANIM_THETA_M30_DEG
    1000,  // ARDRONE_ANIM_THETA_30_DEG
    1000,  // ARDRONE_ANIM_THETA_20DEG_YAW_200DEG
    1000,  // ARDRONE_ANIM_THETA_20DEG_YAW_M200DEG
    5000,  // ARDRONE_ANIM_TURNAROUND
    5000,  // ARDRONE_ANIM_TURNAROUND_GODOWN
    2000,  // ARDRONE_ANIM_YAW_SHAKE
    5000,  // ARDRONE_ANIM_YAW_DANCE
    5000,  // ARDRONE_ANIM_PHI_DANCE
    5000,  // ARDRONE_ANIM_THETA_DANCE
    5000,  // ARDRONE_ANIM_VZ_DANCE
    5000,  // ARDRONE_ANIM_WAVE
    5000,  // ARDRONE_ANIM_PHI_THETA_MIXED
    5000,  // ARDRONE_ANIM_DOUBLE_PHI_THETA_MIXED
};

#define NAVDATA_SEQUENCE_DEFAULT  1

#define NAVDATA_HEADER  0x55667788

#define NAVDATA_MAX_SIZE 2048
#define NAVDATA_MAX_CUSTOM_TIME_SAVE 20

/* !!! Warning !!! - changing the value below would break compatibility with older applications
 * DO NOT CHANGE THIS  */
#define NB_NAVDATA_DETECTION_RESULTS 4


/**
 * @brief Tags identifying navdata blocks in a Navdata UDP packet
 * This tag is stored in the first two bytes of any navdata block (aka 'option').
 */

#define NAVDATA_OPTION_DEMO(STRUCTURE,NAME,TAG)  TAG = 0,
#define NAVDATA_OPTION(STRUCTURE,NAME,TAG)       TAG ,
#define NAVDATA_OPTION_CKS(STRUCTURE,NAME,TAG)   NAVDATA_NUM_TAGS, TAG = 0xFFFF

typedef enum _navdata_tag_t {
	#include <navdata_keys.h>
} navdata_tag_t;

#define NAVDATA_OPTION_MASK(option) ( 1 << (option) )
#define NAVDATA_OPTION_FULL_MASK    ((1<<NAVDATA_NUM_TAGS)-1)


typedef struct _navdata_option_t {
  uint16_t  tag;
  uint16_t  size;
#if defined _MSC_VER
  uint8_t   *data;
#else
  uint8_t   data[];
#endif
} navdata_option_t;


/**
 * @brief Navdata structure sent over the network.
 */
typedef struct _navdata_t {
  uint32_t    header;			/*!< Always set to NAVDATA_HEADER */
  uint32_t    ardrone_state;    /*!< Bit mask built from def_ardrone_state_mask_t */
  uint32_t    sequence;         /*!< Sequence number, incremented for each sent packet */
  bool_t      vision_defined;

  navdata_option_t  options[1];
}_ATTRIBUTE_PACKED_ navdata_t;


/**
 * All navdata options can be extended (new values AT THE END) except navdata_demo whose size must be constant across versions
 * New navdata options may be added, but must not be sent in navdata_demo mode unless requested by navdata_options.
 */

/*----------------------------------------------------------------------------*/
/**
 * @brief Minimal navigation data for all flights.
 */
typedef struct _navdata_demo_t {
  uint16_t    tag;					  /*!< Navdata block ('option') identifier */
  uint16_t    size;					  /*!< set this to the size of this structure */

  uint32_t    ctrl_state;             /*!< Flying state (landed, flying, hovering, etc.) defined in CTRL_STATES enum. */
  uint32_t    vbat_flying_percentage; /*!< battery voltage filtered (mV) */

  float32_t   theta;                  /*!< UAV's pitch in milli-degrees */
  float32_t   phi;                    /*!< UAV's roll  in milli-degrees */
  float32_t   psi;                    /*!< UAV's yaw   in milli-degrees */

  int32_t     altitude;               /*!< UAV's altitude in centimeters */

  float32_t   vx;                     /*!< UAV's estimated linear velocity */
  float32_t   vy;                     /*!< UAV's estimated linear velocity */
  float32_t   vz;                     /*!< UAV's estimated linear velocity */

  uint32_t    num_frames;			  /*!< streamed frame index */ // Not used -> To integrate in video stage.

  // Camera parameters compute by detection
  matrix33_t  detection_camera_rot;   /*!<  Deprecated ! Don't use ! */
  vector31_t  detection_camera_trans; /*!<  Deprecated ! Don't use ! */
  uint32_t	  detection_tag_index;    /*!<  Deprecated ! Don't use ! */

  uint32_t	  detection_camera_type; /*!<  Type of tag searched in detection */

  // Camera parameters compute by drone
  matrix33_t  drone_camera_rot;		/*!<  Deprecated ! Don't use ! */
  vector31_t  drone_camera_trans;	/*!<  Deprecated ! Don't use ! */
}_ATTRIBUTE_PACKED_ navdata_demo_t;



/*----------------------------------------------------------------------------*/
/**
 * @brief Last navdata option that *must* be included at the end of all navdata packets
 * + 6 bytes
 */
typedef struct _navdata_cks_t {
  uint16_t  tag;
  uint16_t  size;

  // Checksum for all navdatas (including options)
  uint32_t  cks;
}_ATTRIBUTE_PACKED_ navdata_cks_t;


/*----------------------------------------------------------------------------*/
/**
 * @brief Timestamp
 * + 6 bytes
 */
typedef struct _navdata_time_t {
  uint16_t  tag;
  uint16_t  size;

  uint32_t  time;  /*!< 32 bit value where the 11 most significant bits represents the seconds, and the 21 least significant bits are the microseconds. */
}_ATTRIBUTE_PACKED_ navdata_time_t;



/*----------------------------------------------------------------------------*/
/**
 * @brief Raw sensors measurements
 */
typedef struct _navdata_raw_measures_t {
  uint16_t  tag;
  uint16_t  size;

  // +12 bytes
  uint16_t  raw_accs[NB_ACCS];    // filtered accelerometers
  uint16_t  raw_gyros[NB_GYROS];  // filtered gyrometers
  uint16_t  raw_gyros_110[2];     // gyrometers  x/y 110 deg/s
  uint32_t  vbat_raw;             // battery voltage raw (mV)
  uint16_t  us_debut_echo;
  uint16_t  us_fin_echo;
  uint16_t  us_association_echo;
  uint16_t  us_distance_echo;
  uint16_t  us_courbe_temps;
  uint16_t  us_courbe_valeur;
  uint16_t  us_courbe_ref;
  uint16_t  flag_echo_ini;
}_ATTRIBUTE_PACKED_ navdata_raw_measures_t;


typedef struct _navdata_phys_measures_t {
  uint16_t   tag;
  uint16_t   size;

  float32_t   accs_temp;
  uint16_t    gyro_temp;
  float32_t   phys_accs[NB_ACCS];
  float32_t   phys_gyros[NB_GYROS];
  uint32_t    alim3V3;              // 3.3volt alim [LSB]
  uint32_t    vrefEpson;            // ref volt Epson gyro [LSB]
  uint32_t    vrefIDG;              // ref volt IDG gyro [LSB]
}_ATTRIBUTE_PACKED_ navdata_phys_measures_t;


typedef struct _navdata_gyros_offsets_t {
  uint16_t   tag;
  uint16_t   size;

  float32_t offset_g[NB_GYROS];
}_ATTRIBUTE_PACKED_ navdata_gyros_offsets_t;


typedef struct _navdata_euler_angles_t {
  uint16_t   tag;
  uint16_t   size;

  float32_t   theta_a;
  float32_t   phi_a;
}_ATTRIBUTE_PACKED_ navdata_euler_angles_t;


typedef struct _navdata_references_t {
  uint16_t   tag;
  uint16_t   size;

  int32_t   ref_theta;
  int32_t   ref_phi;
  int32_t   ref_theta_I;
  int32_t   ref_phi_I;
  int32_t   ref_pitch;
  int32_t   ref_roll;
  int32_t   ref_yaw;
  int32_t   ref_psi;
}_ATTRIBUTE_PACKED_ navdata_references_t;


typedef struct _navdata_trims_t {
  uint16_t   tag;
  uint16_t   size;

  float32_t angular_rates_trim_r;
  float32_t euler_angles_trim_theta;
  float32_t euler_angles_trim_phi;
}_ATTRIBUTE_PACKED_ navdata_trims_t;

typedef struct _navdata_rc_references_t {
  uint16_t   tag;
  uint16_t   size;

  int32_t    rc_ref_pitch;
  int32_t    rc_ref_roll;
  int32_t    rc_ref_yaw;
  int32_t    rc_ref_gaz;
  int32_t    rc_ref_ag;
}_ATTRIBUTE_PACKED_ navdata_rc_references_t;


typedef struct _navdata_pwm_t {
  uint16_t   tag;
  uint16_t   size;

  uint8_t     motor1;
  uint8_t     motor2;
  uint8_t     motor3;
  uint8_t     motor4;
  uint8_t	  sat_motor1;
  uint8_t	  sat_motor2;
  uint8_t	  sat_motor3;
  uint8_t	  sat_motor4;
  int32_t     gaz_feed_forward;
  int32_t     gaz_altitude;
  float32_t   altitude_integral;
  float32_t   vz_ref;
  int32_t     u_pitch;
  int32_t     u_roll;
  int32_t     u_yaw;
  float32_t   yaw_u_I;
  int32_t     u_pitch_planif;
  int32_t     u_roll_planif;
  int32_t     u_yaw_planif;
  float32_t   u_gaz_planif;
  uint16_t    current_motor1;
  uint16_t    current_motor2;
  uint16_t    current_motor3;
  uint16_t    current_motor4;
}_ATTRIBUTE_PACKED_ navdata_pwm_t;


typedef struct _navdata_altitude_t {
  uint16_t   tag;
  uint16_t   size;

  int32_t   altitude_vision;
  float32_t altitude_vz;
  int32_t   altitude_ref;
  int32_t   altitude_raw;

	float32_t	obs_accZ;
	float32_t obs_alt;
	vector31_t obs_x;
	uint32_t 	obs_state;
	vector21_t	est_vb;
	uint32_t 	est_state ;

}_ATTRIBUTE_PACKED_ navdata_altitude_t;


typedef struct _navdata_vision_raw_t {
  uint16_t   tag;
  uint16_t   size;

  float32_t vision_tx_raw;
  float32_t vision_ty_raw;
  float32_t vision_tz_raw;
}_ATTRIBUTE_PACKED_ navdata_vision_raw_t;


typedef struct _navdata_vision_t {
  uint16_t   tag;
  uint16_t   size;

  uint32_t   vision_state;
  int32_t    vision_misc;
  float32_t  vision_phi_trim;
  float32_t  vision_phi_ref_prop;
  float32_t  vision_theta_trim;
  float32_t  vision_theta_ref_prop;

  int32_t    new_raw_picture;
  float32_t  theta_capture;
  float32_t  phi_capture;
  float32_t  psi_capture;
  int32_t    altitude_capture;
  uint32_t   time_capture;     // time in TSECDEC format (see config.h)
  velocities_t body_v;

  float32_t  delta_phi;
  float32_t  delta_theta;
  float32_t  delta_psi;

	uint32_t  gold_defined;
	uint32_t  gold_reset;
	float32_t gold_x;
	float32_t gold_y;
}_ATTRIBUTE_PACKED_ navdata_vision_t;


typedef struct _navdata_vision_perf_t {
  uint16_t   tag;
  uint16_t   size;

  // +44 bytes
  float32_t  time_szo;
  float32_t  time_corners;
  float32_t  time_compute;
  float32_t  time_tracking;
  float32_t  time_trans;
  float32_t  time_update;
	float32_t  time_custom[NAVDATA_MAX_CUSTOM_TIME_SAVE];
}_ATTRIBUTE_PACKED_ navdata_vision_perf_t;


typedef struct _navdata_trackers_send_t {
  uint16_t   tag;
  uint16_t   size;

  int32_t locked[DEFAULT_NB_TRACKERS_WIDTH * DEFAULT_NB_TRACKERS_HEIGHT];
  screen_point_t point[DEFAULT_NB_TRACKERS_WIDTH * DEFAULT_NB_TRACKERS_HEIGHT];
}_ATTRIBUTE_PACKED_ navdata_trackers_send_t;


typedef struct _navdata_vision_detect_t {
	/* !! Change the function 'navdata_server_reset_vision_detect()' if this structure is modified !! */
  uint16_t   tag;
  uint16_t   size;

  uint32_t   nb_detected;
  uint32_t   type[NB_NAVDATA_DETECTION_RESULTS];
  uint32_t   xc[NB_NAVDATA_DETECTION_RESULTS];
  uint32_t   yc[NB_NAVDATA_DETECTION_RESULTS];
  uint32_t   width[NB_NAVDATA_DETECTION_RESULTS];
  uint32_t   height[NB_NAVDATA_DETECTION_RESULTS];
  uint32_t   dist[NB_NAVDATA_DETECTION_RESULTS];
  float32_t  orientation_angle[NB_NAVDATA_DETECTION_RESULTS];
}_ATTRIBUTE_PACKED_ navdata_vision_detect_t;

typedef struct _navdata_vision_of_t {
  uint16_t   tag;
  uint16_t   size;

  float32_t   of_dx[5];
  float32_t   of_dy[5];
}_ATTRIBUTE_PACKED_ navdata_vision_of_t;


typedef struct _navdata_watchdog_t {
  uint16_t   tag;
  uint16_t   size;

  // +4 bytes
  int32_t    watchdog;
}_ATTRIBUTE_PACKED_ navdata_watchdog_t;

typedef struct _navdata_adc_data_frame_t {
  uint16_t  tag;
  uint16_t  size;

  uint32_t  version;
  uint8_t   data_frame[32];
}_ATTRIBUTE_PACKED_ navdata_adc_data_frame_t;

typedef struct _navdata_video_stream_t {
  uint16_t  tag;
  uint16_t  size;

  uint8_t 	quant;			// quantizer reference used to encode frame [1:31]
  uint32_t	frame_size;		// frame size (bytes)
  uint32_t	frame_number;	// frame index
  uint32_t	atcmd_ref_seq;  // atmcd ref sequence number
  uint32_t	atcmd_mean_ref_gap;	// mean time between two consecutive atcmd_ref (ms)
  float32_t atcmd_var_ref_gap;
  uint32_t	atcmd_ref_quality; // estimator of atcmd link quality
}_ATTRIBUTE_PACKED_ navdata_video_stream_t;


typedef struct _navdata_games_t {
  uint16_t  tag;
  uint16_t  size;
  uint32_t  double_tap_counter;
  uint32_t  finish_line_counter;
}_ATTRIBUTE_PACKED_ navdata_games_t;

#if defined(_MSC_VER)
	/* Go back to default packing policy */
	#pragma pack()
#endif

#endif // _NAVDATA_COMMON_H_

