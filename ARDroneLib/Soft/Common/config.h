/**
***************************************************************************
*
* Copyright (C) 2007 Parrot S.A.
*
***************************************************************************
*/

#ifndef _CONFIG_H_
#define _CONFIG_H_

#include <VP_Os/vp_os_types.h>
#ifdef _WIN32
	#include <win32_custom.h>
#else
	#include <generated_custom.h>
	#include <autoconf.h>
#endif

#undef ARDRONE_PIC_VERSION

#define USE_NAVDATA_IP
#define USE_AT_IP
#define USE_VIDEO_IP

///////////////////////////////////////////////
// Video configuration
#define VIDEO_ENABLE            1

///////////////////////////////////////////////
// Vision configuration

#define VISION_ENABLE           1
// #define VISION_TEST_MODE
#define ARDRONE_VISION_DETECT

///////////////////////////////////////////////
// Navdata configuration

#define NAVDATA_ENABLE          1
#define ND_WRITE_TO_FILE

# define NAVDATA_SUBSAMPLING     13 /* 200 / 15 fps = 13.3333 */

#if defined(NAVDATA_ENABLE)

# define NAVDATA_VISION_DETECT_INCLUDED
# define NAVDATA_TRIMS_INCLUDED
# define NAVDATA_WATCHDOG
# define NAVDATA_EULER_ANGLES_INCLUDED
# define NAVDATA_PHYS_MEASURES_INCLUDED
# define NAVDATA_TIME_INCLUDED
# define NAVDATA_RAW_MEASURES_INCLUDED
# define NAVDATA_GYROS_OFFSETS_INCLUDED
# define NAVDATA_REFERENCES_INCLUDED
# define NAVDATA_RC_REFERENCES_INCLUDED
# define NAVDATA_PWM_INCLUDED
# define NAVDATA_ALTITUDE_INCLUDED
# define NAVDATA_VISION_INCLUDED
# define NAVDATA_VISION_PERF_INCLUDED
# define NAVDATA_TRACKERS_SEND
# define NAVDATA_VIDEO_STREAM_INCLUDED

#endif // ! NAVDATA_ENABLE

#ifndef ARDRONE_VISION_DETECT
# undef NAVDATA_VISION_DETECT_INCLUDED
#endif // ! ARDRONE_VISION_DETECT

///////////////////////////////////////////////
// Wifi configuration

#define USE_AUTOIP              VP_COM_AUTOIP_DISABLE /* VP_COM_AUTOIP_ENABLE */

#define WIFI_NETMASK            "255.255.255.0"
#define WIFI_GATEWAY            WIFI_ARDRONE_IP
#define WIFI_SERVER             WIFI_ARDRONE_IP
#define WIFI_SECURE             0

#define WIFI_BASE_ADDR          0xc0a80100      			// 192.168.1.0
#define MULTICAST_BASE_ADDR     0xe0010100      			// 224.1.1.0
#define WIFI_BROADCAST_ADDR 	(WIFI_BASE_ADDR | 0xff)    //XXX.XXX.XXX.255

// Configure infrastructure mode given wifi driver compilation
#define WIFI_INFRASTRUCTURE     0

#define WIFI_PASSKEY            "9F1C3EE11CBA230B27BF1C1B6F"

#define FTP_PORT				5551
#define AUTH_PORT				5552
#define NAVDATA_PORT            5554
#define VIDEO_PORT              5555
#define AT_PORT                 5556
#define RAW_CAPTURE_PORT        5557
#define PRINTF_PORT             5558
#define CONTROL_PORT            5559

///////////////////////////////////////////////
// Wired configuration

#define WIRED_MOBILE_IP         WIFI_MOBILE_IP

///////////////////////////////////////////////
// Serial link configuration

#ifdef USE_ELINUX

#define SERIAL_LINK_0           "/dev/ttyPA0"
#define SERIAL_LINK_1           "/dev/ttyPA1"
#define SERIAL_LINK_2           "/dev/ttyPA2"

#endif

#ifdef USE_LINUX

#ifdef USE_MINGW32

#define SERIAL_LINK_0           ""
#define SERIAL_LINK_1           ""
#define SERIAL_LINK_2           ""

#else

// Only USE_LINUX is defined
#define SERIAL_LINK_0           "/dev/ttyUSB0" /* Serial link for navdata & ATCmd */
#define SERIAL_LINK_1           "/dev/ttyUSB1" /* Serial link for video */
#define SERIAL_LINK_2           "/dev/ser2" /* Serial link for adc */

#endif // USE_MINGW32

#endif // USE_LINUX

#define SL0_BAUDRATE             VP_COM_BAUDRATE_460800 /* baud rate for serial link 0 */
#define SL1_BAUDRATE             VP_COM_BAUDRATE_460800 /* baud rate for serial link 1 */
#define SL2_BAUDRATE             VP_COM_BAUDRATE_460800 /* baud rate for serial link 2 */

///////////////////////////////////////////////
// Defines & types used in shared data

#define ARDRONE_ORIENTATION_HISTORY_SIZE  256

#define TSECDEC     21 /* Defines used to format time ( seconds << 21 + useconds ) */
#define TUSECMASK   ((1 << TSECDEC) - 1)
#define TSECMASK    (0xffffffff & ~TUSECMASK)
#define TIME_TO_USEC    (0xffffffff & ~TUSECMASK)

#define VBAT_POWERING_OFF   9000    /* Minimum Battery Voltage [mV] to prevent damaging */

/* Syslog Configuration */
#define SYSLOG_NUM_BUFFERS        4     /* Number of actives buffers. When a buffer is full, it's dumped in file */
#define SYSLOG_BUFFER_SIZE        2048  /* Max number of bytes in a syslog buffer */
#define SYSLOG_BUFFER_DUMP_SIZE   128   /* Max number of bytes wrote at once during dump */


#define DEFAULT_MISC1_VALUE 2
#define DEFAULT_MISC2_VALUE 20
#define DEFAULT_MISC3_VALUE 2000
#define DEFAULT_MISC4_VALUE 3000


typedef enum {
  MISC_VAR1 = 0,
  MISC_VAR2,
  MISC_VAR3,
  MISC_VAR4,
  NB_MISC_VARS
} misc_var_t;

// Mayday scenarii
typedef enum {
	ARDRONE_ANIM_PHI_M30_DEG= 0,
	ARDRONE_ANIM_PHI_30_DEG,
	ARDRONE_ANIM_THETA_M30_DEG,
	ARDRONE_ANIM_THETA_30_DEG,
	ARDRONE_ANIM_THETA_20DEG_YAW_200DEG,
	ARDRONE_ANIM_THETA_20DEG_YAW_M200DEG,
	ARDRONE_ANIM_TURNAROUND,
	ARDRONE_ANIM_TURNAROUND_GODOWN,
	ARDRONE_ANIM_YAW_SHAKE,
	ARDRONE_ANIM_YAW_DANCE,
	ARDRONE_ANIM_PHI_DANCE,
	ARDRONE_ANIM_THETA_DANCE,
    ARDRONE_ANIM_VZ_DANCE,
	ARDRONE_ANIM_WAVE,
	ARDRONE_ANIM_PHI_THETA_MIXED,
	ARDRONE_ANIM_DOUBLE_PHI_THETA_MIXED,
	ARDRONE_NB_ANIM_MAYDAY
} anim_mayday_t;

// Bitfield definition for user input

typedef enum {
  ARDRONE_UI_BIT_AG             = 0,
  ARDRONE_UI_BIT_AB             = 1,
  ARDRONE_UI_BIT_AD             = 2,
  ARDRONE_UI_BIT_AH             = 3,
  ARDRONE_UI_BIT_L1             = 4,
  ARDRONE_UI_BIT_R1             = 5,
  ARDRONE_UI_BIT_L2             = 6,
  ARDRONE_UI_BIT_R2             = 7,
  ARDRONE_UI_BIT_SELECT         = 8,
  ARDRONE_UI_BIT_START          = 9,
  ARDRONE_UI_BIT_TRIM_THETA     = 18,
  ARDRONE_UI_BIT_TRIM_PHI       = 20,
  ARDRONE_UI_BIT_TRIM_YAW       = 22,
  ARDRONE_UI_BIT_X              = 24,
  ARDRONE_UI_BIT_Y              = 28,
} ardrone_ui_bitfield_t;


/// \enum def_ardrone_state_mask_t is a bit field representing ARDrone' state


// Define masks for ARDrone state
// 31                                                             0
//  x x x x x x x x x x x x x x x x x x x x x x x x x x x x x x x x -> state
//  | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | |
//  | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | FLY MASK : (0) ardrone is landed, (1) ardrone is flying
//  | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | VIDEO MASK : (0) video disable, (1) video enable
//  | | | | | | | | | | | | | | | | | | | | | | | | | | | | | VISION MASK : (0) vision disable, (1) vision enable
//  | | | | | | | | | | | | | | | | | | | | | | | | | | | | CONTROL ALGO : (0) euler angles control, (1) angular speed control
//  | | | | | | | | | | | | | | | | | | | | | | | | | | | ALTITUDE CONTROL ALGO : (0) altitude control inactive (1) altitude control active
//  | | | | | | | | | | | | | | | | | | | | | | | | | | USER feedback : Start button state
//  | | | | | | | | | | | | | | | | | | | | | | | | | Control command ACK : (0) None, (1) one received
//  | | | | | | | | | | | | | | | | | | | | | | | | Trim command ACK : (0) None, (1) one received
//  | | | | | | | | | | | | | | | | | | | | | | | Trim running : (0) none, (1) running
//  | | | | | | | | | | | | | | | | | | | | | | Trim result : (0) failed, (1) succeeded
//  | | | | | | | | | | | | | | | | | | | | | Navdata demo : (0) All navdata, (1) only navdata demo
//  | | | | | | | | | | | | | | | | | | | | Navdata bootstrap : (0) options sent in all or demo mode, (1) no navdata options sent
//  | | | | | | | | | | | | | | | | | | | | Motors status : (0) Ok, (1) Motors Com is down
//  | | | | | | | | | | | | | | | | | |
//  | | | | | | | | | | | | | | | | | Bit means that there's an hardware problem with gyrometers
//  | | | | | | | | | | | | | | | | VBat low : (1) too low, (0) Ok
//  | | | | | | | | | | | | | | | VBat high (US mad) : (1) too high, (0) Ok
//  | | | | | | | | | | | | | | Timer elapsed : (1) elapsed, (0) not elapsed
//  | | | | | | | | | | | | | Power : (0) Ok, (1) not enough to fly
//  | | | | | | | | | | | | Angles : (0) Ok, (1) out of range
//  | | | | | | | | | | | Wind : (0) Ok, (1) too much to fly
//  | | | | | | | | | | Ultrasonic sensor : (0) Ok, (1) deaf
//  | | | | | | | | | Cutout system detection : (0) Not detected, (1) detected
//  | | | | | | | | PIC Version number OK : (0) a bad version number, (1) version number is OK
//  | | | | | | | ATCodec thread ON : (0) thread OFF (1) thread ON
//  | | | | | | Navdata thread ON : (0) thread OFF (1) thread ON
//  | | | | | Video thread ON : (0) thread OFF (1) thread ON
//  | | | | Acquisition thread ON : (0) thread OFF (1) thread ON
//  | | | CTRL watchdog : (1) delay in control execution (> 5ms), (0) control is well scheduled // Check frequency of control loop
//  | | ADC Watchdog : (1) delay in uart2 dsr (> 5ms), (0) uart2 is good // Check frequency of uart2 dsr (com with adc)
//  | Communication Watchdog : (1) com problem, (0) Com is ok // Check if we have an active connection with a client
//  Emergency landing : (0) no emergency, (1) emergency

typedef enum {
  ARDRONE_FLY_MASK            = 1 << 0,  /*!< FLY MASK : (0) ardrone is landed, (1) ardrone is flying */
  ARDRONE_VIDEO_MASK          = 1 << 1,  /*!< VIDEO MASK : (0) video disable, (1) video enable */
  ARDRONE_VISION_MASK         = 1 << 2,  /*!< VISION MASK : (0) vision disable, (1) vision enable */
  ARDRONE_CONTROL_MASK        = 1 << 3,  /*!< CONTROL ALGO : (0) euler angles control, (1) angular speed control */
  ARDRONE_ALTITUDE_MASK       = 1 << 4,  /*!< ALTITUDE CONTROL ALGO : (0) altitude control inactive (1) altitude control active */
  ARDRONE_USER_FEEDBACK_START = 1 << 5,  /*!< USER feedback : Start button state */
  ARDRONE_COMMAND_MASK        = 1 << 6,  /*!< Control command ACK : (0) None, (1) one received */
  ARDRONE_FW_FILE_MASK        = 1 << 7,  /* Firmware file is good (1) */
  ARDRONE_FW_VER_MASK         = 1 << 8,  /* Firmware update is newer (1) */
//  ARDRONE_FW_UPD_MASK         = 1 << 9,  /* Firmware update is ongoing (1) */
  ARDRONE_NAVDATA_DEMO_MASK   = 1 << 10, /*!< Navdata demo : (0) All navdata, (1) only navdata demo */
  ARDRONE_NAVDATA_BOOTSTRAP   = 1 << 11, /*!< Navdata bootstrap : (0) options sent in all or demo mode, (1) no navdata options sent */
  ARDRONE_MOTORS_MASK  	      = 1 << 12, /*!< Motors status : (0) Ok, (1) Motors problem */
  ARDRONE_COM_LOST_MASK       = 1 << 13, /*!< Communication Lost : (1) com problem, (0) Com is ok */
  ARDRONE_VBAT_LOW            = 1 << 15, /*!< VBat low : (1) too low, (0) Ok */
  ARDRONE_USER_EL             = 1 << 16, /*!< User Emergency Landing : (1) User EL is ON, (0) User EL is OFF*/
  ARDRONE_TIMER_ELAPSED       = 1 << 17, /*!< Timer elapsed : (1) elapsed, (0) not elapsed */
  ARDRONE_ANGLES_OUT_OF_RANGE = 1 << 19, /*!< Angles : (0) Ok, (1) out of range */
  ARDRONE_ULTRASOUND_MASK     = 1 << 21, /*!< Ultrasonic sensor : (0) Ok, (1) deaf */
  ARDRONE_CUTOUT_MASK         = 1 << 22, /*!< Cutout system detection : (0) Not detected, (1) detected */
  ARDRONE_PIC_VERSION_MASK    = 1 << 23, /*!< PIC Version number OK : (0) a bad version number, (1) version number is OK */
  ARDRONE_ATCODEC_THREAD_ON   = 1 << 24, /*!< ATCodec thread ON : (0) thread OFF (1) thread ON */
  ARDRONE_NAVDATA_THREAD_ON   = 1 << 25, /*!< Navdata thread ON : (0) thread OFF (1) thread ON */
  ARDRONE_VIDEO_THREAD_ON     = 1 << 26, /*!< Video thread ON : (0) thread OFF (1) thread ON */
  ARDRONE_ACQ_THREAD_ON       = 1 << 27, /*!< Acquisition thread ON : (0) thread OFF (1) thread ON */
  ARDRONE_CTRL_WATCHDOG_MASK  = 1 << 28, /*!< CTRL watchdog : (1) delay in control execution (> 5ms), (0) control is well scheduled */
  ARDRONE_ADC_WATCHDOG_MASK   = 1 << 29, /*!< ADC Watchdog : (1) delay in uart2 dsr (> 5ms), (0) uart2 is good */
  ARDRONE_COM_WATCHDOG_MASK   = 1 << 30, /*!< Communication Watchdog : (1) com problem, (0) Com is ok */
  ARDRONE_EMERGENCY_MASK      = 1 << 31  /*!< Emergency landing : (0) no emergency, (1) emergency */
} def_ardrone_state_mask_t;

static INLINE uint32_t ardrone_set_state_with_mask( uint32_t state, uint32_t mask, bool_t value )
{
  state &= ~mask;
  if( value )
    state |= mask;

  return state;
}

/** Returns a bit value of state from a mask
 * This function is used to test bits from a bit field like def_ardrone_state_mask_t
 *
 * @param state a 32 bits word we want to test
 * @param mask a mask that tells the bit to test
 * @return TRUE if bit is set, FALSE otherwise
 */
static INLINE bool_t ardrone_get_mask_from_state( uint32_t state, uint32_t mask )
{
  return state & mask ? TRUE : FALSE;
}

/** Convert time value from proprietary format to (unsigned int) micro-second value
 *
 * @param time value in proprietary format
 * @return time value in micro-second value (unsigned int)
 */
static INLINE uint32_t ardrone_time_to_usec( uint32_t time )
{
  return ((uint32_t)(time >> TSECDEC) * 1000000 + (uint32_t)(time & TUSECMASK));
}


#ifdef DEBUG_MODE
#define POSIX_DEBUG
#endif // DEBUG_MODE

#endif // _CONFIG_H_
