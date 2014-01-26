/**
 *  \file     control_states.h
 *  \brief    Control states declaration for control loop & ihm display
 *  \author   Sylvain Gaeremynck <sylvain.gaeremynck@parrot.com>
 *  \version  1.0
 */

#ifndef _CONTROL_STATES_H_
#define _CONTROL_STATES_H_

#ifdef CTRL_STATES_STRING
typedef char ctrl_string_t[32];
#endif

// Macros to customize preprocessing
// If this CTRL_STATES_STRING is not defined, this file declare several enumeration
// If user defines CTRL_STATES_STRING, this file will define several array of strings.
// Each string being equivalent to an enumeration entry

#ifdef CTRL_STATES_STRING
#define CVAR(a) #a  /* Using # causes the first argument after the # to be returned as a string in quotes */
#define CVARZ(a) #a
#else
#define CVARZ(a) a = 0
#define CVAR(a) a
#endif

/**
 * \enum  CONTROL_STATE
 * \brief control loop thread states.
 * \brief this defines major states
*/
#ifndef DO_NOT_INCLUDE_MAJOR_CTRL_STATES
#ifdef CTRL_STATES_STRING
static ctrl_string_t ctrl_states[] = {
#else
typedef enum {
#endif
  CVARZ( CTRL_DEFAULT ),
  CVAR( CTRL_INIT ),
  CVAR( CTRL_LANDED ),
  CVAR( CTRL_FLYING ),
  CVAR( CTRL_HOVERING ),
  CVAR( CTRL_TEST ),
  CVAR( CTRL_TRANS_TAKEOFF ),
  CVAR( CTRL_TRANS_GOTOFIX ),
  CVAR( CTRL_TRANS_LANDING ),
  CVAR( CTRL_TRANS_LOOPING ),
  //CVAR( CTRL_TRANS_NO_VISION ),
#ifndef CTRL_STATES_STRING
  CTRL_NUM_STATES
} CTRL_STATES;
#else
};
#endif
#endif

/**
 * \enum  FLYING_STATES
 * \brief flying states.
 * \brief this is one of the minor state
*/
#ifndef DO_NOT_INCLUDE_MINOR_CTRL_STATES
#ifdef CTRL_STATES_STRING
static ctrl_string_t flying_states[] = {
#else
typedef enum {
#endif
  CVARZ( FLYING_OK ),
  CVAR( FLYING_LOST_ALT ),
  CVAR( FLYING_LOST_ALT_GO_DOWN ),
  CVAR( FLYING_ALT_OUT_ZONE ),
  CVAR( FLYING_COMBINED_YAW ), 
  CVAR( FLYING_BRAKE ),
  CVAR( FLYING_NO_VISION ),
  #ifndef CTRL_STATES_STRING
} FLYING_STATES;
#else
};
#endif
#endif

/**
 * \enum  HOVERING_STATES
 * \brief flying states.
 * \brief this is one of the minor state
*/
#ifndef DO_NOT_INCLUDE_MINOR_CTRL_STATES
#ifdef CTRL_STATES_STRING
static ctrl_string_t hovering_states[] = {
#else
typedef enum {
#endif
  CVARZ( HOVERING_OK ),
  CVAR( HOVERING_YAW ),
  CVAR( HOVERING_YAW_LOST_ALT),
  CVAR( HOVERING_YAW_LOST_ALT_GO_DOWN),
  CVAR( HOVERING_ALT_OUT_ZONE),
  CVAR( HOVERING_YAW_ALT_OUT_ZONE),
  CVAR( HOVERING_LOST_ALT ),
  CVAR( HOVERING_LOST_ALT_GO_DOWN ),
  CVAR( HOVERING_LOST_COM ),
  CVAR( LOST_COM_LOST_ALT ),
  CVAR( LOST_COM_LOST_ALT_TOO_LONG ),
  CVAR( LOST_COM_ALT_OK ),
  CVAR( HOVERING_MAGNETO_CALIB ),
  CVAR( HOVERING_DEMO )
#ifndef CTRL_STATES_STRING
} HOVERING_STATES;
#else
};
#endif
#endif

/**
 * \enum  TAKEOFF_TRANS_STATES
 * \brief take off states.
 * \brief this is one of the minor state
*/
#ifndef DO_NOT_INCLUDE_MINOR_CTRL_STATES
#ifdef CTRL_STATES_STRING
static ctrl_string_t takeoff_trans_states[] = {
#else
typedef enum {
#endif
  CVARZ( TAKEOFF_GROUND ),
  CVAR( TAKEOFF_AUTO ),
  #ifndef CTRL_STATES_STRING
} TAKEOFF_TRANS_STATES;
#else
};
#endif
#endif

/**
 * \enum  GOTOFIX_TRANS_STATES
 * \brief gotofix substates.
 * \brief this is one of the minor state
*/
#ifndef DO_NOT_INCLUDE_MINOR_CTRL_STATES
#ifdef CTRL_STATES_STRING
static ctrl_string_t gotofix_trans_states[] = {
#else
typedef enum {
#endif
  CVARZ( GOTOFIX_OK ),
  CVAR( GOTOFIX_LOST_ALT ),
  CVAR( GOTOFIX_YAW ),
  #ifndef CTRL_STATES_STRING
} GOTOFIX_TRANS_STATES;
#else
};
#endif
#endif

/**
 * \enum  LANDING_TRANS_STATES
 * \brief landing states.
 * \brief this is one of the minor state
*/
#ifndef DO_NOT_INCLUDE_MINOR_CTRL_STATES
#ifdef CTRL_STATES_STRING
static ctrl_string_t landing_trans_states[] = {
#else
typedef enum {
#endif
  CVARZ( LANDING_CLOSED_LOOP ),
  CVAR( LANDING_OPEN_LOOP ),
  CVAR( LANDING_OPEN_LOOP_FAST )
#ifndef CTRL_STATES_STRING
} LANDING_TRANS_STATES;
#else
};
#endif
#endif

/**
 * \enum  TAKEOFF_TRANS_STATES
 * \brief take off states.
 * \brief this is one of the minor state
*/
#ifndef DO_NOT_INCLUDE_MINOR_CTRL_STATES
#ifdef CTRL_STATES_STRING
static ctrl_string_t looping_trans_states[] = {
#else
typedef enum {
#endif
	CVARZ( LOOPING_IMPULSION),
	CVAR( LOOPING_OPEN_LOOP_CTRL ),
	CVAR( LOOPING_PLANIF_CTRL )
  #ifndef CTRL_STATES_STRING
} LOOPING_TRANS_STATES;
#else
};
#endif
#endif




#ifdef CTRL_STATES_STRING
#ifndef DO_NOT_INCLUDE_CTRL_STATES_LINK
/**
 * control link tates
 * \brief This array is used to link minor states & major state's strings
*/
static ctrl_string_t* control_states_link[] = {
  NULL,
  NULL,
  NULL,
  flying_states,
  hovering_states,
  NULL,
  takeoff_trans_states,
  gotofix_trans_states,
  landing_trans_states,
  looping_trans_states
};
#endif
#endif

#endif // _CONTROL_STATES_H_
