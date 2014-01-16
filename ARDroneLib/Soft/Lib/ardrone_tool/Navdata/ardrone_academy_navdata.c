#include <stdio.h>
#include <sys/time.h>
#include <ardrone_api.h>
#include <control_states.h>

#include <ardrone_tool/ardrone_tool.h>
#include <ardrone_tool/ardrone_tool_configuration.h>
#include <ardrone_tool/Academy/academy_stage_recorder.h>
#include <ardrone_tool/Academy/academy_download.h>
#include <ardrone_tool/Navdata/ardrone_navdata_client.h>
#include <ardrone_tool/Navdata/ardrone_academy_navdata.h>

#include <ardrone_tool/UI/ardrone_input.h>
#include <ardrone_tool/ardrone_version.h>
#include <ardrone_tool/Video/video_com_stage.h>
#include <ardrone_tool/Video/video_stage_encoded_recorder.h>

#include <VP_Os/vp_os_assert.h>
#include <VP_Os/vp_os_print.h>

#include <utils/ardrone_date.h>

#define TAKEOFF_TIMEOUT_SEC (5)

typedef enum
{
	SCREENSHOT_STATE_IDLE,
	SCREENSHOT_STATE_NEEDED,
	SCREENSHOT_STATE_INPROGRESS,
} SCREENSHOT_STATE;

typedef enum
{
	TAKEOFF_STATE_IDLE,
	TAKEOFF_STATE_NEEDED,
	TAKEOFF_STATE_WAIT_USERBOX,
        TAKEOFF_STATE_CANCELLING,
} TAKEOFF_STATE;

typedef enum
{
	RECORD_STATE_IDLE,
	RECORD_STATE_NEEDED,
	RECORD_STATE_WAIT_USERBOX,
} RECORD_STATE;

typedef enum
{
	USERBOX_STATE_STARTING,
	USERBOX_STATE_STARTED,
	USERBOX_STATE_STOPPING,
	USERBOX_STATE_STOPPED,
} USERBOX_STATE;

typedef struct
{
	/**
	 * variable to know if setting is needed
	 */
	bool_t wasEmergency;
	bool_t needSetEmergency;
	TAKEOFF_STATE takeoff_state;
        time_t takeoff_time;
	RECORD_STATE record_state;
	USERBOX_STATE userbox_state;
	SCREENSHOT_STATE screenshot_state;
	time_t userbox_time;
        vp_os_mutex_t aan_mutex;
    uint32_t usbFreeTime;
    bool_t droneStoppedUsbRecording;
    bool_t takeOffWasCancelled;

	/**
	 * Strings to display in interface
	 */
	bool_t isTakeOff;
	bool_t isEmergency;
    uint32_t lastState;
	bool_t cameraIsReady;
        bool_t usbIsReady;
        bool_t usbRecordInProgress;
    bool_t internalRecordInProgress;
} ardrone_academy_navdata_state_t;

static const codec_type_t usbRecordCodec = MP4_360P_H264_720P_CODEC;
codec_type_t deviceWifiRecordCodec = MP4_360P_H264_720P_CODEC;
static ardrone_academy_navdata_state_t navdata_state;

#define LOCK_ND_MUTEX() vp_os_mutex_lock (&navdata_state.aan_mutex)
#define UNLOCK_ND_MUTEX() vp_os_mutex_unlock (&navdata_state.aan_mutex)
#define TRYLOCK_ND_MUTEX() vp_os_mutex_trylock (&navdata_state.aan_mutex)

bool_t ardrone_academy_navdata_takeoff(void)
{
    bool_t result = FALSE;
    bool_t isBatteryLow = (ARDRONE_VBAT_LOW & navdata_state.lastState) ? TRUE : FALSE;
    bool_t shouldNotDo = (TRUE == isBatteryLow && FALSE == navdata_state.isTakeOff) ? TRUE : FALSE;
    if(navdata_state.takeoff_state == TAKEOFF_STATE_IDLE &&
       FALSE == shouldNotDo)
{
  LOCK_ND_MUTEX ();
      navdata_state.takeoff_state = TAKEOFF_STATE_NEEDED;
      navdata_state.takeoff_time = time (NULL);
        UNLOCK_ND_MUTEX ();
        result = TRUE;
    }
    return result;
}

bool_t ardrone_academy_navdata_emergency(void)
{
    bool_t result = TRUE;
  LOCK_ND_MUTEX ();
  navdata_state.needSetEmergency = TRUE;
  UNLOCK_ND_MUTEX ();
    return result;
}

bool_t ardrone_academy_navdata_record(void)
{
    bool_t result = FALSE;
  if(navdata_state.record_state == RECORD_STATE_IDLE)
    {
      LOCK_ND_MUTEX ();
      navdata_state.record_state = RECORD_STATE_NEEDED;
      UNLOCK_ND_MUTEX ();
        result = TRUE;
    }
    return result;
}

bool_t ardrone_academy_navdata_screenshot(void)
{
    bool_t result = FALSE;
  if(navdata_state.screenshot_state == SCREENSHOT_STATE_IDLE)
    {
      LOCK_ND_MUTEX ();
      navdata_state.screenshot_state = SCREENSHOT_STATE_NEEDED;
      UNLOCK_ND_MUTEX ();
        result = TRUE;
    }
    return result;
}

bool_t ardrone_academy_navdata_get_camera_state(void)
{
  return (navdata_state.cameraIsReady && (navdata_state.screenshot_state == SCREENSHOT_STATE_IDLE));
}

bool_t ardrone_academy_navdata_get_usb_state(void)
{
  return navdata_state.usbIsReady;
}

bool_t ardrone_academy_navdata_get_takeoff_state(void)
{
  return navdata_state.isTakeOff;
}

bool_t ardrone_academy_navdata_get_record_ready(void)
{
  bool_t result = FALSE;
  
  if(1 == ARDRONE_VERSION())
    {
      result = ardrone_academy_stage_recorder_state();
    }
  else if(2 == ARDRONE_VERSION())
    {
#if defined (RECORD_ENCODED_VIDEO)
      result = (video_stage_encoded_recorder_state() || navdata_state.usbRecordInProgress);
#else
        PRINT("ARDRONE VERSION 2 NEEDS RECORD_ENCODED_VIDEO MACRO!!!\n");
#endif
    }
  else
    {
        PRINT("ARDRONE VERSION NOT INITIALIZED !!!\n");
    }
  return result;
}

bool_t ardrone_academy_navdata_get_record_state(void)
{
    bool_t result = FALSE;

    if(IS_ARDRONE1)
    {
        result = ardrone_academy_stage_recorder_state();
    }
    else if(IS_ARDRONE2)
    {
#if defined (RECORD_ENCODED_VIDEO)
        video_encoded_record_state recState = video_stage_encoded_complete_recorder_state ();
        result = navdata_state.usbRecordInProgress ||
            ( VIDEO_ENCODED_STOPPED != recState &&
              VIDEO_ENCODED_WAITING_STREAM_END != recState);
#else
        PRINT("ARDRONE VERSION 2 NEEDS RECORD_ENCODED_VIDEO MACRO!!!\n");
#endif
    }
    else
    {
        PRINT("ARDRONE VERSION NOT INITIALIZED !!!\n");
    }
    return result;
}

bool_t ardrone_academy_navdata_get_emergency_state(void)
{
  return navdata_state.isEmergency;
}

uint32_t ardrone_academy_navdata_get_remaining_usb_time (void)
{
    return navdata_state.usbFreeTime;
}

bool_t ardrone_academy_navdata_check_usb_record_status (void)
{
    LOCK_ND_MUTEX ();
    bool_t retVal = navdata_state.droneStoppedUsbRecording;
    navdata_state.droneStoppedUsbRecording = FALSE;
    UNLOCK_ND_MUTEX ();
    return retVal;
}

bool_t ardrone_academy_navdata_check_takeoff_cancelled (void)
{
    LOCK_ND_MUTEX ();
    bool_t retVal = navdata_state.takeOffWasCancelled;
    navdata_state.takeOffWasCancelled = FALSE;
    UNLOCK_ND_MUTEX ();
    return retVal;
}

void ardrone_academy_navdata_recorder_enable(bool_t enable, uint32_t timestamp)
{
  if(1 == ARDRONE_VERSION())
    {
      ardrone_academy_stage_recorder_enable(enable, timestamp);
    }
  else if(2 == ARDRONE_VERSION())
    {
#if defined (RECORD_ENCODED_VIDEO)
      if (1 == enable)
        {
          /*
            Calling this function to disable the recorder is to be made outside
            a navdata handler, so we can finish a video even if the drone is
            disconnected (battery removed, or out of wifi range)
            The disable call is made inside the UI "button press" callbacks
          */
          video_stage_encoded_recorder_enable(enable, timestamp);
        }
#else
        PRINT("ARDRONE VERSION 2 NEEDS RECORD_ENCODED_VIDEO MACRO!!!\n");
#endif
    }
  else
    {
        PRINT("ARDRONE VERSION NOT INITIALIZED !!!\n");
    }
}

FLYING_STATE ardrone_academy_navdata_get_flying_state(const navdata_unpacked_t* data)
{
  FLYING_STATE tmp_state;
  switch ((data->navdata_demo.ctrl_state >> 16))
    {
    case CTRL_FLYING:
    case CTRL_HOVERING:
    case CTRL_TRANS_GOTOFIX:
    case CTRL_TRANS_LOOPING:
      tmp_state = FLYING_STATE_FLYING;
      break;
      
    case CTRL_TRANS_TAKEOFF:
      tmp_state = FLYING_STATE_TAKING_OFF;
      break;
      
    case CTRL_TRANS_LANDING:
      tmp_state = FLYING_STATE_LANDING;
      break;
      
    case CTRL_DEFAULT:
    case CTRL_LANDED:
    default:
      tmp_state = FLYING_STATE_LANDED;
      break;
    }
  return tmp_state;
}

void ardrone_academy_navdata_userbox_cb(bool_t result)
{
  if(result)
    {
      LOCK_ND_MUTEX ();
      if(navdata_state.userbox_state == USERBOX_STATE_STARTING)
        {
            PRINT("Userbox started\n");
          navdata_state.userbox_state = USERBOX_STATE_STARTED;
          
          switch (navdata_state.takeoff_state)
            {
            case TAKEOFF_STATE_WAIT_USERBOX:
              navdata_state.takeoff_state = TAKEOFF_STATE_IDLE;
              ardrone_tool_set_ui_pad_start(1);
              break;
            case TAKEOFF_STATE_CANCELLING:
              navdata_state.takeoff_state = TAKEOFF_STATE_IDLE;
              ardrone_tool_set_ui_pad_start(0);
              break;
            default:
              break;
            }
              
          
          if(navdata_state.record_state == RECORD_STATE_WAIT_USERBOX)
            {
              navdata_state.record_state = RECORD_STATE_IDLE;
              if (FALSE == navdata_state.usbRecordInProgress)
              {
              ardrone_academy_navdata_recorder_enable(TRUE, navdata_state.userbox_time);
            }
        }
        }
      else if(navdata_state.userbox_state == USERBOX_STATE_STOPPING)
        {
            PRINT("Userbox stopped\n");
          academy_download_resume ();
          if(navdata_state.takeoff_state == TAKEOFF_STATE_WAIT_USERBOX ||
             TAKEOFF_STATE_CANCELLING == navdata_state.takeoff_state)
            {
              navdata_state.takeoff_state = TAKEOFF_STATE_IDLE;
              ardrone_tool_set_ui_pad_start(0);
            }

          if(navdata_state.record_state == RECORD_STATE_WAIT_USERBOX)
            {
              navdata_state.record_state = RECORD_STATE_IDLE;
              ardrone_academy_navdata_recorder_enable(FALSE, navdata_state.userbox_time);
              navdata_state.usbRecordInProgress = FALSE;
            }

          navdata_state.userbox_state = USERBOX_STATE_STOPPED;
        }
      UNLOCK_ND_MUTEX ();
    }
}

void ardrone_academy_navdata_screenshot_cb(bool_t result)
{
  if(result)
    {
      LOCK_ND_MUTEX ();
      if(navdata_state.screenshot_state == SCREENSHOT_STATE_INPROGRESS)
        {
          navdata_state.screenshot_state = SCREENSHOT_STATE_IDLE;
        }
      UNLOCK_ND_MUTEX ();
    }
}

void ardrone_academy_check_take_off_timeout (void)
{
  static char cancelCommand[ARDRONE_DATE_MAXSIZE];
  time_t elapsedTime = time (NULL) - navdata_state.takeoff_time;
  if (TAKEOFF_TIMEOUT_SEC < elapsedTime)
    {
      switch (navdata_state.takeoff_state)
        {
        case TAKEOFF_STATE_WAIT_USERBOX:
          snprintf (cancelCommand, ARDRONE_DATE_MAXSIZE-1, "%d", USERBOX_CMD_CANCEL);
          navdata_state.userbox_state = USERBOX_STATE_STOPPING;
          navdata_state.takeoff_state = TAKEOFF_STATE_CANCELLING;
          ARDRONE_TOOL_CONFIGURATION_ADDEVENT (userbox_cmd, cancelCommand, ardrone_academy_navdata_userbox_cb);
            navdata_state.takeOffWasCancelled = TRUE;
          break;
        case TAKEOFF_STATE_NEEDED:
          navdata_state.takeoff_state = TAKEOFF_STATE_IDLE;
            navdata_state.takeOffWasCancelled = TRUE;
          break;
        default:
          break;
        }
    }
}


void ardrone_academy_navdata_userbox_switch(void)
{
  static char param[ARDRONE_DATE_MAXSIZE + 8];
  static char date[ARDRONE_DATE_MAXSIZE];
  if(navdata_state.userbox_state == USERBOX_STATE_STOPPED)
    {
      navdata_state.userbox_time = time(NULL);
      ardrone_time2date(navdata_state.userbox_time, ARDRONE_FILE_DATE_FORMAT, date);
      sprintf(param, "%d,%s", USERBOX_CMD_START, date);
      navdata_state.userbox_state = USERBOX_STATE_STARTING;
      ARDRONE_TOOL_CONFIGURATION_ADDEVENT(userbox_cmd, param, ardrone_academy_navdata_userbox_cb);
    }
  else if(navdata_state.userbox_state == USERBOX_STATE_STARTED)
    {
      sprintf(param, "%d", USERBOX_CMD_STOP);
      navdata_state.userbox_state = USERBOX_STATE_STOPPING;
      ARDRONE_TOOL_CONFIGURATION_ADDEVENT(userbox_cmd, param, ardrone_academy_navdata_userbox_cb);
    }
}

C_RESULT ardrone_academy_navdata_init(void* data)
{
	navdata_state.wasEmergency = FALSE;
	navdata_state.needSetEmergency = FALSE;
	navdata_state.takeoff_state = TAKEOFF_STATE_IDLE;
	navdata_state.record_state = TAKEOFF_STATE_IDLE;
	navdata_state.userbox_state = USERBOX_STATE_STOPPED;
	navdata_state.isTakeOff = FALSE;
	navdata_state.isEmergency = FALSE;
	navdata_state.cameraIsReady = FALSE;
	navdata_state.usbIsReady = FALSE;
        navdata_state.usbRecordInProgress = FALSE;
	navdata_state.userbox_time = (time_t)0;
        navdata_state.takeoff_time = (time_t)0;
    navdata_state.usbFreeTime = 0;
    navdata_state.droneStoppedUsbRecording = FALSE;
    navdata_state.takeOffWasCancelled = FALSE;
    navdata_state.internalRecordInProgress = FALSE;
    navdata_state.lastState = 0;
        vp_os_mutex_init (&navdata_state.aan_mutex);

	return C_OK;
}

C_RESULT ardrone_academy_navdata_process( const navdata_unpacked_t* const pnd )
{
  static bool_t prevCameraIsReady = FALSE;
    static bool_t prevDroneUsbRecording = FALSE;
  if (C_OK == TRYLOCK_ND_MUTEX ())
    {
      input_state_t* input_state = ardrone_tool_input_get_state();
        bool_t recordIsReady = ! ardrone_academy_navdata_get_record_ready();
        bool_t recordIsInProgress = ardrone_academy_navdata_get_record_state();

        // Save ARDrone State
        navdata_state.lastState = pnd->ardrone_state;

        // Save remaining USB time
        navdata_state.usbFreeTime = pnd->navdata_hdvideo_stream.usbkey_remaining_time;

        bool_t currentDroneUsbRecording = (NAVDATA_HDVIDEO_USBKEY_IS_RECORDING == (NAVDATA_HDVIDEO_USBKEY_IS_RECORDING & pnd->navdata_hdvideo_stream.hdvideo_state));
        // Check for record stop from drone
        if (navdata_state.usbRecordInProgress)
        {
            if (TRUE  == prevDroneUsbRecording &&
                FALSE == currentDroneUsbRecording)
            {
                navdata_state.droneStoppedUsbRecording = TRUE;
                navdata_state.record_state = RECORD_STATE_NEEDED;
            }
        }
        prevDroneUsbRecording = currentDroneUsbRecording;

      ardrone_academy_check_take_off_timeout ();

      // Take off and Emergency management
      if(navdata_state.takeoff_state == TAKEOFF_STATE_NEEDED)
	{
          if(pnd->ardrone_state & ARDRONE_EMERGENCY_MASK)
            {
              navdata_state.needSetEmergency = TRUE;
            }
          else
            {
                if(recordIsInProgress)
                {
                  if(!(pnd->ardrone_state & ARDRONE_USER_FEEDBACK_START))
                    ardrone_tool_set_ui_pad_start(1);
                  else
                    ardrone_tool_set_ui_pad_start(0);
                  navdata_state.takeoff_state = TAKEOFF_STATE_IDLE;

                }
              else
                {
                  navdata_state.takeoff_state = TAKEOFF_STATE_WAIT_USERBOX;
                  ardrone_academy_navdata_userbox_switch();
                }
            }
	}

      if(navdata_state.needSetEmergency)
	{
          ardrone_tool_set_ui_pad_select(1);
          navdata_state.needSetEmergency = FALSE;
	}

      if(pnd->ardrone_state & ARDRONE_EMERGENCY_MASK)
        {
          if(!navdata_state.wasEmergency && (input_state->user_input & (1 << ARDRONE_UI_BIT_SELECT)))
            {
              ardrone_tool_set_ui_pad_select(0);
            }
        
          navdata_state.isEmergency = TRUE;
          navdata_state.isTakeOff = FALSE;

            if(!navdata_state.internalRecordInProgress && !navdata_state.usbRecordInProgress && (navdata_state.userbox_state == USERBOX_STATE_STARTED))
            {
                PRINT("Emergency !! Stopping userbox...\n");
              ardrone_academy_navdata_userbox_switch();
            }

          ardrone_tool_input_start_reset();
        }
      else // Not emergency landing
        {
          if(navdata_state.wasEmergency && (input_state->user_input & (1 << ARDRONE_UI_BIT_SELECT)))
            {
              ardrone_tool_set_ui_pad_select(0);
            }
        
          if(!(pnd->ardrone_state & ARDRONE_TIMER_ELAPSED))
            navdata_state.isEmergency = FALSE;

          if(input_state->user_input & (1 << ARDRONE_UI_BIT_START))
            {
              navdata_state.isTakeOff = (pnd->ardrone_state & ARDRONE_USER_FEEDBACK_START) ? TRUE : FALSE;
            }
          else
            {
              navdata_state.isTakeOff = FALSE;
            }
        }
      navdata_state.wasEmergency = (pnd->ardrone_state & ARDRONE_EMERGENCY_MASK) ? TRUE : FALSE;
    
      // Video record management

        bool_t usbRecordEnable = FALSE;
        if(navdata_state.record_state == RECORD_STATE_NEEDED &&
           TAKEOFF_STATE_IDLE == navdata_state.takeoff_state)
        {
            bool_t continueRecord = TRUE;
            bool_t nextInternalRecordState = FALSE;
            if (IS_LEAST_ARDRONE2)
	{
          static codec_type_t oldCodec;
                codec_type_t cancelCodec;
                if (recordIsReady && ! navdata_state.usbRecordInProgress && !navdata_state.internalRecordInProgress) // We want to enable recording
            {
                if ((ARDRONE_USB_MASK == (pnd->ardrone_state & ARDRONE_USB_MASK)) &&
                        (0 < pnd->navdata_hdvideo_stream.usbkey_remaining_time) &&
                    (TRUE == ardrone_control_config.video_on_usb))
                {
                        usbRecordEnable = TRUE;
                }
                    // Set the "non-record codec" to the codec defined as the application default
                    oldCodec = ardrone_application_default_config.video_codec;
                    cancelCodec = oldCodec;
                    ardrone_control_config.video_codec = (TRUE == usbRecordEnable) ? usbRecordCodec : deviceWifiRecordCodec;
                    PRINT ("Set codec %d -> %d\n", oldCodec, ardrone_control_config.video_codec);
                    nextInternalRecordState = TRUE;
            }
          else // We want to disable recording
            {
                    cancelCodec = ardrone_control_config.video_codec;
              ardrone_control_config.video_codec = oldCodec;
                    PRINT ("Reset codec %d -> %d\n", cancelCodec, oldCodec);
            }
                bool_t addEventSucceeded = ARDRONE_TOOL_CONFIGURATION_ADDEVENT (video_codec, &ardrone_control_config.video_codec, NULL);
                if (FALSE == addEventSucceeded)
                {
                    PRINT ("Unable to send codec switch ... retry later\n");
                    ardrone_control_config.video_codec = cancelCodec;
                    continueRecord = FALSE;
                }
            }
            else if (IS_ARDRONE1)
            {
                nextInternalRecordState = !recordIsInProgress;
            }

            if (TRUE == continueRecord)
            {
                navdata_state.internalRecordInProgress = nextInternalRecordState;
            switch (navdata_state.userbox_state)
            {
            case USERBOX_STATE_STOPPED:
              navdata_state.record_state = RECORD_STATE_WAIT_USERBOX;
                    navdata_state.usbRecordInProgress = usbRecordEnable;
              ardrone_academy_navdata_userbox_switch();
                break;
            case USERBOX_STATE_STARTED:
              if(navdata_state.isTakeOff)
                {
                        if(! recordIsReady)
                    {
                            PRINT("Userbox is started and record is started => Stop record\n");
                      ardrone_academy_navdata_recorder_enable(FALSE, navdata_state.userbox_time);
                      navdata_state.usbRecordInProgress = FALSE;
                    }
                  else
                    {
                            PRINT("Userbox is started and record is stopped => Start record\n");
                            if (FALSE == usbRecordEnable)
                        {
                            // Only activate the local recorder if we don't record on USB
                      ardrone_academy_navdata_recorder_enable(TRUE, navdata_state.userbox_time);
                                navdata_state.usbRecordInProgress = FALSE;
                            }
                            else
                            {
                                navdata_state.usbRecordInProgress = TRUE;
                        }
                    }

                  navdata_state.record_state = RECORD_STATE_IDLE;
                }
              else
                {
                  navdata_state.record_state = RECORD_STATE_WAIT_USERBOX;
                  ardrone_academy_navdata_userbox_switch();
                }
                break;
            case USERBOX_STATE_STARTING:
                    navdata_state.usbRecordInProgress = usbRecordEnable;
                navdata_state.record_state = RECORD_STATE_WAIT_USERBOX;
                break;
            case USERBOX_STATE_STOPPING:
                    navdata_state.usbRecordInProgress = usbRecordEnable;
                // Should never be here
                PRINT ("Don't know what to do for USERBOX_STATE_STOPPING\n");
                VP_OS_ASSERT (0 == 1); // Debug handler
                break;
                }
            }
        }

      // Screenshot management
      prevCameraIsReady = navdata_state.cameraIsReady;
      navdata_state.cameraIsReady = (pnd->ardrone_state & ARDRONE_CAMERA_MASK) ? TRUE : FALSE;
      if (TRUE == navdata_state.cameraIsReady && FALSE == prevCameraIsReady)
      {
          academy_download_resume ();
      }
        
      if((navdata_state.screenshot_state == SCREENSHOT_STATE_NEEDED) && navdata_state.cameraIsReady)
      {
          static char param[ARDRONE_DATE_MAXSIZE + 64];
          static char date[ARDRONE_DATE_MAXSIZE];
          time_t t = time(NULL);
          ardrone_time2date(t, ARDRONE_FILE_DATE_FORMAT, date);
          navdata_state.screenshot_state = SCREENSHOT_STATE_INPROGRESS;
          sprintf(param, "%d,%d,%d,%s", USERBOX_CMD_SCREENSHOT, 0, 0, date);
          ARDRONE_TOOL_CONFIGURATION_ADDEVENT(userbox_cmd, param, ardrone_academy_navdata_screenshot_cb);
      }

      // USB management
      navdata_state.usbIsReady = (pnd->ardrone_state & ARDRONE_USB_MASK) ? TRUE : FALSE;
      UNLOCK_ND_MUTEX ();
    }
  return C_OK;
}

C_RESULT ardrone_academy_navdata_release( void )
{
  if(ardrone_academy_navdata_get_record_state())
    {
      ardrone_academy_navdata_recorder_enable(FALSE, navdata_state.userbox_time);
    }
  vp_os_mutex_destroy (&navdata_state.aan_mutex);
  return C_OK;
}

void ardrone_academy_navdata_set_wifi_record_codec (codec_type_t newCodec)
{
    deviceWifiRecordCodec = newCodec;
}
