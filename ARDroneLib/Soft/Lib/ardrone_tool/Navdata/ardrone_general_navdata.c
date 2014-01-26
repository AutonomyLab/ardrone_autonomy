#include <stdio.h>
#include <sys/time.h>
#include <ardrone_api.h>
#include <ardrone_tool/ardrone_tool.h>
#include <ardrone_tool/ardrone_tool_configuration.h>
#include <ardrone_tool/Navdata/ardrone_navdata_client.h>
#include <ardrone_tool/Navdata/ardrone_general_navdata.h>
#include <utils/ardrone_gen_ids.h>

int configWasDone = 0;

static MULTICONFIG_STATE configState;
static NAVDATA_REQUEST_STATE navdataState;
static int appSwitch = 0;
static int usrSwitch = 0;
static int sesSwitch = 0;
static int navdataNeeded = 0;
static int droneSupportsMulticonfig = 0;

#define __MULTICONFIGURATION_MIN_VERSION_MAJOR__    (1U)
#define __MULTICONFIGURATION_MIN_VERSION_MINOR__    (6U)
#define __MULTICONFIGURATION_MIN_VERSION_REVISION__ (0U)
#define __MULTICONFIGURATION_MIN_VERSION__ ((__MULTICONFIGURATION_MIN_VERSION_MAJOR__ << 16) | (__MULTICONFIGURATION_MIN_VERSION_MINOR__ << 8) | (__MULTICONFIGURATION_MIN_VERSION_REVISION__))
static inline int versionSupportsMulticonfiguration (const char *currentString)
{
  uint32_t currentMajor = 0, currentMinor = 0, currentRevision = 0;
  uint32_t currentVersion;
  sscanf (currentString, "%d.%d.%d", &currentMajor, &currentMinor, &currentRevision);
  currentVersion = ((currentMajor << 16) | (currentMinor << 8) | (currentRevision));
  return currentVersion >= __MULTICONFIGURATION_MIN_VERSION__;
}


#define _GENERAL_NAVDATA_DEBUG (0)
#define _GENERAL_NAVDATA_DEBUG_PREFIX "General Navdata : "
#if _GENERAL_NAVDATA_DEBUG || defined (DEBUG)
#define PRINTDBG(...)                                                   \
    do                                                                  \
    {                                                                   \
        printf ("[%d] %s", __LINE__, _GENERAL_NAVDATA_DEBUG_PREFIX);    \
        printf (__VA_ARGS__);                                           \
        printf ("\n");                                                  \
    } while (0)
#else
#define PRINTDBG(...)
#endif

static void sendInitConfigurations(void)
{
}

static void switchToSession(void)
{
  ARDRONE_TOOL_CONFIGURATION_ADDEVENT (session_id, ses_id, NULL);
}

static void switchToUser(void)
{
  ARDRONE_TOOL_CONFIGURATION_ADDEVENT (profile_id, usr_id, NULL);
}

static void switchToApplication(void)
{
  ARDRONE_TOOL_CONFIGURATION_ADDEVENT (application_id , app_id, NULL);
}

void configurationCallback (int res)
{
  PRINTDBG ("Config callback called with result %d", res);
  if (0 != res)
    {
      PRINTDBG ("State : %d", configState);
      switch (configState)
        {
        case MULTICONFIG_IN_PROGRESS_VERSION: // got config version
          PRINTDBG ("Got config file");
          configState = MULTICONFIG_GOT_DRONE_VERSION;
          break;
        case MULTICONFIG_IN_PROGRESS_LIST: // got multiconfig ids
          PRINTDBG ("Got ids list");
          configState = MULTICONFIG_GOT_IDS_LIST;
          break;
        case MULTICONFIG_IN_PROGRESS_IDS:
          PRINTDBG ("Got current ids");
          configState = MULTICONFIG_GOT_CURRENT_IDS;
          break;
        case MULTICONFIG_IN_PROGRESS_NAVDATA:
          PRINTDBG ("Got navdatas");
          configState = MULTICONFIG_GOT_NAVDATA;
          break;
        case MULTICONFIG_IDLE:
        case MULTICONFIG_GOT_IDS_LIST:
        case MULTICONFIG_GOT_DRONE_VERSION:
        case MULTICONFIG_GOT_CURRENT_IDS:
        case MULTICONFIG_GOT_NAVDATA:
        case MULTICONFIG_NEEDED:
        case MULTICONFIG_REQUEST_NAVDATA:
        default:
          break;
        }
    }
  PRINTDBG ("End of config callback call");
}

void navdataCallback (int res)
{
  PRINTDBG ("Navdata callback called with result %d", res);
  if (0 != res)
    {
      PRINTDBG ("State : %d", navdataState);
      switch (navdataState)
        {
        case NAVDATA_REQUEST_IN_PROGRESS: // Navdata request got acknowledged by the AR.Drone
          navdataState = NAVDATA_REQUEST_DONE;
          break;
        case NAVDATA_REQUEST_IDLE:
        case NAVDATA_REQUEST_NEEDED:
        case NAVDATA_REQUEST_DONE:
        default:
          break;
        }
    }
  PRINTDBG ("End of navdata callback call");
}

C_RESULT ardrone_general_navdata_init( void* data )
{
  ARDRONE_TOOL_CONFIGURATION_ADDEVENT (session_id, "-all", NULL);
  
  navdataState = NAVDATA_REQUEST_IDLE;
  configState = MULTICONFIG_NEEDED;
  appSwitch = 1;
  usrSwitch = 1;
  sesSwitch = 1;
  navdataNeeded = 1;
  
  return C_OK;
}

C_RESULT ardrone_general_navdata_process( const navdata_unpacked_t* const pnd )
{
  navdata_mode_t current_navdata_state = NAVDATA_BOOTSTRAP;
  
  /* Makes sure the navdata stream will be resumed if the drone is disconnected and reconnected.
     * Allows changing the drone battery during debugging sessions.    */
  if( ardrone_get_mask_from_state(pnd->ardrone_state, ARDRONE_NAVDATA_BOOTSTRAP) )
    {
      current_navdata_state = NAVDATA_BOOTSTRAP;
    }
  else
    {
      current_navdata_state = (ardrone_get_mask_from_state(pnd->ardrone_state, ARDRONE_NAVDATA_DEMO_MASK)) ? NAVDATA_DEMO : NAVDATA_FULL ;
    }

  if (current_navdata_state == NAVDATA_BOOTSTRAP && configState == MULTICONFIG_IDLE && navdataState == NAVDATA_REQUEST_IDLE)
    {
        if ((0 != strncmp(ardrone_control_config.application_id, app_id, APPLI_NAME_SIZE)) ||
            (0 != strncmp(ardrone_control_config.profile_id, usr_id, USER_NAME_SIZE)) ||
            (0 != strncmp(ardrone_control_config.session_id, ses_id, SESSION_NAME_SIZE)))
        {
            configState = MULTICONFIG_NEEDED;
        }
        else
        {
            navdataState = NAVDATA_REQUEST_NEEDED;
        }
    }
	
  /* Multiconfig settings */
  int configIndex, userNeedInit, appNeedInit;
  userNeedInit = 0; appNeedInit = 0;
  switch (configState)
    {
    case MULTICONFIG_GOT_DRONE_VERSION:
      PRINTDBG ("Checking drone version ...");
      // Check if drone version is >= 1.6
      if (versionSupportsMulticonfiguration (ardrone_control_config.num_version_soft))
        {
          PRINTDBG ("Drone supports multiconfig");
          configState = MULTICONFIG_IN_PROGRESS_LIST;
          ARDRONE_TOOL_CUSTOM_CONFIGURATION_GET (configurationCallback);
          droneSupportsMulticonfig = 1;
        }
      else
        {
          PRINTDBG ("Drone does not support multiconfig");
          // Drone does not support multiconfig ... don't call init functions because we don't want to mess up things in default config
          configState = MULTICONFIG_REQUEST_NAVDATA;
        }
      break;
    case MULTICONFIG_GOT_IDS_LIST:
      // At this point, we're sure that the AR.Drone supports multiconfiguration, so we'll wheck if our ids exists, and send them.
      PRINTDBG ("Got AR.Drone ID list. Switch->%d,%d,%d. ND->%d", sesSwitch, usrSwitch, appSwitch, navdataNeeded);
      if (1 == sesSwitch)
        {
          switchToSession(); // Go to session ...
        }

      if (1 == appSwitch)
        {
          if (0 != strcmp(ardrone_control_config_default.application_id, app_id)) // Check for application only if we're not asking for the default one
            {
              appNeedInit = 1;
              for (configIndex = 0; configIndex < available_configurations[CAT_APPLI].nb_configurations; configIndex++) // Check all existing app_ids
                {
                  PRINTDBG ("Checking application %s (desc : %s)", available_configurations[CAT_APPLI].list[configIndex].id,
                            available_configurations[CAT_APPLI].list[configIndex].description);
                  if (0 == strcmp(available_configurations[CAT_APPLI].list[configIndex].id, app_id))
                    {
                      PRINTDBG ("Found our application ... should not init");
                      appNeedInit = 0;
                      break;
                    }
                }
              switchToApplication();
            }
          else
            {
              PRINTDBG ("We're requesting default application (%s), do nothing.", app_id);
            }
        }
			
      if (1 == usrSwitch)
        {
          if (0 != strcmp(ardrone_control_config_default.profile_id, usr_id)) // Check for user only if we're not asking for the default one
            {
              userNeedInit = 1;
              for (configIndex = 0; configIndex < available_configurations[CAT_USER].nb_configurations; configIndex++) // Check all existing user_ids
                {
                  PRINTDBG ("Checking user %s (desc : %s)", available_configurations[CAT_USER].list[configIndex].id,
                            available_configurations[CAT_USER].list[configIndex].description);
                  if (0 == strcmp(available_configurations[CAT_USER].list[configIndex].id, usr_id))
                    {
                      PRINTDBG ("Found our user ... should not init");
                      userNeedInit = 0;
                      break;
                    }
                }
              switchToUser();
            }
          else
            {
              PRINTDBG ("We're requesting default user (%s), do nothing.", usr_id);
            }
        }
			
      if (1 == appNeedInit)
        {
          // Send application defined default values
          ardrone_tool_send_application_default();
          PRINTDBG ("Creating app. profile on AR.Drone");
          ARDRONE_TOOL_CONFIGURATION_ADDEVENT (application_desc, app_name, NULL);
        }
      if (1 == userNeedInit)
        {
          // Send user defined default values
          ardrone_tool_send_user_default();
          PRINTDBG ("Creating usr. profile on AR.Drone");
          ARDRONE_TOOL_CONFIGURATION_ADDEVENT (profile_desc, usr_name, NULL);
        }
      if (1 == sesSwitch)
        {
          if (0 != strcmp(ardrone_control_config_default.session_id, ses_id)) // Send session info only if session is not the default one
            {
              ARDRONE_TOOL_CONFIGURATION_ADDEVENT (session_desc, ses_name, NULL);
              // Send session specific default values
              ardrone_tool_send_session_default();
            }
          else
            {
              PRINTDBG ("We're requesting default session (%s), do nothing.", ses_id);
            }
        }
      configState = MULTICONFIG_IN_PROGRESS_IDS;
      ARDRONE_TOOL_CONFIGURATION_GET (configurationCallback);
      break;
			
    case MULTICONFIG_GOT_CURRENT_IDS:
      if (0 != strcmp(ardrone_control_config.session_id, ses_id) ||
          0 != strcmp(ardrone_control_config.profile_id, usr_id) ||
          0 != strcmp(ardrone_control_config.application_id, app_id))
        {
          configState = MULTICONFIG_GOT_DRONE_VERSION; // We failed at setting up the application ids ... restart (but assume that drone supports multiconfig as we already checked)
        }
      else if (1 == navdataNeeded)
        {
          configState = MULTICONFIG_REQUEST_NAVDATA;
        }
      else
        {
          configState = MULTICONFIG_IDLE;
        }
      break;
    case MULTICONFIG_NEEDED:
      PRINTDBG ("Need to check multiconfig ... request config file");
      // Get config file for reset
      configState = MULTICONFIG_IN_PROGRESS_VERSION;
      ARDRONE_TOOL_CONFIGURATION_GET (configurationCallback);
      break;
    case MULTICONFIG_REQUEST_NAVDATA:
      PRINTDBG ("Send application navdata demo/options");
      // Send application navdata demo/options to start navdatas from AR.Drone
      ARDRONE_TOOL_CONFIGURATION_ADDEVENT (navdata_demo, &ardrone_application_default_config.navdata_demo, NULL);
      ARDRONE_TOOL_CONFIGURATION_ADDEVENT (navdata_options, &ardrone_application_default_config.navdata_options, configurationCallback);
      configState = MULTICONFIG_IN_PROGRESS_NAVDATA;
      break;
    case MULTICONFIG_GOT_NAVDATA:
      // Refresh configuration file
      configState = MULTICONFIG_IDLE;
      PRINTDBG ("Refreshing from MULTICONFIG FSM");
      configWasDone = 1;
      ARDRONE_TOOL_CONFIGURATION_GET (NULL);
      break;
    case MULTICONFIG_IDLE:
    case MULTICONFIG_IN_PROGRESS_LIST:
    case MULTICONFIG_IN_PROGRESS_VERSION:
    case MULTICONFIG_IN_PROGRESS_IDS:
    default:
      break;
    }
			
  /* Navdata request settings */
  switch (navdataState)
    {
    case NAVDATA_REQUEST_NEEDED:
      PRINTDBG ("Resetting navdatas to %s", (0 == ardrone_application_default_config.navdata_demo) ? "full" : "demo");
      navdataState = NAVDATA_REQUEST_IN_PROGRESS;
      switchToSession(); // Resend session id when reconnecting.
      sendInitConfigurations();
        ARDRONE_TOOL_CONFIGURATION_ADDEVENT (navdata_demo, &ardrone_application_default_config.navdata_demo, NULL);
      ARDRONE_TOOL_CONFIGURATION_ADDEVENT (navdata_options, &ardrone_application_default_config.navdata_options, navdataCallback);
      break;
    case NAVDATA_REQUEST_DONE:
      // Refresh configuration file
      PRINTDBG ("Refreshing from NAVDATA FSM");
      navdataState = NAVDATA_REQUEST_IDLE;
        ARDRONE_TOOL_CONFIGURATION_GET (NULL);
      break;
    case NAVDATA_REQUEST_IN_PROGRESS:
    case NAVDATA_REQUEST_IDLE:
    default:
      break;
    }

  return C_OK;
}

C_RESULT ardrone_general_navdata_release( void )
{
  return C_OK;
}

/* User switch/list functions */


/* User functions */
void ardrone_refresh_user_list(void)
{
  if (1 == droneSupportsMulticonfig)
    {
      appSwitch = 0;
      usrSwitch = 0;
      sesSwitch = 0;
      navdataNeeded = 0;
      configState = MULTICONFIG_GOT_DRONE_VERSION;
    }
}

void ardrone_switch_to_user(const char *new_user)
{
  if (1 == droneSupportsMulticonfig)
    {
      ardrone_gen_usrid (new_user, usr_id, usr_name, USER_NAME_SIZE);
      appSwitch = 0;
      usrSwitch = 1;
      sesSwitch = 0;
      navdataNeeded = 0;
      configState = MULTICONFIG_GOT_DRONE_VERSION;
    }
}

void ardrone_switch_to_user_id(const char *new_user_id)
{
  if (1 == droneSupportsMulticonfig)
    {
      // We assume that the userlist is up to date
      int userExists = 0;
      int configIndex;
      for (configIndex = 0; configIndex < available_configurations[CAT_USER].nb_configurations; configIndex++) // Check all existing user_ids
        {
          if (0 == strcmp(available_configurations[CAT_USER].list[configIndex].id, new_user_id))
            {
              userExists = 1;
              break;
            }
        }
        
      if (1 == userExists)
        {
          strncpy (usr_id, new_user_id, MULTICONFIG_ID_SIZE);
          appSwitch = 0;
          usrSwitch = 0;
          sesSwitch = 0;
          navdataNeeded = 0;
          configState = MULTICONFIG_GOT_DRONE_VERSION;
        }
    }
}

ardrone_users_t *ardrone_get_user_list(void)
{
  if (0 == droneSupportsMulticonfig)
    {
      return NULL;
    }
  ardrone_users_t *retVal = vp_os_malloc (sizeof (ardrone_users_t));
  if (NULL == retVal)
    return NULL;
    
    
  // Assume that userlist is up to date
  int validUserCount = 0; // User whose descriptions start with a dot ('.') are hidden users that may not be shown to the application user (e.g. default user for each iPhone, or user specific to a control mode
  int configIndex;
  for (configIndex = 0; configIndex < available_configurations[CAT_USER].nb_configurations; configIndex++) // Check all existing user_ids
    {
      if ('.' != available_configurations[CAT_USER].list[configIndex].description[0]) // Not an hidden user
        {
          validUserCount++;
          retVal->userList = vp_os_realloc (retVal->userList, validUserCount * sizeof (ardrone_user_t));
          if (NULL == retVal->userList)
            {
              vp_os_free (retVal);
              retVal = NULL;
              return NULL;
            }
          strncpy (retVal->userList[validUserCount-1].ident, available_configurations[CAT_USER].list[configIndex].id, MULTICONFIG_ID_SIZE);
          strncpy (retVal->userList[validUserCount-1].description, available_configurations[CAT_USER].list[configIndex].description, USER_NAME_SIZE);
        }
    }
  retVal->userCount = validUserCount;
  return retVal;
}

void ardrone_free_user_list (ardrone_users_t **users)
{
  if (NULL != *users)
    {
      if (NULL != (*users)->userList)
        {
          vp_os_free ((*users)->userList);
          (*users)->userList = NULL;
        }
      vp_os_free (*users);
      *users = NULL;
    }
}
