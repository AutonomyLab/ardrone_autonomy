#ifndef SNIPPET_ROS_TO_ARDRONE_CONFIG_H
#define SNIPPET_ROS_TO_ARDRONE_CONFIG_H

// LOAD THE CUSTOM CONFIGURATION FROM ROS PARAMETERS
// all possible configuration parameters are stored in config_keys.h (and documented in the manual)
// taking inspiration from ardrone_tool_configuration.c,
// we define some macros that replace these parameter definitions
// with a function which attempts to read corresponding ros parameters,
// and then if successful, sets the parameter value for the drone
// Note that we don't actually send these parameters to the drone,
// otherwise they will be overwritten when the profiles are created
// in a later stage of the ARDrone initialization.

#undef ARDRONE_CONFIG_KEY_IMM_a10
#undef ARDRONE_CONFIG_KEY_REF_a10
#undef ARDRONE_CONFIG_KEY_STR_a10
#undef LOAD_PARAM_STR
#undef LOAD_PARAM_NUM

#define LOAD_PARAM_NUM(NAME,C_TYPE,DEFAULT)                                                                             \
        {                                                                                                                   \
          double param;                                                                                                     \
          ROS_DEBUG("CHECK: "#NAME" (Default = "#DEFAULT" = %f)",(float)DEFAULT);                                           \
          if(ros::param::get("~"#NAME,param))                                                                               \
          {                                                                                                                 \
            ardrone_application_default_config.NAME = (C_TYPE)param;                                                        \
            ROS_DEBUG("SET: "#NAME" = %f (DEFAULT = %f)", (float)ardrone_application_default_config.NAME, (float)DEFAULT);  \
          }                                                                                                                 \
        }

#define LOAD_PARAM_STR(NAME,DEFAULT)                                                                                    \
        {                                                                                                                   \
          std::string param;                                                                                                \
          ROS_DEBUG("CHECK: "#NAME" (Default = "#DEFAULT" = %s)",DEFAULT);                                                  \
          if(ros::param::get("~"#NAME,param))                                                                               \
          {                                                                                                                 \
            param = param.substr(0,STRING_T_SIZE-1);                                                                        \
            strcpy(ardrone_application_default_config.NAME , param.c_str());                                                \
            ROS_DEBUG("SET: "#NAME" = %s (DEFAULT = %s)", ardrone_application_default_config.NAME, DEFAULT);                \
          }                                                                                                                 \
        }

#define ARDRONE_CONFIG_KEY_REF_a10(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, RW_CUSTOM, DEFAULT, CALLBACK, CATEGORY) //do nothing for reference-only parameters
#define ARDRONE_CONFIG_KEY_IMM_a10(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, RW_CUSTOM, DEFAULT, CALLBACK, CATEGORY) { if(0!=strcmp(KEY,"custom") && ((RW & K_WRITE) != 0 || (RW_CUSTOM & K_WRITE) != 0)) LOAD_PARAM_NUM(NAME,C_TYPE, DEFAULT) } // parameters under the custom key are for control of application/user/session, we don't want to change these!
#define ARDRONE_CONFIG_KEY_STR_a10(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, RW_CUSTOM, DEFAULT, CALLBACK, CATEGORY) { if(0!=strcmp(KEY,"custom") && ((RW & K_WRITE) != 0 || (RW_CUSTOM & K_WRITE) != 0)) LOAD_PARAM_STR(NAME, DEFAULT) }

#include <config_keys.h> // include the parameter definitions, which will be replaced by the above

#undef LOAD_PARAM_NUM
#undef LOAD_PARAM_STR
#undef ARDRONE_CONFIG_KEY_IMM_a10
#undef ARDRONE_CONFIG_KEY_REF_a10
#undef ARDRONE_CONFIG_KEY_STR_a10

// Now we delete any old configuration that we may have stored, this is so the profiles will be reinitialized to drone default before being updated with the potentially new set of custom parameters that we specify above.
// We have to do this because only non-default parameters are sent, so if we delete a ros_param, the local parameter will be not be changed (above), thus will remain default and thus won't be updated on the drone - a problem if old profiles exist.

char buffer[MULTICONFIG_ID_SIZE + 1];

sprintf(buffer, "-%s", usr_id);
printf("Deleting Profile %s\n", buffer);
ARDRONE_TOOL_CONFIGURATION_ADDEVENT(profile_id, buffer, NULL);

sprintf(buffer, "-%s", app_id);
printf("Deleting Application %s\n", buffer);
ARDRONE_TOOL_CONFIGURATION_ADDEVENT(application_id, buffer, NULL);

#endif // SNIPPET_ROS_TO_ARDRONE_CONFIG_H
