#ifndef SNIPPET_CONFIGURE_DRONE_H
#define SNIPPET_CONFIGURE_DRONE_H

// This function will send the custom configuration to the drone
// It doesn't work if sent during the SDK (which runs before the configuration profiles on the drone are setup)
#undef ARDRONE_CONFIG_KEY_IMM_a10
#undef ARDRONE_CONFIG_KEY_REF_a10
#undef ARDRONE_CONFIG_KEY_STR_a10
#undef LOAD_PARAM_STR
#undef LOAD_PARAM_NUM

#define SEND_PARAM_NUM(NAME,CATEGORY,DEFAULT)                                                          \
{                                                                                                      \
    if(ardrone_application_default_config.NAME!=DEFAULT)                                               \
    {                                                                                                  \
        ROS_INFO("  SEND: "#CATEGORY"/"#NAME" = %f (DEFAULT = %f)", (float)ardrone_application_default_config.NAME, (float)DEFAULT);           \
        ARDRONE_TOOL_CONFIGURATION_ADDEVENT (NAME, &ardrone_application_default_config.NAME, NULL);    \
    }                                                                                                  \
}

#define SEND_PARAM_STR(NAME,CATEGORY,DEFAULT)                                                          \
{                                                                                                      \
    if(0!=strcmp(ardrone_application_default_config.NAME,DEFAULT))                                     \
    {                                                                                                  \
        ROS_INFO("SEND: "#CATEGORY"/"#NAME" = %s (DEFAULT = %s)", ardrone_application_default_config.NAME, DEFAULT);           \
        ARDRONE_TOOL_CONFIGURATION_ADDEVENT (NAME, ardrone_application_default_config.NAME, NULL);     \
    }                                                                                                  \
}

// firstly we send the CAT_COMMON parameters, for example outdoor, as these settings can affect where the other parameters are stored
#define ARDRONE_CONFIG_KEY_REF_a10(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, RW_CUSTOM, DEFAULT, CALLBACK, CATEGORY) //do nothing for reference-only parameters
#define ARDRONE_CONFIG_KEY_IMM_a10(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, RW_CUSTOM, DEFAULT, CALLBACK, CATEGORY) { if(0!=strcmp(KEY,"custom") && CATEGORY==CAT_COMMON && ((RW & K_WRITE) != 0 || (RW_CUSTOM & K_WRITE) != 0)) SEND_PARAM_NUM(NAME,CATEGORY,DEFAULT) } // parameters under the custom key are for control of application/user/session, we don't want to change these!
#define ARDRONE_CONFIG_KEY_STR_a10(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, RW_CUSTOM, DEFAULT, CALLBACK, CATEGORY) { if(0!=strcmp(KEY,"custom") && CATEGORY==CAT_COMMON && ((RW & K_WRITE) != 0 || (RW_CUSTOM & K_WRITE) != 0)) SEND_PARAM_STR(NAME,CATEGORY,DEFAULT) }

#include <config_keys.h> // include the parameter definitions, which will be replaced by the above

#undef ARDRONE_CONFIG_KEY_IMM_a10
#undef ARDRONE_CONFIG_KEY_STR_a10

// then we send the rest of the parameters. The problem is if we send euler_angle_max (for example) before sending outdoor, it will get written to the wrong parameter
// (indoor_ not outdoor_euler_angle_max) and then will be overwritten by the default when changing state from indoor to outdoor, so we need to send common parameters first.
#define ARDRONE_CONFIG_KEY_IMM_a10(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, RW_CUSTOM, DEFAULT, CALLBACK, CATEGORY) { if(0!=strcmp(KEY,"custom") && CATEGORY!=CAT_COMMON && ((RW & K_WRITE) != 0 || (RW_CUSTOM & K_WRITE) != 0)) SEND_PARAM_NUM(NAME,CATEGORY,DEFAULT) } // parameters under the custom key are for control of application/user/session, we don't want to change these!
#define ARDRONE_CONFIG_KEY_STR_a10(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, RW_CUSTOM, DEFAULT, CALLBACK, CATEGORY) { if(0!=strcmp(KEY,"custom") && CATEGORY!=CAT_COMMON && ((RW & K_WRITE) != 0 || (RW_CUSTOM & K_WRITE) != 0)) SEND_PARAM_STR(NAME,CATEGORY,DEFAULT) }

#include <config_keys.h> // include the parameter definitions, which will be replaced by the above

#undef SEND_PARAM_NUM
#undef SEND_PARAM_STR
#undef ARDRONE_CONFIG_KEY_IMM_a10
#undef ARDRONE_CONFIG_KEY_REF_a10
#undef ARDRONE_CONFIG_KEY_STR_a10


#define NAVDATA_STRUCTS_INITIALIZE
#include <ardrone_autonomy/NavdataMessageDefinitions.h>
#undef NAVDATA_STRUCTS_INITIALIZE

#endif // SNIPPET_CONFIGURE_DRONE_H
