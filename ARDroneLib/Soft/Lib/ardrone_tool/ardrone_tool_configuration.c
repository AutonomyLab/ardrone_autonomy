//
//  ardrone_tool_configuration.c
//
//  Created by D'HAEYER Frederic on 13/09/10.
//  Copyright 20010 Parrot SA. All rights reserved.
//
#include <ardrone_tool/ardrone_tool_configuration.h>
#include <config.h>
#include <ardrone_tool/ardrone_tool.h>

#define ARDRONE_TOOL_CONFIGURATION_MAX_EVENT	128

#undef ARDRONE_CONFIG_KEY_IMM
#undef ARDRONE_CONFIG_KEY_REF
#undef ARDRONE_CONFIG_KEY_STR
#define ARDRONE_CONFIG_KEY_IMM(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, DEFAULT, CALLBACK) DEFAULT,
#define ARDRONE_CONFIG_KEY_REF(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, DEFAULT, CALLBACK) DEFAULT,
#define ARDRONE_CONFIG_KEY_STR(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, DEFAULT, CALLBACK) DEFAULT,

/* ardrone_control_config and ardrone_control_config_default are now initialized at runtime */
ardrone_config_t ardrone_control_config_default;
ardrone_config_t ardrone_control_config;
ardrone_config_t ardrone_application_default_config;


static ardrone_tool_configuration_data_t ardrone_tool_configuration_data[ARDRONE_TOOL_CONFIGURATION_MAX_EVENT];
static dictionary *ardrone_tool_configuration_dict;
static bool_t ardrone_tool_configuration_is_init = FALSE;
static int ardrone_tool_configuration_current_index = 0;
static int ardrone_tool_configuration_nb_event = 0;
static vp_os_mutex_t ardrone_tool_configuration_mutex;

static void ardrone_tool_configuration_event_configure(void);

void ardrone_tool_reset_configuration (void)
{
#undef ARDRONE_CONFIG_KEY_IMM
#undef ARDRONE_CONFIG_KEY_REF
#undef ARDRONE_CONFIG_KEY_STR
#define ARDRONE_CONFIG_KEY_IMM(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, DEFAULT, CALLBACK) ardrone_control_config.NAME = DEFAULT;
#define ARDRONE_CONFIG_KEY_REF(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, DEFAULT, CALLBACK) ardrone_control_config.NAME = DEFAULT;
#define ARDRONE_CONFIG_KEY_STR(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, DEFAULT, CALLBACK) 	\
{																								\
	strncpy(ardrone_control_config.NAME, DEFAULT, sizeof(ardrone_control_config.NAME) - 1);		\
	ardrone_control_config.NAME[sizeof(ardrone_control_config.NAME) - 1] = '\0';				\
}
#include <config_keys.h>
}

#undef ARDRONE_CONFIG_KEY_IMM
#undef ARDRONE_CONFIG_KEY_REF
#undef ARDRONE_CONFIG_KEY_STR
#define ARDRONE_CONFIG_KEY_IMM(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, DEFAULT, CALLBACK)											\
bool_t ardrone_tool_configuration_addevent_##NAME(C_TYPE_PTR value, ardrone_tool_configuration_callback result_callback)				\
{																																		\
	bool_t res = FALSE;                                                                                                                 \
    vp_os_mutex_lock(&ardrone_tool_configuration_mutex);																				\
    if(ardrone_tool_configuration_current_index == (ardrone_tool_configuration_nb_event + 1) % ARDRONE_TOOL_CONFIGURATION_MAX_EVENT)	\
    {                                                                                                                                   \
        printf("ARDRONE_TOOL_CONFIGURATION QUEUE FILLED !! %s\n", #NAME);																\
    }																																	\
    else																																\
    {																																	\
        ardrone_tool_configuration_data[ardrone_tool_configuration_nb_event].control_mode = ACK_CONTROL_MODE;							\
        ardrone_tool_configuration_data[ardrone_tool_configuration_nb_event].result_callback = result_callback;							\
        ardrone_tool_configuration_data[ardrone_tool_configuration_nb_event].key = #NAME;												\
        ardrone_tool_configuration_data[ardrone_tool_configuration_nb_event].value = vp_os_malloc(sizeof(C_TYPE));						\
        vp_os_memcpy(ardrone_tool_configuration_data[ardrone_tool_configuration_nb_event].value, value, sizeof(C_TYPE));				\
        ardrone_tool_configuration_data[ardrone_tool_configuration_nb_event].callback	= (ardrone_at_configuration_set)&ARDRONE_CONFIGURATION_SET_FUNCTION(NAME);	\
        ardrone_tool_configuration_nb_event = (ardrone_tool_configuration_nb_event + 1) % ARDRONE_TOOL_CONFIGURATION_MAX_EVENT;			\
        if(ardrone_tool_configuration_nb_event == ((ardrone_tool_configuration_current_index + 1) % ARDRONE_TOOL_CONFIGURATION_MAX_EVENT))	\
            ardrone_tool_configuration_event_configure();																				\
        res = TRUE;																														\
    }																																	\
    vp_os_mutex_unlock(&ardrone_tool_configuration_mutex);																				\
	return res;																															\
}

#define ARDRONE_CONFIG_KEY_REF(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, DEFAULT, CALLBACK)
#define ARDRONE_CONFIG_KEY_STR(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, DEFAULT, CALLBACK)												\
bool_t ardrone_tool_configuration_addevent_##NAME(C_TYPE_PTR value, ardrone_tool_configuration_callback result_callback)					\
{																																			\
	bool_t res = FALSE;                                                                                                                     \
	if(value != NULL)                                                                                                                       \
	{																																		\
		vp_os_mutex_lock(&ardrone_tool_configuration_mutex);																				\
		if(ardrone_tool_configuration_current_index == (ardrone_tool_configuration_nb_event + 1) % ARDRONE_TOOL_CONFIGURATION_MAX_EVENT)	\
		{																																	\
			printf("ARDRONE_TOOL_CONFIGURATION QUEUE FILLED !! %s\n", #NAME);																\
		}																																	\
		else																																\
        {																																	\
			ardrone_tool_configuration_data[ardrone_tool_configuration_nb_event].control_mode = ACK_CONTROL_MODE;							\
			ardrone_tool_configuration_data[ardrone_tool_configuration_nb_event].result_callback = result_callback;							\
			ardrone_tool_configuration_data[ardrone_tool_configuration_nb_event].key = #NAME;												\
			ardrone_tool_configuration_data[ardrone_tool_configuration_nb_event].value = vp_os_malloc((strlen((char*)value) + 1) * sizeof(C_TYPE));				\
			vp_os_memcpy(ardrone_tool_configuration_data[ardrone_tool_configuration_nb_event].value, value, (strlen((char*)value) + 1) * sizeof(C_TYPE));		\
			ardrone_tool_configuration_data[ardrone_tool_configuration_nb_event].callback	= (ardrone_at_configuration_set)&ARDRONE_CONFIGURATION_SET_FUNCTION(NAME);	\
			ardrone_tool_configuration_nb_event = (ardrone_tool_configuration_nb_event + 1) % ARDRONE_TOOL_CONFIGURATION_MAX_EVENT;			\
			if(ardrone_tool_configuration_nb_event == ((ardrone_tool_configuration_current_index + 1) % ARDRONE_TOOL_CONFIGURATION_MAX_EVENT))	\
				ardrone_tool_configuration_event_configure();																			 	\
			res = TRUE;																														\
		}																																	\
		vp_os_mutex_unlock(&ardrone_tool_configuration_mutex);																				\
	}																																		\
	return res;																																\
}

#include <config_keys.h>


bool_t ardrone_tool_configuration_get(ardrone_tool_configuration_callback result_callback)
{
	bool_t res = FALSE;
	vp_os_mutex_lock(&ardrone_tool_configuration_mutex);
	if(ardrone_tool_configuration_current_index == (ardrone_tool_configuration_nb_event + 1) % ARDRONE_TOOL_CONFIGURATION_MAX_EVENT)
	{
		printf("ARDRONE_TOOL_CONFIGURATION QUEUE FILLED !!\n");
	}
	else 
	{
		ardrone_tool_configuration_data[ardrone_tool_configuration_nb_event].key = NULL;
		ardrone_tool_configuration_data[ardrone_tool_configuration_nb_event].callback = NULL;
		
		ardrone_tool_configuration_data[ardrone_tool_configuration_nb_event].control_mode = CFG_GET_CONTROL_MODE;
		ardrone_tool_configuration_data[ardrone_tool_configuration_nb_event].result_callback = result_callback;
		ardrone_tool_configuration_nb_event = (ardrone_tool_configuration_nb_event + 1) % ARDRONE_TOOL_CONFIGURATION_MAX_EVENT;
		if(ardrone_tool_configuration_nb_event == ((ardrone_tool_configuration_current_index + 1) % ARDRONE_TOOL_CONFIGURATION_MAX_EVENT))
			ardrone_tool_configuration_event_configure();
		res = TRUE;
	}
	vp_os_mutex_unlock(&ardrone_tool_configuration_mutex);

	return res;
}


/* Multiconfiguration support */
/*============================================================================*/
/** @brief Queues a request to retrieve from the drone the list of available custom configurations.
 *  This functions works like the 'ardrone_tool_configuration_get' function.
 */
bool_t ardrone_tool_custom_configuration_get(ardrone_tool_configuration_callback result_callback)
{
	vp_os_mutex_lock(&ardrone_tool_configuration_mutex);
	//printf("%s %s %i\n",__FILE__,__FUNCTION__,__LINE__);
	if(ardrone_tool_configuration_current_index == (ardrone_tool_configuration_nb_event + 1) % ARDRONE_TOOL_CONFIGURATION_MAX_EVENT)
	{
		printf("ARDRONE_TOOL_CONFIGURATION QUEUE FILLED !!\n");
	}
	else
	{
		ardrone_tool_configuration_data[ardrone_tool_configuration_nb_event].key = NULL;
		ardrone_tool_configuration_data[ardrone_tool_configuration_nb_event].callback = NULL;

		ardrone_tool_configuration_data[ardrone_tool_configuration_nb_event].control_mode = CUSTOM_CFG_GET_CONTROL_MODE;
		ardrone_tool_configuration_data[ardrone_tool_configuration_nb_event].result_callback = result_callback;
		ardrone_tool_configuration_nb_event = (ardrone_tool_configuration_nb_event + 1) % ARDRONE_TOOL_CONFIGURATION_MAX_EVENT;
		if(ardrone_tool_configuration_nb_event == ((ardrone_tool_configuration_current_index + 1) % ARDRONE_TOOL_CONFIGURATION_MAX_EVENT))
			ardrone_tool_configuration_event_configure();
	}
	//printf("%s %s %i\n",__FILE__,__FUNCTION__,__LINE__);
	vp_os_mutex_unlock(&ardrone_tool_configuration_mutex);

	return TRUE;
}

/*============================================================================*/
/** @brief Initializes the configuration manager.
 * This function creates a dictionary, ie. a structure which is used by the
 * .ini file parser to translate into binary values the text file sent by the
 *  drone when the client request the drone configuration file.
 */
void ardrone_tool_configuration_init(void)
{
	if(!ardrone_tool_configuration_is_init)
	{
		/* Create a new dictionary */
		ardrone_tool_configuration_dict = dictionary_new(0);

		/* For each configuration parameter stored in config_keys.h,
		 * we bind it to the address of the corresponding binary data field
		 * inside the 'ardrone_control_config' structure.
		 * This structure will later be filled with the drone configuration.
		 */
#undef ARDRONE_CONFIG_KEY_IMM
#undef ARDRONE_CONFIG_KEY_REF
#undef ARDRONE_CONFIG_KEY_STR
#define ARDRONE_CONFIG_KEY_IMM(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, DEFAULT, CALLBACK) \
iniparser_alias(ardrone_tool_configuration_dict, KEY ":" #NAME, INI_TYPE, &ardrone_control_config.NAME, NULL,RW);
#define ARDRONE_CONFIG_KEY_REF(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, DEFAULT, CALLBACK) \
iniparser_alias(ardrone_tool_configuration_dict, KEY ":" #NAME, INI_TYPE, &ardrone_control_config.NAME, NULL,RW);
#define ARDRONE_CONFIG_KEY_STR(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, DEFAULT, CALLBACK) \
iniparser_alias(ardrone_tool_configuration_dict, KEY ":" #NAME, INI_TYPE, &ardrone_control_config.NAME[0], NULL,RW);
#include <config_keys.h>

		ardrone_tool_configuration_current_index = 0;
		ardrone_tool_configuration_nb_event = 0;
		vp_os_memset(&ardrone_tool_configuration_data[0], 0, sizeof(ardrone_tool_configuration_data_t) * ARDRONE_TOOL_CONFIGURATION_MAX_EVENT);
		vp_os_mutex_init(&ardrone_tool_configuration_mutex);
		ardrone_tool_configuration_is_init = TRUE;
	}
}


/*============================================================================*/
/* Callback function called by the 'control' thread on the client side, once the configuration
 * file was retrieved from the drone and stored in the 'ardrone_control_config' structure.
 */
static void ardrone_tool_configuration_event_configure_end(struct _ardrone_control_event_t* event)
{
	ardrone_tool_configuration_callback callback = NULL;
	int result_callback_argument = FALSE;

	vp_os_mutex_lock(&ardrone_tool_configuration_mutex);

	callback = ardrone_tool_configuration_data[ardrone_tool_configuration_current_index].result_callback;

	switch(event->status)
	{
		case ARDRONE_CONTROL_EVENT_FINISH_SUCCESS:
				result_callback_argument = TRUE;
				if (callback!=NULL)
				{ 
					callback(result_callback_argument); 
				}

			if (NULL!=ardrone_tool_configuration_data[ardrone_tool_configuration_current_index].value)
			{
                vp_os_free(ardrone_tool_configuration_data[ardrone_tool_configuration_current_index].value);
                ardrone_tool_configuration_data[ardrone_tool_configuration_current_index].value = NULL;
			}
			if (NULL!=ardrone_tool_configuration_data[ardrone_tool_configuration_current_index].event)
			{
                vp_os_free(ardrone_tool_configuration_data[ardrone_tool_configuration_current_index].event);
                ardrone_tool_configuration_data[ardrone_tool_configuration_current_index].event = NULL;
			}
			ardrone_tool_configuration_data[ardrone_tool_configuration_current_index].event = NULL;
			ardrone_tool_configuration_data[ardrone_tool_configuration_current_index].value = NULL;
			ardrone_tool_configuration_current_index = (ardrone_tool_configuration_current_index + 1) % ARDRONE_TOOL_CONFIGURATION_MAX_EVENT;
			break;
			
		case ARDRONE_CONTROL_EVENT_FINISH_FAILURE:
				result_callback_argument = FALSE;
				if (callback!=NULL)
				{ 
					callback(result_callback_argument); 
				}
			break;
			
		default:
			// Nothing to do
			break;
	}

    if (ardrone_tool_configuration_current_index != ardrone_tool_configuration_nb_event)
        ardrone_tool_configuration_event_configure();

	vp_os_mutex_unlock(&ardrone_tool_configuration_mutex);

	/*  
	 * Should we trigger the callback after unlocking the mutex, so that
	 * the callback function can add a new event without creating a deadlock ?
	 */

}

static void ardrone_tool_configuration_event_configure(void)
{
	bool_t control_mode_ok = FALSE;
	switch(ardrone_tool_configuration_data[ardrone_tool_configuration_current_index].control_mode)
	{
		case ACK_CONTROL_MODE:
		{
			ardrone_control_ack_event_t *event = NULL;

			if (ardrone_tool_configuration_data[ardrone_tool_configuration_current_index].callback)
			{
			ardrone_tool_configuration_data[ardrone_tool_configuration_current_index].callback(ardrone_tool_configuration_data[ardrone_tool_configuration_current_index].value, ses_id, usr_id, app_id);
			}

			if(ardrone_tool_configuration_data[ardrone_tool_configuration_current_index].event == NULL)
				ardrone_tool_configuration_data[ardrone_tool_configuration_current_index].event = vp_os_malloc(sizeof(ardrone_control_ack_event_t));

			/* ACK_COMMAND_MASK_TRUE means we are going to ask the control thread to negotiate a switch of the ACK bit if it is currently set */
			event = (ardrone_control_ack_event_t*)ardrone_tool_configuration_data[ardrone_tool_configuration_current_index].event;
			event->ack_state = ACK_COMMAND_MASK_TRUE;

			control_mode_ok = TRUE;
		}
		break;

		case CFG_GET_CONTROL_MODE:
		{
			ardrone_control_configuration_event_t *event = NULL;

			if(ardrone_tool_configuration_data[ardrone_tool_configuration_current_index].event == NULL)
				ardrone_tool_configuration_data[ardrone_tool_configuration_current_index].event = vp_os_malloc(sizeof(ardrone_control_configuration_event_t));

			event = (ardrone_control_configuration_event_t*)ardrone_tool_configuration_data[ardrone_tool_configuration_current_index].event;
			event->config_state = CONFIG_REQUEST_INI;
			event->ini_dict = ardrone_tool_configuration_dict;

			control_mode_ok = TRUE;
		}
		break;

		case CUSTOM_CFG_GET_CONTROL_MODE:
		{
			ardrone_control_configuration_event_t *event = NULL;

			if(ardrone_tool_configuration_data[ardrone_tool_configuration_current_index].event == NULL)
				ardrone_tool_configuration_data[ardrone_tool_configuration_current_index].event = vp_os_malloc(sizeof(ardrone_control_configuration_event_t));

			event = (ardrone_control_configuration_event_t*)ardrone_tool_configuration_data[ardrone_tool_configuration_current_index].event;
			event->config_state = CUSTOM_CONFIG_REQUEST;
			event->ini_dict = ardrone_tool_configuration_dict;

			control_mode_ok = TRUE;
		}
		break;


		default:
			break;
	}

	if(control_mode_ok)
	{
		/* Prepare a task for the control thread
		 * We are going to ask this thread to negociate with the drone a ACK bit toggling
		 *  or a configuration file retrieval.
		 */
		ardrone_tool_configuration_data[ardrone_tool_configuration_current_index].event->event = ardrone_tool_configuration_data[ardrone_tool_configuration_current_index].control_mode;
		ardrone_tool_configuration_data[ardrone_tool_configuration_current_index].event->status	= ARDRONE_CONTROL_EVENT_WAITING;
		ardrone_tool_configuration_data[ardrone_tool_configuration_current_index].event->num_retries	= 10;
		ardrone_tool_configuration_data[ardrone_tool_configuration_current_index].event->ardrone_control_event_start = NULL;
		ardrone_tool_configuration_data[ardrone_tool_configuration_current_index].event->ardrone_control_event_end	= ardrone_tool_configuration_event_configure_end;

		/* Add a task in the task-list of the Control thread of the client */
		ardrone_control_send_event( ardrone_tool_configuration_data[ardrone_tool_configuration_current_index].event );
	}
}

/**
 * Sending application defined default application for each category.
 */
void ardrone_tool_send_application_default(void)
{
	printf("Sending default CAT_APPLI settings\n");
#undef ARDRONE_CONFIG_KEY_IMM_a10
#undef ARDRONE_CONFIG_KEY_REF_a10
#undef ARDRONE_CONFIG_KEY_STR_a10
#define ARDRONE_CONFIG_KEY_IMM_a10(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, RW_CUSTOM, DEFAULT, CALLBACK, CATEGORY) \
if (CAT_APPLI == CATEGORY && ardrone_application_default_config.NAME != DEFAULT) { ARDRONE_TOOL_CONFIGURATION_ADDEVENT(NAME, &ardrone_application_default_config.NAME, NULL); }
#define ARDRONE_CONFIG_KEY_REF_a10(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, RW_CUSTOM, DEFAULT, CALLBACK, CATEGORY)
#define ARDRONE_CONFIG_KEY_STR_a10(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, RW_CUSTOM, DEFAULT, CALLBACK, CATEGORY) \
if (CAT_APPLI == CATEGORY && 0 != strcmp(ardrone_application_default_config.NAME, DEFAULT)) { ARDRONE_TOOL_CONFIGURATION_ADDEVENT(NAME, ardrone_application_default_config.NAME, NULL); }
#include <config_keys.h>
}
void ardrone_tool_send_user_default(void)
{
	printf("Sending default CAT_USER settings\n");
#undef ARDRONE_CONFIG_KEY_IMM_a10
#undef ARDRONE_CONFIG_KEY_REF_a10
#undef ARDRONE_CONFIG_KEY_STR_a10
#define ARDRONE_CONFIG_KEY_IMM_a10(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, RW_CUSTOM, DEFAULT, CALLBACK, CATEGORY) \
if (CAT_USER == CATEGORY && ardrone_application_default_config.NAME != DEFAULT) { ARDRONE_TOOL_CONFIGURATION_ADDEVENT(NAME, &ardrone_application_default_config.NAME, NULL); }
#define ARDRONE_CONFIG_KEY_REF_a10(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, RW_CUSTOM, DEFAULT, CALLBACK, CATEGORY)
#define ARDRONE_CONFIG_KEY_STR_a10(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, RW_CUSTOM, DEFAULT, CALLBACK, CATEGORY) \
if (CAT_USER == CATEGORY && 0 != strcmp(ardrone_application_default_config.NAME, DEFAULT)) { ARDRONE_TOOL_CONFIGURATION_ADDEVENT(NAME, ardrone_application_default_config.NAME, NULL); }
#include <config_keys.h>
}
void ardrone_tool_send_session_default(void)
{
	printf("Sending default CAT_SESSION settings\n");
#undef ARDRONE_CONFIG_KEY_IMM_a10
#undef ARDRONE_CONFIG_KEY_REF_a10
#undef ARDRONE_CONFIG_KEY_STR_a10
#define ARDRONE_CONFIG_KEY_IMM_a10(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, RW_CUSTOM, DEFAULT, CALLBACK, CATEGORY) \
if (CAT_SESSION == CATEGORY && ardrone_application_default_config.NAME != DEFAULT) { ARDRONE_TOOL_CONFIGURATION_ADDEVENT(NAME, &ardrone_application_default_config.NAME, NULL); }
#define ARDRONE_CONFIG_KEY_REF_a10(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, RW_CUSTOM, DEFAULT, CALLBACK, CATEGORY)
#define ARDRONE_CONFIG_KEY_STR_a10(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, RW_CUSTOM, DEFAULT, CALLBACK, CATEGORY) \
if (CAT_SESSION == CATEGORY && 0 != strcmp(ardrone_application_default_config.NAME, DEFAULT)) { ARDRONE_TOOL_CONFIGURATION_ADDEVENT(NAME, ardrone_application_default_config.NAME, NULL); }
#include <config_keys.h>
}
