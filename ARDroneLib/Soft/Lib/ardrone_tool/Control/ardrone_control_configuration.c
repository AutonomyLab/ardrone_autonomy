#include <VP_Os/vp_os_malloc.h>
#include <VP_Os/vp_os_print.h>

#include <ardrone_api.h>

#include <ardrone_tool/Control/ardrone_control_configuration.h>

#include <config_keys.h>

static uint8_t ini_buffer[ARDRONE_CONTROL_CONFIGURATION_INI_BUFFER_SIZE];
static uint32_t ini_buffer_index = 0;

static void set_state(ardrone_control_configuration_event_t* event, ardrone_config_state_t s)
{
  DEBUG_PRINT_SDK("[CONTROL CONFIGURATION] Switching to state %d\n", s);

  event->config_state = s;
}

static inline void noprintf(char *s,...){}

/* Activate debugging this file */
	//#define DEBUG_CONFIG_RECEIVE PRINT
	#define DEBUG_CONFIG_RECEIVE noprintf

//extern const char * custom_configuration_headers[NB_CONFIG_CATEGORIES];
//extern custom_configuration_list_t available_configurations[NB_CONFIG_CATEGORIES];

void ardrone_control_reset_custom_configurations_list(custom_configuration_list_t *available_configurations)
{
	int i;

	for (i=0;i<NB_CONFIG_CATEGORIES;i++)
	{
		if (available_configurations[i].list) { vp_os_free(available_configurations[i].list); }
		available_configurations[i].list=NULL;
		available_configurations[i].nb_configurations=0;
	}
}


void ardrone_control_read_custom_configurations_list(/*in*/uint8_t * buffer,
													 /*in*/int buffer_size,
													 /*out*/custom_configuration_list_t *available_configurations)
{
	custom_configuration_list_t * current_scope = NULL;
	char id[CUSTOM_CONFIGURATION_ID_LENGTH+1];
	char description[1024];
	int index = 0;
	uint8_t * pindex; 
	int j;
	uint8_t * end_of_buffer;

	index = 0;
	pindex = buffer;
	end_of_buffer = buffer + buffer_size;

	DEBUG_CONFIG_RECEIVE("Decoding %i bytes",buffer_size);
	DEBUG_CONFIG_RECEIVE("\n");

	while(1)
	{
		//DEBUG_CONFIG_RECEIVE("Analysing <"); for (i=index;i<buffer_size;i++) DEBUG_CONFIG_RECEIVE("[%i]",buffer[i]); DEBUG_CONFIG_RECEIVE(">\n");
		/* Go to the beginning of a section */
			while((*pindex)!='[') { index++; pindex++; if (pindex==end_of_buffer) return; }
		/* Search the end of the section name */
			for (;index<buffer_size;index++) { if (buffer[index]==13 ) { buffer[index]=0; break; } }    if(index==buffer_size) return;
		/* Search the corresponding category */
			for (j=0;j<NB_CONFIG_CATEGORIES;j++){
				if ( strcmp((char*)custom_configuration_headers[j],(char*)pindex)==0 ){
					/* Found the category */
					current_scope = &available_configurations[j];
					DEBUG_CONFIG_RECEIVE(" Found Scope <%s>\n",custom_configuration_headers[j]);
					break;
				}
			}
			if (j==NB_CONFIG_CATEGORIES) { DEBUG_CONFIG_RECEIVE("Unknown category."); return ;}
		/* Reset the list */
			if (current_scope!=NULL)
			{
				current_scope->nb_configurations = 0;
				if (current_scope->list!=NULL) { vp_os_free(current_scope->list); current_scope->list = NULL; }
			}
		/* Points on the first ID */
			index++;
			pindex=buffer+index;

		/* Read the IDs */
			while(pindex<end_of_buffer && (*pindex)!='[' && (*pindex)!=0)
			{
				vp_os_memset(id,0,sizeof(id));
				vp_os_memset(description,0,sizeof(description));

				//DEBUG_CONFIG_RECEIVE("Now scanning <%c> %i\n",*pindex,index);
				for (;index<buffer_size;index++) { if (buffer[index]==',' || buffer[index]=='\r') { buffer[index]=0; break; } }   if(index==buffer_size) return;
				strncpy(id,(char*)pindex,sizeof(id));
				index++;
				pindex=buffer+index;
				for (;index<buffer_size;index++) { if (buffer[index]==0 || buffer[index]=='\r') { buffer[index]=0; break; } }   if(index==buffer_size) return;
				strncpy(description,(char*)pindex,sizeof(description));
				DEBUG_CONFIG_RECEIVE(" Found ID <%s> description <%s>\n",id,description);
				index++;
				pindex=buffer+index;

				/* Store the found ID */
					/* Increase the size of the list by one element */
					current_scope->list = vp_os_realloc(current_scope->list,sizeof(*current_scope->list)*(current_scope->nb_configurations+1));
					/* Store the new element */
					strncpy(current_scope->list[current_scope->nb_configurations].id,
							id,
							sizeof(current_scope->list[current_scope->nb_configurations].id)  );
					strncpy(current_scope->list[current_scope->nb_configurations].description,
							description,
							sizeof(current_scope->list[current_scope->nb_configurations].description)  );
					current_scope->nb_configurations++;
			}
	}
	return;
}



/**
 * \brief Runs a control event managing the configuration file.
 * This functions handles reading the configuration information
 * sent by the drone on the TCP 'control' socket on port 5559.
 * Called by the 'ardrone_control' thread loop in 'ardrone_control.c'.
 */
C_RESULT ardrone_control_configuration_run( uint32_t ardrone_state, ardrone_control_configuration_event_t* event )
{
	int32_t buffer_size;
	char *start_buffer, *buffer;
	char *value, *param, *c;
	bool_t ini_buffer_end, ini_buffer_more;
	C_RESULT res = C_OK;
	int bytes_to_read;

	/* Multiconfiguration support */
	static uint8_t *custom_configuration_list_buffer = NULL;
	static int  custom_configuration_list_buffer_size = 0;
	static int  custom_configuration_list_data_size = 0;
	#define CUSTOM_CFG_BLOCK_SIZE 1024

	int i;

	switch( event->config_state )
	{
		case CONFIG_REQUEST_INI:
			if( ardrone_state & ARDRONE_COMMAND_MASK )
			{
				/* If the ACK bit is set, we must ask the drone to clear it before asking the configuration. */
				DEBUG_CONFIG_RECEIVE("%s %s %i - Requesting ack. bit reset.\n",__FILE__,__FUNCTION__,__LINE__);
				ardrone_at_update_control_mode(ACK_CONTROL_MODE);
			}
			else
			{
				DEBUG_CONFIG_RECEIVE("%s %s %i - Requesting the configuration.\n",__FILE__,__FUNCTION__,__LINE__);
				ini_buffer_index = 0;
				ardrone_at_configuration_get_ctrl_mode();
				set_state(event, CONFIG_RECEIVE_INI);
			}
			break;

		/* multi-configuration support */
		case CUSTOM_CONFIG_REQUEST:
			DEBUG_CONFIG_RECEIVE("%s %s %i\n",__FILE__,__FUNCTION__,__LINE__);
			if( ardrone_state & ARDRONE_COMMAND_MASK )
			{
				DEBUG_CONFIG_RECEIVE("%s %s %i - Requesting ack. bit reset.\n",__FILE__,__FUNCTION__,__LINE__);
				ardrone_at_update_control_mode(ACK_CONTROL_MODE);
			}
			else
			{
				DEBUG_CONFIG_RECEIVE("%s %s %i - Requesting the custom config.\n",__FILE__,__FUNCTION__,__LINE__);
				custom_configuration_list_buffer = NULL;
				custom_configuration_list_buffer_size = 0;
				custom_configuration_list_data_size = 0;
				ardrone_at_custom_configuration_get_ctrl_mode();
				set_state(event, CUSTOM_CONFIG_RECEIVE);
			}
			break;

		case CONFIG_RECEIVE_INI:
			DEBUG_CONFIG_RECEIVE("%s %s %i - Trying to read the control socket.\n",__FILE__,__FUNCTION__,__LINE__);
			// Read data coming from ARDrone
			buffer_size = ARDRONE_CONTROL_CONFIGURATION_INI_BUFFER_SIZE - ini_buffer_index;
			res = ardrone_control_read( &ini_buffer[ini_buffer_index], &buffer_size );
			DEBUG_CONFIG_RECEIVE("Received <<%s>>\n",&ini_buffer[ini_buffer_index]);

			if(VP_SUCCEEDED(res))
			{      
				buffer_size += ini_buffer_index;

				ini_buffer[buffer_size]=0;  // Makes sure the buffer ends with a zero

				// Parse received data
				if( buffer_size > 0 )
				{
					//DEBUG_CONFIG_RECEIVE(" Searching value\n");

					ini_buffer_end  = ini_buffer[buffer_size-1] == '\0'; // End of configuration data
					ini_buffer_more = ini_buffer[buffer_size-1] != '\n'; // Need more configuration data to end parsing current line

					//if (ini_buffer_end) DEBUG_CONFIG_RECEIVE(" ini_buffer_end\n");
					//if (ini_buffer_more) DEBUG_CONFIG_RECEIVE(" ini_buffer_more\n");

					start_buffer = (char*)&ini_buffer[0];
					buffer = strchr(start_buffer, '\n');

					//DEBUG_CONFIG_RECEIVE("Le start buffer : <<%s>>\n",start_buffer);

					while( buffer != NULL )
					{
						//DEBUG_CONFIG_RECEIVE(" Found an '\\n' \n");

						value = start_buffer;
						param = strchr(value, '=');

						*buffer = '\0';
						*param  = '\0';

						// Remove spaces at end of strings
						c = param - 1;
						while( *c == ' ' )
						{
							*c = '\0';
							c  = c-1;
						}

						c = buffer-1;
						while( *c == ' ' )
						{
							*c = '\0';
							c  = c-1;
						}

						// Remove spaces at beginning of strings
						while( *value == ' ' )
						{
							value = value + 1;
						}

						param = param + 1;
						while( *param == ' ' )
						{
							param = param + 1;
						}

						DEBUG_CONFIG_RECEIVE(" Decoding from control stream : <%s>=<%s>\n",value,param);
						iniparser_setstring( event->ini_dict, value, param );

						start_buffer = buffer + 1;
						buffer = strchr(start_buffer, '\n');
					}

					if( ini_buffer_end )
					{
						/* Reading the condfiguration is finished, we ask the drone
						 * to clear the ACK bit */
						ardrone_at_update_control_mode(ACK_CONTROL_MODE);
						set_state(event, CONFIG_RECEIVED);
					}
					else if( ini_buffer_more )
					{
						// Compute number of bytes to copy
						int32_t size = (int32_t)&ini_buffer[buffer_size] - (int32_t)start_buffer;
						vp_os_memcpy( &ini_buffer[0], start_buffer, size );
						ini_buffer_index = size;
					}
					else
					{
						/* The previous line was completely consumed -  next data
						 * from the network will be stored at the beginning of the
						 * buffer. */
						ini_buffer_index = 0;
					}
				}
			}
         else
         {
        	 res = C_FAIL;

        	 DEBUG_CONFIG_RECEIVE("%s %s %i - no data to read from the control socket.\n",__FILE__,__FUNCTION__,__LINE__);

            if(!(ardrone_state & ARDRONE_COMMAND_MASK))
				   set_state(event, CONFIG_REQUEST_INI);
         }
		break;

		case CUSTOM_CONFIG_RECEIVE:
			DEBUG_CONFIG_RECEIVE("%s %s %i - Trying to read the control socket.\n",__FILE__,__FUNCTION__,__LINE__);
			DEBUG_CONFIG_RECEIVE("%s %s %i\n",__FILE__,__FUNCTION__,__LINE__);
			/* Read data until a zero byte is received */

			/* Clear old data from the buffer when receiving the first bytes */
				if (custom_configuration_list_buffer!=NULL && custom_configuration_list_data_size==0){
					vp_os_memset(custom_configuration_list_buffer,0,custom_configuration_list_buffer_size);
				}

			/* Enlarge the buffer if needed */
				if (custom_configuration_list_data_size==custom_configuration_list_buffer_size)
				{
					custom_configuration_list_buffer_size += CUSTOM_CFG_BLOCK_SIZE;
					custom_configuration_list_buffer = vp_os_realloc(custom_configuration_list_buffer,custom_configuration_list_buffer_size);
				}
			/* Read data at the end of the buffer */
				bytes_to_read = custom_configuration_list_buffer_size - custom_configuration_list_data_size;
				res = ardrone_control_read( &custom_configuration_list_buffer[custom_configuration_list_data_size], &bytes_to_read );
				DEBUG_CONFIG_RECEIVE(" Reading %i bytes of the custom config. list\n",bytes_to_read);
				for (i=0;i<bytes_to_read;i++) { DEBUG_CONFIG_RECEIVE("<%i>",custom_configuration_list_buffer[i]); }

			/* Searches a zero */
				if (VP_SUCCEEDED(res))
				{
					custom_configuration_list_data_size += bytes_to_read;
					for (i=bytes_to_read;i>0;i--){
						if (custom_configuration_list_buffer[custom_configuration_list_data_size - i] == 0) {

							/* Reading the condfiguration is finished, we ask the drone
							 * to clear the ACK bit */
							DEBUG_CONFIG_RECEIVE("Finished receiving\n");
							ardrone_at_update_control_mode(ACK_CONTROL_MODE);
							set_state(event, CUSTOM_CONFIG_RECEIVED);

							ardrone_control_reset_custom_configurations_list(available_configurations);

							ardrone_control_read_custom_configurations_list(custom_configuration_list_buffer,
																			custom_configuration_list_data_size,
																			available_configurations);

							/* Clean up */
								vp_os_sfree((void**)&custom_configuration_list_buffer);
								custom_configuration_list_buffer_size = 0;
								custom_configuration_list_data_size = 0;
							res = C_OK;
							break;
						}
					}

				}
				else{
					/* No data are available on the control socket. */
					DEBUG_CONFIG_RECEIVE("%s %s %i - no data to read from the control socket.\n",__FILE__,__FUNCTION__,__LINE__);
					/* Reset the buffer */
					custom_configuration_list_data_size = 0;
					/* The request for the configuration file should have been acknowledged by the drone.
					 * If not, another request is sent.	*/
					if(!(ardrone_state & ARDRONE_COMMAND_MASK))   set_state(event, CUSTOM_CONFIG_REQUEST);
				}

				break;


		case CONFIG_RECEIVED:
		case CUSTOM_CONFIG_RECEIVED:
				/* We finished reading the configuration, we wait for the drone to reset the ACK bit */
				if( ardrone_state & ARDRONE_COMMAND_MASK )
				{
					/* If the ACK bit is set, we must ask the drone to clear it before asking the configuration. */
					DEBUG_PRINT("%s %s %i - Requesting ack. bit reset.\n",__FILE__,__FUNCTION__,__LINE__);
					ardrone_at_update_control_mode(ACK_CONTROL_MODE);
				}
				else
				{
					DEBUG_PRINT("%s %s %i - Finished.\n",__FILE__,__FUNCTION__,__LINE__);
					event->status = ARDRONE_CONTROL_EVENT_FINISH_SUCCESS;
				}
				res = C_OK;
				break;


		default:
         res = C_FAIL;
			break;
	}

	return res;
}
