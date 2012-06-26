#include <unistd.h>
#include <Maths/maths.h>
#include <Maths/matrices.h>

#undef ARDRONE_CONFIG_KEY_IMM
#undef ARDRONE_CONFIG_KEY_REF
#undef ARDRONE_CONFIG_KEY_STR
#include <config_keys.h>

const vector31_t default_accs_offset = {{{ -2048.0f, 2048.0f, 2048.0f}}};
const matrix33_t default_accs_gain = {1.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f, 0.0f, -1.0f };
const vector31_t default_gyros_offset = {{{ 1662.5f, 1662.5f, 1662.5f}}};
const vector31_t default_gyros_gains = {{{ 395.0f * MDEG_TO_RAD, -395.0f * MDEG_TO_RAD, -207.5f * MDEG_TO_RAD }}};
const vector21_t default_gyros110_offset = {{ 1662.5f, 1662.5f}};
const vector21_t default_gyros110_gains = {{ 87.5f * MDEG_TO_RAD, -87.5f * MDEG_TO_RAD }};
const vector31_t default_magneto_offset = {{{ 0.0f, 0.0f, 0.0f}}};
const float32_t  default_magneto_radius = 0.0f;

/* Stephane - multiconfiguration support */

	/* Those data are present both on the client and the drone side */

	const char * custom_configuration_id_keys[NB_CONFIG_CATEGORIES+1]=
	{
			"",
			"custom:application_desc",
			"custom:profile_desc",
			"custom:session_desc",
	};

	const char * custom_configuration_headers[NB_CONFIG_CATEGORIES+1]=
	{
			"[common]",
			"[applis]",
			"[profiles]",
			"[sessions]"
	};

	const char *configuration_switching_commands[NB_CONFIG_CATEGORIES+1]=
	{
			"",
			"custom:application_id",
			"custom:profile_id",
			"custom:session_id",
			NULL
	};

	custom_configuration_list_t available_configurations[NB_CONFIG_CATEGORIES];


	/* Stephane - multiconfiguration support */

	C_RESULT configuration_check_config_id_char(const char session_id_char)
	{
		char c = session_id_char;
		return ( (c>='a' && c<='f') || (c>='A' && c<='F') || (c>='0' && c<='9') );
	}

	C_RESULT configuration_check_config_id(const char * session_id)
	{
		int i;
		unsigned char c;
		// Session IDs should be strings containing a 32-bit integer hexadecimal representation
		if (session_id==NULL) return C_FAIL;
		for (i=0;i<CUSTOM_CONFIGURATION_ID_LENGTH;i++){
			c=session_id[i];
			if (c==0) { return C_FAIL; }  /* string is too short */
			if (!(configuration_check_config_id_char(c))) { return C_FAIL; }  /* character is invalid */
		}
		if (session_id[i]!=0) { return C_FAIL; } /* string is too long */
		return C_OK;
	}
