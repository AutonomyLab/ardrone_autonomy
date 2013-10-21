//
//  academy_common.h
//  ARDroneEngine
//
//  Created by Frédéric D'Haeyer on 2/28/12.
//  Copyright (c) 2012 Parrot SA. All rights reserved.
//

#ifndef _ACADEMY_COMMON_H_
#define _ACADEMY_COMMON_H_
#include <VP_Os/vp_os_types.h>

#define ACADEMY_USERNAME_SIZE		64
#define ACADEMY_PASSWORD_SIZE		64

typedef enum _ACADEMY_RESULT_
{
	ACADEMY_RESULT_NONE = 0,
	ACADEMY_RESULT_OK,
	ACADEMY_RESULT_FAILED,
} ACADEMY_RESULT;

typedef enum _ACADEMY_STATE_
{
	ACADEMY_STATE_NONE = 0,
	ACADEMY_STATE_CONNECTION,
	ACADEMY_STATE_PREPARE_PROCESS,
	ACADEMY_STATE_PROCESS,
	ACADEMY_STATE_FINISH_PROCESS,
	ACADEMY_STATE_DISCONNECTION,
	ACADEMY_STATE_MAX
} ACADEMY_STATE;

typedef struct _academy_state_t_
{
	ACADEMY_STATE 	state;
	ACADEMY_RESULT 	result;
} academy_state_t;

typedef struct _academy_user_t_
{
	char username[ACADEMY_USERNAME_SIZE];
	char password[ACADEMY_PASSWORD_SIZE];
} academy_user_t;

typedef void (*academy_callback)(academy_state_t state);
typedef void (*academy_download_new_media)(const char *mediaPath, bool_t addToQueue);

#endif // _ACADEMY_COMMON_H_
