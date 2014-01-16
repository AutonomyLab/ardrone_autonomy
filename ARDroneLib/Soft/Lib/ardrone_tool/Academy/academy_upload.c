/**
 *  academy_upload.c
 *
 *  Created by Frederic D'HAEYER on 8/04/11.
 *  Copyright 2011 Parrot SA. All rights reserved.
 *
 */
#include <ardrone_tool/Academy/academy_upload.h>

typedef struct _academy_upload_t_
{
	vp_os_mutex_t mutex;
	vp_os_cond_t cond;
	academy_callback callback;
	academy_state_t state;
	academy_user_t user;
	bool_t connected;
} academy_upload_t;

PROTO_THREAD_ROUTINE(academy_upload, data);

static bool_t academy_upload_started = FALSE;
static THREAD_HANDLE academy_upload_thread;
static academy_upload_t academy_upload;

C_RESULT academy_connect(const char *username, const char *password, academy_callback callback)
{
	C_RESULT result = C_FAIL;

	if(academy_upload_started && (academy_upload.state.state == ACADEMY_STATE_NONE))
    {
        if(!academy_upload.connected)
        {
            _ftp_t *academy_ftp = NULL;
            _ftp_status status;
            strcpy(academy_upload.user.username, username ? username : "");
            strcpy(academy_upload.user.password, password ? password : "");

            if(callback != NULL)
                academy_upload.callback = callback;

            academy_ftp = ftpConnectFromName(ACADEMY_SERVERNAME, ACADEMY_PORT, academy_upload.user.username, academy_upload.user.password, &status);
            if(academy_ftp != NULL)
            {
                ftpClose(&academy_ftp);
                vp_os_mutex_lock(&academy_upload.mutex);
                academy_upload.connected = TRUE;
                vp_os_cond_signal(&academy_upload.cond);
                vp_os_mutex_unlock(&academy_upload.mutex);
                result = C_OK;
            }
        }
        else
            result = C_OK;
    }
    
	return result;
}

C_RESULT academy_disconnect(void)
{
	C_RESULT result = C_FAIL;

	if(academy_upload_started)
    {
        if(academy_upload.connected)
        {
            vp_os_mutex_lock(&academy_upload.mutex);
            academy_upload.connected = FALSE;
            vp_os_mutex_unlock(&academy_upload.mutex);
            result = C_OK;
        }
        else
            result = C_OK;
    }
    
	return result;
}

static void academy_upload_private_callback(academy_state_t state)
{
	static char msg[128] = { 0 };
	switch(state.result)
	{
		case ACADEMY_RESULT_NONE:
			switch(state.state)
			{
				case ACADEMY_STATE_CONNECTION:
					sprintf(msg, "Connecting to %s...", ACADEMY_SERVERNAME);
					break;

				case ACADEMY_STATE_PREPARE_PROCESS:
					sprintf(msg, "Preparing upload from local directory to %s...", ACADEMY_SERVERNAME);
					break;

				case ACADEMY_STATE_PROCESS:
					sprintf(msg, "Uploading from local directory to %s...", ACADEMY_SERVERNAME);
					break;

				case ACADEMY_STATE_FINISH_PROCESS:
					sprintf(msg, "Finishing upload from local directory to %s...", ACADEMY_SERVERNAME);
					break;

				case ACADEMY_STATE_DISCONNECTION:
					sprintf(msg, "Disconnection to %s...", ACADEMY_SERVERNAME);
					break;

				case ACADEMY_STATE_NONE:
					strcpy(msg, "");
					break;

				default:
					strcpy(msg, "");
					break;
			}
			break;

		case ACADEMY_RESULT_OK:
			strcat(msg, "OK");
			break;

		case ACADEMY_RESULT_FAILED:
			strcat(msg, "FAILED");
			break;
	}
	if(strlen(msg) != 0)
		PA_DEBUG("===========> %s\n", msg);
}

static void academy_upload_setState(academy_upload_t *academy, ACADEMY_STATE state)
{
	academy_setState(&academy->state, state);
	if(academy->callback)
		academy->callback(academy->state);
}

static void academy_upload_resetState(academy_upload_t *academy)
{
	academy_resetState(&academy->state);
	if(academy->callback)
		academy->callback(academy->state);
}

static void academy_upload_nextState(academy_upload_t *academy)
{
	academy_nextState(&academy->state);
	if(academy->callback)
		academy->callback(academy->state);
}

static void academy_upload_stateOK(academy_upload_t *academy)
{
	academy_stateOK(&academy->state);
	if(academy->callback)
		academy->callback(academy->state);
}

static void academy_upload_stateERROR(academy_upload_t *academy)
{
	academy_stateERROR(&academy->state);
	if(academy->callback)
		academy->callback(academy->state);
	academy_upload_resetState(academy);
}

void academy_upload_init(void)
{
	if(!academy_upload_started)
	{
		vp_os_mutex_init(&academy_upload.mutex);
		vp_os_cond_init(&academy_upload.cond, &academy_upload.mutex);
		academy_upload.connected = FALSE;
		vp_os_memset(&academy_upload.user, 0, sizeof(academy_user_t));
		academy_upload_started = TRUE;
		vp_os_thread_create(thread_academy_upload, (THREAD_PARAMS)&academy_upload, &academy_upload_thread);
	}
}

void academy_upload_shutdown(void)
{
	if(academy_upload_started)
	{
		academy_upload_started = FALSE;
		vp_os_mutex_lock(&academy_upload.mutex);
		vp_os_cond_signal(&academy_upload.cond);
		vp_os_mutex_unlock(&academy_upload.mutex);
		vp_os_thread_join(academy_upload_thread);
	}
}

DEFINE_THREAD_ROUTINE(academy_upload, data)
{
	char *directoryList = NULL;
	char *fileList = NULL;
	char dirname[ACADEMY_MAX_FILENAME];
	_ftp_t *academy_ftp = NULL;
	_ftp_status academy_status;
	char *ptr = NULL;
	academy_upload_t *academy = (academy_upload_t *)data;

	printf("Start thread %s\n", __FUNCTION__);

	while( academy_upload_started && !ardrone_tool_exit() )
	{
		vp_os_mutex_lock(&academy->mutex);
		vp_os_memset(&academy->user, 0, sizeof(academy_user_t));
		academy->callback = &academy_upload_private_callback;
		academy->connected = FALSE;
		academy_upload_resetState(academy);
		vp_os_cond_wait(&academy->cond);
		vp_os_mutex_unlock(&academy->mutex);

		while(academy_upload_started && academy->connected)
		{
			academy_status = FTP_FAIL;
			academy_upload_nextState(academy);

			switch(academy->state.state)
			{
				case ACADEMY_STATE_CONNECTION:
					{
						struct dirent *dirent = NULL;
						DIR *dir = NULL;
						academy_status = FTP_FAIL;
						// Check if flight_* directory exist in local dir
						if((dir = opendir(flight_dir)) != NULL)
						{
							struct stat statbuf;
							while(FTP_FAILED(academy_status) && (dirent = readdir(dir)) != NULL)
							{
								if((strncmp(dirent->d_name, "flight_", strlen("flight_")) == 0))
								{
									char local_dir[ACADEMY_MAX_FILENAME];
									sprintf(dirname, "%s", dirent->d_name);
									sprintf(local_dir, "%s/%s", flight_dir, dirname);
									if((stat(local_dir, &statbuf) == 0) && S_ISDIR(statbuf.st_mode))
										academy_status = FTP_SUCCESS;
								}
							}
                            closedir(dir);
						}

						if(FTP_SUCCEDED(academy_status))
						{
							if(academy_ftp == NULL)
								academy_ftp = ftpConnectFromName(ACADEMY_SERVERNAME, ACADEMY_PORT, academy->user.username, academy->user.password, &academy_status);
						}
					}
					break;

				case ACADEMY_STATE_PREPARE_PROCESS:
					academy_status = ftpCd(academy_ftp, "/Uploaded");
					if(FTP_FAILED(academy_status))
					{
						ftpMkdir(academy_ftp, "/Uploaded");
						academy_status = ftpCd(academy_ftp, "/Uploaded");
					}

					if(FTP_SUCCEDED(academy_status))
					{
						academy_status = ftpList(academy_ftp, &directoryList, NULL);
						if(FTP_SUCCEDED(academy_status))
						{
							bool_t found = FALSE;
							char *next_dir = NULL;

							while(!found && (ptr = academy_get_next_item_with_prefix(directoryList, &next_dir, "flight_", TRUE)))
							{
								if(strcmp(ptr, dirname) == 0)
								{
									found = TRUE;
									academy_upload_setState(academy, ACADEMY_STATE_FINISH_PROCESS);
								}
							}

							if(directoryList != NULL)
							{
								vp_os_free(directoryList);
								directoryList = NULL;
							}
						}
					}
					break;

				case ACADEMY_STATE_PROCESS:
					academy_status = ftpCd(academy_ftp, "/Uploading");
					if(FTP_FAILED(academy_status))
					{
						ftpMkdir(academy_ftp, "/Uploading");
						academy_status = ftpCd(academy_ftp, "/Uploading");
					}

					if(FTP_SUCCEDED(academy_status))
					{
						ftpMkdir(academy_ftp, dirname);
						academy_status = ftpCd(academy_ftp, dirname);
						if(FTP_SUCCEDED(academy_status))
						{
							char local_dir[ACADEMY_MAX_FILENAME];
							struct dirent *dirent = NULL;
							DIR *dir = NULL;

							sprintf(local_dir, "%s/%s", flight_dir, dirname);
							if((dir = opendir(local_dir)) != NULL)
							{
								char local_filename[ACADEMY_MAX_FILENAME];
								struct stat statbuf;
								while(FTP_SUCCEDED(academy_status) && ((dirent = readdir(dir)) != NULL))
								{
									if((strncmp(dirent->d_name, "picture_", strlen("picture_")) == 0) || (strncmp(dirent->d_name, "userbox_", strlen("userbox_")) == 0))
									{
										sprintf(local_filename, "%s/%s", local_dir, dirent->d_name);
										if((stat(local_filename, &statbuf) == 0) && !S_ISDIR(statbuf.st_mode))
										{
											PA_DEBUG("Put %s from local directory to server...", dirent->d_name);
											academy_status = ftpPut(academy_ftp, local_filename, dirent->d_name, 1, NULL);
											if(FTP_SUCCEDED(academy_status))
												PA_DEBUG("OK\n");
											else
												PA_DEBUG("ERROR\n");
										}
									}
								}
                                
                                closedir(dir);
							}
						}
					}
					break;

				case ACADEMY_STATE_FINISH_PROCESS:
					{
						char local_dir[ACADEMY_MAX_FILENAME];
						char src[ACADEMY_MAX_FILENAME];
						char dest[ACADEMY_MAX_FILENAME];
						sprintf(src, "/Uploading/%s", dirname);
						sprintf(dest, "/Uploaded/%s", dirname);
						academy_status = ftpRename(academy_ftp, src, dest);

						// Penser à supprimer le fichier local
						academy_status = FTP_FAIL;
						sprintf(local_dir, "%s/%s", flight_dir, dirname);
						if(!ftw(local_dir, &academy_remove, ACADEMY_MAX_FD_FOR_FTW))
						{
							rmdir(local_dir);
							academy_status = FTP_SUCCESS;
						}
					}
					break;

				case ACADEMY_STATE_DISCONNECTION:
					if(academy_ftp != NULL)
						ftpClose(&academy_ftp);
					academy_status = FTP_SUCCESS;
					break;

				default:
					// Nothing to do
					break;
			}

			if(FTP_SUCCEDED(academy_status))
			{
				PA_DEBUG("continue state from %d to %d\n", academy->state.state, (academy->state.state + 1) % ACADEMY_STATE_MAX);
				academy_upload_stateOK(academy);
			}
			else
			{
				PA_DEBUG("stop\n");
				academy_upload_stateERROR(academy);

				if(fileList != NULL)
				{
					vp_os_free(fileList);
					fileList = NULL;
				}

				if(academy_ftp)
					ftpClose(&academy_ftp);

				vp_os_delay(1000);
			}
		}

		if(fileList != NULL)
		{
			vp_os_free(fileList);
			fileList = NULL;
		}

		if(academy_ftp)
			ftpClose(&academy_ftp);
	} // while

	THREAD_RETURN(C_OK);
}
