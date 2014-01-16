/**
 *  academy_download.c
 *
 *  Created by Frederic D'HAEYER on 8/04/11.
 *  Copyright 2011 Parrot SA. All rights reserved.
 *
 */
#include <ardrone_tool/Academy/academy_download.h>

#define DISPLAY_STRING_LENGTH (128)

typedef struct _academy_download_t_
{
    academy_callback callback;
    academy_download_new_media new_media_callback;
    academy_state_t state;
} academy_download_t;

PROTO_THREAD_ROUTINE(academy_download, data);

static bool_t academy_download_started = FALSE;
static bool_t academy_download_in_pause = FALSE;
static THREAD_HANDLE academy_download_thread;
static vp_os_mutex_t academy_download_mutex;
static vp_os_cond_t academy_download_cond;

static void academy_download_private_callback(academy_state_t state)
{
    static char msg[DISPLAY_STRING_LENGTH] = { 0 };
    switch(state.result)
    {
    case ACADEMY_RESULT_NONE:
        switch(state.state)
        {
        case ACADEMY_STATE_CONNECTION:
            snprintf (msg, DISPLAY_STRING_LENGTH, "Connecting to %s...", &wifi_ardrone_ip[0]);
            break;

        case ACADEMY_STATE_PREPARE_PROCESS:
            snprintf (msg, DISPLAY_STRING_LENGTH, "Preparing download from %s to local directory...", &wifi_ardrone_ip[0]);
            break;

        case ACADEMY_STATE_PROCESS:
            snprintf (msg, DISPLAY_STRING_LENGTH, "Downloading from %s to local directory...", &wifi_ardrone_ip[0]);
            break;

        case ACADEMY_STATE_FINISH_PROCESS:
            snprintf (msg, DISPLAY_STRING_LENGTH, "Finishing download from %s to local directory...", &wifi_ardrone_ip[0]);
            break;

        case ACADEMY_STATE_DISCONNECTION:
            snprintf (msg, DISPLAY_STRING_LENGTH, "Disconnection to %s...", &wifi_ardrone_ip[0]);
            break;

        case ACADEMY_STATE_NONE:
            strncpy(msg, "", DISPLAY_STRING_LENGTH);
            break;

        default:
            strncpy(msg, "", DISPLAY_STRING_LENGTH);
            break;
        }
        break;

    case ACADEMY_RESULT_OK:
        strncat(msg, "OK", DISPLAY_STRING_LENGTH - strlen (msg));
        break;

    case ACADEMY_RESULT_FAILED:
        strncat(msg, "FAILED", DISPLAY_STRING_LENGTH - strlen (msg));
        break;
    }

    if(strlen(msg) != 0)
        PA_DEBUG("===========> %s\n", msg);
}

static void academy_download_resetState(academy_download_t *academy)
{
    academy_resetState(&academy->state);
    if(academy->callback)
        academy->callback(academy->state);
}

static void academy_download_nextState(academy_download_t *academy)
{
    academy_nextState(&academy->state);
    if(academy->callback)
        academy->callback(academy->state);
}

static void academy_download_stateOK(academy_download_t *academy)
{
    academy_stateOK(&academy->state);
    if(academy->callback)
        academy->callback(academy->state);
}

static void academy_download_stateERROR(academy_download_t *academy)
{
    academy_stateERROR(&academy->state);
    if(academy->callback)
        academy->callback(academy->state);
    academy_download_resetState(academy);
}

void academy_download_init(academy_download_new_media academy_download_callback_func)
{
    if(!academy_download_started)
    {
        vp_os_mutex_init (&academy_download_mutex);
        vp_os_cond_init (&academy_download_cond, &academy_download_mutex);
        academy_download_started = TRUE;
        vp_os_thread_create(thread_academy_download, (THREAD_PARAMS)academy_download_callback_func, &academy_download_thread);
    }
}

void academy_download_shutdown(void)
{
    if(academy_download_started)
    {
        academy_download_started = FALSE;
        academy_download_resume ();
        vp_os_thread_join(academy_download_thread);
    }
}

DEFINE_THREAD_ROUTINE(academy_download, data)
{
    char *directoryList = NULL;
    bool_t shouldGoInPause = FALSE;
    char *fileList = NULL;
    char process_dirname[ACADEMY_MAX_FILENAME];
    _ftp_t *academy_ftp = NULL;
    _ftp_status academy_status;
    struct stat statbuf;
    char *ptr = NULL;
    academy_download_t academy;

    academy.new_media_callback = (academy_download_new_media)data;    
    academy.callback = academy_download_private_callback;

    PRINT("Start thread %s \n", __FUNCTION__);
    do
    {
        academy_download_resetState(&academy);
        shouldGoInPause = FALSE;
        if (TRUE == academy_download_in_pause)
        {
            vp_os_mutex_lock (&academy_download_mutex);
            PRINT ("Academy download stage paused\n");
            vp_os_cond_wait (&academy_download_cond);
            vp_os_mutex_unlock (&academy_download_mutex);
        }
        academy_download_pause();
        PRINT ("Academy download stage resumed\n");
        academy_download_nextState(&academy);

        while( academy_download_started && !shouldGoInPause)
        {
            academy_status = FTP_FAIL;

            switch(academy.state.state)
			{				
            case ACADEMY_STATE_NONE:
                academy_status = FTP_SUCCESS;
                break;

            case ACADEMY_STATE_CONNECTION:
                if(academy_ftp == NULL)
                    academy_ftp = ftpConnect(&wifi_ardrone_ip[0], ACADEMY_PORT, "anonymous", "", &academy_status);

                if(academy_ftp != NULL)
                    academy_status = FTP_SUCCESS;
                break;

            case ACADEMY_STATE_PREPARE_PROCESS:
                academy_status = ftpCd(academy_ftp, "/boxes");
                if(FTP_SUCCEDED(academy_status))
                {
                    char *next_dir = NULL;
                    PA_DEBUG("Enter to boxes directory\n");
                    academy_status = ftpList(academy_ftp, &directoryList, NULL);

                    // If directory is empty, we consider that it is an error
                    if(academy_status == FTP_SAMESIZE)
                        academy_status = FTP_FAIL;

                    if(FTP_SUCCEDED(academy_status))
                    {
                        char local_dir[ACADEMY_MAX_FILENAME];
                        // Search if it stay ftpremove_* directories to continue removing
                        PA_DEBUG("Get directory list :\n%s", directoryList);
                        while(FTP_SUCCEDED(academy_status) && (ptr = academy_get_next_item_with_prefix(directoryList, &next_dir, "ftpremove_downloading_", TRUE)))
                        {
                            char remove_dirname[ACADEMY_MAX_FILENAME];
                            strncpy(remove_dirname, ptr, ACADEMY_MAX_FILENAME);
                            academy_status = academy_remove_ftp_directory(academy_ftp, remove_dirname);
                        }

                        academy_status = FTP_FAIL;
                        next_dir = NULL;
                        // Search if it stay downloading_flight_* file to continue downloading.
                        ptr = academy_get_next_item_with_prefix(directoryList, &next_dir, "downloading_flight_", TRUE);
                        // Check if transfer was in progress
                        if(ptr != NULL)
                        {
                            // Transfer was in progress
                            PA_DEBUG("Transfer was in progress\n");
                            strncpy(process_dirname, ptr, ACADEMY_MAX_FILENAME);
                            snprintf(local_dir, ACADEMY_MAX_FILENAME, "%s/%s", flight_dir, process_dirname);
                            if((stat(local_dir, &statbuf) == 0) && S_ISDIR(statbuf.st_mode))
                            {
                                // Transfer was in progress by current device
                                PA_DEBUG("Transfer was in progress by current device\n");
                                academy_status = FTP_SUCCESS;
                            }
                            else
                            {
                                // Transfer was in progress by an other device => Create local directory
                                PA_DEBUG("Transfer was in progress by an other device\n");
                                PRINT("Creating local directory %s\n", local_dir);
                                if(mkdir(local_dir, 0777) >= 0)
                                {
                                    PA_DEBUG("Create local directory %s\n", local_dir);
                                    academy_status = FTP_SUCCESS;
                                }
                            }
                        }
                        else
                        {
                            PA_DEBUG(" Transfer was not in progress\n");
                            // Transfer was not in progress => Get oldest flight, rename remote directory and create local directory
                            next_dir = NULL;
                            ptr = academy_get_next_item_with_prefix(directoryList, &next_dir, "flight_", TRUE);
                            if(ptr != NULL)
                            {
                                // Prepare to process new transfer
                                PA_DEBUG("Prepare to process new transfer\n");
                                snprintf(process_dirname, ACADEMY_MAX_FILENAME, "downloading_%s", ptr);
                                academy_status = ftpRename(academy_ftp, ptr, process_dirname);
                                if(FTP_SUCCEDED(academy_status))
                                {
                                    PA_DEBUG("Rename %s to %s directory\n", ptr, process_dirname);
                                    snprintf(local_dir, ACADEMY_MAX_FILENAME, "%s/%s", flight_dir, process_dirname);
                                    if((stat(local_dir, &statbuf) != 0) && (mkdir(local_dir, 0777) >= 0))
                                        PA_DEBUG("Create local directory %s if not exist\n", local_dir);
                                    academy_status = FTP_SUCCESS;
                                }
                            }
                            else
                            {
                                shouldGoInPause = TRUE;
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
                academy_status = ftpCd(academy_ftp, process_dirname);
                if(FTP_SUCCEDED(academy_status))
                {
                    PA_DEBUG("Enter to %s directory\n", process_dirname);
                    academy_status = ftpList(academy_ftp, &fileList, NULL);
                    if(FTP_SUCCEDED(academy_status))
                    {
                        char *next_file = NULL;
                        char media_dirname[ACADEMY_MAX_FILENAME];
                        char local_filename[ACADEMY_MAX_FILENAME];
                        bool_t userbox_ready = FALSE;

                        PA_DEBUG("Get file list :\n%s", fileList);
                        ptr = academy_get_next_item_with_prefix(fileList, &next_file, "userbox_", FALSE);
                        if(ptr != NULL)
                        {
                            snprintf(local_filename, ACADEMY_MAX_FILENAME, "%s/%s/%s", flight_dir, process_dirname, ptr);
                            PA_DEBUG("Get %s from drone to local directory...", local_filename);
                            academy_status = ftpGet (academy_ftp, ptr, local_filename, 1, NULL);
                            if(FTP_SUCCEDED(academy_status))
                            {
                                userbox_ready = TRUE;
                                PA_DEBUG("OK\n");
                            }
                            else
                                PA_DEBUG("ERROR\n");
                        }

                        snprintf(media_dirname, ACADEMY_MAX_FILENAME, "%s/media_%s", flight_dir, process_dirname + (strlen("downloading_flight_")));
                        //PRINT("medias directory : %s\n", media_dirname);
                        next_file = NULL;
                        while(FTP_SUCCEDED(academy_status) && (ptr = academy_get_next_item_with_prefix(fileList, &next_file, "picture_", FALSE)))
                        {
                            if((stat(media_dirname, &statbuf) != 0) && (mkdir(media_dirname, 0777) >= 0))
                                PA_DEBUG("Create local media directory %s if not exist\n", media_dirname);

                            sprintf(local_filename, "%s/%s", media_dirname, ptr);
                            PA_DEBUG("Get %s from drone to local directory...", local_filename);
                            academy_status = ftpGet (academy_ftp, ptr, local_filename, 1, NULL);
                            if(FTP_SUCCEDED(academy_status))
                            {
                                    if (NULL != academy.new_media_callback)
                                    {
                                academy.new_media_callback((const char *)local_filename, TRUE);
                                    }
                                PA_DEBUG("OK\n");
                            }
                            else
                            {
                                PA_DEBUG("ERROR\n");
                            }
                        }
                        if (NULL != academy.new_media_callback)
                        {
                        academy.new_media_callback((const char *)NULL, FALSE);
                        }

                        if(FTP_SUCCEDED(academy_status) && !userbox_ready)
                        {
                            char local_dirname[ACADEMY_MAX_FILENAME];
                            snprintf(local_dirname, ACADEMY_MAX_FILENAME, "%s/%s", flight_dir, process_dirname);
                            remove(local_dirname);
                        }
                    }

                    if(FTP_SUCCEDED(academy_status))
                        ftpCd(academy_ftp, "..");
                }
                break;

            case ACADEMY_STATE_FINISH_PROCESS:
            {
                char remove_dirname[ACADEMY_MAX_FILENAME];
                char src[ACADEMY_MAX_FILENAME];
                char dest[ACADEMY_MAX_FILENAME];

                // Rename local directory from downloading_flight_* to flight_*
                snprintf(src, ACADEMY_MAX_FILENAME, "%s/%s", flight_dir, process_dirname);
                snprintf(dest, ACADEMY_MAX_FILENAME, "%s/%s", flight_dir, process_dirname + strlen("downloading_"));
                rename(src, dest);

                // Rename ftp remote directory from downloading_flight_* to ftpremove_downloading_flight_*
                snprintf(remove_dirname, ACADEMY_MAX_FILENAME, "ftpremove_%s", process_dirname);
                academy_status = ftpRename(academy_ftp, process_dirname, remove_dirname);

                // Delete ftpremove_downloading_flight_*
                if(FTP_SUCCEDED(academy_status))
                    academy_status = academy_remove_ftp_directory(academy_ftp, remove_dirname);
            }
            break;

            case ACADEMY_STATE_DISCONNECTION:
                if(academy_ftp != NULL)
                    ftpClose(&academy_ftp);
                academy_check_memory();
                academy_status = FTP_SUCCESS;
                break;

            default:
                // Nothing to do
                PA_DEBUG("THIS CASE SHOULD NOT HAPPEN\n");
                break;
            }

            if(FTP_SUCCEDED(academy_status))
            {
                PA_DEBUG("continue state from %d to %d\n", academy.state.state, (academy.state.state + 1) % ACADEMY_STATE_MAX);
                academy_download_stateOK(&academy);
                academy_download_nextState(&academy);
            }
            else
            {
                PA_DEBUG("stop\n");
                shouldGoInPause = TRUE;
                academy_download_stateERROR(&academy);

                if(fileList != NULL)
                {
                    vp_os_free(fileList);
                    fileList = NULL;
                }

                if(academy_ftp)
                    ftpClose(&academy_ftp);
            }
        }

        if(fileList != NULL)
        {
            vp_os_free(fileList);
            fileList = NULL;
        }

        if(academy_ftp)
            ftpClose(&academy_ftp);

    }
    while(academy_download_started && !ardrone_tool_exit());

    vp_os_cond_destroy(&academy_download_cond);
    vp_os_mutex_destroy(&academy_download_mutex);
    THREAD_RETURN(C_OK);
}

void academy_download_pause (void)
{
    vp_os_mutex_lock (&academy_download_mutex);
    academy_download_in_pause = TRUE;
    vp_os_mutex_unlock (&academy_download_mutex);
}

void academy_download_resume (void)
{
    vp_os_mutex_lock (&academy_download_mutex);
    academy_download_in_pause = FALSE;
    vp_os_cond_signal (&academy_download_cond);
    vp_os_mutex_unlock (&academy_download_mutex);
}
