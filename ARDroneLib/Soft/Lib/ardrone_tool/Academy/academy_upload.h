/**
 *  academy_upload.h
 *  
 *  Created by Frederic D'HAEYER on 8/04/11.
 *  Copyright 2011 Parrot SA. All rights reserved.
 *
 */
#ifndef _ACADEMY_UPLOAD_H_
#define _ACADEMY_UPLOAD_H_

#include <ardrone_tool/Academy/academy.h>

void academy_setState(academy_state_t *state, ACADEMY_STATE academy_state);
void academy_resetState(academy_state_t *state);
void academy_nextState(academy_state_t *state);
void academy_stateOK(academy_state_t *state);
void academy_stateERROR(academy_state_t *state);
int academy_remove(const char* fpath, const struct stat *sb, int typeflag);
char *academy_get_next_item_with_prefix(const char *list, char **next_item, const char *prefix, bool_t isDirectory);
_ftp_status academy_remove_ftp_directory(_ftp_t *ftp, const char *directory_name);

void academy_upload_init(void);
void academy_upload_shutdown(void);

#endif // _ACADEMY_UPLOAD_H_
