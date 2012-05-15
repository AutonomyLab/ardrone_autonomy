#ifndef _ARDRONE_GEN_APPID_H_
#define _ARDRONE_GEN_APPID_H_

#include <VP_Os/vp_os_types.h>

void ardrone_gen_appid (const char *appName, const char *sdkVersion, char appId [9], char *appDesc, int descLen);
void ardrone_gen_usrid (const char *usrName, char usrId [9], char *usrDesc, int descLen);
void ardrone_gen_sessionid (char sessId [9], char *sessDesc, int descLen);

#endif /* _ARDRONE_GEN_APPID_H_ */
