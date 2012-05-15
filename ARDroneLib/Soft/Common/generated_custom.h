#ifndef _GENERATED_CUSTOM_CONFIGURATION_H_
#define _GENERATED_CUSTOM_CONFIGURATION_H_

#if  defined(BR2_PACKAGE_BCM4318_AP)
#  define AP
#else
#  define STA
#endif
#define CURRENT_NUM_VERSION_SOFT "0.0.0"
#define CURRENT_BUILD_DATE "2011-08-04 17:17"

#define USE_VIDEO_YUV

#define WIFI_NETWORK_NAME "ardronenetwork"
#define WIFI_BROADCAST "192.168.1.255"
#define WIFI_ARDRONE_IP "192.168.1.1"

#if defined(__linux__) || defined(USE_MINGW32)
# define WIFI_MOBILE_IP "192.168.1.2"
# define WIRED_ITFNAME ""
#endif // ! __linux__


#endif // ! _GENERATED_CUSTOM_CONFIGURATION_H_
