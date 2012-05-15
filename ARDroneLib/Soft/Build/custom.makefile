#########################################################
# Common definitions (CUSTOM)
#########################################################
ifndef IPHONE_MODE
IPHONE_MODE	     = no
endif
ifndef MINGW32_MODE
MINGW32_MODE         = no
endif
ifndef USE_NDS
USE_NDS              = no
endif
ifndef USE_ANDROID
USE_ANDROID          = no
endif
ifndef USE_LINUX
USE_LINUX            = yes
endif
MAJOR_VERSION        = 0
MINOR_VERSION        = 0
MODIF_VERSION        = 0

#########################################################
# ARDroneTool options definitions
#########################################################
USE_ARDRONE_MAINLOOP=yes
USE_CHECK_WIFI_CONFIG=no

ifeq ($(IPHONE_MODE),yes)
USE_ARDRONE_MAINLOOP=no
IPHONE_SDK_PATH=/Developer/Platforms/iPhoneOS.platform/Developer/SDKs/iPhoneOS3.0
# iphoneos or iphonesimulator
ARDRONE_TARGET_OS=iphoneos
ARDRONE_TARGET_ARCH=armv6
else
   ifeq ($(USE_ANDROID),yes)
      USE_ARDRONE_MAINLOOP=no
   endif
ARDRONE_TARGET_OS=Linux
endif

################## Wifi Options ##################
# Name of the network you want to join or create
WIFI_NETWORK_NAME    = "ardronenetwork"
WIFI_BROADCAST       = "192.168.1.255"

################## Video Options ##################
# Tells if we want to record video on pc side
RECORD_VIDEO         = yes
FFMPEG_RECORDING_SUPPORT = yes
# Tells if we want to add vision data to video stream (in raw mode)
# Vision data are saved into file only if we define RECORD_VIDEO too
RECORD_VISION_DATA   = no
# If the yuv mode is choosen then video is displayed & recorded in color
# Otherwise only luminances are displayed & recorded
VIDEO_YUV            = yes

#########################################################
# Embedded definitions (CUSTOM)
#########################################################
WIFI_ARDRONE_IP      = "192.168.1.1"

#########################################################
# Linux definitions (CUSTOM)
#########################################################
WIFI_MOBILE_IP       = "192.168.1.2"

