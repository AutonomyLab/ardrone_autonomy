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
USE_LINUX            = no
endif
ifndef PROJECT
# set default to ardrone2 for video TCP com.
PROJECT              = ardrone2
endif
MAJOR_VERSION        = 0
MINOR_VERSION        = 0
MODIF_VERSION        = 0

#########################################################
# ARDroneTool options definitions
#########################################################
USE_ARDRONE_TOOL=yes
USE_CHECK_WIFI_CONFIG=no

################## Wifi Options ##################
# Name of the network you want to join or create
WIFI_NETWORK_NAME    = "ardronenetwork"
WIFI_BROADCAST       = "192.168.1.255"

################## Video Options ##################
# Tells if we want to record video on pc side
RECORD_ENCODED_VIDEO     = yes
RECORD_RAW_VIDEO         = no
RECORD_FFMPEG_VIDEO      = no

# Tells if we want to add vision data to video stream (in raw mode)
# Vision data are saved into file only if we define RECORD_RAW_VIDEO too
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

