
###########################################################################################
#
# Common to sdk.makefile and app.makefile
# ----------------------------------------------------------------------------------
# Author : aurelien.morelle@parrot.com
# Date   : 2007/05/16
#
###########################################################################################

# To be reordered (CommonSoft)
########################
XCC=$(GENERIC_COMMAND_PREFIX)gcc
XAR=$(GENERIC_COMMAND_PREFIX)ar

ifdef MYKONOS
GENERIC_INCLUDES+=					\
	-I$(ALL_TARGETS)/CommonSoft/include		\
	-I$(ALL_TARGETS)/CommonSoft/include/xmlparser	\
	-I$(ALL_TARGETS)/CommonSoft/include/crypto
endif

# Used for include paths
########################
ifdef MYKONOS
SDK_PATH:=$(ALL_SOURCES)/ardrone_api/$(SDK_VERSION)/ARDrone_API/ARDroneLib
SDK_SOURCE_DIR=$(SDK_PATH)/VP_SDK
VLIB_SOURCE_DIR=$(SDK_PATH)/VLIB

COMMONSOFT=$(ALL_SOURCES)/commonsoft/$(COMMONSOFT_VERSION)/CommonSoft
PARROTOS_CORE_SOURCE_DIR=$(COMMONSOFT)/ParrotOS/core/src
PARROTOS_CORE_INCLUDE_DIR=$(COMMONSOFT)/ParrotOS/core/include
PARROTOS_UTILS_SOURCE_DIR=$(COMMONSOFT)/ParrotOS/utils/src
PARROTOS_UTILS_INCLUDE_DIR=$(COMMONSOFT)/ParrotOS/utils/include
PARROTOS_DRIVERS_SOURCE_DIR=$(COMMONSOFT)/ParrotOS/drivers/src
PARROTOS_DRIVERS_INCLUDE_DIR=$(COMMONSOFT)/ParrotOS/drivers/include
PARROTOS_DEVS_SOURCE_DIR=$(COMMONSOFT)/ParrotOS/devs/
PARROTOS_DEVS_INCLUDE_DIR=$(COMMONSOFT)/ParrotOS/devs/
PARROTOS_CODEC_SOURCE_DIR=$(COMMONSOFT)/ParrotOS/codec
PARROTOS_CODEC_INCLUDE_DIR=$(COMMONSOFT)/ParrotOS/codec
LIBPLF_SOURCE_DIR=$(COMMONSOFT)/libplf/src
LIBPLF_INCLUDE_DIR=$(COMMONSOFT)/libplf/include
else
SDK_SOURCE_DIR=../
VLIB_SOURCE_DIR=../../VLIB
endif

# Sdk relative paths
########################
API_PATH      = VP_Api
ATCODEC_PATH  = ATcodec
COM_PATH      = VP_Com
OS_PATH       = VP_Os
EXAMPLES_PATH = Examples
STAGES_PATH   = VP_Stages

ifeq ($(USE_NDS),yes)
    OS=nds
else
  ifeq ($(USE_MINGW32),yes)
    OS=win32
  else
    ifeq ($(USE_ELINUX),yes)
      OS=elinux
    else
      OS=linux
    endif
  endif
endif

# Include paths
########################
ifeq ($(USE_ANDROID),yes)
GENERIC_INCLUDES+=   \
     -I$(NDK_PATH)/build/platforms/$(NDK_PLATFORM_VERSION)/arch-arm/usr/include
endif

ifeq ($(USE_IPHONE),yes)
  GENERIC_INCLUDES+=					\
	-isysroot $(IPHONE_SDK_PATH) \
	-I$(IPHONE_SDK_PATH)/usr/include/gcc/darwin/4.2
endif

ifeq ($(USE_ELINUX),yes)
  GENERIC_INCLUDES+=					\
	-I$(ALL_SOURCES)/linux/$(ELINUX_VERSION)/Linux/lucie/build/staging-dir_$(BOARD_CPU)_$(BOARD_NAME)/usr/include	\
	-I$(ALL_SOURCES)/linux/$(ELINUX_VERSION)/Linux/lucie/build/staging-dir_$(BOARD_CPU)_$(BOARD_NAME)/usr/include/linux	\
	-I$(ALL_SOURCES)/linux/$(ELINUX_VERSION)/Linux/kernel/linux/include	\
	-I$(ALL_SOURCES)/linux/$(ELINUX_VERSION)/Linux/kernel/linux/drivers	\
	-I$(ALL_SOURCES)/linux/$(ELINUX_VERSION)/Linux/packages/drivers
	
  ifeq ($(USE_WIFI),yes)
  GENERIC_INCLUDES+=					\
	-Dlinux -I$(ALL_SOURCES)/linux/$(ELINUX_VERSION)/Linux/packages/drivers/bcm4318/src_4_170_55/include	\
	-I$(ALL_SOURCES)/linux/$(ELINUX_VERSION)/Linux/lucie/build/staging-dir_$(BOARD_CPU)_$(BOARD_NAME)/include
  endif
endif
ifeq ($(USE_NDS),yes)
  GENERIC_INCLUDES+=					\
	-I$(DEVKITARM)/include				\
	-I$(DEVKITPRO)/libnds/include
endif

ifeq ($(USE_VLIB),yes)
  GENERIC_INCLUDES+=					\
	-I$(VLIB_SOURCE_DIR)/..				\
	-I$(VLIB_SOURCE_DIR)/P263
ifeq ($(TARGET_CPU_ARM),1)
  GENERIC_INCLUDES+=			\
	-I$(VLIB_SOURCE_DIR)/Platform/arm9 \
	-I$(VLIB_SOURCE_DIR)/Platform/arm11
else
ifeq ($(TARGET_CPU_X86),1)
  GENERIC_INCLUDES+=			\
	-I$(VLIB_SOURCE_DIR)/Platform/x86
endif
endif
endif
ifeq ($(NO_COM),no)
  GENERIC_INCLUDES+=					\
	-I$(COM_TARGET_DIR)/include
endif
ifeq ($(USE_SDK),yes)
  GENERIC_INCLUDES+=					\
	-I$(SDK_SOURCE_DIR)				\
	-I$(SDK_SOURCE_DIR)/$(COM_PATH)

ifeq ($(USE_LIBPLF),yes)
  GENERIC_INCLUDES+=					\
	-I$(LIBPLF_INCLUDE_DIR)
endif

ifeq ($(USE_PARROTOS_CORE),yes)
  GENERIC_INCLUDES+=					\
	-I$(PARROTOS_CORE_INCLUDE_DIR)		\
	-I$(PARROTOS_CORE_INCLUDE_DIR)/generic		\
	-I$(PARROTOS_CORE_INCLUDE_DIR)/linux		\
	-I$(PARROTOS_CORE_INCLUDE_DIR)/posix		\
	-I$(PARROTOS_UTILS_INCLUDE_DIR)		\
  -I$(PARROTOS_DRIVERS_INCLUDE_DIR) \
  -I$(PARROTOS_DRIVERS_INCLUDE_DIR)/linux \
  -I$(PARROTOS_DEVS_INCLUDE_DIR) \
  -I$(PARROTOS_CODEC_INCLUDE_DIR) \
	-I$(SDK_SOURCE_DIR)/$(OS_PATH)/parrotos
else
  GENERIC_INCLUDES+=					\
	-I$(SDK_SOURCE_DIR)/$(OS_PATH)/$(OS)
endif
ifeq ($(NO_COM),no)
  GENERIC_INCLUDES+=					\
	-I$(SDK_SOURCE_DIR)/$(COM_PATH)/$(OS)
endif
endif

  GENERIC_INCLUDES+=					\
	-I$(PFFMPEG_SOURCE_DIR)/specific

ifeq ($(USE_JPEG_P6),yes)
     GENERIC_INCLUDES+=                  			\
	-I$(ALL_SOURCES)/linux/$(ELINUX_VERSION)/Linux/lucie/build/build_$(BOARD_CPU)_$(BOARD_NAME)/jpeg-6b
endif
ifeq ($(USE_BONJOUR),yes)
  GENERIC_INCLUDES+=-I$(BONJOUR_SOURCE_DIR)
endif

# Libraries
########################

GENERIC_LIB_PATHS+=					\
	-L$(OS_TARGET_DIR)/install/lib			\
	-L$(SDK_TARGET_DIR)

INITIAL_GENERIC_LIBS:=$(GENERIC_LIBS)

GENERIC_LIBS+=						\
	-lsdk

GENERIC_BINARIES_LIBS_DEPS=$(SDK_TARGET_DIR)/libsdk.a

ifeq ($(USE_LIBPLF),yes)
  GENERIC_LIB_PATHS+=					\
	-L$(LIBPLF_TARGET_DIR)

  GENERIC_LIBS+=					\
	-lplf \
	-lz

endif

ifeq ($(USE_PARROTOS_CORE),yes)
  GENERIC_LIB_PATHS+=					\
	-L$(PARROTOS_CORE_TARGET_DIR) \
	-L$(PARROTOS_UTILS_TARGET_DIR) \
	-L$(PARROTOS_DRIVERS_TARGET_DIR) \
	-L$(PARROTOS_CODEC_TARGET_DIR) \
	-L$(PARROTOS_DEVS_TARGET_DIR)

  GENERIC_LIBS+=					\
	-lparrotOS_core					\
	-lparrotOS_utils					\
	-lparrotOS_devs 			\
	-lparrotOS_codec 			\
	-lparrotOS_drivers 			\
	-lrt
  GENERIC_BINARIES_LIBS_DEPS+=		\
		$(PARROTOS_CORE_TARGET_DIR)/libparrotOS_core.a \
		$(PARROTOS_UTILS_TARGET_DIR)/libparrotOS_utils.a \
		$(PARROTOS_DRIVERS_TARGET_DIR)/libparrotOS_drivers.a \
		$(PARROTOS_CODEC_TARGET_DIR)/libparrotOS_codec.a \
		$(PARROTOS_DEVS_TARGET_DIR)/libparrotOS_devs.a
endif

ifeq ($(USE_VLIB),yes)
   ifeq ($(USE_ELINUX),yes)

    GENERIC_LIB_PATHS+=		\
	-L$(ALL_SOURCES)/linux/$(ELINUX_VERSION)/Linux/lucie/build/staging-dir_$(BOARD_CPU)_$(BOARD_NAME)/lib	\
 	-L$(ALL_SOURCES)/linux/$(ELINUX_VERSION)/Linux/lucie/build/staging-dir_$(BOARD_CPU)_$(BOARD_NAME)/usr/lib 
    
    GENERIC_LIBS+=					\
    -luiomap -ldma_alloc
   endif
   GENERIC_LIB_PATHS+= 					\
	-L$(CODEC_TARGET_DIR)
   GENERIC_LIBS+=					\
	-lvlib
   GENERIC_BINARIES_LIBS_DEPS+=				\
	$(CODEC_TARGET_DIR)/libvlib.a
endif

ifeq ($(USE_NDS),yes)
    GENERIC_LIB_PATHS+= 				\
	-L$(DEVKITPRO)/libnds/lib
    ifeq ($(NDS_CPU),ARM9)
      GENERIC_LIBS+=					\
	-ldswifi9					\
	-lnds9
    endif
    ifeq ($(NDS_CPU),ARM7)
      GENERIC_LIBS+=					\
	-ldswifi7					\
	-lnds7
    endif
else
    ifeq ($(USE_MINGW32),yes)
      GENERIC_LIBS+=					\
	-lws2_32
    else
      GENERIC_LIBS+=					\
	-lpthread
      ifneq ($(USE_ELINUX),yes)
         ifeq ($(USE_IWLIB),yes)
            GENERIC_LIBS+=					\
	            -liw
         endif
        GENERIC_LIBS+=					\
	       -lSDL						\
	       -lGL						\
	       -lGLU
      endif
    endif
endif

ifeq ($(USE_ELINUX),yes)
  ifeq ($(USE_WIFI),yes)
    GENERIC_LIB_PATHS+= 				\
	-L$(ALL_SOURCES)/linux/$(ELINUX_VERSION)/Linux/lucie/build/staging-dir_$(BOARD_CPU)_$(BOARD_NAME)/lib

    ifeq ($(USE_IWLIB), yes)
      GENERIC_LIBS+=-liw
    endif
  endif
  ifeq ($(USE_JPEG_P6),yes)
    
    GENERIC_LIB_PATHS+= 				\
	-L$(ALL_SOURCES)/linux/$(ELINUX_VERSION)/Linux/lucie/build/build_$(BOARD_CPU)_$(BOARD_NAME)/jpeg-6b  \
	-L$(ALL_SOURCES)/linux/$(ELINUX_VERSION)/Linux/lucie/build/staging-dir_$(BOARD_CPU)_$(BOARD_NAME)/usr/lib

    GENERIC_LIBS+= \
    -ljpeg \
    -ldma_alloc \
    -luiomap
  endif
endif

ifeq ($(USE_ARDRONELIB),yes)
GENERIC_LIB_PATHS+= 					\
	-L$(ARDRONELIB_TARGET_DIR)
endif

ifeq ($(USE_ARDRONE_VISION),yes)
GENERIC_LIB_PATHS+= 					\
	-L$(ARDRONE_VISION_TARGET_DIR)
endif

ifeq ($(USE_ARDRONE_POLARIS),yes)
GENERIC_LIB_PATHS+= 					\
	-L$(ARDRONE_POLARIS_TARGET_DIR)
endif

ifeq ($(USE_ARDRONE_VICON),yes)
GENERIC_LIB_PATHS+= 					\
	-L$(ARDRONE_VICON_TARGET_DIR)
endif

ifeq ($(USE_ARDRONE_TEST_BENCHS),yes)
GENERIC_LIB_PATHS+= 					\
	-L$(ARDRONE_TEST_BENCHS_TARGET_DIR)
endif

ifeq ($(USE_ARDRONE_CALIBRATION),yes)
GENERIC_LIB_PATHS+= 					\
	-L$(ARDRONE_CALIBRATION_TARGET_DIR)
endif

GENERIC_LIBS+=$(INITIAL_GENERIC_LIBS)


