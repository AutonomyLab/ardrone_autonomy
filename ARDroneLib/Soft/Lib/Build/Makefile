NO_COLOR=\033[0m
OK_COLOR=\033[32;01m

ifdef ARDRONE_CUSTOM_CONFIG 
  ifdef ARDRONE_BUILD_CONFIG
    include $(ARDRONE_CUSTOM_CONFIG)
    include $(ARDRONE_BUILD_CONFIG)
  else
    include ../../Build/custom.makefile
    include ../../Build/config.makefile
  endif
else
  include ../../Build/custom.makefile
  include ../../Build/config.makefile
endif

LIB_ID=ardrone
ifeq ($(USE_ARDRONE_TOOL),no)
LIB_ID:=$(LIB_ID)_notool
endif
GENERIC_TARGET_LIBRARY:=lib$(TARGET)$(LIB_ID).a

GENERIC_CFLAGS+=-DAT_MESSAGES_HEADER="\"$(COMMON_DIR)/at_msgs.h\""
GENERIC_CFLAGS+=-DUSE_NEW_ATCODEC

ifeq ("$(USE_CHECK_WIFI_CONFIG)","yes")
   GENERIC_CFLAGS+=-DCHECK_WIFI_CONFIG
endif

SDK_FLAGS+="USE_APP=no"
SDK_FLAGS+="USE_LIB=yes"
SDK_FLAGS+="LIB_ID=ardrone_lib"

MATHS_DIR=Maths
CONTROL_DIR=Control
ARDRONE_TOOL_DIR=ardrone_tool
UTILS_DIR=utils
INIPARSER_DIR=iniparser3.0b/src
LIBCALIBRATION_DIR=libCalibration

GENERIC_LIBRARY_SOURCE_FILES+=                          \
  $(INIPARSER_DIR)/iniparser.c                          \
  $(INIPARSER_DIR)/dictionary.c                         \
  $(MATHS_DIR)/filter.c                                 \
  $(MATHS_DIR)/maths.c                                  \
  $(MATHS_DIR)/matrices.c                               \
  $(MATHS_DIR)/matrix3d.c                               \
  $(MATHS_DIR)/quaternions.c                            \
  $(MATHS_DIR)/time.c                                   \
  $(MATHS_DIR)/vision_math.c                            \
  $(ARDRONE_TOOL_DIR)/Navdata/navdata.c                 \
  $(ARDRONE_TOOL_DIR)/config_keys.c                     \
  $(ARDRONE_TOOL_DIR)/ardrone_version.c                 \
  $(UTILS_DIR)/ardrone_time.c 							\
  $(UTILS_DIR)/ardrone_date.c 							\
  $(UTILS_DIR)/ardrone_crc_32.c                         \
  $(UTILS_DIR)/ardrone_gen_ids.c                        \
  $(UTILS_DIR)/ardrone_ftp.c							\
  $(ARDRONE_TOOL_DIR)/Video/video_encapsulation.c 

# The following files are needed to record video on the drone's flash memory
GENERIC_LIBRARY_SOURCE_FILES+=                          \
  $(UTILS_DIR)/ardrone_video_atoms.c                    \
  $(UTILS_DIR)/ardrone_video_encapsuler.c               \
  $(ARDRONE_TOOL_DIR)/Video/video_stage_encoded_recorder.c

ifneq ($(ELINUX_TARGET),yes)
#Controller only files
GENERIC_LIBRARY_SOURCE_FILES+= \
  $(ARDRONE_TOOL_DIR)/Video/video_stage_tcp.c           \
  $(ARDRONE_TOOL_DIR)/Video/video_stage_merge_slices.c  \
  $(ARDRONE_TOOL_DIR)/Video/video_stage_latency_estimation.c \
  $(UTILS_DIR)/ardrone_ftp.c     \
  $(ARDRONE_TOOL_DIR)/Video/video_stage_ffmpeg_recorder.c  \
  $(ARDRONE_TOOL_DIR)/Video/video_stage_ffmpeg_decoder.c   \
  $(ARDRONE_TOOL_DIR)/Video/video_stage_ittiam_decoder.c   \
  $(ARDRONE_TOOL_DIR)/Video/video_stage_decoder.c          \
  $(ARDRONE_TOOL_DIR)/Video/video_recorder_pipeline.c      \
  $(ARDRONE_TOOL_DIR)/Video/video_stage.c
  ifeq ($(USE_ANDROID),yes)
  GENERIC_LIBRARY_SOURCE_FILES+= \
    $(UTILS_DIR)/AR_Ftw.c
  endif
endif

ifneq ($(CONTROL_DLL),yes)
  ifeq ($(USE_ARDRONE_TOOL),yes)
    ifeq ($(PC_USE_POLARIS),yes)
      GENERIC_CFLAGS+=-DPC_USE_POLARIS
    endif

    GENERIC_LIBRARY_SOURCE_FILES+=                      \
      $(ARDRONE_TOOL_DIR)/AT/ardrone_at_mutex.c			       

    GENERIC_LIBRARY_SOURCE_FILES+=                              	\
	  $(ARDRONE_TOOL_DIR)/Academy/academy.c							\
	  $(ARDRONE_TOOL_DIR)/Academy/academy_download.c				\
	  $(ARDRONE_TOOL_DIR)/Academy/academy_upload.c					\
	  $(ARDRONE_TOOL_DIR)/Academy/academy_stage_recorder.c			\
      $(ARDRONE_TOOL_DIR)/Control/ardrone_control_configuration.c  	\
      $(ARDRONE_TOOL_DIR)/Control/ardrone_control_ack.c         	\
      $(ARDRONE_TOOL_DIR)/Navdata/ardrone_navdata_file.c        	\
      $(ARDRONE_TOOL_DIR)/Navdata/ardrone_general_navdata.c    	    \
	  $(ARDRONE_TOOL_DIR)/Navdata/ardrone_academy_navdata.c			\
      $(ARDRONE_TOOL_DIR)/UI/ardrone_input.c                       	\
      $(ARDRONE_TOOL_DIR)/ardrone_api.c                            	\
      $(ARDRONE_TOOL_DIR)/ardrone_tool_configuration.c             	\
      $(ARDRONE_TOOL_DIR)/ardrone_tool.c                           	\
      $(ARDRONE_TOOL_DIR)/Com/config_wifi.c

     ifneq ($(USE_MINGW32),yes)
       GENERIC_LIBRARY_SOURCE_FILES+=                             \
         $(ARDRONE_TOOL_DIR)/Video/video_com_stage.c              \
         $(ARDRONE_TOOL_DIR)/Video/video_navdata_handler.c        \
         $(ARDRONE_TOOL_DIR)/Control/ardrone_control.c            \
         $(ARDRONE_TOOL_DIR)/Control/ardrone_navdata_control.c    \
         $(ARDRONE_TOOL_DIR)/Navdata/ardrone_navdata_client.c


       ifeq ($(USE_IPHONE),no)
         GENERIC_LIBRARY_SOURCE_FILES+=				  \
           $(ARDRONE_TOOL_DIR)/Com/config_serial.c
       endif

       ifeq ($(USE_ARDRONE_VICON),yes)
         GENERIC_CFLAGS+=-DUSE_ARDRONE_VICON
       endif
     endif # USE_MINGW32
  endif # USE_ARDRONE_TOOL

  ifneq ($(USE_MINGW32),yes)
    GENERIC_LIBRARY_SOURCE_FILES+= \
      $(ARDRONE_TOOL_DIR)/Video/video_stage_recorder.c
  endif

  ifeq ($(USE_ELINUX),yes)
    HARDWARE_ARDRONE:=$(SRC_PATH)/Soft/Toy/Sources/$(CARD_HARDWARE_DIR)
    ifeq ($(USE_MINGW32),yes)
      HARDWARE_ARDRONE_CALIBRATION:=c:/$(CARD_HARDWARE_DIR)
    else
      HARDWARE_ARDRONE_CALIBRATION:=$(HARDWARE_ARDRONE)
    endif

    GENERIC_CFLAGS+=-DHARDWARE_ARDRONE=\"$(HARDWARE_ARDRONE)\" \
      -DHARDWARE_ARDRONE_CALIBRATION=\"$(HARDWARE_ARDRONE_CALIBRATION)\"
  endif # USE_ELINUX

endif # neq CONTROL_DLL

BASE_RELATIVE_PATH_FROM_SDK=../../

GENERIC_INCLUDES+=                               \
  -I$(BASE_RELATIVE_PATH_FROM_SDK)/Soft/Common   \
  -I$(BASE_RELATIVE_PATH_FROM_SDK)               \
  -I$(BASE_RELATIVE_PATH_FROM_SDK)/Soft/Lib

export GENERIC_INCLUDES
export GENERIC_LIBRARY_SOURCE_DIR=$(BASE_RELATIVE_PATH_FROM_SDK)/Soft/Lib
export GENERIC_BINARIES_SOURCE_DIR=$(BASE_RELATIVE_PATH_FROM_SDK)/Soft/Lib

export GENERIC_CFLAGS
export GENERIC_LIBS
export GENERIC_LIBRARY_SOURCE_FILES
export GENERIC_TARGET_LIBRARY

export GENERIC_BINARIES_COMMON_SOURCE_FILES=
export GENERIC_BINARIES_SOURCE_ENTRYPOINTS=

all $(MAKECMDGOALS):
	@if [ '$(MAKECMDGOALS)' != 'clean' ]; then echo "$(OK_COLOR)Building ARDroneTool/Lib$(NO_COLOR)"; fi
	@$(MAKE) -C ../../../VP_SDK/Build $(TMP_SDK_FLAGS) $(SDK_FLAGS) $(MAKECMDGOALS)

makefileverbose:
	@echo "Dump compilation flags:"
	@echo "GENERIC_CFLAGS : $(GENERIC_CFLAGS)"
	@echo "GENERIC_LIBS : $(GENERIC_LIBS)"
	@echo "GENERIC_LIB_PATHS : $(GENERIC_LIB_PATHS)"
	@echo "GENERIC_INCLUDES : $(GENERIC_INCLUDES)"
	@echo "GENERIC_BINARIES_SOURCE_DIR : $(GENERIC_BINARIES_SOURCE_DIR)"
	@echo "GENERIC_BINARIES_COMMON_SOURCE_FILES : $(GENERIC_BINARIES_COMMON_SOURCE_FILES)"
	@echo "GENERIC_TARGET_BINARIES_PREFIX : $(GENERIC_TARGET_BINARIES_PREFIX)"
	@echo "GENERIC_TARGET_BINARIES_DIR : $(GENERIC_TARGET_BINARIES_DIR)"
	@echo "GENERIC_BINARIES_SOURCE_ENTRYPOINTS : $(GENERIC_BINARIES_SOURCE_ENTRYPOINTS)"
	@echo "TMP_SDK_FLAGS: $(TMP_SDK_FLAGS)"
	@echo "SDK_FLAGS: $(SDK_FLAGS)"
	@echo "Make cmd goals: $(MAKECMDGOALS)"
