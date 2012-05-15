
###########################################################################################
#
# Designed to build Sdk using generic.makefile
# ----------------------------------------------------------------------------------
# Author : aurelien.morelle@parrot.com
# Date   : 2007/05/16
#
###########################################################################################

include common.makefile

GENERIC_LIBRARY_TARGET_DIR=$(SDK_TARGET_DIR)
GENERIC_TARGET_LIBRARY=$(SDK_TARGET_DIR)/libsdk.a

GENERIC_LIBRARY_SOURCE_FILES=				\
	$(OS_PATH)/vp_os_malloc.c			\
	$(OS_PATH)/vp_os_error_handling.c		\
	$(API_PATH)/vp_api.c				\
	$(API_PATH)/vp_api_error.c			\
	$(API_PATH)/vp_api_io_multi_stage.c		\
	$(API_PATH)/vp_api_stage.c			\
	$(API_PATH)/vp_api_supervisor.c			\
	$(API_PATH)/vp_api_thread_helper.c		\
	$(STAGES_PATH)/vp_stages_frame_pipe.c		\
	$(STAGES_PATH)/vp_stages_configs.c		\
	$(STAGES_PATH)/vp_stages_io_buffer.c		\
	$(STAGES_PATH)/vp_stages_io_console.c		\
	$(STAGES_PATH)/vp_stages_io_file.c		\
	$(STAGES_PATH)/vp_stages_yuv2rgb.c		\
	$(STAGES_PATH)/vp_stages_io_com.c		\
	$(STAGES_PATH)/vp_stages_buffer_to_picture.c	\
	$(ATCODEC_PATH)/ATcodec_Memory.c		\
	$(ATCODEC_PATH)/ATcodec_Buffer.c		\
	$(ATCODEC_PATH)/ATcodec_Sorted_List.c		\
	$(ATCODEC_PATH)/ATcodec.c			\
	$(ATCODEC_PATH)/ATcodec_Tree.c			\
	$(ATCODEC_PATH)/ATcodec_api.c			\
	$(COM_PATH)/$(OS)/vp_com.c			\
	$(COM_PATH)/vp_com_error.c

ifeq ($(USE_PARROTOS_CORE),yes)
GENERIC_LIBRARY_SOURCE_FILES+=				\
	$(OS_PATH)/parrotos/vp_os_signal.c		\
	$(OS_PATH)/parrotos/vp_os_delay.c		\
	$(OS_PATH)/parrotos/vp_os_thread.c
else
GENERIC_LIBRARY_SOURCE_FILES+=				\
	$(OS_PATH)/$(OS)/vp_os_signal.c			\
	$(OS_PATH)/$(OS)/vp_os_delay.c			\
	$(OS_PATH)/$(OS)/vp_os_thread.c
endif

ifneq ($(USE_ANDROID),yes)
ifneq ($(USE_ELINUX),yes)
ifneq ($(USE_MINGW32),yes)
ifneq ($(USE_IPHONE),yes)
  GENERIC_LIBRARY_SOURCE_FILES+=			\
	$(STAGES_PATH)/vp_stages_o_sdl.c		
endif
endif
endif
endif

#ifneq ($(USE_MINGW32),yes)
ifneq ($(USE_ANDROID),yes)
ifneq ($(USE_NDS),yes)
ifneq ($(USE_IPHONE),yes)
  GENERIC_LIBRARY_SOURCE_FILES+=			\
	$(COM_PATH)/$(OS)/vp_com_serial.c
endif
endif
endif
#endif

ifeq ($(USE_ELINUX),yes) 
    GENERIC_LIBRARY_SOURCE_FILES += 			\
  $(OS_PATH)/$(OS)/vp_os_ltt.c
endif

ifneq ($(NO_COM),yes)

# Com Linux source files
########################

ifneq ($(USE_MINGW32),yes)

ifeq ($(USE_ELINUX),yes)
  BUILD_COM_BASE:=yes
  ifeq ($(USE_WIFI),yes)
    GENERIC_LIBRARY_SOURCE_FILES += 			\
	$(COM_PATH)/elinux/vp_com_wifi.c		\
	$(COM_PATH)/elinux/vp_com_wlc.c			\
	$(COM_PATH)/elinux/vp_com_interface.c
  endif
endif

ifeq ($(USE_LINUX),yes)
  BUILD_COM_BASE:=yes
ifeq ($(USE_WIFI),yes)
  GENERIC_LIBRARY_SOURCE_FILES +=			\
	$(COM_PATH)/linux/vp_com_wifi.c
endif
endif

ifeq ($(USE_ANDROID),yes)
  BUILD_COM_BASE:=yes
ifeq ($(USE_WIFI),yes)
  GENERIC_LIBRARY_SOURCE_FILES +=			\
	$(COM_PATH)/linux/vp_com_wifi.c
endif
endif

ifeq ($(USE_IPHONE),yes)
  BUILD_COM_BASE:=yes
ifeq ($(USE_WIFI),yes)
  GENERIC_LIBRARY_SOURCE_FILES +=			\
	$(COM_PATH)/linux/vp_com_wifi.c
endif
endif

ifeq ($(BUILD_COM_BASE),yes)
  GENERIC_LIBRARY_SOURCE_FILES +=			\
	$(COM_PATH)/linux/vp_com_wired.c		\
	$(COM_PATH)/vp_com_socket.c			\
	$(COM_PATH)/vp_com_socket_utils.c		\
	$(COM_PATH)/linux/vp_com_config_itf.c
endif
ifeq ($(USE_BLUEZ),yes)
  GENERIC_LIBRARY_SOURCE_FILES +=			\
	$(COM_PATH)/linux/bluez.c			\
	$(COM_PATH)/linux/vp_com_bluetooth.c
endif
else
  GENERIC_LIBRARY_SOURCE_FILES +=			\
	$(COM_PATH)/vp_com_socket.c			\
	$(COM_PATH)/vp_com_socket_utils.c
endif

endif



GENERIC_BINARIES_COMMON_SOURCE_FILES=			\
	$(EXAMPLES_PATH)/common/common.c

ifneq ($(USE_NDS),yes)
  GENERIC_BINARIES_COMMON_SOURCE_FILES+=		\
	$(EXAMPLES_PATH)/common/atcodec_server.c
endif
GENERIC_BINARIES_COMMON_SOURCE_FILES+=			\
	$(EXAMPLES_PATH)/common/atcodec_client.c

GENERIC_BINARIES_TARGET_DIR=$(GENERIC_LIBRARY_TARGET_DIR)

GENERIC_BINARIES_SOURCE_DIR=$(SDK_SOURCE_DIR)
GENERIC_LIBRARY_SOURCE_DIR=$(SDK_SOURCE_DIR)

GENERIC_TARGET_BINARIES_PREFIX:=Build/$(BUILD_MODE)_$(GENERIC_TARGET_BINARIES_PREFIX)$(GCC_ID)_
GENERIC_TARGET_BINARIES_DIR=$(SDK_SOURCE_DIR)

define FILTER_OUT_PATTERN
  $(2):=$$(shell for i in $$($(2)) ; do echo $$$$i ; done | grep -v $(1))
endef

ifeq ($(USE_ELINUX),yes)
    GENERIC_BINARIES_SOURCE_ENTRYPOINTS=				\
	$(EXAMPLES_PATH)/$(OS)/api_ifile_upper_ofile.c			\
	$(EXAMPLES_PATH)/$(OS)/api_serial_ofile.c			\
	$(EXAMPLES_PATH)/$(OS)/atcodec_sorted_list.c			\
	$(EXAMPLES_PATH)/$(OS)/api_threads.c				\
	$(EXAMPLES_PATH)/$(OS)/atcodec_server.c				\
	$(EXAMPLES_PATH)/$(OS)/atcodec_client.c				\
	$(EXAMPLES_PATH)/$(OS)/api_v4l_raw_ethernet.c			\
	$(EXAMPLES_PATH)/$(OS)/api_v4l_vlib_ethernet.c	
    ifeq ($(USE_CAMIF),yes)
    endif
    ifeq ($(USE_WIFI),yes)
      GENERIC_BINARIES_SOURCE_ENTRYPOINTS+=				\
          $(EXAMPLES_PATH)/$(OS)/api_wifiClientTCP_console.c		
    endif

else
    ifeq ($(USE_NDS),yes)
      GENERIC_BINARIES_SOURCE_ENTRYPOINTS=				\
	  $(EXAMPLES_PATH)/$(OS)/hello_world.c				\
	  $(EXAMPLES_PATH)/$(OS)/atcodec_client.c
    
    else
      ifeq ($(USE_MJPEG),yes)
        GENERIC_BINARIES_SOURCE_ENTRYPOINTS+=				\
	$(EXAMPLES_PATH)/$(OS)/api_encode_decode.c
      endif
      GENERIC_BINARIES_SOURCE_ENTRYPOINTS+=				\
	$(EXAMPLES_PATH)/$(OS)/api_ethernet_raw_sdl.c			\
	$(EXAMPLES_PATH)/$(OS)/api_ethernet_vlib_sdl.c			\
	$(EXAMPLES_PATH)/$(OS)/api_ifile_raw_sdl.c			\
	$(EXAMPLES_PATH)/$(OS)/api_ifile_MJPEG_sdl.c			\
	$(EXAMPLES_PATH)/$(OS)/api_serial_MJPEG_sdl.c			\
	$(EXAMPLES_PATH)/$(OS)/api_serial_raw_sdl.c			\
	$(EXAMPLES_PATH)/$(OS)/api_serial_decoder_sdl.c			\
	$(EXAMPLES_PATH)/$(OS)/api_wifiClientTCP_raw_sdl.c		\
	$(EXAMPLES_PATH)/$(OS)/api_bluetoothClientTCP_decoder_sdl.c	\
	$(EXAMPLES_PATH)/$(OS)/api_wifiClientTCP_decoder_sdl.c		\
	$(EXAMPLES_PATH)/$(OS)/api_wifiClientTCP_MJPEG_sdl.c		\
	$(EXAMPLES_PATH)/$(OS)/api_bluetoothClientTCP_raw_sdl.c		\
	$(EXAMPLES_PATH)/$(OS)/api_BTclientTCP_MJPEG_sdl.c
    endif
endif

ifeq ($(NO_COM),yes)
  $(eval $(call FILTER_OUT_PATTERN,bluetooth,GENERIC_BINARIES_SOURCE_ENTRYPOINTS))
  $(eval $(call FILTER_OUT_PATTERN,wifi,GENERIC_BINARIES_SOURCE_ENTRYPOINTS))
  $(eval $(call FILTER_OUT_PATTERN,videoEncoder,GENERIC_BINARIES_SOURCE_ENTRYPOINTS))
endif
ifneq ($(USE_MJPEG),yes)
ifneq ($(USE_FFMPEG),yes)
  $(eval $(call FILTER_OUT_PATTERN,videoEncoder,GENERIC_BINARIES_SOURCE_ENTRYPOINTS))
  $(eval $(call FILTER_OUT_PATTERN,encoder,GENERIC_BINARIES_SOURCE_ENTRYPOINTS))
  $(eval $(call FILTER_OUT_PATTERN,decoder,GENERIC_BINARIES_SOURCE_ENTRYPOINTS))
  $(eval $(call FILTER_OUT_PATTERN,video_tracker,GENERIC_BINARIES_SOURCE_ENTRYPOINTS))
endif
endif
ifneq ($(USE_VLIB),yes)
  $(eval $(call FILTER_OUT_PATTERN,vlib,GENERIC_BINARIES_SOURCE_ENTRYPOINTS))
endif
ifneq ($(USE_MJPEG),yes)
  $(eval $(call FILTER_OUT_PATTERN,MJPEG,GENERIC_BINARIES_SOURCE_ENTRYPOINTS))
endif


ifeq ($(USE_APP),yes)
  GENERIC_BINARIES_COMMON_SOURCE_FILES=
  GENERIC_BINARIES_SOURCE_ENTRYPOINTS=
endif
ifeq ($(USE_DLL),yes)
  GENERIC_BINARIES_COMMON_SOURCE_FILES=
  GENERIC_BINARIES_SOURCE_ENTRYPOINTS=
endif
ifeq ($(NO_EXAMPLES),yes)
  GENERIC_BINARIES_COMMON_SOURCE_FILES=
  GENERIC_BINARIES_SOURCE_ENTRYPOINTS=
endif


# All that needs to be exported
########################
export GENERIC_LIBRARY_TARGET_DIR
export GENERIC_TARGET_LIBRARY
export GENERIC_ADD_OFILES
export GENERIC_INCLUDES
export GENERIC_LIBRARY_SOURCE_FILES
export GENERIC_BINARIES_SOURCE_DIR
export GENERIC_BINARIES_COMMON_SOURCE_FILES
export GENERIC_BINARIES_TARGET_DIR
export GENERIC_LIBRARY_SOURCE_DIR
export GENERIC_TARGET_BINARIES_PREFIX
export GENERIC_TARGET_BINARIES_DIR
export GENERIC_LIB_PATHS
export GENERIC_LIBS
export GENERIC_BINARIES_LIBS_DEPS
export GENERIC_BINARIES_SOURCE_ENTRYPOINTS


all $(MAKECMDGOALS):
	@$(MAKE) -f generic.makefile PARROTOS_MAKEFILE= $(MAKECMDGOALS)

