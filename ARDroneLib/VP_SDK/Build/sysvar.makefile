
###########################################################################################
#
# Creates needed build system variables
# ----------------------------------------------------------------------------------
# Author : aurelien.morelle@parrot.com
# Date   : 2007/05/16
#
# Included from global Makefile
#
###########################################################################################

ifndef ALL_TARGETS
export ALL_TARGETS=../../Soft/Build/targets_versions
endif

ifeq ($(MAKECMDGOALS),check)
  export QUIET_BUILD=no
##  export MAKEFLAGS+=--no-print-directory
endif

ifeq ($(QUIET_BUILD),yes)
##  export MAKEFLAGS+=-s --no-print-directory
  GENERIC_COMMAND_PREFIX:=@$(GENERIC_COMMAND_PREFIX)
  RM=@rm -rfv
else
  RM=rm -rf
endif


ifeq "$(RELEASE_BUILD)" "yes"
  BUILD_MODE=PROD_MODE
endif
ifeq "$(RELEASE_BUILD)" "no"
  BUILD_MODE=DEBUG_MODE
endif


ifeq ($(USE_NDS),yes)
    export DEVKITARM=/usr/local/devkitARM-eabi_release_21/devkitARM
    export DEVKITPRO=$(DEVKITARM)
    OS_TARGET_ID=nds_$(NDS_CPU)
    GNUTOOLS_PATH=$(DEVKITARM)/bin
    GENERIC_COMMAND_PREFIX=$(GNUTOOLS_PATH)/arm-eabi-
    ifeq ($(wildcard $(GNUTOOLS_PATH)),)
      $(error ERROR Bad GNUTOOLS_PATH [ $(GNUTOOLS_PATH) does not exist ])
    endif
    ifeq ($(wildcard $(GENERIC_COMMAND_PREFIX)gcc),)
      $(error ERROR Bad GENERIC_COMMAND_PREFIX [ $(GENERIC_COMMAND_PREFIX)gcc does not exist ])
    endif
else
    ifeq ($(USE_MINGW32),yes)
      OS_TARGET_ID=mingw32_gtk+-2.14.7
      GENERIC_COMMAND_PREFIX=i586-mingw32msvc-
      ifeq ($(shell which $(GENERIC_COMMAND_PREFIX)gcc 2> /dev/null),)
        $(error ERROR You need Minimalist GNU Win32 cross compiler. (under Debian: apt-get install mingw32))
      endif
    else
      ifeq ($(USE_ELINUX),yes)
        OS_TARGET_ID=eLinux
        TOOLCHAIN_PATH=/opt/$(TOOLCHAIN_VERSION)/bin
        ifeq ($(TOOLCHAIN_VERSION),arm-2009q1)
          GENERIC_COMMAND_PREFIX=$(TOOLCHAIN_PATH)/arm-none-linux-gnueabi-
        else
        ifeq ($(TOOLCHAIN_VERSION),arm-eglibc)
          GENERIC_COMMAND_PREFIX=$(TOOLCHAIN_PATH)/arm-none-linux-gnueabi-
        else
        ifeq ($(TOOLCHAIN_VERSION),arm-2010.09)
          GENERIC_COMMAND_PREFIX=$(TOOLCHAIN_PATH)/arm-none-linux-gnueabi-
        else
        ifeq ($(TOOLCHAIN_VERSION),arm-uclibc)
          GENERIC_COMMAND_PREFIX=$(TOOLCHAIN_PATH)/arm-linux-uclibcgnueabi-
        else
          $(error ERROR Unsupported toolchain.)
        endif
        endif
        endif
		endif
      else
        ifeq ($(USE_IPHONE),yes)
			OS_TARGET_ID=$(PLATFORM_NAME)
			OS_TARGET_NAME:=$(OS_TARGET_ID)
			OS_TARGET_NAME:=$(shell echo "$(OS_TARGET_NAME)" | sed -e "s/iphone/iPhone/g")
			OS_TARGET_NAME:=$(shell echo "$(OS_TARGET_NAME)" | sed -e "s/os/OS/g")
			OS_TARGET_NAME:=$(shell echo "$(OS_TARGET_NAME)" | sed -e "s/simulator/Simulator/g")
			TOOLCHAIN_PATH=$(PLATFORM_DEVELOPER_BIN_DIR)
			GENERIC_COMMAND_PREFIX=$(TOOLCHAIN_PATH)/
	else
	     OS_TARGET_ID=$(shell uname -sor | sed -e "s/[ \/]/_/g")
        ifeq ($(USE_ANDROID),yes)
          TOOLCHAIN_PATH=$(NDK_PATH)/toolchains/$(TOOLCHAIN_VERSION)/prebuilt/linux-x86/bin
          GENERIC_COMMAND_PREFIX=$(TOOLCHAIN_PATH)/arm-linux-androideabi-
        else
	TOOLCHAIN_PATH=$(shell which gcc | sed "s:/gcc::")
	GENERIC_COMMAND_PREFIX=$(TOOLCHAIN_PATH)/
        endif
	endif
      endif
    endif
endif

GCC_VERSION=$(shell $(GENERIC_COMMAND_PREFIX)gcc -v 2>&1 | grep --color=never version | grep -v [cC]onfigur | sed -e "s/\(^version gcc \)\([^ ]*\)\(.*\)/\2/" | sed -e "s/\(^gcc version \)\([^ ]*\)\(.*\)/\2/")

ifeq ($(USE_NDS),no)
ifeq ($(USE_MINGW32),no)
ifeq ($(USE_ELINUX), no)
  ifneq ($(GCC_VERSION),4.3.3)
#    $(error GCC version not supported)
  endif
endif
endif
endif

GCC_ID:=$(subst $(GNUTOOLS_PATH)/,,$(GENERIC_COMMAND_PREFIX))gcc_$(GCC_VERSION)

ifneq ($(NO_COM),yes)
  ifeq ($(USE_BLUEZ),yes)
    COM_TARGET_ID=libbluetooth3
    COM_TARGET_ID:=$(COM_TARGET_ID)_$(shell apt-cache show libbluetooth-dev | grep "^Version" | cut -d" " -f2 | head -n 1)
    COM_TARGET_ID:=$(COM_TARGET_ID)_libiw
    COM_TARGET_ID:=$(COM_TARGET_ID)_$(shell apt-cache show libiw-dev | grep "^Version" | cut -d" " -f2 | head -n 1)
  endif
endif

ifeq ($(USE_BONJOUR),yes)
  BONJOUR_TARGET_ID:=bonjour_$(BONJOUR_VERSION)
  COM_TARGET_ID:=$(COM_TARGET_ID)_$(BONJOUR_TARGET_ID)
endif

ARCH_TARGET_ID=$(PLATFORM_PREFERRED_ARCH)
PARROTOS_CORE_TARGET_ID:=parrotOS_core_$(COMMONSOFT_VERSION)
PARROTOS_UTILS_TARGET_ID:=parrotOS_utils_$(COMMONSOFT_VERSION)
PARROTOS_DRIVERS_TARGET_ID:=parrotOS_drivers_$(COMMONSOFT_VERSION)
PARROTOS_DEVS_TARGET_ID:=parrotOS_devs_$(COMMONSOFT_VERSION)
PARROTOS_CODEC_TARGET_ID:=parrotOS_codec_$(COMMONSOFT_VERSION)
LIBPLF_TARGET_ID:=libplf_$(COMMONSOFT_VERSION)

ifeq ($(FFMPEG_SUPPORT),yes)
   FFMPEG_SUPPORT_TARGET_ID=ffmpeg_static_$(BUILD_MODE)
endif

ifeq ($(USE_ARDRONELIB),yes)
   ARDRONELIB_TARGET_ID=ardrone_lib_$(BUILD_MODE)
endif

ifeq ($(USE_ARDRONE_VISION),yes)
   ARDRONE_VISION_TARGET_ID=vision_lib_$(BUILD_MODE)
endif

ifeq ($(USE_ARDRONE_POLARIS),yes)
   ARDRONE_POLARIS_TARGET_ID=polaris_lib_$(BUILD_MODE)
endif

ifeq ($(USE_ARDRONE_VICON),yes)
   ARDRONE_VICON_TARGET_ID=vicon_lib_$(BUILD_MODE)
endif

ifeq ($(USE_ARDRONE_TEST_BENCHS),yes)
   ARDRONE_TEST_BENCHS_TARGET_ID=test_benchs_lib_$(BUILD_MODE)
endif

ifeq ($(USE_ARDRONE_CALIBRATION),yes)
   ARDRONE_CALIBRATION_TARGET_ID=calibration_lib_$(BUILD_MODE)
endif

ifeq ($(USE_SDK),yes)
  SDK_TARGET_ID=sdk_$(BUILD_MODE)
endif
ifeq ($(USE_APP),yes)
  APP_TARGET_ID:=$(APP_ID)_$(BUILD_MODE)
endif
ifeq ($(USE_DLL),yes)
  DLL_TARGET_ID:=$(DLL_ID)_$(BUILD_MODE)
endif
ifeq ($(USE_LIB),yes)
  LIB_TARGET_ID:=$(LIB_ID)_$(BUILD_MODE)
endif

ifeq ($(USE_MJPEG),yes)
  CODEC_TARGET_ID=mjpeg
endif
ifeq ($(USE_VLIB),yes)
  CODEC_TARGET_ID=vlib
endif

ifeq ($(USE_ARDRONELIB),yes)
ifneq ($(COM_TARGET_ID),)
  ARDRONELIB_TARGET_ID:=$(ARDRONELIB_TARGET_ID)_$(COM_TARGET_ID)
endif
ifneq ($(CODEC_TARGET_ID),)
  ARDRONELIB_TARGET_ID:=$(ARDRONELIB_TARGET_ID)_$(CODEC_TARGET_ID)
endif
ifneq ($(ARCH_TARGET_ID),)
  ARDRONELIB_TARGET_ID:=$(ARDRONELIB_TARGET_ID)_$(ARCH_TARGET_ID)
endif
endif

ifeq ($(USE_ARDRONE_VISION),yes)
ifneq ($(COM_TARGET_ID),)
  ARDRONE_VISION_TARGET_ID:=$(ARDRONE_VISION_TARGET_ID)_$(COM_TARGET_ID)
endif
ifneq ($(CODEC_TARGET_ID),)
  ARDRONE_VISION_TARGET_ID:=$(ARDRONE_VISION_TARGET_ID)_$(CODEC_TARGET_ID)
endif
ifneq ($(ARCH_TARGET_ID),)
  ARDRONE_VISION_TARGET_ID:=$(ARDRONE_VISION_TARGET_ID)_$(ARCH_TARGET_ID)
endif
endif

ifeq ($(USE_ARDRONE_POLARIS),yes)
ifneq ($(COM_TARGET_ID),)
  ARDRONE_POLARIS_TARGET_ID:=$(ARDRONE_POLARIS_TARGET_ID)_$(COM_TARGET_ID)
endif
ifneq ($(CODEC_TARGET_ID),)
  ARDRONE_POLARIS_TARGET_ID:=$(ARDRONE_POLARIS_TARGET_ID)_$(CODEC_TARGET_ID)
endif
ifneq ($(ARCH_TARGET_ID),)
  ARDRONE_POLARIS_TARGET_ID:=$(ARDRONE_POLARIS_TARGET_ID)_$(ARCH_TARGET_ID)
endif
endif

ifeq ($(USE_ARDRONE_VICON),yes)
ifneq ($(COM_TARGET_ID),)
  ARDRONE_VICON_TARGET_ID:=$(ARDRONE_VICON_TARGET_ID)_$(COM_TARGET_ID)
endif
ifneq ($(CODEC_TARGET_ID),)
  ARDRONE_VICON_TARGET_ID:=$(ARDRONE_VICON_TARGET_ID)_$(CODEC_TARGET_ID)
endif
ifneq ($(ARCH_TARGET_ID),)
  ARDRONE_VICON_TARGET_ID:=$(ARDRONE_VICON_TARGET_ID)_$(ARCH_TARGET_ID)
endif
endif

ifeq ($(USE_ARDRONE_TEST_BENCHS),yes)
ifneq ($(COM_TARGET_ID),)
  ARDRONE_TEST_BENCHS_TARGET_ID:=$(ARDRONE_TEST_BENCHS_TARGET_ID)_$(COM_TARGET_ID)
endif
ifneq ($(CODEC_TARGET_ID),)
  ARDRONE_TEST_BENCHS_TARGET_ID:=$(ARDRONE_TEST_BENCHS_TARGET_ID)_$(CODEC_TARGET_ID)
endif
ifneq ($(ARCH_TARGET_ID),)
  ARDRONE_TEST_BENCHS_TARGET_ID:=$(ARDRONE_TEST_BENCHS_TARGET_ID)_$(ARCH_TARGET_ID)
endif
endif

ifeq ($(USE_ARDRONE_CALIBRATION),yes)
ifneq ($(COM_TARGET_ID),)
  ARDRONE_CALIBRATION_TARGET_ID:=$(ARDRONE_CALIBRATION_TARGET_ID)_$(COM_TARGET_ID)
endif
ifneq ($(CODEC_TARGET_ID),)
  ARDRONE_CALIBRATION_TARGET_ID:=$(ARDRONE_CALIBRATION_TARGET_ID)_$(CODEC_TARGET_ID)
endif
ifneq ($(ARCH_TARGET_ID),)
  ARDRONE_CALIBRATION_TARGET_ID:=$(ARDRONE_CALIBRATION_TARGET_ID)_$(ARCH_TARGET_ID)
endif
endif

ifeq ($(VIDEO_CODEC),ITTIAM_MP4ARM)
  ITTIAM_MPEG4_TARGET_ID:=ittiam_wrapper_$(BUILD_MODE)
ifneq ($(CODEC_TARGET_ID),)
  ITTIAM_MPEG4_TARGET_ID:=$(ITTIAM_MPEG4_TARGET_ID)_$(CODEC_TARGET_ID)
endif
ifneq ($(ARCH_TARGET_ID),)
  ITTIAM_MPEG4_TARGET_ID:=$(ITTIAM_MPEG4_TARGET_ID)_$(ARCH_TARGET_ID)
endif
endif




ifeq ($(USE_SDK),yes)
ifneq ($(COM_TARGET_ID),)
  SDK_TARGET_ID:=$(SDK_TARGET_ID)_$(COM_TARGET_ID)
endif
ifneq ($(CODEC_TARGET_ID),)
  SDK_TARGET_ID:=$(SDK_TARGET_ID)_$(CODEC_TARGET_ID)
endif
ifneq ($(ARCH_TARGET_ID),)
  SDK_TARGET_ID:=$(SDK_TARGET_ID)_$(ARCH_TARGET_ID)
endif
endif

ifeq ($(USE_APP),yes)
ifneq ($(COM_TARGET_ID),)
  APP_TARGET_ID:=$(APP_TARGET_ID)_$(COM_TARGET_ID)
endif
ifneq ($(CODEC_TARGET_ID),)
  APP_TARGET_ID:=$(APP_TARGET_ID)_$(CODEC_TARGET_ID)
endif
ifneq ($(ARCH_TARGET_ID),)
  APP_TARGET_ID:=$(APP_TARGET_ID)_$(ARCH_TARGET_ID)
endif
endif

ifeq ($(USE_LIB),yes)
ifneq ($(COM_TARGET_ID),)
  LIB_TARGET_ID:=$(LIB_TARGET_ID)_$(COM_TARGET_ID)
endif
ifneq ($(CODEC_TARGET_ID),)
  LIB_TARGET_ID:=$(LIB_TARGET_ID)_$(CODEC_TARGET_ID)
endif
ifneq ($(ARCH_TARGET_ID),)
  LIB_TARGET_ID:=$(LIB_TARGET_ID)_$(ARCH_TARGET_ID)
endif
endif

ifneq ($(CODEC_TARGET_ID),)
  CODEC_TARGET_ID:=$(CODEC_TARGET_ID)_$(BUILD_MODE)
ifneq ($(ARCH_TARGET_ID),)
  CODEC_TARGET_ID:=$(CODEC_TARGET_ID)_$(ARCH_TARGET_ID)
endif
endif
ifneq ($(COM_TARGET_ID),)
  COM_TARGET_ID:=$(COM_TARGET_ID)_$(BUILD_MODE)
ifneq ($(ARCH_TARGET_ID),)
  COM_TARGET_ID:=$(COM_TARGET_ID)_$(ARCH_TARGET_ID)
endif
endif

ifneq ($(BONJOUR_TARGET_ID),)
  BONJOUR_TARGET_ID:=$(BONJOUR_TARGET_ID)_$(BUILD_MODE)_$(_COM_TARGET_ID_)
endif

define ADD_OS_TARGET_ID
  ifneq ($(1),OS)
    $(1)_TARGET_ID:=$$($(1)_TARGET_ID)_$$(OS_TARGET_ID)
  endif
endef

define FINALIZE_TARGET_ID
  ifneq ($$($(1)_TARGET_ID),)
    $(1)_TARGET_ID:=$$($(1)_TARGET_ID)_$$(GCC_ID)
    $(1)_TARGET_DIR:=$$(ALL_TARGETS)/$$($(1)_TARGET_ID)
  endif
  export $(1)_TARGET_DIR
endef

TARGET_IDS:=OS APP DLL LIB SDK COM CODEC JPEG BONJOUR PARROTOS_CORE PARROTOS_UTILS PARROTOS_DRIVERS PARROTOS_DEVS PARROTOS_CODEC LIBPLF ARDRONE_VISION ARDRONE_POLARIS ARDRONE_VICON ARDRONE_TEST_BENCHS ARDRONE_CALIBRATION ARDRONELIB ITTIAM_MPEG4 FFMPEG_SUPPORT ITTIAM_SUPPORT

$(foreach id,$(filter-out OS,$(TARGET_IDS)),$(eval $(call ADD_OS_TARGET_ID,$(id))))
$(foreach id,$(TARGET_IDS),$(eval $(call FINALIZE_TARGET_ID,$(id))))


ifeq ($(USE_NDS),yes)
    ifeq ($(NDS_CPU),ARM7)
      TARGET_CPU_ARM=1
      TARGET_CPU_X86=0
    endif
    ifeq ($(NDS_CPU),ARM9)
      TARGET_CPU_ARM=1
      TARGET_CPU_X86=0
    endif
else
    ifeq ($(USE_ANDROID),yes)
      TARGET_CPU_ARM=1
      TARGET_CPU_X86=0
    else
    ifeq ($(USE_ELINUX),yes)
      TARGET_CPU_ARM=1
      TARGET_CPU_X86=0
    else
      ifeq ($(USE_IPHONE),yes)
  	      ifeq ($(PLATFORM_NAME),iphoneos)
            TARGET_CPU_ARM=1
            TARGET_CPU_X86=0
	      else
            TARGET_CPU_ARM=0
            TARGET_CPU_X86=1
	      endif
      else
	     TARGET_CPU_ARM=0
        TARGET_CPU_X86=1
      endif
    endif
   endif
endif

# All that needs to be exported
########################

export MAKE
export RM

export GENERIC_COMMAND_PREFIX
export GNUTOOLS_PATH
export GCC_ID

export BUILD_MODE

export TARGET_CPU_ARM
export TARGET_CPU_X86

