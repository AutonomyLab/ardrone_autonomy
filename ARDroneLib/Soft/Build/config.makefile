
#########################################################
# Common build definitions (CUSTOM)
#########################################################

RELEASE_BUILD        = yes
QUIET_BUILD          = yes

#########################################################
# System utility definitions (STATIC)
#########################################################
define CHECK_UNDEFINITION
  ifdef $(1)
    $$(warning ERROR : $(1) defined $(2))
    ERROR=1
  endif
endef

define EXIT_IF_ERROR
  ifeq "$$(ERROR)" "1"
    $$(error There has been some errors)
  endif
endef


#########################################################
# Validity control (STATIC)
#########################################################

ifdef PC_TARGET
  $(eval $(call CHECK_UNDEFINITION,CONSOLE_TARGET,(should not be defined when PC_TARGET is defined)))
endif

$(eval $(call EXIT_IF_ERROR))


#########################################################
# Common definitions (STATIC)
#########################################################

ifeq "$(QUIET_BUILD)" "yes"
  MAKE=@make -s
else
  MAKE=make
endif

COMMON_DIR:=../Common

SDK_FLAGS:="NO_EXAMPLES=yes"
SDK_FLAGS+="USE_SDK=yes"
SDK_FLAGS+="QUIET_BUILD=$(QUIET_BUILD)"
SDK_FLAGS+="RELEASE_BUILD=$(RELEASE_BUILD)"
SDK_FLAGS+="SDK_VERSION=$(SDK_VERSION)"

ifeq ($(filter NO_COM=%,$(TMP_SDK_FLAGS)),)
  SDK_FLAGS+="NO_COM=no"
endif

#########################################################
# PC_TARGET specific definitions (STATIC)
#########################################################
ifdef PC_TARGET
  SDK_FLAGS+="NO_COM=no"

  ifeq ($(ARDRONE_TARGET_OS),Linux)
    OS_DEFINE=GNU_LINUX
  else
   ifeq ($(ARDRONE_TARGET_OS),iphoneos)
    OS_DEFINE=GNU_LINUX
   else
      ifeq ($(ARDRONE_TARGET_OS),iphonesimulator)
         OS_DEFINE=GNU_LINUX
      else
         TARGET:=$(TARGET).exe
         OS_DEFINE=WINDOW
      endif
    endif
  endif

  GENERIC_CFLAGS+=-D_MOBILE

  ifeq ("$(PC_USE_TABLE_PILOTAGE)","yes")
    GENERIC_CFLAGS+=-DUSE_TABLE_PILOTAGE
  endif

  ifeq ("$(RECORD_VIDEO)","yes")
    GENERIC_CFLAGS+=-DRECORD_VIDEO
  endif

  GENERIC_CFLAGS+=-D$(OS_DEFINE)
  ifeq ($(IPHONE_MODE),yes)
     ifeq ($(ARDRONE_TARGET_OS),iphoneos)
        GENERIC_CFLAGS+=-DTARGET_OS_IPHONE
     else
        GENERIC_CFLAGS+=-DTARGET_IPHONE_SIMULATOR
     endif
  endif

  ifneq ("$(USE_MINGW32)","yes")
    GENERIC_CFLAGS+=$(shell pkg-config --cflags gtk+-2.0)
    GENERIC_LIBS+=$(shell pkg-config --libs gtk+-2.0)
  endif

  ifeq ("$(USE_LINUX)","yes")
     SDK_FLAGS+="USE_LINUX=yes"
  else
     SDK_FLAGS+="USE_LINUX=no"
  endif
  
  SDK_FLAGS+="USE_ELINUX=no"
  
  ifneq ($(findstring iphone,$(ARDRONE_TARGET_OS)),)
	SDK_FLAGS+="USE_IPHONE=yes"
  	SDK_FLAGS+="IPHONE_PLATFORM=$(ARDRONE_TARGET_OS)"
    SDK_FLAGS+="IPHONE_SDK_PATH=$(IPHONE_SDK_PATH)"
  else
	SDK_FLAGS+="USE_IPHONE=no"
  endif
  
  SDK_FLAGS+="ARDRONE_TARGET_ARCH=$(ARDRONE_TARGET_ARCH)"
  
  ifeq ("$(USE_NDS)","yes")
     SDK_FLAGS+="USE_NDS=yes"
     SDK_FLAGS+="NDS_CPU=ARM7"
  else
     SDK_FLAGS+="USE_NDS=no"
  endif
  
  ifeq ("$(USE_ANDROID)","yes")
     SDK_FLAGS+="USE_ANDROID=yes"
     SDK_FLAGS+="TOOLCHAIN_VERSION=arm-eabi-4.4.0"
     SDK_FLAGS+="NDK_PLATFORM_VERSION=android-5"
  else
     SDK_FLAGS+="USE_ANDROID=no"
  endif

  ifeq ($(FFMPEG_RECORDING_SUPPORT),yes)
  ifeq ($(USE_LINUX),yes)
	  SDK_FLAGS+="USE_FFMPEG=yes"
	  GENERIC_CFLAGS+=-DUSE_FFMPEG
  endif
  endif

  ifeq ($(filter USE_BLUEZ=%,$(TMP_SDK_FLAGS)),)
    SDK_FLAGS+="USE_BLUEZ=no"
  endif

  SDK_FLAGS+="USE_VLIB=yes"
  SDK_FLAGS+="USE_BONJOUR=no"
  SDK_FLAGS+="USE_WIFI=yes"
  
  SDK_FLAGS+="USE_BROADCOM=no"
  SDK_FLAGS+="USE_IWLIB=no"

  SDK_FLAGS+="FF_ARCH=Intel"

  SDK_FLAGS+="USE_PARROTOS_CORE=no"
  SDK_FLAGS+="USE_PARROTOS_DRIVERS=no"
  SDK_FLAGS+="USE_PARROTOS_DEVS=no"
  SDK_FLAGS+="USE_PARROTOS_CODEC=no"

  
  SDK_FLAGS+="USE_ARDRONELIB=yes"
  SDK_FLAGS+="USE_ARDRONE_VISION=yes"
  SDK_FLAGS+="USE_ARDRONE_POLARIS=no"
  SDK_FLAGS+="USE_ARDRONE_VICON=no"
  SDK_FLAGS+="USE_ARDRONE_TEST_BENCHS=no"
  SDK_FLAGS+="USE_ARDRONE_CALIBRATION=no"

endif

