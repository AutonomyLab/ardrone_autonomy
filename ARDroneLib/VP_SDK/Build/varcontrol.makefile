
###########################################################################################
#
# Controls validity of variables values
# ----------------------------------------------------------------------------------
# Author : aurelien.morelle@parrot.com
# Date   : 2007/05/16
#
# Included from global Makefile
#
###########################################################################################

#
# Control macros
###################################################

define EXIT_IF_ERROR
  ifeq "$$(ERROR)" "1"
    $$(error There has been some errors)
  endif
endef

define CHECK_DEFINITION
  ifndef $(1)
    $$(warning ERROR : $(1) undefined $(2))
    ERROR=1
  endif
endef

define CHECK_YES_NO
  ifneq "$$($(1))" "yes"
  ifneq "$$($(1))" "no"
    $$(warning ERROR : $(1) must be set to "yes" or "no")
    ERROR=1
  endif
  endif
endef

define CHECK_VALUE
  ifneq "$$($(1))" "$(2)"
    $$(warning ERROR : $(1) must be set to "$(2)" when $(3))
    ERROR=1
  endif
endef

define CHECK_TWO_VALUES
  ifneq "$$($(1))" "$(2)"
    ifneq "$$($(1))" "$(3)"
      $$(warning ERROR : $(1) must be set to "$(2)" or "$(3)" when $(4))
      ERROR=1
    endif
  endif
endef

define CHECK_DIR
  ifeq ($$(wildcard $(1)),)
    $$(warning ERROR Bad $(2) : "$$($(2))" [ $(1) does not exist ])
    ERROR=1
  endif
endef

#
# All that need to be defined
###################################################

ifeq "$(USE_ELINUX)" "yes"
$(eval $(call CHECK_DEFINITION,ALL_SOURCES,: should be defined as an environment variable.))
$(eval $(call CHECK_DEFINITION,ALL_TARGETS,: should be defined as an environment variable.))
$(eval $(call CHECK_DEFINITION,LINUX_CONFIG_PATH,: should be defined as an environment variable.))
endif 
$(eval $(call CHECK_DEFINITION,USE_ANDROID,: should be "yes" or "no"))
$(eval $(call CHECK_DEFINITION,USE_LINUX,: should be "yes" or "no"))
$(eval $(call CHECK_DEFINITION,USE_ELINUX,: should be "yes" or "no"))
$(eval $(call CHECK_DEFINITION,USE_NDS,: should be "yes" or "no"))
$(eval $(call CHECK_DEFINITION,USE_IPHONE,: should be "yes" or "no"))
$(eval $(call CHECK_DEFINITION,NO_COM,: should be "yes" or "no"))
$(eval $(call CHECK_DEFINITION,USE_BLUEZ,: should be "yes" or "no"))
$(eval $(call CHECK_DEFINITION,USE_VLIB,: should be "yes" or "no"))
$(eval $(call CHECK_DEFINITION,RELEASE_BUILD,: should be "yes" or "no"))
$(eval $(call CHECK_DEFINITION,USE_SDK,: should be "yes" or "no"))
$(eval $(call CHECK_DEFINITION,USE_APP,: should be "yes" or "no"))
$(eval $(call CHECK_DEFINITION,NO_EXAMPLES,: should be "yes" or "no"))
#
#ifeq "$(USE_SDK)" "yes"
#  $(eval $(call CHECK_DEFINITION,SDK_VERSION,( example : "head" )))
#endif

ifeq "$(USE_ELINUX)" "yes"
  $(eval $(call CHECK_DEFINITION,TOOLCHAIN_VERSION,( example : "arm-uclibc" )))
  $(eval $(call CHECK_DEFINITION,USE_PARROTOS_CORE,: should be "yes" or "no"))
endif

ifeq "$(USE_NDS)" "yes"
  $(eval $(call CHECK_DEFINITION,NDS_CPU,: should be "ARM7" or "ARM9"))
endif

ifeq "$(USE_BONJOUR)" "yes"
  $(eval $(call CHECK_DEFINITION,BONJOUR_VERSION,( example : "head" )))
endif

$(eval $(call EXIT_IF_ERROR))

ifeq "$(USE_APP)" "yes"
ifndef PARROTOS_MAKEFILE
  $(eval $(call CHECK_DEFINITION,GENERIC_BINARIES_SOURCE_DIR,))
  $(eval $(call CHECK_DEFINITION,GENERIC_BINARIES_SOURCE_DIR,))
  $(eval $(call CHECK_DEFINITION,GENERIC_TARGET_BINARIES_DIR,))
  $(eval $(call CHECK_DEFINITION,GENERIC_BINARIES_SOURCE_ENTRYPOINTS,))
endif
endif

#
# Validity test
###################################################
$(eval $(call CHECK_YES_NO,USE_ANDROID))
$(eval $(call CHECK_YES_NO,USE_LINUX))
$(eval $(call CHECK_YES_NO,USE_ELINUX))
$(eval $(call CHECK_YES_NO,USE_NDS))
$(eval $(call CHECK_YES_NO,USE_IPHONE))

$(eval $(call CHECK_YES_NO,NO_COM))
$(eval $(call CHECK_YES_NO,USE_BLUEZ))
$(eval $(call CHECK_YES_NO,USE_SDK))
$(eval $(call CHECK_YES_NO,USE_APP))
$(eval $(call CHECK_YES_NO,USE_VLIB))
$(eval $(call CHECK_YES_NO,NO_EXAMPLES))
$(eval $(call CHECK_YES_NO,RELEASE_BUILD))
$(eval $(call CHECK_YES_NO,USE_ARDRONELIB))
$(eval $(call CHECK_YES_NO,USE_ARDRONE_VISION))
$(eval $(call CHECK_YES_NO,USE_ARDRONE_POLARIS))
$(eval $(call CHECK_YES_NO,USE_ARDRONE_VICON))
$(eval $(call CHECK_YES_NO,USE_ARDRONE_TEST_BENCHS))
$(eval $(call CHECK_YES_NO,USE_ARDRONE_CALIBRATION))

ifeq "$(USE_ELINUX)" "yes"
  $(eval $(call CHECK_YES_NO,USE_PARROTOS_CORE))
  ifneq "$(TOOLCHAIN_VERSION)" "arm-uclibc"
  ifneq "$(TOOLCHAIN_VERSION)" "arm-2009q1"
  ifneq "$(TOOLCHAIN_VERSION)" "arm-eglibc"
  ifneq "$(TOOLCHAIN_VERSION)" "arm-2010.09"
    $(warning ERROR Bad TOOLCHAIN_VERSION : must be "arm-uclibc", "arm-2009q1", or "arm-2010.09")
    ERROR=1
  endif
  endif
  endif
  endif
endif

ifeq "$(USE_NDS)" "yes"
  ifneq "$(NDS_CPU)" "ARM7"
  ifneq "$(NDS_CPU)" "ARM9"
    $(warning ERROR Bad NDS_CPU : must be "ARM7" or "ARM9")
    ERROR=1
  endif
  endif
endif

$(eval $(call EXIT_IF_ERROR))

#
# Compatibility test
###################################################
ifeq "$(USE_ANDROID)" "yes"
  $(eval $(call CHECK_DEFINITION,NDK_PATH,: should be defined as an environment variable.))
  $(eval $(call CHECK_VALUE,USE_LINUX,no,android is used))
  $(eval $(call CHECK_VALUE,USE_NDS,no,android is used))
  $(eval $(call CHECK_VALUE,USE_ELINUX,no,android is used))
  $(eval $(call CHECK_VALUE,USE_PARROTOS_CORE,no,android is used (parrotos is available only on elinux)))
  $(eval $(call CHECK_VALUE,USE_IPHONE,no,android is used))
endif

ifeq "$(USE_APP)" "yes"
  $(eval $(call CHECK_DEFINITION,APP_ID,(example : "ardrone")))
endif
ifeq "$(USE_SDK)" "yes"
  $(eval $(call CHECK_DIR,$(ALL_SOURCES)/$(SDK_SOURCE_DIR),SDK_VERSION))
  $(eval $(call EXIT_IF_ERROR))
endif
ifeq "$(USE_BONJOUR)" "yes"
   $(eval $(call CHECK_DIR,$(ALL_SOURCES)/bonjour/$(BONJOUR_VERSION)/Bonjour,BONJOUR_VERSION))
endif

ifeq "$(USE_LINUX)" "yes"
  $(eval $(call CHECK_VALUE,USE_ANDROID,no,linux is used))
  $(eval $(call CHECK_VALUE,USE_ELINUX,no,linux is used))
  $(eval $(call CHECK_VALUE,USE_NDS,no,linux is used))
  $(eval $(call CHECK_VALUE,USE_PARROTOS_CORE,no,linux is used (parrotos is available only on elinux)))
  $(eval $(call CHECK_VALUE,USE_IPHONE,no,linux is used))
endif

ifeq "$(USE_ELINUX)" "yes"
  $(eval $(call CHECK_VALUE,USE_ANDROID,no,linux is used))
  $(eval $(call CHECK_DIR,$(ALL_SOURCES)/commonsoft/$(COMMONSOFT_VERSION)/CommonSoft,COMMONSOFT_VERSION))
  $(eval $(call CHECK_VALUE,USE_LINUX,no,embedded linux is used))
  $(eval $(call CHECK_VALUE,USE_NDS,no,embedded linux is used))
  $(eval $(call CHECK_VALUE,USE_IPHONE,no,embedded linux is used))
endif

ifeq "$(USE_NDS)" "yes"
  $(eval $(call CHECK_VALUE,USE_ANDROID,no,linux is used))
  $(eval $(call CHECK_VALUE,USE_LINUX,no,nintendo ds is used))
  $(eval $(call CHECK_VALUE,USE_ELINUX,no,nintendo ds is used))
  $(eval $(call CHECK_VALUE,USE_PARROTOS_CORE,no,nintendo ds is used (parrotos is available only on elinux)))
  $(eval $(call CHECK_VALUE,USE_IPHONE,no,nintendo ds is used))
endif


ifeq "$(USE_IPHONE)" "yes"
  $(eval $(call CHECK_VALUE,USE_ANDROID,no,linux is used))
  $(eval $(call CHECK_DEFINITION,SDKROOT,(example : "SDKROOT is the root path of iphone sdk")))
  $(eval $(call CHECK_DEFINITION,PLATFORM_DEVELOPER_USR_DIR,("PLATFORM_DEVELOPER_USR_DIR is the user directory of platform path")))
  $(eval $(call CHECK_DEFINITION,PLATFORM_DEVELOPER_BIN_DIR,("PLATFORM_DEVELOPER_BIN_DIR is the binary directory of platform path")))
  $(eval $(call CHECK_DEFINITION,PLATFORM_PREFERRED_ARCH,("PLATFORM_PREFERRED_ARCH is the preferred architecture of platforms")))
  $(eval $(call CHECK_DEFINITION,PLATFORM_NAME,("PLATFORM_NAME is the name of platform (iphoneos, iphonesimulator)")))
  $(eval $(call CHECK_VALUE,USE_NDS,no,iphone is used))
  $(eval $(call CHECK_VALUE,USE_LINUX,no,iphone is used))
  $(eval $(call CHECK_VALUE,USE_ELINUX,no,iphone is used))
  $(eval $(call CHECK_VALUE,USE_PARROTOS_CORE,no,iphone is used (parrotos is available only on elinux)))
  $(eval $(call CHECK_TWO_VALUES,PLATFORM_NAME,iphoneos,iphonesimulator, PLATFORM_NAME must be set to iphoneos or iphonesimulator))
endif

$(eval $(call CHECK_VALUE,USE_BONJOUR,no,ecos is not used))

ifeq "$(NO_COM)" "yes"
  $(eval $(call CHECK_VALUE,USE_BLUEZ,no,NO_COM is used))
endif

ifeq "$(USE_BROADCOM)" "yes"
  $(eval $(call CHECK_VALUE,USE_IWLIB,no,BROADCOM is used))
endif

ifeq "$(USE_IWLIB)" "yes"
  $(eval $(call CHECK_VALUE,USE_BROADCOM,no,iwlib is used))
endif

$(eval $(call EXIT_IF_ERROR))

