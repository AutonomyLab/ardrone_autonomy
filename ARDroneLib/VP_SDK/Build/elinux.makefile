###########################################################################################
#
# Designed to help building vp_sdk's application with eLinux
# ----------------------------------------------------------------------------------
# Author : pierre.eline@parrot.com
# Date   : 2009/07/01
#
###########################################################################################


ifeq ($(USE_ELINUX),yes)
  ifeq ($(USE_BROADCOM),yes)
    ELINUX_BCM4318_AP=$(shell cat $(LINUX_CONFIG_PATH) | grep BR2_PACKAGE_BCM4318_AP | grep y > /dev/null && echo yes || echo no)
    ifeq ($(ELINUX_BCM4318_AP),yes)
      GENERIC_CFLAGS+=-DBR2_PACKAGE_BCM4318_AP
    endif
    USE_WIFI=$(shell cat $(LINUX_CONFIG_PATH) | grep BR2_PACKAGE_BCM4318* | grep y > /dev/null && echo yes || echo no)
  endif
endif

USE_WIFI=yes

USE_CAMIF = yes

# All that needs to be exported
########################
export USE_MEMPROT
export USE_SDCARD
export USE_CAMIF
export USE_LCD
export USE_WIFI
export USE_BASEBAND
export GENERIC_CFLAGS


