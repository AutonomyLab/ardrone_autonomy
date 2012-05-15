
###########################################################################################
#
# Designed to build an application using generic.makefile, linked with Sdk
# ----------------------------------------------------------------------------------
# Author : aurelien.morelle@parrot.com
# Date   : 2007/05/16
#
###########################################################################################

include common.makefile

ifeq ($(USE_APP),yes)
  GENERIC_BINARIES_TARGET_DIR=$(APP_TARGET_DIR)
else
  GENERIC_BINARIES_TARGET_DIR=$(DLL_TARGET_DIR)
endif

# All that needs to be exported
########################
export GENERIC_ADD_OFILES
export GENERIC_INCLUDES
export GENERIC_LIB_PATHS
export GENERIC_LIBS
export GENERIC_BINARIES_LIBS_DEPS
export GENERIC_BINARIES_TARGET_DIR

# Bug fix ...
export GENERIC_LIBRARY_TARGET_DIR=$(GENERIC_BINARIES_TARGET_DIR)


all $(MAKECMDGOALS):
	@$(MAKE) -f generic.makefile $(MAKECMDGOALS)

