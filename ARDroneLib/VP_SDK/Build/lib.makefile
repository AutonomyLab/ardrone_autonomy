
###########################################################################################
#
# Designed to build a library using generic.makefile, linked with Sdk
# ----------------------------------------------------------------------------------
# Author : aurelien.morelle@parrot.com
# Date   : 2007/05/16
#
###########################################################################################

include common.makefile

GENERIC_LIBRARY_TARGET_DIR=$(LIB_TARGET_DIR)
GENERIC_TARGET_LIBRARY:=$(GENERIC_LIBRARY_TARGET_DIR)/$(GENERIC_TARGET_LIBRARY)

export GENERIC_TARGET_LIBRARY
export GENERIC_LIBRARY_TARGET_DIR=$(LIB_TARGET_DIR)

# Bug fix ...
export GENERIC_BINARIES_TARGET_DIR=$(GENERIC_LIBRARY_TARGET_DIR)


all $(MAKECMDGOALS):
	@$(MAKE) -f generic.makefile $(MAKECMDGOALS)

