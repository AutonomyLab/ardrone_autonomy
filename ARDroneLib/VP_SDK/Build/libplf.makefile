
###########################################################################################
#
# Designed to build plf library using generic.makefile
# ----------------------------------------------------------------------------------
# Author : sylvain.gaeremynck@parrot.com
# Date   : 2009/06/09
#
###########################################################################################

include common.makefile


GENERIC_LIBRARY_TARGET_DIR=$(LIBPLF_TARGET_DIR)
GENERIC_TARGET_LIBRARY=$(GENERIC_LIBRARY_TARGET_DIR)/libplf.a

GENERIC_LIBRARY_SOURCE_FILES=			\
			lib_plf.c

GENERIC_LIBRARY_SOURCE_DIR=$(LIBPLF_SOURCE_DIR)

# All that needs to be exported
########################
export GENERIC_LIBRARY_TARGET_DIR
export GENERIC_TARGET_LIBRARY
export GENERIC_INCLUDES
export GENERIC_LIBRARY_SOURCE_DIR
export GENERIC_LIBRARY_SOURCE_FILES

# All that shall not be defined
########################
export GENERIC_BINARIES_SOURCE_ENTRYPOINTS=
export GENERIC_BINARIES_COMMON_SOURCE_FILES=


all $(MAKECMDGOALS):
	@$(MAKE) -f generic.makefile $(MAKECMDGOALS)

