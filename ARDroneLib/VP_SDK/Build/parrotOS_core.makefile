
###########################################################################################
#
# Designed to build ParrotOS Core library using generic.makefile
# ----------------------------------------------------------------------------------
# Author : sylvain.gaeremynck@parrot.com
# Date   : 2009/06/09
#
###########################################################################################

include common.makefile


GENERIC_LIBRARY_TARGET_DIR=$(PARROTOS_CORE_TARGET_DIR)
GENERIC_TARGET_LIBRARY=$(GENERIC_LIBRARY_TARGET_DIR)/libparrotOS_core.a

# Check these defines against ParrotOs/core/Makefile.linux.global
GENERIC_CFLAGS+=-DPOS_LINUX
GENERIC_CFLAGS+=-D_XOPEN_SOURCE=600
GENERIC_CFLAGS+=-DSUP_U32_IS_ATOMIC

GENERIC_LIBRARY_SOURCE_FILES=			\
				posix/posix_alarm.c    \
            posix/posix_cond.c     \
            posix/posix_mutex.c    \
            posix/posix_sem.c      \
            posix/posix_thread.c   \
            posix/posix_main.c     \
            posix/posix_time.c     \
            posix/posix_sys.c      \
            generic/generic_flag.c \
            generic/generic_mbox2.c \
            generic/generic_diag.c \
            generic/generic_mbox.c


GENERIC_LIBRARY_SOURCE_DIR=$(PARROTOS_CORE_SOURCE_DIR)

# All that needs to be exported
########################
export GENERIC_LIBRARY_TARGET_DIR
export GENERIC_TARGET_LIBRARY
export GENERIC_INCLUDES
export GENERIC_LIBRARY_SOURCE_DIR
export GENERIC_LIBRARY_SOURCE_FILES

# All that shall not be defined
########################
export PARROTOS_MAKEFILE=
export GENERIC_BINARIES_SOURCE_ENTRYPOINTS=
export GENERIC_BINARIES_COMMON_SOURCE_FILES=


all $(MAKECMDGOALS): 
	@$(MAKE) -f generic.makefile $(MAKECMDGOALS)

