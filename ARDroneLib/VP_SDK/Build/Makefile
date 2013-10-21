
###########################################################################################
#
# Makefile called externally to build the SDK, alone or to be used in an application.
# ----------------------------------------------------------------------------------
# Author : aurelien.morelle@parrot.com
# Date   : 2007/05/16
#
# Builds libraries and examples.
#
# Needs some variables to be passed for it to know what to do.
# Variables definitions tests are done in varcontrol.makefile
#
###########################################################################################


                               ##############################                              
###########################################################################################
# Variables definitions management
###########################################################################################
include varcontrol.makefile
###########################################################################################
                               ##############################                              


#
# At that point, following global variables are defined :
# a : always / c : conditional

#	(a) NO_COM
#	(a) USE_BLUEZ
#	(a) USE_SDK
#	(a) RELEASE_BUILD

#	(c) SDK_VERSION
#	(c) PROJECT
#	(c) FF_ARCH


                               ##############################                              
###########################################################################################
# System variable definitions
###########################################################################################
include sysvar.makefile
###########################################################################################
                               ##############################                              


#
# At that point, following global variables are defined :
# a : always / c : conditional

#	(a) OS_TARGET_DIR
#	(a) VLIB_TARGET_DIR
#	(c) COM_TARGET_DIR
#	(c) CODEC_TARGET_DIR
#	(c) JPEG_TARGET_DIR
#	(c) SDK_TARGET_DIR


###########################################################################################
# Main rules
###########################################################################################


# If we are building a library, just build this library.
# If we are building an application that depends on some Parrot libraries (USE_xxx=yes), then build these dependencies too.
ifeq ($(USE_LIB),yes)
    TO_BUILD+=build_lib
endif

ifeq ($(USE_APP),yes)
    TO_BUILD+=build_app
endif

.PHONY: $(foreach target,$(TARGETS),build_$(target) clean_$(target)) 

all $(MAKECMDGOALS): $(TO_BUILD)
	@if [ $(MAKECMDGOALS) ] && [ $(MAKECMDGOALS) = check ] ; then $(SDK_SOURCE_DIR)/Build/cvsstatus.sh $(SDK_SOURCE_DIR)/Build END ; fi

before_check:
	@if [ $(MAKECMDGOALS) ] && [ $(MAKECMDGOALS) = check ] ; then rm -rf *.log ; fi


#------------------------------------------------------------------------------------------------------------------------
TARGETS=vlib libplf sdk lib app  

# When building on Ubuntu, only the final application (USE_APP) depends on the above-mentioned libraries.
# The iPhone applications do not use USE_APP; the above-mentioned libraries must therefore be built
#  when the iPhone makefile builds itself.

define ADD_RULE_TEMPLATE
  ifneq ($(1),app)
  ifeq ($$(USE_$(shell echo $(1) | tr "a-z" "A-Z")),yes)
     build_app: build_$(1)
  endif
  endif

  ifneq ($(1),app)
  ifneq ($(1),lib)
  ifeq ($$(USE_$(shell echo $(1) | tr "a-z" "A-Z")),yes)
  ifeq ($(IPHONE_MODE),yes)
     build_lib: build_$(1)
  endif
  ifeq ($(USE_ANDROID),yes)
     build_lib: build_$(1)
  endif
  endif
  endif
  endif
endef

# Make 'app' dependant on all above-mentioned 'TARGETS'
$(foreach target,$(TARGETS),$(eval $(call ADD_RULE_TEMPLATE,$(target))))
#------------------------------------------------------------------------------------------------------------------------



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

##
## To prevent conflicts during parallel builds, locks are used when building standard blocks like AR.Drone SDK, VLib, etc.
## This is necessary because all libraries linked against these blocks trigger the blocks Makefile
## For example, building two independant libraries which both use the SDK would trigger the SDK makefile twice at the same time.
##
## The targets 'lib' and 'app' used to build a new library or application are considered parallel-safe.
## The master Makefile is supposed to correctly handle dependencies between these libraries/applications.
##
## 'setlock' is standard on Ubuntu, not on MacOSX


define GENERIC_RULES_TEMPLATE
  ifneq ($(1),dll)
    build_$(1): before_check
#		@echo "\nCalling mkfile for $(1)..."
   ifeq ("$$(VPSDK_PARALLEL_BUILD)","yes")
		@setlock /tmp/$(1).lock $$(MAKE) -f $(1).makefile $$(MAKECMDGOALS)
   else
		@$$(MAKE) -f $(1).makefile $$(MAKECMDGOALS)
   endif
#		@echo "...$(1) done.\n"
  endif
endef

build_dll: before_check
	@$(MAKE) -f app.makefile $(MAKECMDGOALS)


# Build rules for each target
$(foreach target,$(TARGETS),$(eval $(call GENERIC_RULES_TEMPLATE,$(target))))

###########################################################################################
                               ##############################                              


                               ##############################                              
###########################################################################################
# Update generic flags
###########################################################################################
FLAGS_FILTER_OUT=-O0 -O1 -O2 -O3 -Os -fno-rtti -frtti -fvtable-gc -Winline -Woverloaded-virtual -lstdc++
ifneq ($(USE_RTMON),yes)
  FLAGS_FILTER_OUT+=-g
endif

DEFINE_FLAGS=USE_WIFI USE_CAMIF USE_BASEBAND NO_COM USE_SERIAL USE_VLIB USE_MINGW32 USE_DLL USE_LINUX USE_ELINUX USE_PARROTOS_CORE USE_PARROTOS_UTILS USE_PARROTOS_DRIVERS USE_PARROTOS_DEVS USE_PARROTOS_CODEC USE_LIBPLF USE_BROADCOM USE_IWLIB USE_ANDROID FFMPEG_SUPPORT ITTIAM_SUPPORT USE_VIDEO_TCP USE_VIDEO_HD


ifeq ($(USE_ELINUX),yes)
   include elinux.makefile
else
   GENERIC_CFLAGS+= -Wall
endif

ifeq ($(USE_LIBPLF),yes)
   GENERIC_CFLAGS+= -DOS_LINUX
endif

GENERIC_CFLAGS+=-Wall 
#-Werror -Wfatal-errors -Wno-unused-variable

GENERIC_LDFLAGS+=-Wl,--warn-common

ifeq ("$(BUILD_MODE)","PROD_MODE")
  GENERIC_CFLAGS+=-O3 -DCOMPIL_MODE_PROD
  GENERIC_CFLAGS:=$(filter-out -DNDEBUG,$(GENERIC_CFLAGS)) -DNDEBUG
  GENERIC_LDFLAGS+=-O3
  ifeq ($(USE_RTMON),yes)
    GENERIC_CFLAGS+=-g
    GENERIC_LDFLAGS+=-g
  endif
endif
ifeq ("$(BUILD_MODE)","DEBUG_MODE")
  GENERIC_CFLAGS+=-g -O0 -DDEBUG_MODE -DDEBUG -D_DEBUG
  GENERIC_LDFLAGS+=-g -O0
endif

ifeq ($(USE_NDS),yes)
    GENERIC_CFLAGS+=-D__NDS__ -DUSE_NDS
    ifeq ($(NDS_CPU),ARM7)
      GENERIC_CFLAGS+=-DARM7 -DSDK_ARM7 -mcpu=arm7tdmi -mtune=arm7tdmi
      GENERIC_LDFLAGS+=-specs=ds_arm7.specs
    endif
    ifeq ($(NDS_CPU),ARM9)
      GENERIC_CFLAGS+=-DARM9 -DSDK_ARM9 -mtune=arm9tdmi -march=armv5te
      GENERIC_LDFLAGS+=-specs=ds_arm9.specs
    endif
    GENERIC_CFLAGS+=-fomit-frame-pointer -ffast-math
    GENERIC_LDFLAGS+=-mno-fpu -Wl,-Map,.map
else
    ifeq ($(USE_ELINUX),yes)    
      GENERIC_CFLAGS+=-D_ELINUX=$(USE_ELINUX) -D__elinux__ -D__ELINUX__ -DPARROT5
      ifeq ($(PROCESSOR),ARM_CORTEX_A8)
## Check http://processors.wiki.ti.com/index.php/Cortex-A8	
	GENERIC_CFLAGS+=-march=armv7-a -mtune=cortex-a8 -mfpu=neon -ftree-vectorize -mfloat-abi=softfp
      endif
      ifeq ($(PROCESSOR),ARM_926EJS)
## Check http://www.codesourcery.com/sgpp/lite/arm/portal/kbentry26
	GENERIC_CFLAGS+=-mcpu=arm926ej-s
      endif
    else
	     ifneq ($(USE_MINGW32),yes)
		     GENERIC_CFLAGS+=-D__linux__

           ifeq ($(USE_IPHONE),yes)
		        GENERIC_CFLAGS+=-D__MACOSX__ -std=gnu99
	            GENERIC_CFLAGS+=-arch $(PLATFORM_PREFERRED_ARCH)
		     else  
		        ifeq ($(USE_LINUX), yes)        
			       GENERIC_CFLAGS+=-D__LINUX__
              else
                 ifeq ($(USE_ANDROID),yes)
						  TARGET_arm_release_CFLAGS :=  -O2 \
								-fomit-frame-pointer \
								-fstrict-aliasing    \
								-funswitch-loops     \
								-finline-limit=300

						  TARGET_arm_debug_CFLAGS := $(TARGET_arm_release_CFLAGS) \
								-fno-omit-frame-pointer \
								-fno-strict-aliasing

						  GENERIC_CFLAGS+= $(TARGET_arm_debug_CFLAGS) \
								 -fstack-protector \
								 -fno-short-enums \
								 -D__ARM_ARCH_5__ -D__ARM_ARCH_5T__ \
								 -D__ARM_ARCH_5E__ -D__ARM_ARCH_5TE__
                 endif
		        endif
	        endif
        endif
	   endif
endif

GENERIC_CFLAGS+=-DTARGET_CPU_ARM=$(TARGET_CPU_ARM) -DTARGET_CPU_X86=$(TARGET_CPU_X86)

ifeq ("$(USE_MEMPROT)","yes")
  GENERIC_CFLAGS+=-fno-common -mlong-calls
endif

ifeq ("$(USE_PVSP)","yes")
  GENERIC_CFLAGS+=-DBLUES_SUPPORT_PVSP_BLOCK
endif

ifeq ("$(USE_BONJOUR)","yes")
  GENERIC_CFLAGS+=				\
	-DMDNS_DEBUGMSGS=0			\
	-DMDNS_LOG_ANSWER_SUPPRESSION_TIMES=0	\
	-DCOMPILER_LIKES_PRAGMA_MARK=0		\
	-DHAVE_IPV6=0				\
	-DUSES_NETLINK=0			\
	-DTARGET_OS_LINUX=1			\
	-DNOT_HAVE_DAEMON=1			\
	-Dsigset_t=cyg_uint32
endif

define ADD_DEFINE_FLAG_TEMPLATE
  ifeq ($$($(1)),yes)
    GENERIC_CFLAGS+=-D$(1)
  endif
endef

# Add define flags
$(foreach flag,$(DEFINE_FLAGS),$(eval $(call ADD_DEFINE_FLAG_TEMPLATE,$(flag))))

# All that needs to be exported
########################
export GENERIC_CFLAGS
export GENERIC_LDFLAGS
export GENERIC_ARFLAGS=rcs
###########################################################################################
                               ##############################                              

