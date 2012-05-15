
###########################################################################################
#
# Generic makefile, used everywhere
# ----------------------------------------------------------------------------------
# Author : aurelien.morelle@parrot.com
# Date   : 2007/05/16
#
###########################################################################################

INTERNAL_LOG_FILE=build.log

ifdef PARROTOS_MAKEFILE
  TWEAKED_PARROTOS_MAKEFILE=$(GENERIC_LIBRARY_TARGET_DIR)/tweaked_parrotos.mak
  include $(TWEAKED_PARROTOS_MAKEFILE)
  GENERIC_LIBRARY_SOURCE_FILES=$(filter-out mDNSPosix/Responder.c,$(CFILES:P5_FFT.c=P5_FFT.s) $(SFILES) $(PSFILES))
  GENERIC_INCLUDES+=$(patsubst %,-I$(GENERIC_LIBRARY_SOURCE_DIR)/%,$(MODULES))
endif

ifeq ($(QUIET_BUILD),yes)
  Q=@
  SILENT=2>&1 > /dev/null
endif

ifeq ($(USE_NDS),yes)
  NDS_CPU_TYPE=$(shell echo $(NDS_CPU) | tr "A-Z" "a-z" | sed -e "s@[0-9][0-9]*@@")
  NDS_CPU_NUM=$(shell echo $(NDS_CPU) | sed -e "s@[a-zA-Z][a-zA-Z]*@@")
  INTERNAL_NDSTOOL:=$(Q)ndstool
  INTERNAL_DSBUILD:=$(Q)dsbuild
endif

INTERNAL_SOURCE_EXTENSIONS= .c .S .s .cpp

INTERNAL_MKDIR=mkdir -p
INTERNAL_ECHO=echo

# (in) GENERIC_COMMAND_PREFIX
INTERNAL_CC:=$(GENERIC_COMMAND_PREFIX)gcc
INTERNAL_AR:=$(GENERIC_COMMAND_PREFIX)ar
INTERNAL_OBJCOPY:=$(GENERIC_COMMAND_PREFIX)objcopy
INTERNAL_STRIP:=$(GENERIC_COMMAND_PREFIX)strip

# (in) GENERIC_CFLAGS
# (in) GENERIC_LDFLAGS
# (in) GENERIC_ARFLAGS

# (in) GENERIC_INCLUDES

# (in) GENERIC_ADD_OFILES : for linking with

# (in) GENERIC_LIBRARY_SOURCE_DIR
# (in) GENERIC_LIBRARY_SOURCE_FILES
# (in) GENERIC_LIBRARY_TARGET_DIR : for .o files
INTERNAL_LIBRARY_SOURCE_FILES:=$(patsubst %,$(GENERIC_LIBRARY_SOURCE_DIR)/%,$(GENERIC_LIBRARY_SOURCE_FILES))
INTERNAL_LIBRARY_TARGET_OFILES:=$(foreach ext,$(INTERNAL_SOURCE_EXTENSIONS),\
	$(patsubst $(GENERIC_LIBRARY_SOURCE_DIR)/%$(ext),$(GENERIC_LIBRARY_TARGET_DIR)/%.o,$(filter %$(ext),$(INTERNAL_LIBRARY_SOURCE_FILES))))

# (in) GENERIC_BINARIES_SOURCE_DIR
# (in) GENERIC_BINARIES_COMMON_SOURCE_FILES
# (in) GENERIC_BINARIES_SOURCE_ENTRYPOINTS
# (in) GENERIC_BINARIES_TARGET_DIR : for .o files
INTERNAL_BINARIES_COMMON_SOURCE_FILES:=$(patsubst %,$(GENERIC_BINARIES_SOURCE_DIR)/%,$(GENERIC_BINARIES_COMMON_SOURCE_FILES))
INTERNAL_BINARIES_COMMON_TARGET_OFILES:=$(foreach ext,$(INTERNAL_SOURCE_EXTENSIONS),\
	$(patsubst $(GENERIC_BINARIES_SOURCE_DIR)/%$(ext),$(GENERIC_BINARIES_TARGET_DIR)/%.o,$(filter %$(ext),$(INTERNAL_BINARIES_COMMON_SOURCE_FILES))))
INTERNAL_BINARIES_TARGET_OENTRYPOINTS:=$(foreach ext,$(INTERNAL_SOURCE_EXTENSIONS),\
	$(patsubst %$(ext),$(GENERIC_BINARIES_TARGET_DIR)/%.o,$(filter %$(ext),$(GENERIC_BINARIES_SOURCE_ENTRYPOINTS))))

# (in) GENERIC_TARGET_LIBRARY

# (in) GENERIC_TARGET_BINARIES_PREFIX : for binaries naming
# (in) GENERIC_TARGET_BINARIES_DIR : for binaries places
INTERNAL_TARGET_BINARIES:=$(foreach entrypoint,$(GENERIC_BINARIES_SOURCE_ENTRYPOINTS),$(shell basename $(entrypoint) .c))
INTERNAL_TARGET_BINARIES:=$(patsubst %.c,%,$(GENERIC_BINARIES_SOURCE_ENTRYPOINTS))
INTERNAL_TARGET_BINARIES:=$(patsubst %,$(GENERIC_TARGET_BINARIES_DIR)/$(GENERIC_TARGET_BINARIES_PREFIX)%,$(INTERNAL_TARGET_BINARIES))

_INTERNAL_LIBRARY_DEPFILES:=$(foreach ext,$(INTERNAL_SOURCE_EXTENSIONS),\
	$(patsubst $(GENERIC_LIBRARY_SOURCE_DIR)/%$(ext),$(GENERIC_LIBRARY_TARGET_DIR)/%$(ext).d,$(filter %$(ext),$(INTERNAL_LIBRARY_SOURCE_FILES))))
_INTERNAL_BINARIES_DEPFILES:=$(foreach ext,$(INTERNAL_SOURCE_EXTENSIONS),\
	$(patsubst $(GENERIC_BINARIES_SOURCE_DIR)/%$(ext),$(GENERIC_BINARIES_TARGET_DIR)/%$(ext).d,$(filter %$(ext),$(INTERNAL_BINARIES_COMMON_SOURCE_FILES))))
_INTERNAL_BINARIES_DEPFILES+=$(foreach ext,$(INTERNAL_SOURCE_EXTENSIONS),\
	$(patsubst %$(ext),$(GENERIC_BINARIES_TARGET_DIR)/%$(ext).d,$(filter %$(ext),$(GENERIC_BINARIES_SOURCE_ENTRYPOINTS))))

ifneq ($(MAKECMDGOALS),clean)
  INTERNAL_LIBRARY_DEPFILES:=$(_INTERNAL_LIBRARY_DEPFILES)
  INTERNAL_BINARIES_DEPFILES:=$(_INTERNAL_BINARIES_DEPFILES)
endif

ALL_TO_BE_DONE=						\
	$(INTERNAL_LIBRARY_DEPFILES)			\
	$(INTERNAL_BINARIES_DEPFILES)			\
	$(INTERNAL_LIBRARY_TARGET_OFILES)		\
	$(GENERIC_TARGET_LIBRARY)			\
	$(INTERNAL_BINARIES_COMMON_TARGET_OFILES)	\
	$(INTERNAL_BINARIES_TARGET_OENTRYPOINTS)	\
	$(INTERNAL_TARGET_BINARIES)

ifeq ($(GENERIC_TARGET_LIBRARY),)
ifeq ($(USE_DLL),yes)
  ALL_TO_BE_DONE+=$(GENERIC_TARGET_BINARIES_DIR)/$(GENERIC_TARGET_BINARIES_PREFIX)$(DLL_ID).dll
endif
endif

CREATE_TARGET_DIRECTORY=$(INTERNAL_MKDIR) $(@D)

all: clean_log $(ALL_TO_BE_DONE)

clean_log:
	@rm -f $(GENERIC_BINARIES_TARGET_DIR)/$(INTERNAL_LOG_FILE) $(GENERIC_LIBRARY_TARGET_DIR)/$(INTERNAL_LOG_FILE)

check:
	@(cd $(GENERIC_LIBRARY_SOURCE_DIR) ; $(SDK_SOURCE_DIR)/Build/cvsstatus.sh $(SDK_SOURCE_DIR)/Build ADD)

clean:
	$(RM) $(ALL_TO_BE_DONE) $(_INTERNAL_LIBRARY_DEPFILES) $(_INTERNAL_BINARIES_DEPFILES)

update:

$(TWEAKED_PARROTOS_MAKEFILE): $(PARROTOS_MAKEFILE)
	@$(CREATE_TARGET_DIRECTORY)
	@echo "all:" >> $(TWEAKED_PARROTOS_MAKEFILE)
	@cat $(PARROTOS_MAKEFILE) | grep -v "^[-]*include" | grep -v "^all:" | sed -e "s@^\([A-Z]*[=]*\)\([ \t]*\)\(/\)@\1\2@" | sed -e "s@^clean:@NOCLEAN:@" >> $(TWEAKED_PARROTOS_MAKEFILE)

ifneq ("$(INTERNAL_LIBRARY_TARGET_OFILES)","")
$(GENERIC_TARGET_LIBRARY): $(INTERNAL_LIBRARY_TARGET_OFILES)
	@$(CREATE_TARGET_DIRECTORY)
	@$(INTERNAL_ECHO) "ar $(GENERIC_ARFLAGS)" $(subst $(GENERIC_LIBRARY_TARGET_DIR)/,,$@ $^)
	@rm -f $@
	@$(INTERNAL_AR) $(GENERIC_ARFLAGS) $@ $^
endif

ifeq ($(USE_DLL),yes)
$(GENERIC_TARGET_BINARIES_DIR)/$(GENERIC_TARGET_BINARIES_PREFIX)$(DLL_ID).dll: $(INTERNAL_BINARIES_COMMON_TARGET_OFILES) $(GENERIC_ADD_OFILES) $(GENERIC_BINARIES_LIBS_DEPS)
else
$(GENERIC_TARGET_BINARIES_DIR)/$(GENERIC_TARGET_BINARIES_PREFIX)%: $(GENERIC_BINARIES_TARGET_DIR)/%.o $(INTERNAL_BINARIES_COMMON_TARGET_OFILES) $(GENERIC_ADD_OFILES) $(GENERIC_BINARIES_LIBS_DEPS)
endif
	@$(CREATE_TARGET_DIRECTORY)
  ifeq ($(QUIET_BUILD),yes)
  ifeq ($(USE_NDS),yes)
	@$(INTERNAL_ECHO) "dsbuild $(GENERIC_TARGET_BINARIES_PREFIX)$*.nds"
  else
    ifeq ($(USE_DLL),yes)
	@$(INTERNAL_ECHO) "ld $(GENERIC_TARGET_BINARIES_PREFIX)$(DLL_ID).dll"
    else
	@$(INTERNAL_ECHO) "ld $(GENERIC_TARGET_BINARIES_PREFIX)$*"
    endif
  endif
  endif
  ifeq ($(USE_DLL),yes)
	$(GENERIC_COMMAND_PREFIX)dlltool -e $(GENERIC_TARGET_BINARIES_DIR)/$(DLL_ID)_exports.o --export-all-symbols -l $(@:.dll=.lib) $(INTERNAL_BINARIES_COMMON_TARGET_OFILES)
	$(INTERNAL_CC) --shared -o $@ $(INTERNAL_BINARIES_COMMON_TARGET_OFILES) $(GENERIC_TARGET_BINARIES_DIR)/$(DLL_ID)_exports.o $(GENERIC_ADD_OFILES) $(GENERIC_LIB_PATHS) $(GENERIC_LIBS) $(GENERIC_LDFLAGS) $(LDFLAGS_$(subst /,_,$*))
	$(RM) $(GENERIC_TARGET_BINARIES_DIR)/$(DLL_ID)_exports.o
  else
	$(INTERNAL_CC) -o $@ $(GENERIC_BINARIES_TARGET_DIR)/$*.o $(INTERNAL_BINARIES_COMMON_TARGET_OFILES) $(GENERIC_ADD_OFILES) $(GENERIC_LIB_PATHS) $(GENERIC_LIBS) $(GENERIC_LDFLAGS) $(LDFLAGS_$(subst /,_,$*))
  endif
  ifeq ($(RELEASE_BUILD),yes)
	cp $@ $(shell dirname $@)/sym_$(shell basename $@)
	$(INTERNAL_STRIP) $@
  endif
  ifeq ($(USE_NDS),yes)
	$(INTERNAL_OBJCOPY) -O binary $@ $@.$(NDS_CPU_TYPE)$(NDS_CPU_NUM)
	$(INTERNAL_NDSTOOL) -c $@.nds -$(NDS_CPU_NUM) $@.$(NDS_CPU_TYPE)$(NDS_CPU_NUM) $(SILENT)
	$(INTERNAL_DSBUILD) $@.nds $(SILENT)
  endif

# Template build rules
# first param  $(1) : rule type (LIBRARY or BINARIES)
# second param $(2) : source extension (.c for example)
define BUILD_OFILE_TEMPLATE
  $$(GENERIC_$(1)_TARGET_DIR)/%$(2).d: $$(GENERIC_$(1)_SOURCE_DIR)/%$(2)
	@$$(CREATE_TARGET_DIRECTORY)
  ifeq ($$(QUIET_BUILD),yes)
	@$$(INTERNAL_ECHO) "dep $$*$(2)"
  endif
	$$(INTERNAL_CC) -M $$(filter-out -Wall,$$(GENERIC_CFLAGS)) -w $$(CFLAGS_$$(subst /,_,$$*)) $$(GENERIC_INCLUDES) $$(GENERIC_$(1)_SOURCE_DIR)/$$*$(2) > $$@
ifeq ($(USE_IPHONE),yes)
   @sed -ie 's,\(.*\.o\)\([ :]*\),\1 $$@\2,g' $$@
else
	@sed -i 's,\(.*\.o\)\([ :]*\),\1 $$@\2,g' $$@	
endif
  $$(GENERIC_$(1)_TARGET_DIR)/%.o: $$(GENERIC_$(1)_TARGET_DIR)/%$(2).d
	@$$(CREATE_TARGET_DIRECTORY)
  ifeq ($$(QUIET_BUILD),yes)
	@$$(INTERNAL_ECHO) "cc $$*$(2)"
  endif
	@echo -E $$(INTERNAL_CC) $$(GENERIC_CFLAGS) $$(CFLAGS_$$(subst /,_,$$*)) $$(GENERIC_INCLUDES) -c -o $$@ $$(GENERIC_$(1)_SOURCE_DIR)/$$*$(2) >> $$(GENERIC_$(1)_TARGET_DIR)/$(INTERNAL_LOG_FILE)
	$$(INTERNAL_CC) $$(GENERIC_CFLAGS) $$(CFLAGS_$$(subst /,_,$$*)) $$(GENERIC_INCLUDES) -c -o $$@ $$(GENERIC_$(1)_SOURCE_DIR)/$$*$(2) 2>&1 >> $$(GENERIC_$(1)_TARGET_DIR)/$(INTERNAL_LOG_FILE)
  ifeq ("$$(USE_MEMPROT)","yes")
  ifdef GENERIC_DOMAIN
    ifeq ($$(QUIET_BUILD),yes)
	@$$(INTERNAL_ECHO) "put_in_domain $$(GENERIC_DOMAIN) $$*.o"
	@$$(PUT_IN_DOMAIN) $$(GENERIC_DOMAIN) $$@
      else
	$$(PUT_IN_DOMAIN) $$(GENERIC_DOMAIN) $$@
      endif
  else
    ifeq ($$(QUIET_BUILD),yes)
	@if [ ! -z $$(DOMAIN_$$(subst /,_,$$*)) ]; then					\
		$$(INTERNAL_ECHO) "put_in_domain $$(DOMAIN_$$(subst /,_,$$*)) $$*.o";	\
		$$(PUT_IN_DOMAIN) $$(DOMAIN_$$(subst /,_,$$*)) $$@;			\
	fi
    else
	@if [ ! -z $$(DOMAIN_$$(subst /,_,$$*)) ]; then					\
		$$(INTERNAL_ECHO) $$(PUT_IN_DOMAIN) $$(DOMAIN_$$(subst /,_,$$*)) $$@;	\
		$$(PUT_IN_DOMAIN) $$(DOMAIN_$$(subst /,_,$$*)) $$@;			\
	fi
    endif
  endif
  endif
endef

# Build rules for each extension
$(foreach ext,$(INTERNAL_SOURCE_EXTENSIONS),$(eval $(call BUILD_OFILE_TEMPLATE,LIBRARY,$(ext))))
$(foreach ext,$(INTERNAL_SOURCE_EXTENSIONS),$(eval $(call BUILD_OFILE_TEMPLATE,BINARIES,$(ext))))

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(MAKECMDGOALS),check)
  -include $(INTERNAL_LIBRARY_DEPFILES) $(INTERNAL_BINARIES_DEPFILES)
endif
endif

