SILENT="YES"

ifeq ($(SILENT), "YES")
SILENT_MAKE=@
REDIRECT_STDOUT=> /dev/null
REDIRECT_STDERR=2> /dev/null
endif

RELEASE_BUILD=yes
ifeq ($(RELEASE_BUILD),yes)
RELEASE_OPT="release"
else
RELEASE_OPT="debug"
endif

FFMPEG_CONFIG=both


FFMPEG_DIR_EXIST=$(shell bash -c "if [ -d ffmpeg ]; then echo \"YES\"; else echo \"NO\"; fi")
FFMPEG_ARCHIVE=$(shell bash -c "if [ -e ffmpeg-0.8.tar.bz2 ]; then echo \"ffmpeg-0.8.tar.bz2\"; else echo \"no_ffmpeg_archive_found\"; fi")
FFMPEG_ARCHIVE_ROOTDIR=$(shell tar tjf $(FFMPEG_ARCHIVE) $(REDIRECT_STDERR) | head -n 1)

ifeq "$(MAKECMDGOALS)" ""
MAKECMDGOALS=host
endif

all: extract build
	$(SILENT_MAKE)echo "Build done."

extract:
ifneq "$(MAKECMDGOALS)" "clean"
ifeq ($(FFMPEG_DIR_EXIST),YES)
	$(SILENT_MAKE)echo "Libs already extracted"
else
	$(SILENT_MAKE)echo -n "Extracting libs ... "
	$(SILENT_MAKE)tar xjf $(FFMPEG_ARCHIVE)
ifneq ($(FFMPEG_ARCHIVE_ROOTDIR), ffmpeg/)
	$(SILENT_MAKE)mkdir ffmpeg $(REDIRECT_STDERR)
	$(SILENT_MAKE)mv $(FFMPEG_ARCHIVE_ROOTDIR)* ffmpeg/
	$(SILENT_MAKE)rm -rf $(FFMPEG_ARCHIVE_ROOTDIR)
endif
	$(SILENT_MAKE)tar xzf parrot_ffmpeg_build_utils.tar.gz
	$(SILENT_MAKE)echo "Done"
endif
endif

build:
ifneq "$(MAKECMDGOALS)" "clean"
	$(SILENT_MAKE)echo "Building target $(MAKECMDGOALS)"
	$(SILENT_MAKE)cd ffmpeg && ./autoConf.bash $(MAKECMDGOALS) $(RELEASE_OPT) $(FFMPEG_CONFIG) && cd - $(REDIRECT_STDOUT)
endif

clean:
ifeq ($(FFMPEG_DIR_EXIST),YES)
	$(SILENT_MAKE)cd ffmpeg && ./autoConf.bash clean release $(REDIRECT_STDERR) ; cd - $(REDIRECT_STDOUT)
endif
	$(SILENT_MAKE)rm -rf ffmpeg/
	$(SILENT_MAKE)rm -rf Includes/

ifneq "$(MAKECMDGOALS)" "clean"
ifneq "$(MAKECMDGOALS)" "extract"
ifneq "$(MAKECMDGOALS)" "build"
$(MAKECMDGOALS): all
endif
endif
endif
