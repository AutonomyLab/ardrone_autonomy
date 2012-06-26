
###########################################################################################
#
# Designed to build VLIB using generic.makefile
# ----------------------------------------------------------------------------------
# Author : aurelien.morelle@parrot.com
# Date   : 2008/10/30
#
###########################################################################################

include common.makefile

GENERIC_LIBRARY_TARGET_DIR=$(CODEC_TARGET_DIR)
GENERIC_TARGET_LIBRARY=$(GENERIC_LIBRARY_TARGET_DIR)/libvlib.a

GENERIC_LIBRARY_SOURCE_FILES=			\
	video_codec.c				\
	video_controller.c			\
	video_mem32.c				\
	video_dct.c				\
	video_huffman.c				\
	video_macroblock.c			\
	video_packetizer.c			\
	video_picture.c				\
	video_quantizer.c			\
	P263/p263_codec.c			\
	P263/p263_huffman.c			\
	P263/p263_mb_layer.c			\
	P263/p263_gob_layer.c			\
	P263/p263_picture_layer.c		\
  P264/p264_zigzag.c		\
	P264/p264_transform.c		\
	P264/p264_Qp.c		\
	P264/p264_intra_pred.c		\
  P264/p264_inter_mc.c		\
	P264/p264_merge.c		\
	P264/p264_codec.c		\
	P264/p264.c				\
	P264/p264_gob_layer.c			\
	P264/p264_mb_layer.c			\
	P264/p264_picture_layer.c  \
	P264/video_p264.c  \
	Stages/vlib_stage_decode.c		\
	Stages/vlib_stage_encode.c		\
	UVLC/uvlc_codec.c			\
	UVLC/uvlc.c				\
	UVLC/uvlc_gob_layer.c			\
	UVLC/uvlc_mb_layer.c			\
	UVLC/uvlc_picture_layer.c

ifeq ($(USE_ELINUX),yes)
GENERIC_LIBRARY_SOURCE_FILES+=			\
	Platform/arm9_P6/video_utils.c		\
	Platform/arm9_P6/video_dct_p6.c		\
	Platform/arm9_P6/video_p264_p6.c		\
	Platform/arm9_P6/video_quantizer_p6.c	\
	Platform/arm9_P6/video_packetizer_p6.S	\
	Platform/arm9_P6/UVLC/uvlc_p6.S		\
	Platform/arm9_P6/UVLC/uvlc_mb_layer_p6.S
endif

ifeq ($(USE_IPHONE),yes)
  ifeq ($(PLATFORM_NAME),iphoneos)
	GENERIC_LIBRARY_SOURCE_FILES+=			\
		Platform/arm11/video_utils.c		\
		Platform/arm11/UVLC/uvlc_codec.c
  else
	GENERIC_LIBRARY_SOURCE_FILES+=			\
		Platform/x86/video_utils.c		\
		Platform/x86/UVLC/uvlc_codec.c
  endif
else
   ifeq ($(USE_ANDROID),yes)
	GENERIC_LIBRARY_SOURCE_FILES+=			\
		Platform/arm11/video_utils.c		\
		Platform/arm11/UVLC/uvlc_codec.c
   else
      ifeq ($(FF_ARCH),Intel)
	     GENERIC_LIBRARY_SOURCE_FILES+=			\
		   Platform/x86/video_utils.c		\
		   Platform/x86/UVLC/uvlc_codec.c
      endif
   endif
endif

GENERIC_LIBRARY_SOURCE_DIR=$(VLIB_SOURCE_DIR)


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

