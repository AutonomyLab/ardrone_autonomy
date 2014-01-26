//
//  ittiam_stage_decode.c
//  ARDroneEngine
//
//  Created by Mykonos on 08/07/11.
//  Copyright 2011 PARROT SA. All rights reserved.
//

#ifdef ITTIAM_SUPPORT

#include <VP_Os/vp_os_malloc.h>
#include <VP_Os/vp_os_print.h>
#include <ardrone_tool/Video/video_stage_ittiam_decoder.h>
#include <Maths/time.h>
#include <math.h>
#include <sys/time.h>
#include <video_encapsulation.h>
#include <unistd.h>
#include <fcntl.h>

#define H264_DEC
#define MPEG4_DEC

static parrot_video_encapsulation_t current_PaVE, previous_PaVE;
static uint32_t old_num_frame = 0;

#ifdef H264_DEC
/* H264 CONFIGURATION */

#define H264_MAX_LEVEL_SUPPORTED 			42
#define H264_MAX_FRAME_WIDTH				1920
#define H264_MAX_FRAME_HEIGHT				1088
#define H264_MAX_REF_FRAMES                 2
#define H264_MAX_REORDER_FRAMES				0

#define H264_MIN_FRAME_WIDTH				64
#define H264_MIN_FRAME_HEIGHT				64



iv_obj_t * H264_DECHDL = NULL;
it_mem_t * h264_ps_it_mem;
iv_mem_rec_t * h264_mem_rec;
ivd_video_decode_ip_t h264_video_decode_ip;
ivd_video_decode_op_t h264_video_decode_op;

ivd_get_display_frame_ip_t h264_get_display_frame_ip;
ivd_get_display_frame_op_t h264_get_display_frame_op;

#endif

#ifdef MPEG4_DEC
/* MPEG4 CONFIGURATION */

iv_obj_t * MPEG4_DECHDL = NULL;
it_mem_t * mpeg4_ps_it_mem;
iv_mem_rec_t * mpeg4_mem_rec;
ivd_video_decode_ip_t mpeg4_video_decode_ip;
ivd_video_decode_op_t mpeg4_video_decode_op;

ivd_get_display_frame_ip_t mpeg4_get_display_frame_ip;
ivd_get_display_frame_op_t mpeg4_get_display_frame_op;
#endif


/**
 * Enable (1) to display FPS
 */
#define DISPLAY_FPS (0)
/**
 * Display FPS after N frames
 * - default : 20
 */
#define DISPLAY_FPS_FRAMES (20)

/**
 * Enable (1) to display missed/dropped frames
 */
#define DISPLAY_DROPPED_FRAMES (0)

/**
 * Enable (1) to allow decoding of non-PaVE MPEG4/h264 frames
 * Disable (0) for compatibility with VLIB/P264 (won't try to decode if no PaVE is found)
 */
#define FAKE_PaVE (0)
/**
 * Default codec for non-PaVE frames :
 * (0) - h264
 * (1) - MPEG4
 */
#define FAKE_PaVE_CODEC_ID (0)

/**
 * Wait for I-Frame mode :
 * - (0) : display each received frame
 * - (1) : when a frame is lost, wait for next I-Frame
 */
#define WAIT_FOR_I_FRAME (1)

/**
 * Use UDP/TCP connexion :
 * - (0) UDP : we get one frame per buffer
 * - (1) TCP : we get each frame in multiple buffers
 */
//#define VIDEO_USES_TCP (0) // NOT FUNCTIONNAL !

/* END OF CONFIGURATION */


#if DISPLAY_FPS
#define NUM_SAMPLES DISPLAY_FPS_FRAMES
#endif

#if FAKE_PaVE_CODEC_ID == 0
#define FAKE_PaVE_CODEC CODEC_MPEG4_AVC
#elif FAKE_PaVE_CODEC_ID == 1
#define FAKE_PaVE_CODEC CODEC_MPEG4_VISUAL
#else
#warning FAKE_PaVE_CODEC_ID : unknown value -> falling back to MPEG4
#define FAKE_PaVE_CODEC CODEC_MPEG4_VISUAL
#endif


#define ITTIAM_DEBUG_ENABLED (0)
#if ITTIAM_DEBUG_ENABLED
#define ITTIAM_DEBUG_PRINT(...)       \
    do{                           \
        printf ("ITTIAM-DEBUG (%s @ %d) : ", __FUNCTION__, __LINE__); \
        printf (__VA_ARGS__);   \
        printf ("\n");          \
    } while (0)
#else
#define ITTIAM_DEBUG_PRINT(...)
#endif



#if DISPLAY_DROPPED_FRAMES
int missed_frames = 0;
int dropped_frames = 0;
int previous_ok_frame = 0;
#endif

const vp_api_stage_funcs_t ittiam_decoding_funcs = {
    (vp_api_stage_handle_msg_t) NULL,
    (vp_api_stage_open_t) ittiam_stage_decoding_open,
    (vp_api_stage_transform_t) ittiam_stage_decoding_transform,
    (vp_api_stage_close_t) ittiam_stage_decoding_close
};

void ITTIAM_dumpPave(parrot_video_encapsulation_t *PaVE) {
    printf("Codec : %s\n", (PaVE->video_codec == CODEC_MPEG4_VISUAL) ? "MP4" : ((PaVE->video_codec == CODEC_MPEG4_AVC) ? "H264" : "Unknown"));
    printf("Header 1 : %d\n",PaVE->header1_size);
    printf("Header 2 : %d\n",PaVE->header2_size);
    printf("Encoded dims : %d x %d\n", PaVE->encoded_stream_width, PaVE->encoded_stream_height);
    printf("Display dims : %d x %d\n", PaVE->display_width, PaVE->display_height);
    printf("Header size  : %d (PaVE size : %lu)\n", PaVE->header_size, sizeof (parrot_video_encapsulation_t));
    printf("Payload size : %d\n", PaVE->payload_size);
    printf("Frame Type / Number : %s : %d\n", (PaVE->frame_type == FRAME_TYPE_P_FRAME) ? "P-Frame" : ((PaVE->frame_type == FRAME_TYPE_I_FRAME) ? "I-Frame" : "IDR-Frame"), PaVE->frame_number);
}

static inline bool_t check_and_copy_PaVE(parrot_video_encapsulation_t *PaVE, vp_api_io_data_t *data, parrot_video_encapsulation_t *prevPaVE, bool_t *dimChanged) {
    parrot_video_encapsulation_t *localPaVE = (parrot_video_encapsulation_t *) data->buffers[data->indexBuffer];
    if (localPaVE->signature[0] == 'P' &&
            localPaVE->signature[1] == 'a' &&
            localPaVE->signature[2] == 'V' &&
            localPaVE->signature[3] == 'E') {
        //ITTIAM_DEBUG_PRINT("Found a PaVE");
        vp_os_memcpy(prevPaVE, PaVE, sizeof (parrot_video_encapsulation_t)); // Make a backup of previous PaVE so we can check if things have changed
        vp_os_memcpy(PaVE, localPaVE, sizeof (parrot_video_encapsulation_t)); // Copy PaVE to our local one

#if ITTIAM_DEBUG_ENABLED
        //printf ("PREV : ");
        //ITTIAM_dumpPave (prevPaVE);
        printf("CURR : ");
        ITTIAM_dumpPave(PaVE);
#endif

        if (prevPaVE->encoded_stream_width != PaVE->encoded_stream_width ||
                prevPaVE->encoded_stream_height != PaVE->encoded_stream_height ||
                prevPaVE->display_width != PaVE->display_width ||
                prevPaVE->display_width != PaVE->display_width) {
            *dimChanged = TRUE;
        } else {
            *dimChanged = FALSE;
        }

        data->size = localPaVE->payload_size;
        memmove(data->buffers[data->indexBuffer], &(data->buffers[data->indexBuffer])[localPaVE->header_size], data->size);

#if DISPLAY_DROPPED_FRAMES
        missed_frames += PaVE->frame_number - prevPaVE->frame_number - 1;
#endif

        return TRUE;
    } else {
        ITTIAM_DEBUG_PRINT("No PaVE, signature was [%c][%c][%c][%c]", localPaVE->signature[0]
                , localPaVE->signature[1]
                , localPaVE->signature[2]
                , localPaVE->signature[3]);
#if FAKE_PaVE
        PaVE->encoded_stream_width = 1280; //640;
        PaVE->encoded_stream_height = 720; //368;
        PaVE->display_width = 1280; //640;
        PaVE->display_height = 720; //360;
        PaVE->video_codec = FAKE_PaVE_CODEC;
        PaVE->frame_type = FRAME_TYPE_I_FRAME;
        vp_os_memcpy(prevPaVE, PaVE, sizeof (parrot_video_encapsulation_t));
        *dimChanged = FALSE;
        return TRUE;
#else
        return FALSE;
#endif
    }
}

C_RESULT ittiam_stage_decoding_open(ittiam_stage_decoding_config_t *cfg) {
    ITTIAM_DEBUG_PRINT("ITTIAM OPEN");
    return C_OK;
}

C_RESULT ittiam_stage_decoding_transform(ittiam_stage_decoding_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out) {
    bool_t is_frame_dim_changed = FALSE;
    UWORD32 i;

#if WAIT_FOR_I_FRAME
    static bool_t waitForIFrame = TRUE;
#endif

#ifdef NUM_SAMPLES
    static struct timeval start_time, start_time2;
    static int numsamples = 0;
#endif	

    check_and_copy_PaVE(&current_PaVE, in, &previous_PaVE, &is_frame_dim_changed);

    if (out->status == VP_API_STATUS_INIT && (current_PaVE.video_codec == CODEC_MPEG4_AVC || current_PaVE.video_codec == CODEC_MPEG4_VISUAL)) {
        if (current_PaVE.video_codec == CODEC_MPEG4_AVC) {
#ifdef H264_DEC
            //////////////////// H264 INIT SECTION //////////////////////////////////////////////////////////////////////////////////////

            /****************************************************************************/
            /* H264 ====== Initialize the memory records
             *****************************************************************************/
            cfg->num_picture_decoded = old_num_frame;
            ITTIAM_DEBUG_PRINT("ITTIAM INIT");

            iv_num_mem_rec_ip_t h264_num_mem_rec_ip;
            iv_num_mem_rec_op_t h264_num_mem_rec_op;

            h264_num_mem_rec_ip.e_cmd = IV_CMD_GET_NUM_MEM_REC;
            h264_num_mem_rec_ip.u4_size = sizeof (iv_num_mem_rec_ip_t);
            h264_num_mem_rec_op.u4_size = sizeof (iv_num_mem_rec_op_t);

            if (ih264d_cxa8_api_function(H264_DECHDL, (void*) (&h264_num_mem_rec_ip), (void*) (&h264_num_mem_rec_op)) == IV_SUCCESS) {
                ITTIAM_DEBUG_PRINT("IV_CMD_GET_NUM_MEM_REC    [ OK ]");
            } else {
                ITTIAM_DEBUG_PRINT("IV_CMD_GET_NUM_MEM_REC    [ NOK ] with error %d", (UWORD32) h264_num_mem_rec_op.u4_error_code);
            }

            ITTIAM_DEBUG_PRINT("Number of records : %d", h264_num_mem_rec_op.u4_num_mem_rec);


            /****************************************************************************/
            /* H264 ====== Allocate the pointers
             *****************************************************************************/
            h264_ps_it_mem = (it_mem_t *) vp_os_malloc(sizeof (it_mem_t));
            if (h264_ps_it_mem == NULL) {
                ITTIAM_DEBUG_PRINT("\nAllocation failure\n");
                return 0;
            }
            it_mem_init(h264_ps_it_mem);

            h264_mem_rec = h264_ps_it_mem->alloc(h264_ps_it_mem, (h264_num_mem_rec_op.u4_num_mem_rec) * sizeof (iv_mem_rec_t));
            if (h264_mem_rec == NULL) {
                ITTIAM_DEBUG_PRINT("\nAllocation failure\n");
                return 0;
            }

            /****************************************************************************/
            /* H264 ====== Fill the memory with some information
             *****************************************************************************/

            printf("current_PaVE.encoded_stream_width = %d, current_PaVE.encoded_stream_height = %d\n", current_PaVE.encoded_stream_width, current_PaVE.encoded_stream_height);

            ih264d_cxa8_fill_mem_rec_ip_t h264_fill_mem_rec_ip;
            ih264d_cxa8_fill_mem_rec_op_t h264_fill_mem_rec_op;

            h264_fill_mem_rec_ip.s_ivd_fill_mem_rec_ip_t.e_cmd = IV_CMD_FILL_NUM_MEM_REC;
            h264_fill_mem_rec_ip.s_ivd_fill_mem_rec_ip_t.pv_mem_rec_location = h264_mem_rec;
            h264_fill_mem_rec_ip.s_ivd_fill_mem_rec_ip_t.u4_max_frm_wd = current_PaVE.encoded_stream_width; //for example  640;
            h264_fill_mem_rec_ip.s_ivd_fill_mem_rec_ip_t.u4_max_frm_ht = current_PaVE.encoded_stream_height; //for example 368;
            h264_fill_mem_rec_ip.s_level = H264_MAX_LEVEL_SUPPORTED;
            h264_fill_mem_rec_ip.s_num_ref_frames = H264_MAX_REF_FRAMES;
            h264_fill_mem_rec_ip.s_num_reorder_frames = H264_MAX_REORDER_FRAMES;
            h264_fill_mem_rec_ip.s_ivd_fill_mem_rec_ip_t.u4_size = sizeof (ih264d_cxa8_fill_mem_rec_ip_t);
            h264_fill_mem_rec_op.s_ivd_fill_mem_rec_op_t.u4_size = sizeof (ih264d_cxa8_fill_mem_rec_op_t);

            for (i = 0; i < h264_num_mem_rec_op.u4_num_mem_rec; i++)
                h264_mem_rec[i].u4_size = sizeof (iv_mem_rec_t);

            if (ih264d_cxa8_api_function(H264_DECHDL, (void*) (&h264_fill_mem_rec_ip), (void*) (&h264_fill_mem_rec_op)) == IV_SUCCESS) {
                ITTIAM_DEBUG_PRINT("IV_CMD_FILL_NUM_MEM_REC    [ OK ]");
            } else {
                ITTIAM_DEBUG_PRINT("IV_CMD_FILL_NUM_MEM_REC    [ NOK ]");
            }

            //Do some allocation on the mem_rec pointer
            iv_mem_rec_t * h264_temp_mem_rec = h264_mem_rec;
            for (i = 0; i < h264_num_mem_rec_op.u4_num_mem_rec; i++) {
                h264_temp_mem_rec->pv_base = h264_ps_it_mem->align_alloc(h264_ps_it_mem, h264_temp_mem_rec->u4_mem_size, h264_temp_mem_rec->u4_mem_alignment);
                if (h264_temp_mem_rec->pv_base == NULL) {
                    ITTIAM_DEBUG_PRINT("\nAllocation failure\n");
                }
                h264_temp_mem_rec++;
            }


            /****************************************************************************/
            /* H264 ====== Init the DECHDL
             *****************************************************************************/
            ih264d_cxa8_init_ip_t h264_init_ip;
            ih264d_cxa8_init_op_t h264_init_op;

            void *h264_fxns = &ih264d_cxa8_api_function;

            iv_mem_rec_t *h264_mem_tab = (iv_mem_rec_t*) h264_mem_rec;

            h264_init_ip.s_ivd_init_ip_t.e_cmd = IV_CMD_INIT;
            h264_init_ip.s_ivd_init_ip_t.pv_mem_rec_location = h264_mem_tab;
            h264_init_ip.s_ivd_init_ip_t.u4_frm_max_wd = current_PaVE.encoded_stream_width; //for example 640;
            h264_init_ip.s_ivd_init_ip_t.u4_frm_max_ht = current_PaVE.encoded_stream_height; //for example 368;
            h264_init_ip.s_level = H264_MAX_LEVEL_SUPPORTED;
            h264_init_ip.s_num_ref_frames = H264_MAX_REF_FRAMES;
            h264_init_ip.s_num_reorder_frames = H264_MAX_REORDER_FRAMES;
            h264_init_ip.s_ivd_init_ip_t.u4_num_mem_rec = h264_num_mem_rec_op.u4_num_mem_rec;
            h264_init_ip.s_ivd_init_ip_t.e_output_format = IV_RGB_565; //IV_YUV_420P;
            h264_init_ip.s_ivd_init_ip_t.u4_size = sizeof (ih264d_cxa8_init_ip_t);
            h264_init_op.s_ivd_init_op_t.u4_size = sizeof (ih264d_cxa8_init_op_t);

            H264_DECHDL = (iv_obj_t*) h264_mem_tab[0].pv_base;
            H264_DECHDL->pv_fxns = h264_fxns;
            H264_DECHDL->u4_size = sizeof (iv_obj_t);

            if (ih264d_cxa8_api_function(H264_DECHDL, (void *) &h264_init_ip, (void *) &h264_init_op) == IV_SUCCESS) {
                ITTIAM_DEBUG_PRINT("IV_CMD_INIT    [ OK ]");
            } else {
                ITTIAM_DEBUG_PRINT("IV_CMD_INIT    [ NOK ]");
            }

            /****************************************************************************/
            /* H264 ====== Set decoder config
             *****************************************************************************/
            ivd_ctl_set_config_ip_t h264_ctl_set_config_ip;
            ivd_ctl_set_config_op_t h264_ctl_set_config_op;
            h264_ctl_set_config_ip.u4_disp_wd = current_PaVE.encoded_stream_width; //for example 640;
            h264_ctl_set_config_ip.e_frm_skip_mode = IVD_NO_SKIP;
            h264_ctl_set_config_ip.e_frm_out_mode = IVD_DISPLAY_FRAME_OUT;
            h264_ctl_set_config_ip.e_vid_dec_mode = IVD_DECODE_FRAME;
            h264_ctl_set_config_ip.e_cmd = IVD_CMD_VIDEO_CTL;
            h264_ctl_set_config_ip.e_sub_cmd = IVD_CMD_CTL_SETPARAMS;
            h264_ctl_set_config_ip.u4_size = sizeof (ivd_ctl_set_config_ip_t);
            h264_ctl_set_config_op.u4_size = sizeof (ivd_ctl_set_config_op_t);

            if (ih264d_cxa8_api_function(H264_DECHDL, (void *) &h264_ctl_set_config_ip, (void *) &h264_ctl_set_config_op) == IV_SUCCESS) {
                ITTIAM_DEBUG_PRINT("IVD_CMD_VIDEO_CTL => IVD_CMD_CTL_SETPARAMS   [ OK ]");
            } else {
                ITTIAM_DEBUG_PRINT("IVD_CMD_VIDEO_CTL => IVD_CMD_CTL_SETPARAMS   [ NOK ]");
            }

            /****************************************************************************/
            /* H264 ====== Decode the in buffer INIT PART
             *****************************************************************************/
            h264_video_decode_ip.e_cmd = IVD_CMD_VIDEO_DECODE;
            h264_video_decode_ip.u4_size = sizeof (ivd_video_decode_ip_t);
            h264_video_decode_op.u4_size = sizeof (ivd_video_decode_op_t);

            /****************************************************************************/
            /* H264 ====== Display the buffer INIT PART
             *****************************************************************************/
            h264_get_display_frame_ip.e_cmd = IVD_CMD_GET_DISPLAY_FRAME;
            h264_get_display_frame_ip.u4_size = sizeof (ivd_get_display_frame_ip_t);
            h264_get_display_frame_op.u4_size = sizeof (ivd_get_display_frame_op_t);

            /****************************************************************************/
            /* H264 ====== Get the buffers information to re-use them
             *****************************************************************************/
            ivd_ctl_getbufinfo_ip_t h264_ctl_dec_ip;
            ivd_ctl_getbufinfo_op_t h264_ctl_dec_op;

            h264_ctl_dec_ip.e_cmd = IVD_CMD_VIDEO_CTL;
            h264_ctl_dec_ip.e_sub_cmd = IVD_CMD_CTL_GETBUFINFO;
            h264_ctl_dec_ip.u4_size = sizeof (ivd_ctl_getbufinfo_ip_t);
            h264_ctl_dec_op.u4_size = sizeof (ivd_ctl_getbufinfo_op_t);

            if (ih264d_cxa8_api_function(H264_DECHDL, (void *) &h264_ctl_dec_ip, (void *) &h264_ctl_dec_op) == IV_SUCCESS) {
                ITTIAM_DEBUG_PRINT("IVD_CMD_VIDEO_CTL => IVD_CMD_CTL_GETBUFINFO   [ OK ]");
            } else {
                ITTIAM_DEBUG_PRINT("IVD_CMD_VIDEO_CTL => IVD_CMD_CTL_GETBUFINFO   [ NOK ]");
            }

            //Allocate the output buffer used to store decoded frame (RGB 565)

            h264_get_display_frame_ip.s_out_buffer.u4_min_out_buf_size[0] = h264_ctl_dec_op.u4_min_out_buf_size[0];
            h264_get_display_frame_ip.s_out_buffer.pu1_bufs[0] = h264_ps_it_mem->alloc(h264_ps_it_mem, h264_ctl_dec_op.u4_min_out_buf_size[0]);
            h264_get_display_frame_ip.s_out_buffer.u4_num_bufs = h264_ctl_dec_op.u4_min_num_out_bufs;

            ITTIAM_DEBUG_PRINT("min buf size = %d", h264_get_display_frame_ip.s_out_buffer.u4_min_out_buf_size[0]);
            ITTIAM_DEBUG_PRINT("num out buf  = %d", h264_get_display_frame_ip.s_out_buffer.u4_num_bufs);
            ITTIAM_DEBUG_PRINT("@buffer      = %p", h264_get_display_frame_ip.s_out_buffer.pu1_bufs[0]);
            
            puts("******************************** ITTIAM H264 decoding init *********************************");
#endif
            ///////////////////////////// END H264 INIT SECTION /////////////////////////////////////////////////////////////////////////
            //================================================================================================================================
        } else if (current_PaVE.video_codec == CODEC_MPEG4_VISUAL) {
            //////////////////// MPEG4 INIT SECTION //////////////////////////////////////////////////////////////////////////////////////
#ifdef MPEG4_DEC
            /****************************************************************************/
            /* MPEG4 ====== Initialize the memory records
             *****************************************************************************/
            cfg->num_picture_decoded = old_num_frame;
            ITTIAM_DEBUG_PRINT("ITTIAM INIT");

            iv_num_mem_rec_ip_t mpeg4_num_mem_rec_ip;
            iv_num_mem_rec_op_t mpeg4_num_mem_rec_op;

            mpeg4_num_mem_rec_ip.e_cmd = IV_CMD_GET_NUM_MEM_REC;
            mpeg4_num_mem_rec_ip.u4_size = sizeof (iv_num_mem_rec_ip_t);
            mpeg4_num_mem_rec_op.u4_size = sizeof (iv_num_mem_rec_op_t);

            if (imp4d_cxa8_api_function(MPEG4_DECHDL, (void*) (&mpeg4_num_mem_rec_ip), (void*) (&mpeg4_num_mem_rec_op)) == IV_SUCCESS) {
                ITTIAM_DEBUG_PRINT("IV_CMD_GET_NUM_MEM_REC    [ OK ]");
            } else {
                ITTIAM_DEBUG_PRINT("IV_CMD_GET_NUM_MEM_REC    [ NOK ] with error %d", (UWORD32) mpeg4_num_mem_rec_op.u4_error_code);
            }

            ITTIAM_DEBUG_PRINT("Number of records : %d", mpeg4_num_mem_rec_op.u4_num_mem_rec);


            /****************************************************************************/
            /* MPEG4 ====== Allocate the pointers
             *****************************************************************************/
            mpeg4_ps_it_mem = (it_mem_t *) vp_os_malloc(sizeof (it_mem_t));
            if (mpeg4_ps_it_mem == NULL) {
                ITTIAM_DEBUG_PRINT("\nAllocation failure\n");
                return 0;
            }
            it_mem_init(mpeg4_ps_it_mem);

            mpeg4_mem_rec = mpeg4_ps_it_mem->alloc(mpeg4_ps_it_mem, (mpeg4_num_mem_rec_op.u4_num_mem_rec) * sizeof (iv_mem_rec_t));
            if (mpeg4_mem_rec == NULL) {
                ITTIAM_DEBUG_PRINT("\nAllocation failure\n");
                return 0;
            }

            /****************************************************************************/
            /* MPEG4 ====== Fill the memory with some information
             *****************************************************************************/

            printf("current_PaVE.encoded_stream_width = %d, current_PaVE.encoded_stream_height = %d\n", current_PaVE.encoded_stream_width, current_PaVE.encoded_stream_height);

            imp4d_cxa8_fill_mem_rec_ip_t mpeg4_fill_mem_rec_ip;
            imp4d_cxa8_fill_mem_rec_op_t mpeg4_fill_mem_rec_op;

            mpeg4_fill_mem_rec_ip.s_ivd_fill_mem_rec_ip_t.e_cmd = IV_CMD_FILL_NUM_MEM_REC;
            mpeg4_fill_mem_rec_ip.s_ivd_fill_mem_rec_ip_t.pv_mem_rec_location = mpeg4_mem_rec;
            mpeg4_fill_mem_rec_ip.s_ivd_fill_mem_rec_ip_t.u4_max_frm_wd = current_PaVE.encoded_stream_width; //for example  640;
            mpeg4_fill_mem_rec_ip.s_ivd_fill_mem_rec_ip_t.u4_max_frm_ht = current_PaVE.encoded_stream_height; //for example 360;
            mpeg4_fill_mem_rec_ip.s_ivd_fill_mem_rec_ip_t.u4_size = sizeof (imp4d_cxa8_fill_mem_rec_ip_t);
            mpeg4_fill_mem_rec_op.s_ivd_fill_mem_rec_op_t.u4_size = sizeof (imp4d_cxa8_fill_mem_rec_op_t);

            for (i = 0; i < mpeg4_num_mem_rec_op.u4_num_mem_rec; i++)
                mpeg4_mem_rec[i].u4_size = sizeof (iv_mem_rec_t);

            if (imp4d_cxa8_api_function(MPEG4_DECHDL, (void*) (&mpeg4_fill_mem_rec_ip), (void*) (&mpeg4_fill_mem_rec_op)) == IV_SUCCESS) {
                ITTIAM_DEBUG_PRINT("IV_CMD_FILL_NUM_MEM_REC    [ OK ]");
            } else {
                ITTIAM_DEBUG_PRINT("IV_CMD_FILL_NUM_MEM_REC    [ NOK ]");
            }

            //Do some allocation on the mem_rec pointer
            iv_mem_rec_t * mpeg4_temp_mem_rec = mpeg4_mem_rec;
            for (i = 0; i < mpeg4_num_mem_rec_op.u4_num_mem_rec; i++) {
                mpeg4_temp_mem_rec->pv_base = mpeg4_ps_it_mem->align_alloc(mpeg4_ps_it_mem, mpeg4_temp_mem_rec->u4_mem_size, mpeg4_temp_mem_rec->u4_mem_alignment);
                if (mpeg4_temp_mem_rec->pv_base == NULL) {
                    ITTIAM_DEBUG_PRINT("\nAllocation failure\n");
                }
                mpeg4_temp_mem_rec++;
            }

            /****************************************************************************/
            /* MPEG4 ====== Init the DECHDL
             *****************************************************************************/
            imp4d_cxa8_init_ip_t mpeg4_init_ip;
            imp4d_cxa8_init_op_t mpeg4_init_op;

            void *mpeg4_fxns = &imp4d_cxa8_api_function;

            iv_mem_rec_t *mpeg4_mem_tab = (iv_mem_rec_t*) mpeg4_mem_rec;

            mpeg4_init_ip.s_ivd_init_ip_t.e_cmd = IV_CMD_INIT;
            mpeg4_init_ip.s_ivd_init_ip_t.pv_mem_rec_location = mpeg4_mem_tab;
            mpeg4_init_ip.s_ivd_init_ip_t.u4_frm_max_wd = current_PaVE.encoded_stream_width; //for example 640;
            mpeg4_init_ip.s_ivd_init_ip_t.u4_frm_max_ht = current_PaVE.encoded_stream_height; //for example 360;
            mpeg4_init_ip.s_ivd_init_ip_t.u4_num_mem_rec = mpeg4_num_mem_rec_op.u4_num_mem_rec;
            mpeg4_init_ip.s_ivd_init_ip_t.e_output_format = IV_RGB_565; //IV_YUV_420P;
            mpeg4_init_ip.s_ivd_init_ip_t.u4_size = sizeof (imp4d_cxa8_init_ip_t);
            mpeg4_init_op.s_ivd_init_op_t.u4_size = sizeof (imp4d_cxa8_init_op_t);

            MPEG4_DECHDL = (iv_obj_t*) mpeg4_mem_tab[0].pv_base;
            MPEG4_DECHDL->pv_fxns = mpeg4_fxns;
            MPEG4_DECHDL->u4_size = sizeof (iv_obj_t);

            if (imp4d_cxa8_api_function(MPEG4_DECHDL, (void *) &mpeg4_init_ip, (void *) &mpeg4_init_op) == IV_SUCCESS) {
                ITTIAM_DEBUG_PRINT("IV_CMD_INIT    [ OK ]");
            } else {
                ITTIAM_DEBUG_PRINT("IV_CMD_INIT    [ NOK ]");
            }

            /****************************************************************************/
            /* MPEG4 ====== Set decoder config
             *****************************************************************************/
            ivd_ctl_set_config_ip_t mpeg4_ctl_set_config_ip;
            ivd_ctl_set_config_op_t mpeg4_ctl_set_config_op;
            mpeg4_ctl_set_config_ip.u4_disp_wd = current_PaVE.encoded_stream_width; //for example 640;
            mpeg4_ctl_set_config_ip.e_frm_skip_mode = IVD_NO_SKIP;
            mpeg4_ctl_set_config_ip.e_frm_out_mode = IVD_DISPLAY_FRAME_OUT;
            mpeg4_ctl_set_config_ip.e_vid_dec_mode = IVD_DECODE_FRAME;
            mpeg4_ctl_set_config_ip.e_cmd = IVD_CMD_VIDEO_CTL;
            mpeg4_ctl_set_config_ip.e_sub_cmd = IVD_CMD_CTL_SETPARAMS;
            mpeg4_ctl_set_config_ip.u4_size = sizeof (ivd_ctl_set_config_ip_t);
            mpeg4_ctl_set_config_op.u4_size = sizeof (ivd_ctl_set_config_op_t);

            if (imp4d_cxa8_api_function(MPEG4_DECHDL, (void *) &mpeg4_ctl_set_config_ip, (void *) &mpeg4_ctl_set_config_op) == IV_SUCCESS) {
                ITTIAM_DEBUG_PRINT("IVD_CMD_VIDEO_CTL => IVD_CMD_CTL_SETPARAMS   [ OK ]");
            } else {
                ITTIAM_DEBUG_PRINT("IVD_CMD_VIDEO_CTL => IVD_CMD_CTL_SETPARAMS   [ NOK ]");
            }

            /****************************************************************************/
            /* MPEG4 ====== Decode the in buffer INIT PART
             *****************************************************************************/
            mpeg4_video_decode_ip.e_cmd = IVD_CMD_VIDEO_DECODE;
            mpeg4_video_decode_ip.u4_size = sizeof (ivd_video_decode_ip_t);
            mpeg4_video_decode_op.u4_size = sizeof (ivd_video_decode_op_t);

            /****************************************************************************/
            /* Display the buffer INIT PART
             *****************************************************************************/
            mpeg4_get_display_frame_ip.e_cmd = IVD_CMD_GET_DISPLAY_FRAME;
            mpeg4_get_display_frame_ip.u4_size = sizeof (ivd_get_display_frame_ip_t);
            mpeg4_get_display_frame_op.u4_size = sizeof (ivd_get_display_frame_op_t);

            /****************************************************************************/
            /* MPEG4 ====== Get the buffers information to re-use them
             *****************************************************************************/
            ivd_ctl_getbufinfo_ip_t mpeg4_ctl_dec_ip;
            ivd_ctl_getbufinfo_op_t mpeg4_ctl_dec_op;

            mpeg4_ctl_dec_ip.e_cmd = IVD_CMD_VIDEO_CTL;
            mpeg4_ctl_dec_ip.e_sub_cmd = IVD_CMD_CTL_GETBUFINFO;
            mpeg4_ctl_dec_ip.u4_size = sizeof (ivd_ctl_getbufinfo_ip_t);
            mpeg4_ctl_dec_op.u4_size = sizeof (ivd_ctl_getbufinfo_op_t);

            if (imp4d_cxa8_api_function(MPEG4_DECHDL, (void *) &mpeg4_ctl_dec_ip, (void *) &mpeg4_ctl_dec_op) == IV_SUCCESS) {
                ITTIAM_DEBUG_PRINT("IVD_CMD_VIDEO_CTL => IVD_CMD_CTL_GETBUFINFO   [ OK ]");
            } else {
                ITTIAM_DEBUG_PRINT("IVD_CMD_VIDEO_CTL => IVD_CMD_CTL_GETBUFINFO   [ NOK ]");
            }

            //Allocate the output buffer used to store decoded frame (RGB 565)

            mpeg4_get_display_frame_ip.s_out_buffer.u4_min_out_buf_size[0] = mpeg4_ctl_dec_op.u4_min_out_buf_size[0];
            mpeg4_get_display_frame_ip.s_out_buffer.pu1_bufs[0] = mpeg4_ps_it_mem->alloc(mpeg4_ps_it_mem, mpeg4_ctl_dec_op.u4_min_out_buf_size[0]);
            mpeg4_get_display_frame_ip.s_out_buffer.u4_num_bufs = mpeg4_ctl_dec_op.u4_min_num_out_bufs;

            ITTIAM_DEBUG_PRINT("min buf size = %d", mpeg4_get_display_frame_ip.s_out_buffer.u4_min_out_buf_size[0]);
            ITTIAM_DEBUG_PRINT("num out buf  = %d", mpeg4_get_display_frame_ip.s_out_buffer.u4_num_bufs);
            ITTIAM_DEBUG_PRINT("@buffer      = %p", mpeg4_get_display_frame_ip.s_out_buffer.pu1_bufs[0]);
            
            puts("******************************** ITTIAM MPEG4 decoding init ********************************");

#endif
        }

        ///////////////////////////// END MPEG4 INIT SECTION /////////////////////////////////////////////////////////////////////////

        vp_os_mutex_lock(&out->lock);

        out->numBuffers = 1;
        if(out->buffers == NULL)
            out->buffers = (uint8_t **) vp_os_malloc(sizeof (uint8_t*));
        out->buffers[0] = NULL;
        out->indexBuffer = 0;
        out->lineSize = 0;

        cfg->src_picture.width = current_PaVE.display_width; //for example 640;
        cfg->src_picture.height = current_PaVE.display_height; //for example 360;
        cfg->dst_picture.format = PIX_FMT_RGB565;
        cfg->dst_picture.width = current_PaVE.display_width; //for example 640;
        cfg->dst_picture.height = current_PaVE.display_height; //for example 360;


        //Adress of output ittiam pointer
        if (current_PaVE.video_codec == CODEC_MPEG4_AVC) {
            out->buffers[0] = (uint8_t*) h264_get_display_frame_ip.s_out_buffer.pu1_bufs[0];
            out->size = h264_get_display_frame_ip.s_out_buffer.u4_min_out_buf_size[0];
        } else if (current_PaVE.video_codec == CODEC_MPEG4_VISUAL) {
            out->buffers[0] = (uint8_t*) mpeg4_get_display_frame_ip.s_out_buffer.pu1_bufs[0];
            out->size = mpeg4_get_display_frame_ip.s_out_buffer.u4_min_out_buf_size[0];
        }
        
        out->status = VP_API_STATUS_PROCESSING;

        vp_os_mutex_unlock(&out->lock);


#ifdef NUM_SAMPLES
        gettimeofday(&start_time, NULL);
#endif


    }


    ///////////////////////////////////// PROCESS SECTION //////////////////////////////////////////////////////////////////
#if	WAIT_FOR_I_FRAME
    if (current_PaVE.frame_number != (previous_PaVE.frame_number + 1)) {
        waitForIFrame = TRUE;
    }
#endif

    if (waitForIFrame == FALSE || current_PaVE.frame_type == FRAME_TYPE_IDR_FRAME || current_PaVE.frame_type == FRAME_TYPE_I_FRAME) {

#ifdef NUM_SAMPLES
        struct timeval end_time;
        static float32_t frame_decoded_time = 0;

        gettimeofday(&start_time2, NULL);
#endif

        waitForIFrame = FALSE;

        if (current_PaVE.video_codec == CODEC_MPEG4_AVC) {

#ifdef H264_DEC
            /****************************************************************************/
            /* Decode the in buffer EXEC PART
             *****************************************************************************/

            h264_video_decode_ip.u4_ts = cfg->num_picture_decoded;
            h264_video_decode_ip.pv_stream_buffer = ((unsigned char*) in->buffers[in->indexBuffer]);
            h264_video_decode_ip.u4_num_Bytes = in->size;

            ITTIAM_DEBUG_PRINT("In size = %d", in->size);

            if (ih264d_cxa8_api_function(H264_DECHDL, (void *) &h264_video_decode_ip, (void *) &h264_video_decode_op) == IV_SUCCESS) {
                ITTIAM_DEBUG_PRINT("IVD_CMD_VIDEO_DECODE   [ OK ]");
            } else {
                ITTIAM_DEBUG_PRINT("IVD_CMD_VIDEO_DECODE   [ NOK ]");
            }

            /****************************************************************************/
            /* Display the buffer EXEC PART
             *****************************************************************************/

            if (ih264d_cxa8_api_function(H264_DECHDL, (void *) &h264_get_display_frame_ip, (void *) &h264_get_display_frame_op) == IV_SUCCESS) {
                ITTIAM_DEBUG_PRINT("IVD_CMD_GET_DISPLAY_FRAME   [ OK ]");

                if (h264_video_decode_op.u4_frame_decoded_flag == 1) {

                    cfg->num_picture_decoded++;

                    //Display the FPS
#ifdef NUM_SAMPLES
                    gettimeofday(&end_time, NULL);
                    frame_decoded_time += ((end_time.tv_sec * 1000.0 + end_time.tv_usec / 1000.0)
                            - (start_time2.tv_sec * 1000.0 + start_time2.tv_usec / 1000.0));

                    if (numsamples++ > NUM_SAMPLES) {
                        float32_t value = ((end_time.tv_sec * 1000.0 + end_time.tv_usec / 1000.0)
                                - (start_time.tv_sec * 1000.0 + start_time.tv_usec / 1000.0));

                        printf("Frames decoded in average %f fps, received and decoded in average %f fps\n",
                                (1000.0 / (frame_decoded_time / (float32_t) NUM_SAMPLES)),
                                1000.0 / (value / (float32_t) NUM_SAMPLES)
                                );
                        //printf("%f\n", (1000.0 / (frame_decoded_time / (float32_t)NUM_SAMPLES)));
                        gettimeofday(&start_time, NULL);
                        frame_decoded_time = 0;
                        numsamples = 0;
                    }
#endif

                }
            } else {
                ITTIAM_DEBUG_PRINT("IVD_CMD_GET_DISPLAY_FRAME   [ NOK ]");
            }
#endif
        } else if (current_PaVE.video_codec == CODEC_MPEG4_VISUAL) {

#ifdef MPEG4_DEC
            /****************************************************************************/
            /* Decode the in buffer EXEC PART
             *****************************************************************************/

            mpeg4_video_decode_ip.u4_ts = cfg->num_picture_decoded;
            if(current_PaVE.frame_type == FRAME_TYPE_P_FRAME){
                mpeg4_video_decode_ip.pv_stream_buffer = ((unsigned char*) in->buffers[in->indexBuffer]);
                mpeg4_video_decode_ip.u4_num_Bytes = in->size;
            } else {
                mpeg4_video_decode_ip.pv_stream_buffer = ((unsigned char*) in->buffers[in->indexBuffer]);
                mpeg4_video_decode_ip.u4_num_Bytes = in->size;               
            }
            

            ITTIAM_DEBUG_PRINT("In size = %d", in->size);

            if (imp4d_cxa8_api_function(MPEG4_DECHDL, (void *) &mpeg4_video_decode_ip, (void *) &mpeg4_video_decode_op) == IV_SUCCESS) {
                ITTIAM_DEBUG_PRINT("IVD_CMD_VIDEO_DECODE   [ OK ]");
            } else {
                ITTIAM_DEBUG_PRINT("IVD_CMD_VIDEO_DECODE   [ NOK ] with error 0x%04X ==> %d", mpeg4_video_decode_op.u4_error_code, mpeg4_video_decode_op.u4_error_code);
            }

            /****************************************************************************/
            /* Display the buffer EXEC PART
             *****************************************************************************/

            if (imp4d_cxa8_api_function(MPEG4_DECHDL, (void *) &mpeg4_get_display_frame_ip, (void *) &mpeg4_get_display_frame_op) == IV_SUCCESS) {
                ITTIAM_DEBUG_PRINT("IVD_CMD_GET_DISPLAY_FRAME   [ OK ]");

                if (mpeg4_video_decode_op.u4_frame_decoded_flag == 1) {

                    cfg->num_picture_decoded++;

                    //Display the FPS
#ifdef NUM_SAMPLES
                    gettimeofday(&end_time, NULL);
                    frame_decoded_time += ((end_time.tv_sec * 1000.0 + end_time.tv_usec / 1000.0)
                            - (start_time2.tv_sec * 1000.0 + start_time2.tv_usec / 1000.0));

                    if (numsamples++ > NUM_SAMPLES) {
                        float32_t value = ((end_time.tv_sec * 1000.0 + end_time.tv_usec / 1000.0)
                                - (start_time.tv_sec * 1000.0 + start_time.tv_usec / 1000.0));

                        printf("Frames decoded in average %f fps, received and decoded in average %f fps\n",
                                (1000.0 / (frame_decoded_time / (float32_t) NUM_SAMPLES)),
                                1000.0 / (value / (float32_t) NUM_SAMPLES)
                                );
                        //printf("%f\n", (1000.0 / (frame_decoded_time / (float32_t)NUM_SAMPLES)));
                        gettimeofday(&start_time, NULL);
                        frame_decoded_time = 0;
                        numsamples = 0;
                    }
#endif

                }
            } else {
                ITTIAM_DEBUG_PRINT("IVD_CMD_GET_DISPLAY_FRAME   [ NOK ]");
            }
#endif
        }
    } else {
        return C_OK;
    }

    ///////////////////////////////////// END PROCESS SECTION //////////////////////////////////////////////////////////////    

    ITTIAM_DEBUG_PRINT("ITTIAM DECODE");
    return C_OK;
}

C_RESULT ittiam_stage_decoding_close(ittiam_stage_decoding_config_t *cfg) {
    if (current_PaVE.video_codec == CODEC_MPEG4_AVC) {
        //H264
        /****************************************************************************/
        /* H264 ====== Reset the memory records
         *****************************************************************************/
        ITTIAM_DEBUG_PRINT("ITTIAM RESET");

        ivd_ctl_reset_ip_t h264_ctl_reset_ip;
        ivd_ctl_reset_op_t h264_ctl_reset_op;

        h264_ctl_reset_ip.e_cmd = IVD_CMD_VIDEO_CTL;
        h264_ctl_reset_ip.e_sub_cmd = IVD_CMD_CTL_RESET;
        h264_ctl_reset_ip.u4_size = sizeof (ivd_ctl_reset_ip_t);
        h264_ctl_reset_op.u4_size = sizeof (ivd_ctl_reset_op_t);

        if (ih264d_cxa8_api_function(H264_DECHDL, (void*) (&h264_ctl_reset_ip), (void*) (&h264_ctl_reset_op)) == IV_SUCCESS) {
            ITTIAM_DEBUG_PRINT("IVD_CMD_CTL_RESET    [ OK ]");
        } else {
            ITTIAM_DEBUG_PRINT("IVD_CMD_CTL_RESET    [ NOK ] with error %d", (UWORD32) h264_ctl_reset_op.u4_error_code);

        }

        vp_os_free(h264_mem_rec);
        h264_mem_rec = NULL;
        vp_os_free(h264_ps_it_mem);
        h264_ps_it_mem = NULL;

    } else if (current_PaVE.video_codec == CODEC_MPEG4_VISUAL) {
        //MPEG4
        /****************************************************************************/
        /* H264 ====== Reset the memory records
         *****************************************************************************/
        ITTIAM_DEBUG_PRINT("ITTIAM RESET");

        ivd_ctl_reset_ip_t mpeg4_ctl_reset_ip;
        ivd_ctl_reset_op_t mpeg4_ctl_reset_op;

        mpeg4_ctl_reset_ip.e_cmd = IVD_CMD_VIDEO_CTL;
        mpeg4_ctl_reset_ip.e_sub_cmd = IVD_CMD_CTL_RESET;
        mpeg4_ctl_reset_ip.u4_size = sizeof (ivd_ctl_reset_ip_t);
        mpeg4_ctl_reset_op.u4_size = sizeof (ivd_ctl_reset_op_t);

        if (imp4d_cxa8_api_function(MPEG4_DECHDL, (void*) (&mpeg4_ctl_reset_ip), (void*) (&mpeg4_ctl_reset_op)) == IV_SUCCESS) {
            ITTIAM_DEBUG_PRINT("IVD_CMD_CTL_RESET    [ OK ]");
        } else {
            ITTIAM_DEBUG_PRINT("IVD_CMD_CTL_RESET    [ NOK ] with error %d", (UWORD32) mpeg4_ctl_reset_op.u4_error_code);

        }

        vp_os_free(mpeg4_mem_rec);
        mpeg4_mem_rec = NULL;
        vp_os_free(mpeg4_ps_it_mem);
        mpeg4_ps_it_mem = NULL;
    }
    old_num_frame = cfg->num_picture_decoded;
    ITTIAM_DEBUG_PRINT("ITTIAM CLEAN");
    return C_OK;
}

#endif
