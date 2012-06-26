#ifndef _VP_STAGES_FRAME_PIPE_H_
#define _VP_STAGES_FRAME_PIPE_H_


#include <VP_Api/vp_api.h>
#include <VP_Api/vp_api_picture.h>

///////////////////////////////////////////////
// TYPEDEFS


typedef enum _BUFFER_PIPE_STATE
{
  PAUSE = 0,
  WAIT_RECEPTION,
  FETCH,
  FETCH_PAUSE,
  SENDER_ERROR
} BUFFER_PIPE_STATE;

typedef enum {
  LOW_LATENCY=0, /* stage will wait for the current frame to be transmitted*/
  FAST           /* stage will trigger frame copy and return immediately the last transfered frame */
} FRAME_PIPE_MODE;

/**
 * \typedef pipe parameters definition
 */
typedef struct _vp_stages_frame_pipe_config_
{
  // public
  vp_api_picture_t* inPicture;
  vp_api_picture_t  outPicture;
  FRAME_PIPE_MODE mode;
  void *userInfo;

  // private
  vp_api_picture_t  copyPicture;
  BUFFER_PIPE_STATE pipe_state;
  vp_os_mutex_t	pipe_mut;
  vp_os_cond_t buffer_sent;
  uint32_t frame_size;

  void (*callback)(void);

  int nb_buffers;
  int index_buffer;
  uint8_t ** output_buffers;
}
vp_stages_frame_pipe_config_t;


///////////////////////////////////////////////
// FUNCTIONS

// Sender function
C_RESULT
vp_stages_frame_pipe_sender_open(vp_stages_frame_pipe_config_t *cfg);

C_RESULT
vp_stages_frame_pipe_sender_transform(vp_stages_frame_pipe_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out);

C_RESULT
vp_stages_frame_pipe_sender_close(vp_stages_frame_pipe_config_t *cfg);

// Receiver function
C_RESULT
vp_stages_frame_pipe_receiver_open(vp_stages_frame_pipe_config_t *cfg);

C_RESULT
vp_stages_frame_pipe_receiver_transform(vp_stages_frame_pipe_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out);

C_RESULT
vp_stages_frame_pipe_receiver_close(vp_stages_frame_pipe_config_t *cfg);

// Fetch function
C_RESULT
vp_stages_frame_pipe_fetch_open(vp_stages_frame_pipe_config_t *cfg);

C_RESULT
vp_stages_frame_pipe_fetch_transform(vp_stages_frame_pipe_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out);

C_RESULT
vp_stages_frame_pipe_fetch_close(vp_stages_frame_pipe_config_t *cfg);

#endif


