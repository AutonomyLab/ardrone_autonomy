#ifndef _VP_STAGES_FRAME_PIPE_H_
#define _VP_STAGES_FRAME_PIPE_H_


#include <VP_Api/vp_api.h>
#include <VP_Api/vp_api_picture.h>

///////////////////////////////////////////////
// TYPEDEFS


typedef enum _BUFFER_PIPE_STATE
{
  WAIT_RECEPTION,
  PAUSE
} BUFFER_PIPE_STATE;


/**
 * \typedef pipe parameters definition
 */
typedef struct _vp_stages_frame_pipe_config_
{
  // public
  vp_api_picture_t* inPicture;
  vp_api_picture_t  outPicture;

  // private
  BUFFER_PIPE_STATE pipe_state;
  vp_os_mutex_t	pipe_mut;
  vp_os_cond_t buffer_sent;
  uint32_t frame_size;

  void (*callback)(void);
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
#endif


