/**
 *  \brief    VP Stages. Output SDL stage declaration
 *  \author   Sylvain Gaeremynck <sylvain.gaeremynck@parrot.fr>
 *  \author   Aurelien Morelle <aurelien.morelle@parrot.fr>
 *  \author   Thomas Landais <thomas.landais@parrot.fr>
 *  \version  2.0
 *  \date     first release 16/03/2007
 *  \date     modification  19/03/2007
 */

#ifndef _VP_STAGES_O_SDL_H_
#define _VP_STAGES_O_SDL_H_

/**
 * @defgroup VP_SDK
 * @{ */

/**
 * @defgroup VP_Stages
 * @{ */

/**
 * @defgroup vp_stages_o_sdl output sdl stage
 * @{ */

#if !defined(__NDS__)

///////////////////////////////////////////////
// INCLUDES

#include <VP_Api/vp_api.h>

#include <SDL/SDL.h>

#if defined(_CK4215_) && defined(WIN32)
#include <SDL/SDL_syswm.h>
#endif

///////////////////////////////////////////////
// TYEPDEFS

typedef struct _vp_stages_output_sdl_config_
{
  uint32_t width;         // in
  uint32_t height;        // in
  uint32_t bpp;           // in

#if defined(_CK4215_) && defined(WIN32)
  uint32_t window_pos_x;  // in
  uint32_t window_pos_y;  // in
#endif

  uint32_t window_width;  // in
  uint32_t window_height; // in

  uint32_t pic_width;     // in
  uint32_t pic_height;    // in

  uint32_t y_size;        // in
  uint32_t c_size;        // in

  // private

  SDL_Surface *surface;
  SDL_Overlay *overlay;

  uint32_t received;

} vp_stages_output_sdl_config_t;


///////////////////////////////////////////////
// FUNCTIONS


/**
 * @fn      vp_stages_buffer_to_overlay
 * @param   SDL_Overlay *overlay
 * @param   output_sdl_config_t *cfg
 * @param   vp_api_io_data_t *data
 * @todo    A COMMENTER
 * @return  VOID
 */
/*
void
vp_stages_buffer_to_overlay(SDL_Overlay *overlay, vp_stages_output_sdl_config_t *cfg, vp_api_io_data_t *data);
*/
/**
 * @fn      vp_stages_display_frame
 * @param   output_sdl_config_t *cfg
 * @param   vp_api_io_data_t *dat
 * @todo    A COMMENTER
 * @return  0
 */
/*
int
vp_stages_display_frame(vp_stages_output_sdl_config_t *cfg, vp_api_io_data_t *data);
*/

#ifdef __cplusplus
extern "C"
{
#endif

#if defined(_CK4215_) && defined(WIN32)
void
vp_stages_init_display(void * handle);

void *
vp_stages_get_child_window( void );

#endif
/**
 * @fn      Open the output sdl stage
 * @param   vp_stages_output_sdl_config_t *cfg
 * @todo    A COMMENTER
 * @return  VP_SUCCESS or VP_FAILURE
 */
C_RESULT
vp_stages_output_sdl_stage_open(vp_stages_output_sdl_config_t *cfg);

/**
 * @fn      Open the output sdl stage
 * @param   vp_stages_output_sdl_config_t *cfg
 * @param   vp_api_io_data_t *in
 * @param   vp_api_io_data_t *out
 * @todo    A COMMENTER
 * @return  VP_SUCCESS or VP_FAILURE
 */
C_RESULT
vp_stages_output_sdl_stage_transform(vp_stages_output_sdl_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out);

/**
 * @fn      Open the output sdl stage
 * @param   vp_stages_output_sdl_config_t *cfg
 * @todo    A COMMENTER
 * @return  VP_SUCCESS or VP_FAILURE
 */
C_RESULT
vp_stages_output_sdl_stage_close(vp_stages_output_sdl_config_t *cfg);

extern const vp_api_stage_funcs_t vp_stages_output_sdl_funcs;

#endif // ! __NDS__

// vp_stages_o_sdl
/** @} */
// VP_Stages
/** @} */
// VP_SDK
/** @} */

#ifdef __cplusplus
}
#endif

#endif // _VP_STAGES_O_SDL_H_
