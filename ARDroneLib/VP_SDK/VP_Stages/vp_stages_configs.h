
/**
 *  \brief    VP Stages. Vision Stage default configurations
 *  \author   Aurelien Morelle <aurelien.morelle@parrot.com>
 *  \version  1.0
 *  \date     first release 2007/11/06
 */

#ifndef _VP_STAGES_DEFAULT_CONFIG_
#define _VP_STAGES_DEFAULT_CONFIG_


#include <VP_Api/vp_api.h>
#include <VP_Api/vp_api_picture.h>


extern vp_api_picture_t vp_api_picture;


typedef enum _DEFAULT_STAGE_CONFIG_ 
  {
    CONFIG_LB,

    CAMIF_RAW_QCIF_VS6524_CONFIG,
    CAMIF_ENCODING_QCIF_VS6524_CONFIG,
    CAMIF_RAW_QCIF_OV7720_CONFIG,
    CAMIF_ENCODING_QCIF_OV7720_CONFIG,

    ENCODER_MPEG4_CONFIG,
    DECODER_MPEG4_CONFIG,

    SDL_RAW_QCIF_CONFIG,
    SDL_RAW_QQCIF_CONFIG,
    SDL_DECODING_QCIF_CONFIG,
    SDL_RAW_QVGA_CONFIG,
    SDL_DECODING_QVGA_CONFIG,
    SDL_RAW_TWEAKY_QQVGA_CONFIG,

    UART0_COM_CONFIG,
    UART1_COM_CONFIG,
    BLUETOOTH_COM_CONFIG,
    WIFI_COM_CONFIG,

    CONFIG_UB
  }
DEFAULT_STAGE_CONFIG;


C_RESULT
vp_stages_fill_default_config(DEFAULT_STAGE_CONFIG config, void *cfg, size_t size);


#endif // ! _VP_STAGES_DEFAULT_CONFIG_

