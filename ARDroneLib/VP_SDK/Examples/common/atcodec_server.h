#ifndef _ATCODEC_SERVER_H_
#define _ATCODEC_SERVER_H_


#include <ATcodec/ATcodec_api.h>


AT_CODEC_ERROR_CODE at_cgmi(ATcodec_Memory_t *mem, ATcodec_Memory_t *output, int *len);
AT_CODEC_ERROR_CODE at_pm_qw(ATcodec_Memory_t *mem, ATcodec_Memory_t *output, int *len);
AT_CODEC_ERROR_CODE at_pm_exe(ATcodec_Memory_t *mem, ATcodec_Memory_t *output, int *len);


AT_CODEC_ERROR_CODE AT_CODEC_init(void);
AT_CODEC_ERROR_CODE AT_CODEC_shutdown(void);
AT_CODEC_ERROR_CODE AT_CODEC_open(void);
AT_CODEC_ERROR_CODE AT_CODEC_close(void);
AT_CODEC_ERROR_CODE AT_CODEC_write(int8_t *buffer, int32_t *len);
AT_CODEC_ERROR_CODE AT_CODEC_read(int8_t *buffer, int32_t *len);


#endif // ! _ATCODEC_SERVER_H_
