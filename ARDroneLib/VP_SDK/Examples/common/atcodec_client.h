#ifndef _ATCODEC_SERVER_H_
#define _ATCODEC_SERVER_H_


#include <ATcodec/ATcodec_api.h>


typedef struct _AT_CODEC_MSG_IDS_
{
# define ATCODEC_DEFINE_AT_CMD(ID,Str,From,Cb,Prio) AT_CODEC_MSG_ID ID;
# define ATCODEC_DEFINE_AT_RESU(ID,Str,From,Cb) AT_CODEC_MSG_ID ID;

  AT_CODEC_MSG_ID AT_MSG_ATCMD_DEFAULT;

# include AT_MESSAGES_HEADER
}
AT_CODEC_MSG_IDS;


AT_CODEC_ERROR_CODE atresu_ok(ATcodec_Memory_t *mem, ATcodec_Memory_t *output, int *len);
AT_CODEC_ERROR_CODE atresu_error(ATcodec_Memory_t *mem, ATcodec_Memory_t *output, int *len);
AT_CODEC_ERROR_CODE atresu_cgmi(ATcodec_Memory_t *mem, ATcodec_Memory_t *output, int *len);


AT_CODEC_ERROR_CODE AT_CODEC_Client_init(void);
AT_CODEC_ERROR_CODE AT_CODEC_Client_shutdown(void);
AT_CODEC_ERROR_CODE AT_CODEC_Client_open(void);
AT_CODEC_ERROR_CODE AT_CODEC_Client_close(void);
AT_CODEC_ERROR_CODE AT_CODEC_Client_write(int8_t *buffer, int32_t *len);
AT_CODEC_ERROR_CODE AT_CODEC_Client_read(int8_t *buffer, int32_t *len);


#endif // ! _ATCODEC_SERVER_H_
