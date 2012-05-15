/**
 *  \file     at_msgs_ids.h
 *  \brief    ids for ATCodec
 *  \author   Aurelien Morelle <aurelien.morelle@parrot.com>
 *  \version  1.0
 */

#ifndef _AT_MSGS_IDS_H_
#define _AT_MSGS_IDS_H_

#include <ATcodec/ATcodec_api.h>

typedef struct _AT_CODEC_MSG_IDS_
{
# define ATCODEC_DEFINE_AT_CMD(ID,Str,From,Cb,Prio) AT_CODEC_MSG_ID ID;
# define ATCODEC_DEFINE_AT_RESU(ID,Str,From,Cb) AT_CODEC_MSG_ID ID;

  AT_CODEC_MSG_ID AT_MSG_ATCMD_DEFAULT;

# include <at_msgs.h>
} AT_CODEC_MSG_IDS;


extern AT_CODEC_MSG_IDS ids;

#endif // _AT_MSGS_IDS_H_
