/**
 * @file ATcodec_Error.h
 * @author aurelien.morelle@parrot.fr
 * @date 2006/12/06
 */

#ifndef _AT_CODEC_ERROR_INCLUDE_
#define _AT_CODEC_ERROR_INCLUDE_


// AT codec library error codes

typedef enum _AT_CODEC_GENERAL_ERROR_CODE_
{

  // General AT Codec library error codes
  AT_CODEC_GENERAL_ERROR,
  AT_CODEC_GENERAL_OK,

  // "init" AT Codec library error codes
  AT_CODEC_INIT_ERROR,
  AT_CODEC_INIT_OK,

  // "shutdown" AT Codec library error codes
  AT_CODEC_SHUTDOWN_ERROR,
  AT_CODEC_SHUTDOWN_OK,

  // "enable" AT Codec library return values
  AT_CODEC_ENABLE_ERROR,
  AT_CODEC_ENABLE_OK,

  // "open" AT Codec library error codes
  AT_CODEC_OPEN_ERROR,
  AT_CODEC_OPEN_OK,

  // "close" AT Codec library error codes
  AT_CODEC_CLOSE_ERROR,
  AT_CODEC_CLOSE_OK,

  // "write" AT Codec library error codes
  AT_CODEC_WRITE_ERROR,
  AT_CODEC_WRITE_OK,

  // "read" AT Codec library error codes
  AT_CODEC_READ_ERROR,
  AT_CODEC_READ_OK,

}
AT_CODEC_ERROR_CODE;


#endif // ! _AT_CODEC_ERROR_INCLUDE_
