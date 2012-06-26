/**
 * @file ATcodec_api.h
 * @author aurelien.morelle@parrot.fr
 * @date 2006/12/06
 * modified on 2010/07/19 by stephane.piskorski.ext@parrot.fr (bug fix+comments)
 */

#ifndef _AT_CODEC_LIBRARY_INCLUDE_
#define _AT_CODEC_LIBRARY_INCLUDE_


#include <VP_Os/vp_os_types.h>
#include <VP_Os/vp_os_thread.h>

#include <ATcodec/ATcodec.h>
#include <ATcodec/ATcodec_Error.h>
#include <ATcodec/ATcodec_Memory.h>
#include <ATcodec/ATcodec_Tree.h>


#ifndef ATCODEC_STACK_SIZE
# define ATCODEC_STACK_SIZE 40000
#endif // ! ATCODEC_STACK_SIZE


 /**
 * AT Codec reading mode.
 * AT Codec can read incoming commands from a stream or from packets.
 */
typedef enum {
	ATCODEC_READ_FROM_STREAM=1, /**< AT Commands are read from a stream; incoming data is fused until a full command is received. */
 	ATCODEC_READ_FROM_PACKETS /**< AT Commands are read from packets which are processed independently, one at a time. */
} AT_CODEC_READING_MODE;


// Functions pointers typedefs

typedef struct _AT_CODEC_FUNCTIONS_PTRS_
{
  AT_CODEC_ERROR_CODE (*init)     (void);
  AT_CODEC_ERROR_CODE (*shutdown) (void);
  AT_CODEC_ERROR_CODE (*enable)   (void);
  AT_CODEC_ERROR_CODE (*open)     (void);
  AT_CODEC_ERROR_CODE (*close)    (void);
  AT_CODEC_ERROR_CODE (*write)    (uint8_t *buffer, int32_t *len);
  AT_CODEC_ERROR_CODE (*read)     (uint8_t *buffer, int32_t *len);
}
AT_CODEC_FUNCTIONS_PTRS;


// Others typedefs

typedef int AT_CODEC_MSG_ID;
typedef AT_CODEC_ERROR_CODE (*AT_CODEC_Message_Received)(ATcodec_Memory_t *input, ATcodec_Memory_t *output, AT_CODEC_MSG_ID *id);


typedef struct _ATcodec_Message_Data_
{
  int static_str;
  int dynamic_str;
  int total_str;

  AT_CODEC_MSG_ID from_cmd;
  AT_CODEC_Message_Received func_ptr;

  int priority; // \todo To be used
}
ATcodec_Message_Data_t;


// API

/**
 * Initialization of the ATcodec library.
 *
 * @param  funcs                   Functions pointers
 */
void
ATcodec_Init_Library          (AT_CODEC_FUNCTIONS_PTRS *funcs);


/**
 * Initialization of the ATcodec library reading mode (default is STREAMING).
 *
 * @param  _mode Tells AT Codec to read data in a streaming manner or packet by packet.
 */
void
ATcodec_Set_Reading_Mode(AT_CODEC_READING_MODE _mode);


/**
 * Initialization of the ATcodec library.
 *
 * @param  tree                    Messages tree
 * @param  funcs                   Functions pointers
 */
void
ATcodec_Init_Library_Tree     (ATcodec_Tree_t *tree, AT_CODEC_FUNCTIONS_PTRS *funcs);

/**
 * Cleans all allocated data.
 */
void
ATcodec_Shutdown_Library      (void);

/**
 * Cleans all allocated data.
 *
 * @param  tree                    Messages tree
 */
void
ATcodec_Shutdown_Library_Tree (ATcodec_Tree_t *tree);

/**
 * AT message received default callback that does nothing.
 *
 * @param  input                   Input parameters of the message received
 * @param  output                  Output parameters of the message answer to send
 * @param  size_max                Maximum size of the ouput memory
 * @param  id                      ID of the message answer to send
 *
 * @retVal AT_CODEC_GENERAL_OK     If OK
 */
AT_CODEC_ERROR_CODE
at_default_cb(ATcodec_Memory_t *input, ATcodec_Memory_t *output, int *id);

/**
 * Add a message that will be automatically processed by the ATcodec library when it is received.
 *
 * @param  str                     String of the message
 * @param  from_cmd                For a result, the command ID it is from
 * @param  func_ptr                Pointer to the callback when message is received
 *
 * @retVal -1                      If an error occurs
 * @retVal A new ID                If initialization OK
 */
AT_CODEC_MSG_ID
ATcodec_Add_Hashed_Message       (const char *str, AT_CODEC_MSG_ID from_cmd, AT_CODEC_Message_Received func_ptr, int priority);

/**
 * Add a message that will be automatically processed by the ATcodec library when it is received.
 *
 * @param  tree                    Messages tree
 * @param  str                     String of the message
 * @param  from_cmd                For a result, the command ID it is from
 * @param  func_ptr                Pointer to the callback when message is received
 *
 * @retVal -1                      If an error occurs
 * @retVal A new ID                If initialization OK
 */
AT_CODEC_MSG_ID
ATcodec_Add_Hashed_Message_Tree  (ATcodec_Tree_t *tree, const char *str, AT_CODEC_MSG_ID from_cmd, AT_CODEC_Message_Received func_ptr, int priority);

/**
 * Add a message just for the user to have the ID to be able to send it via the ATcodec library.
 *
 * @param  str                     String of the message
 *
 * @retVal -1                      If an error occurs
 * @retVal A new ID                If initialization OK
 */
AT_CODEC_MSG_ID
ATcodec_Add_Defined_Message      (const char *str);

/**
 * Add a message just for the user to have the ID to be able to send it via the ATcodec library.
 *
 * @param  tree                    Messages tree
 * @param  str                     String of the message
 *
 * @retVal -1                      If an error occurs
 * @retVal A new ID                If initialization OK
 */
AT_CODEC_MSG_ID
ATcodec_Add_Defined_Message_Tree (ATcodec_Tree_t *tree, const char *str);

/**
 * Requires ATcodec AT client to be started.
 * Add an AT message to the sending queue.
 * Sending is done automatically.
 *
 * @param  id                      ID of the AT command to generate
 * @param  params                  Params used for the generation of the command
 *
 * @retVal ATCODEC_TRUE            If it is OK
 * @retVal ATCODEC_FALSE           If it fails
 */
ATCODEC_RET
ATcodec_Queue_Message_params     (AT_CODEC_MSG_ID id, ATcodec_Memory_t *params);

/**
 * Requires ATcodec AT client to be started.
 * Add an AT message to the sending queue.
 * Sending is done automatically.
 *
 * @param  tree                    Messages tree
 * @param  id                      ID of the AT command to generate
 * @param  params                  Params used for the generation of the command
 *
 * @retVal ATCODEC_TRUE            If it is OK
 * @retVal ATCODEC_FALSE           If it fails
 */
ATCODEC_RET
ATcodec_Queue_Message_params_Tree(ATcodec_Tree_t *tree, AT_CODEC_MSG_ID id, ATcodec_Memory_t *params);

/**
 * Requires ATcodec AT client to be started.
 * Same function as ATcodec_Queue_Message_params with parameters passed through a valist.
 *
 * @param  id                      ID of the AT command to generate
 * @param  params                  Params used for the generation of the command
 *
 * @retVal ATCODEC_TRUE            If it is OK
 * @retVal ATCODEC_FALSE           If it fails
 */
ATCODEC_RET
ATcodec_Queue_Message_valist     (AT_CODEC_MSG_ID id, ...);

/**
 * Requires ATcodec AT client to be started.
 * Same function as ATcodec_Queue_Message_params with parameters passed through a valist.
 *
 * @param  tree                    Messages tree
 * @param  id                      ID of the AT command to generate
 * @param  params                  Params used for the generation of the command
 *
 * @retVal ATCODEC_TRUE            If it is OK
 * @retVal ATCODEC_FALSE           If it fails
 */
ATCODEC_RET
ATcodec_Queue_Message_valist_Tree(ATcodec_Tree_t *tree, AT_CODEC_MSG_ID id, ...);

/**
 * Send pushed messages.
 *
 * @retVal ATCODEC_TRUE            If it is OK
 * @retVal ATCODEC_FALSE           If it fails
 */
ATCODEC_RET
ATcodec_Send_Messages(void);

/**
 * ATcodec AT server that processes AT commands.
 *
 * @param  data                    Data passed to thread routine
 */
DEFINE_THREAD_ROUTINE(ATcodec_Commands_Server, data);

/**
 * ATcodec AT client that sends AT commands.
 *
 * @param  data                    Data passed to thread routine
 */
DEFINE_THREAD_ROUTINE(ATcodec_Commands_Client, data);

/**
 * Exits from current thread execution (either client and server)
 *
 * @param  void
 */
void
ATcodec_exit_thread(void);

/**
 * Print the tree created by the adding of the messages.
 */
void
ATcodec_Print_Tree(void);


#endif // -> _AT_CODEC_LIBRARY_INCLUDE_

