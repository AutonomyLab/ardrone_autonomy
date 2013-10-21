/**
 * @file ardrone_ftp.h
 * @author nicolas.brulez@parrot.com
 * @date 2011/04/06
 * Copyright Parrot SA. 2011
 */

#ifndef _ARDRONE_FTP_H_
#define _ARDRONE_FTP_H_

#include <VP_Com/vp_com_socket.h>

/**
 * Enum type for ftp function return/error codes
 */
typedef enum
  {
    FTP_FAIL=0,
    FTP_BUSY,
    FTP_SUCCESS,
    FTP_TIMEOUT,
    FTP_BADSIZE,
    FTP_SAMESIZE,
    FTP_PROGRESS,
    FTP_ABORT,
  } _ftp_status;

/**
 * Success test for _ftp_status
 */
static inline int
FTP_SUCCEDED (_ftp_status ftp_result)
{
  return ((FTP_SUCCESS == ftp_result) ||
          (FTP_SAMESIZE == ftp_result));
}

/**
 * Failure test for _ftp_status
 */
static inline int
FTP_FAILED (_ftp_status ftp_result)
{
  return ((FTP_FAIL == ftp_result) ||
          (FTP_TIMEOUT == ftp_result) ||
          (FTP_BADSIZE == ftp_result) ||
          (FTP_ABORT == ftp_result) ||
          (FTP_BUSY == ftp_result));
}

/**
 * FTP structure. Must be allocated by ftpConnect, and cleaned by ftpClose
 */
typedef struct _ftp_s
{
  vp_com_socket_t *socket;
  Read readSock;
  Write writeSock;
  int connected;
  int opInProgress;
  int abortCurrentOp;
  _ftp_status lastStatus;
  char *lastFileList;
  void * tag; // Allows to put any pointer to some useful data that is associated with this ftp connectionS
} _ftp_t;

/**
 * Last error message from ardrone_ftp lib
 * (Useful when FTP_PRINT_ERROR is defined to 0 in ardrone_ftp.c)
 */
extern char *FTPlastErrorMessage;

/**
 * Callback type for all asynchronous operations on FTP.
 * The callback may be called several times with "FTP_PROGRESS" as the status
 * But only once with any other status.
 * @param arg for ftpList : char *pointer to list string (or NULL in case of failure), for ftpGet/ftpPut, float *pointer to progression if status is FTP_PROGRESS, NULL otherwise
 */
typedef void (*ftp_callback)(_ftp_status status, void *arg, _ftp_t *callingFtp);

/**
 * @brief Connect to a FTP server
 * @param ip IP address (ipv4, string format) of the server. (e.g. "192.168.1.1"). Must not be NULL.
 * @param port Port of the FTP server (default ftp port is 21)
 * @param username Username on the server (put "anonymous" for an anonymous connexion). Must not be NULL.
 * @param password Password for the user (put "" for an anonymous connexion). Must not be NULL.
 * @param status Pointer to a _ftp_status value which will hold error codes from this call. Must not be NULL.
 * @return A pointer to the FTP structure allocated during the call, or NULL if the connexion failed.
 */
_ftp_t *ftpConnect (const char *ip, int port, const char *username, const char *password, _ftp_status *status);

/**
 * @brief Connect to a FTP server
 * @param name Host name (string format) of the server. (e.g. "www.google.fr"). Must not be NULL.
 * @param port Port of the FTP server (default ftp port is 21)
 * @param username Username on the server (put "anonymous" for an anonymous connexion). Must not be NULL.
 * @param password Password for the user (put "" for an anonymous connexion). Must not be NULL.
 * @param status Pointer to a _ftp_status value which will hold error codes from this call. Must not be NULL.
 * @return A pointer to the FTP structure allocated during the call, or NULL if the connexion failed.
 */
_ftp_t *ftpConnectFromName (const char *name, int port, const char *username, const char *password, _ftp_status *status);

/**
 * @brief Close a connexion to a FTP server (Server disconnexion and cleanup)
 * @param ftp Pointer to a FTP structure pointer (works with NULL FTP structure pointers).
 * @return Returns FTP_SUCCESS if the connexion to the server was opened, else FTP_FAIL. In any case of failure, the ftp pointer is reset to NULL and all possible cleanup was done.
 */
_ftp_status ftpClose (_ftp_t **ftp);

/**
 * @brief Send a file to a connected FTP server
 * @param ftp A pointer to a connected (created by ftpConnect) FTP server.
 * @param localName Name of the file on the local filesystem (relative or absolute path). Must not be NULL.
 * @param remoteName Name of the file that will be created on the FTP server (in server PWD !). Must not be NULL.
 * @param useResume Flag for using or not the transfert resume function. If activated, this function will try to complete the distant file with bytes from local file.
 * @param callback Callback that will be called upon progress/completion of the command. If NULL is specified, the function won't return until completion/failure (This functionnality is NOT thread safe !).
 * @return If callback is not NULL, the return values can be :
 * @return  - FTP_SUCCESS : Operation is launched (Result of the operation will be passed as the "status" arg of callback)
 * @return  - FTP_BUSY : An operation is already in progress for this FTP connexion.
 * @return  - FTP_FAIL : Unexpected error (see error message)
 * @return If callback is NULL, the return values can be :
 * @return  - FTP_SUCCESS : No errors, transfert was OK
 * @return  - FTP_TIMEOUT : Timeout during transfert (retry with useResume=1 to resume the transfert)
 * @return  - FTP_BADSIZE : useResume was set to 1 and the distant file is greater than the local file
 * @return  - FTP_SAMESIZE : useResume was set to 1 and the distant file has the same size as the local file (no data transfert was done)
 * @return  - FTP_FAIL : Unexpected error (see error message)
 * @return  - FTP_ABORT : Operation was aborted
 */
_ftp_status ftpPut (_ftp_t *ftp, const char *localName, const char *remoteName, int useResume, ftp_callback callback);

/**
 * @brief Get a file from a connected FTP server
 * @param ftp A pointer to a connected (created by ftpConnect) FTP server.
 * @param remoteName Name of the file that will be got from the FTP server (in server PWD !). Must not be NULL.
 * @param localName Name of the file on the local filesystem (relative or absolute path). Must not be NULL.
 * @param useResume Flag for using or not the transfert resume function. If activated, this function will try to complete the local file with bytes from distant file.
 * @param callback Callback that will be called upon progress/completion of the command. If NULL is specified, the function won't return until completion/failure (This functionnality is NOT thread safe !).
 * @return If callback is not NULL, the return values can be :
 * @return  - FTP_SUCCESS : Operation is launched (Result of the operation will be passed as the "status" arg of callback)
 * @return  - FTP_BUSY : An operation is already in progress for this FTP connexion.
 * @return  - FTP_FAIL : Unexpectee error (see error message)
 * @return If callback is NULL, the return values can be :
 * @return  - FTP_SUCCESS : No errors, transfert was OK
 * @return  - FTP_TIMEOUT : Timeout during transfert (retry with useResume=1 to resume the transfert)
 * @return  - FTP_BADSIZE : useResume was set to 1 and the local file is greater than the distant file
 * @return  - FTP_SAMESIZE : useResume was set to 1 and the local file has the same size as the distant file (no data transfert was done)
 * @return  - FTP_FAIL : Unexpected error (see error message)
 * @return  - FTP_ABORT : Operation was aborted
 */
_ftp_status ftpGet (_ftp_t *ftp, const char *remoteName, const char *localName, int useResume, ftp_callback callback);

/**
 * @brief Get a list of the current directory on a connected FTP server
 * @param ftp A pointer to a connected (created by ftpConnect) FTP server.
 * @param fileList Must to be NULL if callback is NULL. This function allocate list result. YOU MUST TO DISALLOCATE THE POINTER
 * @param callback Callback that will be called upon progress/completion of the command. If NULL is specified, the function won't return until completion/failure (This functionnality is NOT thread safe !).
 * @return If callback is not NULL, the return values can be :
 * @return  - FTP_SUCCESS : Operation is launched (Result of the operation will be passed as the "status" arg of callback)
 * @return  - FTP_BUSY : An operation is already in progress for this FTP connexion.
 * @return  - FTP_FAIL : Unexpectee error (see error message)
 * @return If callback is NULL, the return values can be :
 * @return  - FTP_SUCCESS : No errors, transfert was OK
 * @return  - FTP_TIMEOUT : Timeout during transfert
 * @return  - FTP_FAIL : Unexpected error (see error message)
 * @return  - FTP_ABORT : Operation was aborted
 */
_ftp_status ftpList (_ftp_t *ftp, char **fileList,  ftp_callback callback);

/**
 * @brief Remove a file from a connected FTP server
 * @param ftp A pointer to a connected (created by ftpConnect) FTP server.
 * @param remoteName Name (in server PWD) of the file to remove. Must not be NULL.
 * @return The return values can be :
 * @return  - FTP_SUCCESS : Deletion was OK (even if the file did not exist on server)
 * @return  - FTP_TIMEOUT : Timeout during command transfert
 * @return  - FTP_FAIL : Unexpected error (see error message)
 */
_ftp_status ftpRemove (_ftp_t *ftp, const char *remoteName);

/**
 * @brief Rename a file/directory on a connected FTP server
 * @param ftp A pointer to a connected (created by ftpConnect) FTP server.
 * @param origin Initial name of the file/directory on the server. Must not be NULL.
 * @param dest New name of the file/directory on the server. Must not be NULL.
 * @return The return values can be :
 * @return  - FTP_SUCCESS : Renaming was OK
 * @return  - FTP_SAMESIZE : origin file does not exist on server
 * @return  - FTP_TIMEOUT : Timeout during command transfert
 * @return  - FTP_FAIL : Unexpected error (see error message)
 */
_ftp_status ftpRename (_ftp_t *ftp, const char *origin, const char *dest);

/**
 * @brief Change working directory on a connected FTP server
 * @param ftp A pointer to a connected (created by ftpConnect) FTP server.
 * @param nextDir New working directory on server (relative path). Must not be NULL.
 * @return The return values can be :
 * @return  - FTP_SUCCESS : Changed directory
 * @return  - FTP_TIMEOUT : Timeout during command transfert
 * @return  - FTP_FAIL : Unexpected error (see error message)
 */
_ftp_status ftpCd (_ftp_t *ftp, const char *nextDir);

/**
 * @brief Get the working directory of a connected FTP server
 * @param ftp A pointer to a connected (created by ftpConnect) FTP server.
 * @param workingDir Pointer that will hold the server PWD. Must not be NULL.
 * @param wdLen Size of workingDir array (used internally for strncpy)
 * @return The return values can be :
 * @return  - FTP_SUCCESS : Got server PWD in workingDir
 * @return  - FTP_TIMEOUT : Timeout during command transfert
 * @return  - FTP_FAIL : Unexpected error (see error message)
 */
_ftp_status ftpPwd (_ftp_t *ftp, char *workingDir, int wdLen);

/**
 * @brief Create a directory on a connected FTP server
 * @param ftp A pointer to a connected (created by ftpConnect) FTP server.
 * @param dirName Name of the directory to create (in server PWD). Must not be NULL.
 * @return The return values can be :
 * @return  - FTP_SUCCESS : Successfully created dirName directory
 * @return  - FTP_TIMEOUT : Timeout during command transfert
 * @return  - FTP_FAIL : Unexpected error (see error message)
 */
_ftp_status ftpMkdir (_ftp_t *ftp, const char *dirName);

/**
 * @brief Remove a directory from a connected FTP server
 * @param ftp A pointer to a connected (created by ftpConnect) FTP server.
 * @param dirName Name of the directory to delete (in server PWD). Must not be NULL.
 * @return The return values can be :
 * @return  - FTP_SUCCESS : Successfully deleted dirName directory
 * @return  - FTP_BADSIZE : dirName directory didn't exist, or was not empty
 * @return  - FTP_TIMEOUT : Timeout during command transfert
 * @return  - FTP_FAIL : Unexpected error (see error message)
 */
_ftp_status ftpRmdir (_ftp_t *ftp, const char *dirName);

/**
 * @brief Abort current FTP operation
 * @param ftp A pointer to a connected (created by ftpConnect) FTP server.
 * @return The return values can be :
 * @return  - FTP_SUCCESS : Successfully aborted and operation
 * @return  - FTP_SAMESIZE : FTP had no operation in progress
 * @return  - FTP_FAIL : FTP is not open/initialized
 */
_ftp_status ftpAbort (_ftp_t *ftp);

#endif // _ARDRONE_FTP_H_
