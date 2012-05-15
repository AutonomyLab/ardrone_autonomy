/**
 * @file ardrone_ftp.c
 * @author nicolas.brulez@parrot.com
 * @date 2011/04/06
 * Copyright Parrot SA. 2011
 */

#include <utils/ardrone_ftp.h>

#include <stdio.h>
#include <netdb.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <VP_Os/vp_os_thread.h>
#include <VP_Os/vp_os_malloc.h>
#include <VP_Os/vp_os_signal.h>

#include <errno.h>

/* CONFIGURATION */

// All following macros MUST be defined !
// To activate, define to (1)
// To deactivate, define to (0)

#ifdef DEBUG // Debug options
#define _FTP_DEBUG (1) // Common debug informations
#define _FTP_VERBOSE (0) // Extended debug information (many outputs on ftpList)
#define _FTP_ERRORS_PRINT (1) // Display of error messages
#else // Release options
#define _FTP_DEBUG (0)
#define _FTP_VERBOSE (0)
#define _FTP_ERRORS_PRINT (1)
#endif

#define FTP_PREFIX "FTP : "

/* LOCAL PRINT MACROS */

#if _FTP_ERRORS_PRINT
#define FTP_ERROR(...)                                                  \
  do                                                                    \
    {                                                                   \
      fprintf (stderr, "Error in function %s at line %d : ", __FUNCTION__, __LINE__); \
      fprintf (stderr, __VA_ARGS__);                                    \
      char errorBuffer [512] = {0};                                     \
      snprintf (errorBuffer, sizeof (errorBuffer)-1, __VA_ARGS__);      \
      FTPlastErrorMessageSize = strlen (errorBuffer) + 1;               \
      FTPlastErrorMessage = vp_os_realloc (FTPlastErrorMessage, FTPlastErrorMessageSize); \
      if (NULL != FTPlastErrorMessage)                                  \
        {                                                               \
          strncpy (FTPlastErrorMessage, errorBuffer, FTPlastErrorMessageSize); \
        }                                                               \
    } while (0)
#else
#define FTP_ERROR(...)                                                  \
  do                                                                    \
    {                                                                   \
      char errorBuffer [512] = {0};                                     \
      snprintf (errorBuffer, sizeof (errorBuffer)-1, __VA_ARGS__);      \
      FTPlastErrorMessageSize = strlen (errorBuffer) + 1;               \
      FTPlastErrorMessage = vp_os_realloc (FTPlastErrorMessage, FTPlastErrorMessageSize); \
      if (NULL != FTPlastErrorMessage)                                  \
        {                                                               \
          strncpy (FTPlastErrorMessage, errorBuffer, FTPlastErrorMessageSize); \
        }                                                               \
    } while (0)
#endif

#if _FTP_DEBUG
#define FTP_DEBUG(...)                                                  \
  do                                                                    \
    {                                                                   \
      printf ("Debug from function %s at line %d : ", __FUNCTION__, __LINE__); \
      printf (__VA_ARGS__);                                             \
    } while (0)
#else
#define FTP_DEBUG(...)
#endif

#if _FTP_VERBOSE
#define FTP_PRINT(...)                          \
  do                                            \
    {                                           \
      printf (FTP_PREFIX);                      \
      printf (__VA_ARGS__);                     \
    } while (0)
#else
#define FTP_PRINT(...)
#endif

/* SIZE MACROS */

#ifndef MAX_SIZE_MSG
#define MAX_SIZE_MSG 32768
#endif
#define IP_STRING_SIZE 16 // IP strings goes from 8 ("w.x.y.z\0") to 16 ("www.xxx.yyy.zzz\0") chars
#define LIST_BUFFER_BLOCKSIZE 1024
#define FILE_NAME_MAX_SIZE 512

/* TIMEOUT MACROS */

/* Total socket timeout : SOCK_TO_SEC + SOCK_TO_USEC */
#define SOCK_TO_SEC 1 // Socket timeout (seconds)
#define SOCK_TO_USEC 0 // Socket timeout (useconds)

/* GLOBAL ERROR MESSAGE STRING */
char *FTPlastErrorMessage = NULL;
int FTPlastErrorMessageSize = 0;

/* THREAD STRUCTURES */

typedef struct _ftp_list_param_s _ftp_list_param;
typedef struct _ftp_get_param_s _ftp_get_param;
typedef struct _ftp_put_param_s _ftp_put_param;

struct _ftp_put_param_s
{
  _ftp_t *ftp;
  char localName [FILE_NAME_MAX_SIZE];
  char remoteName [FILE_NAME_MAX_SIZE];
  int useResume;
  ftp_callback callback;
  char *fileList;
};

struct _ftp_get_param_s
{
  _ftp_t *ftp;
  char localName [FILE_NAME_MAX_SIZE];
  char remoteName [FILE_NAME_MAX_SIZE];
  int useResume;
  ftp_callback callback;
  char *fileList;
};

struct _ftp_list_param_s
{
  _ftp_t *ftp;
  char *fileList;
  int listSize;
  ftp_callback callback;
};

/* LOCAL FUNCTIONS PROTOTYPES */
void emptyCallback (_ftp_status status, void *arg, _ftp_t *callingFtp);
_ftp_status waitFor226Answer (_ftp_t *ftp);
int setSockTimeout (int socket, int timeoutSec, int timeoutUsec);
_ftp_status goToBinaryMode (_ftp_t *ftp);
void flushFtp (_ftp_t *ftp);
int getFileSize (_ftp_t *ftp, const char *distPath);
int getLocalFileSize (const char *localPath);
int getResponseCode (const char *response);
_ftp_status getPassiveIpAndPort (const char *response, char *ip, int *port, int ipLen);
_ftp_status ftpTransfert (_ftp_t *ftp, const char *message, char *answer, int answSize);
_ftp_status ftpSend (_ftp_t *ftp, const char *message);
_ftp_status ftpRecv (_ftp_t *ftp, char *answer, int answSize);
DEFINE_THREAD_ROUTINE (ftpGet, param);
DEFINE_THREAD_ROUTINE (ftpPut, param);
DEFINE_THREAD_ROUTINE (ftpList, param);

/* GLOBAL CALLBACK RESULT */
_ftp_status lastStatusFromEmptyCallback = FTP_FAIL;
char *lastFileListFromEmptyCallback = NULL;

/* FUNCTIONS IMPLEMENTATION */

void
emptyCallback (_ftp_status status, void *arg, _ftp_t *callingFtp)
{
  FTP_PRINT ("Called with status %d\n", status);
#if _FTP_VERBOSE
  if (FTP_PROGRESS == status)
    {
      FTP_PRINT ("Trying float arg : %f\n", (NULL != arg) ? *(float *)arg : -1.0);
    }
#endif
  lastStatusFromEmptyCallback = status;
  if(FTP_SUCCESS == status && NULL != arg)
    {
      lastFileListFromEmptyCallback = (char *)arg;
    }
}

#define FTP_MAX_NUM_RETRIES 5
_ftp_status
waitFor226Answer (_ftp_t *ftp)
{
  char srvMsg[MAX_SIZE_MSG] = {0};
  int repCode = 0;
  int numretries = FTP_MAX_NUM_RETRIES;

  _ftp_status ftp_result = FTP_SUCCESS;
  while (226 != repCode && 0 < numretries)
    {
      ftp_result = ftpRecv (ftp, srvMsg, MAX_SIZE_MSG-1);
      if (FTP_FAILED (ftp_result))
        {
          numretries--;
        }
      repCode = getResponseCode (srvMsg);
    }
  return ftp_result;
}

int
setSockTimeout (int socket, int timeoutSec, int timeoutUsec)
{
#ifdef _WIN32
  int winTO = (1000 * timeoutSec) + (timeoutUsec / 1000);
#else
  struct timeval posixTO;
  posixTO.tv_sec = timeoutSec;
  posixTO.tv_usec = timeoutUsec;
#endif

  if (0 > setsockopt (socket, SOL_SOCKET, SO_RCVTIMEO,
#ifdef _WIN32
                      (const char *)&winTO, sizeof (winTO)
#else
                      (const char *)&posixTO, sizeof (posixTO)
#endif
                      ))
    {
      FTP_ERROR ("Unable to set recv timeout\n");
      return -1;
    }

  if (0 > setsockopt (socket, SOL_SOCKET, SO_SNDTIMEO,
#ifdef _WIN32
                      (const char *)&winTO, sizeof (winTO)
#else
                      (const char *)&posixTO, sizeof (posixTO)
#endif
                      ))
    {
      FTP_ERROR ("Unable to set send timeout\n");
      return -1;
    }
  return 0;
}

_ftp_status
goToBinaryMode (_ftp_t *ftp)
{
  char ftpAnswer[256] = {0};
  _ftp_status ftp_result = ftpTransfert (ftp, "TYPE I\r\n\0", ftpAnswer, 255);
  if (FTP_FAILED (ftp_result))
    {
      FTP_ERROR ("Unable to go to binary mode\n");
    }
  return ftp_result;
}

void
flushFtp (_ftp_t *ftp)
{
  FTP_DEBUG ("Starting flush\n");
  char c = 0;
  int bytes = 1;
  int flushedBytes = 0;
  VP_COM_SOCKET_BLOCKING_OPTIONS oldOptions = ftp->socket->block;
  ftp->socket->block = VP_COM_DONTWAIT;
  C_RESULT vp_result = ftp->readSock (ftp->socket, (int8_t *)&c, &bytes);
  while (0 < bytes && VP_SUCCEEDED (vp_result))
    {
      flushedBytes++;
#if _FTP_DEBUG
      printf ("%c", c);
#endif
      vp_result = ftp->readSock (ftp->socket, (int8_t *)&c, &bytes);
    }
  FTP_DEBUG ("Flushed %d bytes\n", flushedBytes);
  ftp->socket->block = oldOptions;
}

int
getFileSize (_ftp_t *ftp, const char *distPath)
{
  char ftpCommand[256] = {0};
  snprintf (ftpCommand, sizeof (ftpCommand)-1, "SIZE %s\r\n", distPath);
  char ftpAnswer[256] = {0};
  _ftp_status ftp_result = ftpTransfert (ftp, ftpCommand, ftpAnswer, sizeof (ftpAnswer)-1);
  if (FTP_FAILED (ftp_result))
    {
      FTP_ERROR ("Unable to get file size\n");
      return -1;
    }
  int size = -1;
  int repCode = 0;
  sscanf (ftpAnswer, "%d %d", &repCode, &size);
  return size;
}

int
getLocalFileSize (const char *localPath)
{
  FILE *localFile = fopen (localPath, "r");
  if (NULL == localFile)
    {
      FTP_DEBUG ("File %s does not exist\n", localPath);
      return -1;
    }
  fseek (localFile, 0, SEEK_END);
  int size = (int)ftell (localFile);
  FTP_DEBUG ("Size of file %s : %d o\n", localPath, size);
  fclose (localFile);
  return size;
}

int
getResponseCode (const char *response)
{
  int retVal = -1;
  sscanf (response, "%d", &retVal);
  return retVal;
}

_ftp_status
getPassiveIpAndPort (const char *response, char *ip, int *port, int ipLen)
{
  int ip1, ip2, ip3, ip4, port1, port2;
  int indexOfFirstIpField = 0;
  char atIndex = '\0';
  int maxIndex = strlen (response);
  while (indexOfFirstIpField < maxIndex && '(' != atIndex)
    {
      atIndex = response[indexOfFirstIpField++];
    }
  int numread = sscanf (&response[indexOfFirstIpField], "%d,%d,%d,%d,%d,%d)", &ip1, &ip2, &ip3, &ip4, &port1, &port2);
  _ftp_status result = FTP_SUCCESS;
  if (6 == numread)
    {
      snprintf (ip, ipLen-1, "%d.%d.%d.%d", ip1, ip2, ip3, ip4);
      *port = 256 * port1 + port2;
      FTP_DEBUG ("IP : %s | Port : %d\n", ip, *port);
    }
  else
    {
      result = FTP_FAIL;
    }
  return result;
}

_ftp_status
ftpTransfert (_ftp_t *ftp, const char *message, char *answer, int answSize)
{
  flushFtp (ftp);
  _ftp_status ftp_result = ftpSend (ftp, message);
  if (FTP_FAILED (ftp_result))
    {
      return ftp_result;
    }
  return ftpRecv (ftp, answer, answSize);
}

_ftp_status
ftpSend (_ftp_t *ftp, const char *message)
{
  if (NULL == ftp)
    {
      FTP_ERROR ("FTP not open\n");
      return FTP_FAIL;
    }
  FTP_DEBUG ("Sending %sto FTP at %s:%d\n", message, ftp->socket->serverHost, ftp->socket->port);

  int bytes = strlen (message);
  C_RESULT vp_result = ftp->writeSock (ftp->socket, (int8_t *)message, &bytes);
  if (VP_FAILED (vp_result))
    {
      FTP_ERROR ("Error while sending data\n");
      return FTP_FAIL;
    }
  if (0 == bytes)
    {
      FTP_ERROR ("Unable to send data\n");
      return FTP_TIMEOUT;
    }
  return FTP_SUCCESS;
}

_ftp_status
ftpRecv (_ftp_t *ftp, char *answer, int answSize)
{
  if (NULL == ftp)
    {
      FTP_ERROR ("FTP not open\n");
      return FTP_FAIL;
    }

  vp_os_memset (answer, 0x0, answSize);
  int index = 0;
  do
    {
      int bytes = 1;
      C_RESULT vp_result = ftp->readSock (ftp->socket, (int8_t *)(&answer [index]), &bytes);
      if (VP_FAILED (vp_result))
        {
          FTP_ERROR ("Error while reading data\n");
          return FTP_FAIL;
        }
      if (0 == bytes)
        {
          FTP_ERROR ("Recv timeout\n");
          return FTP_TIMEOUT;
        }
    }
  while (index < answSize && '\n' != answer [index++]);

  FTP_DEBUG ("Answer:\n<---START--->\n%s\n<---END--->\n", answer);
  return FTP_SUCCESS;
}

_ftp_status
ftpClose (_ftp_t **ftp)
{
  FTP_DEBUG ("Closing ftp\n");
  _ftp_status retVal = FTP_FAIL;
  if (NULL != *ftp)
    {
      FTP_DEBUG ("Not null ftp\n");
      if (NULL != (*ftp)->socket)
        {
          FTP_DEBUG ("Not null socket\n");
          if (1 == (*ftp)->connected)
            {
              if (FTP_SUCCESS == ftpAbort ((*ftp))) // An operation was in progress, abort and let time to cleanup.
              {
                usleep (100000); // 100ms
              }
              ftpSend ((*ftp), "QUIT\r\n\0");
              (*ftp)->connected = 0;
            }
          vp_com_close_socket ((*ftp)->socket);
          vp_os_free ((*ftp)->socket);
          (*ftp)->socket = NULL;
          retVal = FTP_SUCCESS;
        }
      vp_os_free (*ftp);
      *ftp = NULL;
    }
  return retVal;
}

_ftp_t *
ftpConnectFromName (const char *name, int port, const char *username, const char *password, _ftp_status *status)
{
	struct hostent *hostent = gethostbyname(name);
	return ftpConnect(inet_ntoa( *( struct in_addr*)( hostent->h_addr)), port, username, password, status);
}

_ftp_t *
ftpConnect (const char *ip, int port, const char *username, const char *password, _ftp_status *status)
{
  if (NULL == ip ||
      NULL == username ||
      NULL == password ||
      NULL == status)
    {
      FTP_ERROR ("Must not pass NULL pointers to ftpConnect\n");
      if (NULL != status) { *status = FTP_FAIL; }
      return NULL;
    }
  int isAnonymous = ((0 == strcmp (username, "anonymous")) || (0 == strcmp (username, ""))) ? 1 : 0;
  if (1 == isAnonymous)
    {
      FTP_DEBUG ("Connecting to %s:%d, anonymous\n", ip, port);
    }
  else
    {
      FTP_DEBUG ("Connecting to %s:%d, USER = %s, Password = %s\n", ip, port, username, password);
    }

  *status = FTP_FAIL;
  _ftp_t *retFtp = vp_os_malloc (sizeof (_ftp_t));
  if (NULL == retFtp)
    {
      FTP_ERROR ("Unable to allocate a ftp structure\n");
      return NULL;
    }
  retFtp->socket = vp_os_malloc (sizeof (vp_com_socket_t));
  if (NULL == retFtp->socket)
    {
      FTP_ERROR ("Unable to allocate socket filed of the ftp structure\n");
      ftpClose (&retFtp);
      return NULL;
    }

  retFtp->connected = 0;

  retFtp->socket->type = VP_COM_CLIENT;
  retFtp->socket->protocol = VP_COM_TCP;
  retFtp->socket->port = port;
  strncpy (retFtp->socket->serverHost, ip, VP_COM_NAME_MAXSIZE-1);
  retFtp->socket->is_multicast = 0;
  retFtp->socket->block = VP_COM_DEFAULT;

  C_RESULT vp_result = vp_com_open_socket (retFtp->socket, &(retFtp->readSock), &(retFtp->writeSock));
  if (VP_FAILED (vp_result))
    {
      FTP_ERROR ("Unable to connect\n");
      ftpClose (&retFtp);
      return NULL;
    }
  
  int result = setSockTimeout ((int)retFtp->socket->priv, SOCK_TO_SEC, SOCK_TO_USEC);
  if (0 > result)
    {
      FTP_ERROR ("Unable to set socket timeout\n");
      ftpClose (&retFtp);
      return NULL;
    }

  char srvMsg[MAX_SIZE_MSG] = {0};
  if (FTP_FAILED (ftpRecv (retFtp, srvMsg, MAX_SIZE_MSG-1)))
    {
      FTP_ERROR ("Unable to recieve data from server\n");
      ftpClose (&retFtp);
      return NULL;
    }

  int repCode = getResponseCode (srvMsg);
  if (220 != repCode)
    {
      FTP_ERROR ("Bad response from server (%d, expected 220)\n", repCode);
      ftpClose (&retFtp);
      return NULL;
    }


  char buffer[256] = {0};
  if (1 == isAnonymous)
    {
      snprintf (buffer, sizeof (buffer)-1, "USER anonymous\r\n");
    }
  else
    {
      snprintf (buffer, sizeof (buffer)-1, "USER %s\r\n", username);
    }
  if (FTP_FAILED (ftpTransfert (retFtp, buffer, srvMsg, MAX_SIZE_MSG-1)))
    {
      FTP_ERROR ("Error while sending command\n");
      ftpClose (&retFtp);
      return NULL;
    }
  repCode = getResponseCode (srvMsg);
  int goodRepCode = 331;
  if (1 == isAnonymous)
    {
      goodRepCode = 230;
    }
  if (goodRepCode != repCode)
    {
      FTP_ERROR ("Bad response from server (%d, expected %d)\n", repCode, goodRepCode);
      ftpClose (&retFtp);
      return NULL;
    }

  if (0 == isAnonymous)
    {
      vp_os_memset (buffer, 0x0, sizeof (buffer));
      snprintf (buffer, sizeof (buffer)-1, "PASS %s\r\n", password);
      if (FTP_FAILED (ftpTransfert (retFtp, buffer, srvMsg, MAX_SIZE_MSG-1)))
        {
          FTP_ERROR ("Error while sending command\n");
          ftpClose (&retFtp);
          return NULL;
        }
      repCode = getResponseCode (srvMsg);
      if (230 != repCode)
        {
          FTP_ERROR ("Bad response from server (%d, expected 230)\n", repCode);
          ftpClose (&retFtp);
          return NULL;
        }
    }

  *status = FTP_SUCCESS;
  retFtp->connected = 1;
  retFtp->opInProgress = 0;
  retFtp->abortCurrentOp = 0;
  return retFtp;
}


#undef CLEAN_PARAMS
#define CLEAN_PARAMS(status) CLEAN_PARAMS_WITH_ARG(status,NULL)
#undef CLEAN_PARAMS_WITH_ARG
#define CLEAN_PARAMS_WITH_ARG(status,arg)       \
  do                                            \
    {                                           \
      params->ftp->opInProgress = 0;            \
      CLEAN_PARAMS_ABORT (status,arg);          \
    } while (0)
#undef CLEAN_PARAMS_ABORT
#define CLEAN_PARAMS_ABORT(status,arg)                                  \
  do                                                                    \
    {                                                                   \
      _ftp_status locStat = (status);                                   \
      if (NULL != localFile) fclose (localFile);                        \
      if (NULL != dataSocket)                                           \
        {                                                               \
          vp_com_close_socket (dataSocket);                             \
          vp_os_free (dataSocket);                                      \
          dataSocket = NULL;                                            \
        }                                                               \
      params->callback (locStat, arg, params->ftp);                     \
      if (FTP_SUCCESS != status && NULL != params->fileList)            \
        {                                                               \
          vp_os_free (params->fileList);                                \
          params->fileList = NULL;                                      \
        }                                                               \
      vp_os_free (param);                                               \
      FTP_DEBUG ("Returning from thread %s with status %d\n", __FUNCTION__, locStat); \
      THREAD_RETURN (locStat);                                          \
    }                                                                   \
  while (0)
#undef CHECK_ABORT
#define CHECK_ABORT                                                     \
  do                                                                    \
    {                                                                   \
      if (1 <= params->ftp->abortCurrentOp)                             \
        {                                                               \
          vp_os_memset (srvMsg, 0x0, MAX_SIZE_MSG);                     \
          if (NULL != dataSocket)                                       \
            {                                                           \
              vp_com_close_socket (dataSocket);                         \
              vp_os_free (dataSocket);                                  \
              dataSocket = NULL;                                        \
            }                                                           \
          ftpTransfert (params->ftp, "ABOR\r\n\0", srvMsg, MAX_SIZE_MSG-1); \
          flushFtp (params->ftp);                                       \
          params->ftp->abortCurrentOp = 0;                              \
          params->ftp->opInProgress = 0;                                \
          CLEAN_PARAMS_ABORT (FTP_ABORT, NULL);                         \
        }                                                               \
    } while (0)

DEFINE_THREAD_ROUTINE (ftpList, param)
{
  FILE *localFile = NULL; // Compatibilty with macros
  vp_com_socket_t *dataSocket = NULL;
  Read dataRead = NULL;
  Write dataWrite = NULL;
  _ftp_list_param *params = (_ftp_list_param *)param;
  if (NULL == params->ftp)
    {
      FTP_ERROR ("FTP not open\n");
      CLEAN_PARAMS (FTP_FAIL);
    }
  flushFtp (params->ftp);
  
  char srvMsg [MAX_SIZE_MSG] = {0};
  _ftp_status ftp_result = ftpTransfert (params->ftp, "PASV\r\n\0", srvMsg, MAX_SIZE_MSG-1);
  if (FTP_FAILED (ftp_result))
    {
      FTP_ERROR ("Error while sending command\n");
      CLEAN_PARAMS (ftp_result);
    }
  int repCode = getResponseCode (srvMsg);
  if (227 != repCode)
    {
      FTP_ERROR ("Bad response from server (%d, expected 227)\n", repCode);
      CLEAN_PARAMS (FTP_FAIL);
    }
  
  char dataIp [IP_STRING_SIZE] = {0};
  int dataPort = 0;
  getPassiveIpAndPort (srvMsg, dataIp, &dataPort, IP_STRING_SIZE);
  
  FTP_DEBUG ("Thread will connect to %s:%d\n", dataIp, dataPort);
  dataSocket = vp_os_malloc (sizeof (vp_com_socket_t));
  if (NULL == dataSocket)
    {
      FTP_ERROR ("Unable to allocate socket structure\n");
      CLEAN_PARAMS (FTP_FAIL);
    }
  dataSocket->type = VP_COM_CLIENT;
  dataSocket->protocol = VP_COM_TCP;
  dataSocket->port = dataPort;
  strncpy (dataSocket->serverHost, dataIp, VP_COM_NAME_MAXSIZE);
  dataSocket->is_multicast = 0;
  dataSocket->block = VP_COM_WAITALL;
  
  C_RESULT vp_result = vp_com_open_socket (dataSocket, &dataRead, &dataWrite);
  if (VP_FAILED (vp_result))
    {
      FTP_ERROR ("Unable to connect\n");
      CLEAN_PARAMS (FTP_FAIL);
    }
  
  int result = setSockTimeout ((int)dataSocket->priv, SOCK_TO_SEC, SOCK_TO_USEC);
  if (0 > result)
    {
      FTP_ERROR ("Unable to set data socket timeout\n");
      CLEAN_PARAMS (FTP_FAIL);
    }
  
  CHECK_ABORT;
  
  ftp_result = ftpSend (params->ftp, "LIST\r\n\0");
  if (FTP_FAILED (ftp_result))
    {
      FTP_ERROR ("Error while sending LIST command\n");
      CLEAN_PARAMS (ftp_result);
    }
  
  CHECK_ABORT;
  char ftpData [2] = {0};
  int bytes = 1, totalBytes;
  vp_os_memset (params->fileList, 0x0, params->listSize);
  vp_result = dataRead (dataSocket, (int8_t *)ftpData, &bytes);
  if (VP_FAILED (vp_result))
    {
      FTP_ERROR ("Unable to receive data\n");
      CLEAN_PARAMS (FTP_FAIL);
    }
  if(0 == bytes)
    {
      FTP_DEBUG ("Empty folder\n");
      waitFor226Answer (params->ftp);
      flushFtp (params->ftp);
      CLEAN_PARAMS (FTP_SAMESIZE);
    }

  if (strlen (ftpData) > params->listSize - 1)
    {
      params->listSize += LIST_BUFFER_BLOCKSIZE;
      params->fileList = vp_os_realloc(params->fileList, params->listSize);
      if(NULL == params->fileList)
        {
          FTP_ERROR ("Not enough space in response string, can't reallocate buffer list\n");
          CLEAN_PARAMS (FTP_FAIL);
        }
    }
  snprintf (params->fileList, params->listSize-1, "%s", ftpData);
  totalBytes = bytes;
  while (1) // Loop is killed by a return or a break statement
    {
      CHECK_ABORT;
      ftpData [0] = 0;
      bytes = 1;
      vp_result = dataRead (dataSocket, (int8_t *)ftpData, &bytes);
      if (VP_FAILED (vp_result))
        {
          FTP_ERROR ("Unable to receive data\n");
          CLEAN_PARAMS (FTP_FAIL);
        }
      if (0 == bytes && '\n' == params->fileList [totalBytes-1])
        {
          FTP_DEBUG ("Got all listing !\n");
          // Timeouted. We got all the listing !
          // Timeout is voluntary, so still return FTP_SUCCESS
          break;
        }
      else if (0 == bytes)
        {
          // Timeout
          FTP_ERROR ("Recv timeout\n");
          CLEAN_PARAMS (FTP_TIMEOUT);
        }
      if (strlen (ftpData) > (params->listSize - 1 - strlen (params->fileList)))
        {
          params->listSize += LIST_BUFFER_BLOCKSIZE;
          params->fileList = vp_os_realloc(params->fileList, params->listSize);
          if(NULL == params->fileList)
            {
              FTP_ERROR ("Not enough space in response string, can't reallocate buffer list\n");
              CLEAN_PARAMS (FTP_FAIL);
            }
        }
      strcat (params->fileList, ftpData);
      totalBytes += bytes;
      FTP_PRINT ("Progress of listing : finished ? %d : (%d->%d bytes) %s\n", (params->response [totalBytes -1] == '\n') ? 1: 0, bytes, totalBytes, params->response);
    }

  ftp_result = ftpRecv (params->ftp, srvMsg, MAX_SIZE_MSG-1);
  if (FTP_FAILED (ftp_result))
    {
      FTP_ERROR ("Error while getting answer\n"); 
      CLEAN_PARAMS (ftp_result);
    }
  repCode = getResponseCode (srvMsg);
  if (150 != repCode)
    {
      FTP_ERROR ("Bad response from server (%d, expected 150)\n", repCode);
      CLEAN_PARAMS (FTP_FAIL);
    }

  /* Cleaning FTP */
  if (NULL != dataSocket)
    {
      vp_com_close_socket (dataSocket);
      vp_os_free (dataSocket);
      dataSocket = NULL;
    }
  waitFor226Answer (params->ftp);
  flushFtp (params->ftp);


  CLEAN_PARAMS_WITH_ARG (FTP_SUCCESS, (void *)params->fileList);
}

DEFINE_THREAD_ROUTINE (ftpGet, param)
{
  vp_com_socket_t *dataSocket = NULL;
  Read dataRead = NULL;
  Write dataWrite = NULL;
  FILE *localFile = NULL;
  _ftp_get_param *params = (_ftp_get_param *)param;
  FTP_DEBUG ("Downloading %s to %s [resume : %c]\n", params->remoteName, params->localName, params->useResume ? 'y' : 'n');
  if (NULL == params->ftp)
    {
      FTP_ERROR ("FTP not open\n");
      CLEAN_PARAMS (FTP_FAIL);
    }
  flushFtp (params->ftp);

  char buffer[512] = {0};
  char srvMsg[MAX_SIZE_MSG] = {0};
  snprintf (buffer, sizeof (buffer)-1, "PASV\r\n");
  _ftp_status ftp_result = ftpTransfert (params->ftp, buffer, srvMsg, MAX_SIZE_MSG-1);
  if (FTP_FAILED (ftp_result))
    {
      FTP_ERROR ("Error while entering passive mode\n");
      CLEAN_PARAMS (ftp_result);
    }
  int repCode = getResponseCode (srvMsg);
  if (227 != repCode)
    {
      FTP_ERROR ("Bad response from server (%d, expected 227)\n", repCode);
      CLEAN_PARAMS (FTP_FAIL);
    }

  ftp_result = goToBinaryMode (params->ftp);
  if (FTP_FAILED (ftp_result))
    {
      FTP_ERROR ("Unable to go to binary mode\n");
      CLEAN_PARAMS (ftp_result);
    } 

  int fileSize = getFileSize (params->ftp, params->remoteName);
  if (0 >= fileSize)
    {
      FTP_ERROR ("File %s does not exist on server\n", params->remoteName);
      CLEAN_PARAMS (FTP_FAIL);
    }
  int sizeToGet = fileSize;
  int localFileSize = getLocalFileSize (params->localName);
  int appendToFile = params->useResume;
  int appendOffset = 0;
  if (-1 == localFileSize && 1 == params->useResume)
    {
      FTP_DEBUG ("File does not exist ... full download\n");
      appendToFile = 0;
    }
  if (1 == appendToFile)
    {
      if (localFileSize == fileSize)
        {
          FTP_DEBUG ("File already downloaded\n");
          CLEAN_PARAMS (FTP_SAMESIZE);
        }
      else if (localFileSize > fileSize)
        {
          FTP_ERROR ("Local file (%s) is greater than distant file (%s)\n", params->localName, params->remoteName);
          CLEAN_PARAMS (FTP_BADSIZE);
        }
      sizeToGet = fileSize - localFileSize;

      char buffer[50] = {0};
      appendOffset = fileSize - sizeToGet;
      snprintf (buffer, sizeof (buffer)-1, "REST %d\r\n", appendOffset);
      char srvAnsw[MAX_SIZE_MSG] = {0};
      _ftp_status ftp_result = ftpTransfert (params->ftp, buffer, srvAnsw, MAX_SIZE_MSG-1);
      if (FTP_FAILED (ftp_result))
        {
          FTP_ERROR ("Unable to set server offset\n");
          CLEAN_PARAMS (ftp_result);
        }
    }

  char dataIp [IP_STRING_SIZE] = {0};
  int dataPort = 0;
  getPassiveIpAndPort (srvMsg, dataIp, &dataPort, IP_STRING_SIZE);
 
  FTP_DEBUG ("Thread will connect to %s:%d\n", dataIp, dataPort);
  dataSocket = vp_os_malloc (sizeof (vp_com_socket_t));
  if (NULL == dataSocket)
    {
      FTP_ERROR ("Unable to allocate socket structure\n");
      CLEAN_PARAMS (FTP_FAIL);
    }
  dataSocket->type = VP_COM_CLIENT;
  dataSocket->protocol = VP_COM_TCP;
  dataSocket->port = dataPort;
  strncpy (dataSocket->serverHost, dataIp, VP_COM_NAME_MAXSIZE);
  dataSocket->is_multicast = 0;
  dataSocket->block = VP_COM_WAITALL;

  C_RESULT vp_result = vp_com_open_socket (dataSocket, &dataRead, &dataWrite);
  if (VP_FAILED (vp_result))
    {
      FTP_ERROR ("Unable to connect\n");
      CLEAN_PARAMS (FTP_FAIL);
    }

  int result = setSockTimeout ((int)dataSocket->priv, SOCK_TO_SEC, SOCK_TO_USEC);
  if (0 > result)
    {
      FTP_ERROR ("Unable to set data socket timeout\n");
      CLEAN_PARAMS (FTP_FAIL);
    }

  CHECK_ABORT;

  vp_os_memset (buffer, 0x0, sizeof (buffer));
  snprintf (buffer, sizeof (buffer)-1, "RETR %s\r\n", params->remoteName);
  ftp_result = ftpSend (params->ftp, buffer);
  if (FTP_FAILED (ftp_result))
    {
      FTP_ERROR ("Error while sending RETR command\n");
      CLEAN_PARAMS (ftp_result);
    }

  int sizeGot = appendOffset;
  float percentGot = (sizeGot * 100.0) / (fileSize *1.0);
  params->callback (FTP_PROGRESS, (void *)&percentGot, params->ftp);
  
  char filePart [MAX_SIZE_MSG] = {0};


  if (1 == appendToFile)
    {
      localFile = fopen (params->localName, "ab");
    }
  else
    {
      localFile = fopen (params->localName, "wb");
    }
  if (NULL == localFile)
    {
      FTP_ERROR ("Unable to open dest file %s\n", params->localName);
      CLEAN_PARAMS (FTP_FAIL);
    }
  while (sizeGot < fileSize)
    {
      CHECK_ABORT;
      int bytes = MAX_SIZE_MSG-1;
      C_RESULT vp_result = dataRead (dataSocket, (int8_t *)filePart, &bytes);
      if (VP_FAILED (vp_result))
        {
          FTP_ERROR ("Error while receiving data\n");
          CLEAN_PARAMS (FTP_FAIL);
        }
      if (0 >= bytes)
        {
          FTP_ERROR ("Recv timeout\n");
          CLEAN_PARAMS (FTP_TIMEOUT);
        }
      if (0 > fwrite (filePart, 1, bytes, localFile))
        {
          FTP_ERROR ("Unable to write to file\n");
          CLEAN_PARAMS (FTP_FAIL);
        }
      sizeGot += bytes;
      percentGot = (sizeGot * 100.0) / (fileSize * 1.0);
      params->callback (FTP_PROGRESS, (void *)&percentGot, params->ftp);
    }
  ftp_result = ftpRecv (params->ftp, srvMsg, MAX_SIZE_MSG-1);
  if (FTP_FAILED (ftp_result))
    {
      FTP_ERROR ("Error while getting answer\n"); 
      CLEAN_PARAMS (ftp_result);
    }
  repCode = getResponseCode (srvMsg);
  if (150 != repCode)
    {
      FTP_ERROR ("Bad response from server (%d, expected 150)\n", repCode);
      CLEAN_PARAMS (FTP_FAIL);
    }

  /* Cleaning FTP */
  if (NULL != dataSocket)
    {
      vp_com_close_socket (dataSocket);
      vp_os_free (dataSocket);
      dataSocket = NULL;
    }
  waitFor226Answer (params->ftp);
  flushFtp (params->ftp);

  CLEAN_PARAMS (FTP_SUCCESS);
}

DEFINE_THREAD_ROUTINE (ftpPut, param)
{
  vp_com_socket_t *dataSocket = NULL;
  Read dataRead = NULL;
  Write dataWrite = NULL;
  FILE *localFile = NULL;
  _ftp_put_param *params = (_ftp_put_param *)param;
  FTP_DEBUG ("Uploading %s to %s [resume : %c]\n", params->localName, params->remoteName, params->useResume ? 'y' : 'n');
  if (NULL == params->ftp)
    {
      FTP_ERROR ("FTP not open\n");
      CLEAN_PARAMS (FTP_FAIL);
    }
  flushFtp (params->ftp);

  char buffer[512] = {0};
  char srvMsg[MAX_SIZE_MSG] = {0};
  snprintf (buffer, sizeof (buffer)-1, "PASV\r\n");
  _ftp_status ftp_result = ftpTransfert (params->ftp, buffer, srvMsg, MAX_SIZE_MSG-1);
  if (FTP_FAILED (ftp_result))
    {
      FTP_ERROR ("Error while entering passive mode\n");
      CLEAN_PARAMS (ftp_result);
    }
  int repCode = getResponseCode (srvMsg);
  if (227 != repCode)
    {
      FTP_ERROR ("Bad response from server (%d, expected 227)\n", repCode);
      CLEAN_PARAMS (FTP_FAIL);
    }

  ftp_result = goToBinaryMode (params->ftp);
  if (FTP_FAILED (ftp_result))
    {
      FTP_ERROR ("Unable to go to binary mode\n");
      CLEAN_PARAMS (ftp_result);
    }
  
  int localFileSize = getLocalFileSize (params->localName);
  if (0 >= localFileSize)
    {
      FTP_ERROR ("File %s does not exist on filesystem\n", params->localName);
      CLEAN_PARAMS (FTP_FAIL);
    }
  int fileSize = getFileSize (params->ftp, params->remoteName);
  int sizeToPut = localFileSize;
  int appendToFile = params->useResume;
  int appendOffset = 0;
  if (-1 == fileSize && 1 == params->useResume)
    {
      FTP_DEBUG ("File does not exist on server ... full upload\n");
      appendToFile = 0;
    }
  if (1 == appendToFile)
    {
      if (localFileSize == fileSize)
        {
          FTP_DEBUG ("File already uploaded\n");
          CLEAN_PARAMS (FTP_SAMESIZE);
        }
      else if (localFileSize < fileSize)
        {
          FTP_ERROR ("Distant file (%s) is greather than local file (%s)\n", params->remoteName, params->localName);
          CLEAN_PARAMS (FTP_BADSIZE);
        }
      sizeToPut = localFileSize - fileSize;

      char buffer [50] = {0};
      appendOffset = localFileSize - sizeToPut;
      snprintf (buffer, sizeof (buffer)-1, "REST %d\r\n", appendOffset);
      char srvAnswer[MAX_SIZE_MSG] = {0};
      ftp_result = ftpTransfert (params->ftp, buffer, srvAnswer, MAX_SIZE_MSG-1);
      if (FTP_FAILED (ftp_result))
        {
          FTP_ERROR ("Unable to set server offset\n");
          CLEAN_PARAMS (ftp_result);
        }
    }

  char dataIp [IP_STRING_SIZE] = {0};
  int dataPort = 0;
  getPassiveIpAndPort (srvMsg, dataIp, &dataPort, IP_STRING_SIZE);

  FTP_DEBUG ("Thread will connect to %s:%d\n", dataIp, dataPort);
  dataSocket = vp_os_malloc (sizeof (vp_com_socket_t));
  if (NULL == dataSocket)
    {
      FTP_ERROR ("Unable to allocate socket structure\n");
      CLEAN_PARAMS (FTP_FAIL);
    }
  dataSocket->type = VP_COM_CLIENT;
  dataSocket->protocol = VP_COM_TCP;
  dataSocket->port = dataPort;
  strncpy (dataSocket->serverHost, dataIp, VP_COM_NAME_MAXSIZE);
  dataSocket->is_multicast = 0;
  dataSocket->block = VP_COM_DEFAULT;

  C_RESULT vp_result = vp_com_open_socket (dataSocket, &dataRead, &dataWrite);
  if (VP_FAILED (vp_result))
    {
      FTP_ERROR ("Unable to connect\n");
      CLEAN_PARAMS (FTP_FAIL);
    }

  int result = setSockTimeout ((int)dataSocket->priv, SOCK_TO_SEC, SOCK_TO_USEC);
  if (0 > result)
    {
      FTP_ERROR ("Unable to set data socket timeout\n");
      CLEAN_PARAMS (FTP_FAIL);
    }

  CHECK_ABORT;

  vp_os_memset (buffer, 0x0, sizeof (buffer));
  snprintf (buffer, sizeof (buffer)-1, "STOR %s\r\n", params->remoteName);
  ftp_result = ftpTransfert (params->ftp, buffer, srvMsg, MAX_SIZE_MSG-1);
  if (FTP_FAILED (ftp_result))
    {
      FTP_ERROR ("Error while sending command\n");
      CLEAN_PARAMS (ftp_result);
    }
  repCode = getResponseCode (srvMsg);
  if (150 != repCode)
    {
      FTP_ERROR ("Bad response from server (%d, expected 150)\n", repCode);
      CLEAN_PARAMS (FTP_FAIL);
    }

  int sizeSent = appendOffset;
  float percentSend = (sizeSent * 100.0) / (localFileSize *1.0);
  params->callback (FTP_PROGRESS, (void *)&percentSend, params->ftp);

  int numberOfFullSendNeeded = (sizeToPut / (MAX_SIZE_MSG-1));
  int partialSendNeeded = (sizeToPut % (MAX_SIZE_MSG-1)); // Zero if not needed, non-zero if needed
  char filePart [MAX_SIZE_MSG] = {0};

  localFile = fopen (params->localName, "rb");
  if (NULL == localFile)
    {
      FTP_ERROR ("Unable to open source file %s\n", params->localName);
      CLEAN_PARAMS (FTP_FAIL);
    }
  fseek (localFile, appendOffset, SEEK_SET);

  int numSend = 0;
  for (numSend = 0; numSend < numberOfFullSendNeeded; numSend++)
    {
      CHECK_ABORT;
      int bytes = fread (filePart, 1, MAX_SIZE_MSG-1, localFile);
      if (0 > bytes)
        {
          FTP_ERROR ("Unable to read from file\n");
          CLEAN_PARAMS (FTP_FAIL);
        }
      bytes = MAX_SIZE_MSG-1;
      C_RESULT vp_result = dataWrite (dataSocket, (int8_t *)filePart, &bytes);
      if (VP_FAILED (vp_result))
        {
          FTP_ERROR ("Unable to send data\n");
          CLEAN_PARAMS (FTP_FAIL);
        }
      if (MAX_SIZE_MSG-1 > bytes)
        {
          FTP_ERROR ("Send timeout\n");
          CLEAN_PARAMS (FTP_TIMEOUT);
        }
      sizeSent += MAX_SIZE_MSG-1;
      percentSend = (sizeSent * 100.0) / (localFileSize * 1.0);
      params->callback (FTP_PROGRESS, (void *)&percentSend, params->ftp);
    }
  
  if (0 != partialSendNeeded)
    {
      CHECK_ABORT;
      vp_os_memset (filePart, 0x0, MAX_SIZE_MSG);
      int bytes = fread (filePart, 1, MAX_SIZE_MSG-1, localFile);
      FTP_DEBUG ("Read %d bytes\n", bytes);
      if (0 > bytes)
        {
          FTP_ERROR ("Unable to read from file\n");
          CLEAN_PARAMS (FTP_FAIL);
        }
      int sendBytes = bytes;
      C_RESULT vp_result = dataWrite (dataSocket, (int8_t *)filePart, &sendBytes);
      if (VP_FAILED (vp_result))
        {
          FTP_ERROR ("Unable to send data\n");
          CLEAN_PARAMS (FTP_FAIL);
        }
      if (bytes > sendBytes)
        {
          FTP_ERROR ("Send timeout\n");
          CLEAN_PARAMS (FTP_TIMEOUT);
        }
      sizeSent += bytes;
      percentSend = (sizeSent * 100.0) / (localFileSize * 1.0);
      params->callback (FTP_PROGRESS, (void *)&percentSend, params->ftp);
    }

  /* Cleaning FTP */
  if (NULL != dataSocket)
    {
      vp_com_close_socket (dataSocket);
      vp_os_free (dataSocket);
      dataSocket = NULL;
    }
  waitFor226Answer (params->ftp);
  flushFtp (params->ftp);

  CLEAN_PARAMS (FTP_SUCCESS);
}


_ftp_status
ftpPut (_ftp_t *ftp, const char *localName, const char *remoteName, int useResume, ftp_callback callback)
{
  THREAD_HANDLE putThread;
  ftp_callback actualCallback = (NULL != callback) ? callback : emptyCallback;
  if (NULL == ftp)
    {
      actualCallback (FTP_FAIL, NULL, ftp);
      return FTP_FAIL;
    }
  if (1 == ftp->opInProgress)
    {
      actualCallback (FTP_BUSY, NULL, ftp);
      return FTP_BUSY;
    }
  ftp->opInProgress = 1;

  _ftp_put_param *param = vp_os_malloc (sizeof (_ftp_put_param));

  if (NULL == param)
    {
      FTP_ERROR ("Unable to allocate thread param\n");
      actualCallback (FTP_FAIL, NULL, ftp);
      ftp->opInProgress = 0;
      return FTP_FAIL;
    }

  param->ftp = ftp;
  strncpy (param->localName, localName, FILE_NAME_MAX_SIZE);
  param->localName [FILE_NAME_MAX_SIZE-1] = '\0';
  strncpy (param->remoteName, remoteName, FILE_NAME_MAX_SIZE);
  param->remoteName [FILE_NAME_MAX_SIZE-1] = '\0';
  param->useResume = useResume;
  param->callback = actualCallback;
  param->fileList = NULL;
  _ftp_status threadReturn = FTP_SUCCESS; 

  vp_os_thread_create (thread_ftpPut, (THREAD_PARAMS)param, &putThread);

  if (NULL == callback)
    {
      vp_os_thread_join (putThread);
      threadReturn = lastStatusFromEmptyCallback;
    }

  return threadReturn; 
}

_ftp_status
ftpGet (_ftp_t *ftp, const char *remoteName, const char *localName, int useResume, ftp_callback callback)
{
  THREAD_HANDLE getThread;
  ftp_callback actualCallback = (NULL != callback) ? callback : emptyCallback;
  if (NULL == ftp)
    {
      actualCallback (FTP_FAIL, NULL, ftp);
      return FTP_FAIL;
    }
  if (1 == ftp->opInProgress)
    {
      actualCallback (FTP_BUSY, NULL, ftp);
      return FTP_BUSY;
    }
  ftp->opInProgress = 1;

  _ftp_get_param *param = vp_os_malloc (sizeof (_ftp_get_param));

  if (NULL == param)
    {
      FTP_ERROR ("Unable to allocate thread param\n");
      actualCallback (FTP_FAIL, NULL, ftp);
      ftp->opInProgress = 0;
      return FTP_FAIL;
    }

  param->ftp = ftp;
  strncpy (param->localName, localName, FILE_NAME_MAX_SIZE);
  param->localName [FILE_NAME_MAX_SIZE-1] = '\0';
  strncpy (param->remoteName, remoteName, FILE_NAME_MAX_SIZE);
  param->remoteName [FILE_NAME_MAX_SIZE-1] = '\0';
  param->useResume = useResume;
  param->callback = actualCallback;
  param->fileList = NULL;
  _ftp_status threadReturn = FTP_SUCCESS; 

  vp_os_thread_create (thread_ftpGet, (THREAD_PARAMS)param, &getThread);

  if (NULL == callback)
    {
      vp_os_thread_join (getThread);
      threadReturn = lastStatusFromEmptyCallback;
    }

  return threadReturn;
}

_ftp_status
ftpList (_ftp_t *ftp, char **fileList,  ftp_callback callback)
{
  if (NULL == fileList && NULL == callback)
    {
      FTP_ERROR ("file list and callback pointer must not be both NULL\n");
      return FTP_FAIL;
    }
  THREAD_HANDLE listThread;
  ftp_callback actualCallback = (NULL != callback) ? callback : emptyCallback;
  if (NULL == ftp)
    {
      actualCallback (FTP_FAIL, NULL, ftp);
      return FTP_FAIL;
    }
  if (1 == ftp->opInProgress)
    {
      actualCallback (FTP_BUSY, NULL, ftp);
      return FTP_BUSY;
    }
  ftp->opInProgress = 1;

  _ftp_list_param *param = vp_os_malloc (sizeof (_ftp_list_param));

  if (NULL == param)
    {
      FTP_ERROR ("Unable to allocate thread param\n");
      actualCallback (FTP_FAIL, NULL, ftp);
      ftp->opInProgress = 0;
      return FTP_FAIL;
    }

  param->fileList = vp_os_malloc (sizeof (char) * LIST_BUFFER_BLOCKSIZE);
  if (NULL == param->fileList)
    {
      FTP_ERROR ("Unable to allocate list buffer\n");
      actualCallback (FTP_FAIL, NULL, ftp);
      ftp->opInProgress = 0;
      vp_os_free (param);
      return FTP_FAIL;
    }

  param->ftp = ftp;
  param->listSize = LIST_BUFFER_BLOCKSIZE;
  param->callback = actualCallback;
  _ftp_status threadReturn = FTP_SUCCESS;

  vp_os_thread_create (thread_ftpList, (THREAD_PARAMS)param, &listThread);

  if (NULL == callback)
    {
      vp_os_thread_join (listThread);
      threadReturn = lastStatusFromEmptyCallback;
      if (FTP_SUCCESS == threadReturn)
        {
          *fileList = lastFileListFromEmptyCallback;
          lastFileListFromEmptyCallback = NULL;
        }
    }

  return threadReturn;
}

_ftp_status
ftpRemove (_ftp_t *ftp, const char *remoteName)
{
  _ftp_status ftp_result = FTP_FAIL;
  char buffer [256] = {0};
  char srvMsg [MAX_SIZE_MSG] = {0};
  if (NULL == remoteName)
    {
      FTP_ERROR ("remoteName must not be a NULL pointer\n");
      return FTP_FAIL;
    }
  if (NULL == ftp)
    {
      FTP_ERROR ("FTP not open\n");
      return FTP_FAIL;
    }
  snprintf (buffer, sizeof (buffer)-1, "DELE %s\r\n", remoteName);
  ftp_result = ftpTransfert (ftp, buffer, srvMsg, MAX_SIZE_MSG-1);
  if (FTP_FAILED (ftp_result))
    {
      FTP_ERROR ("Error while sending the delete command\n");
      return ftp_result;
    }
  int repCode = getResponseCode (srvMsg);
  if (250 != repCode && 550 != repCode)
    {
      FTP_ERROR ("Bad response from server (%d, expected 250 or 550)\n", repCode);
      ftp_result = FTP_FAIL;
    }
  return ftp_result;
}

_ftp_status
ftpRename (_ftp_t *ftp, const char *origin, const char *dest)
{
  _ftp_status ftp_result = FTP_FAIL;
  char buffer [256] = {0};
  char srvMsg [MAX_SIZE_MSG] = {0};
  if (NULL == origin ||
      NULL == dest)
    {
      FTP_ERROR ("origin and dest pointers must not be NULL\n");
      return FTP_FAIL;
    }
  if (NULL == ftp)
    {
      FTP_ERROR ("FTP not open\n");
      return FTP_FAIL;
    }
  snprintf (buffer, sizeof (buffer)-1, "RNFR %s\r\n", origin);
  ftp_result = ftpTransfert (ftp, buffer, srvMsg, MAX_SIZE_MSG-1);
  if (FTP_FAILED (ftp_result))
    {
      FTP_ERROR ("Error while sending the RNFR command\n");
      return ftp_result;
    }
  int repCode = getResponseCode (srvMsg);
  if (350 != repCode)
    {
      FTP_ERROR ("Bad response from server (%d, expected 350)\n", repCode);
      ftp_result = FTP_FAIL;
    }
  
  vp_os_memset (buffer, 0x0, sizeof (buffer));
  vp_os_memset (srvMsg, 0x0, sizeof (srvMsg));
  
  snprintf (buffer, sizeof (buffer)-1, "RNTO %s\r\n", dest);
  ftp_result = ftpTransfert (ftp, buffer, srvMsg, MAX_SIZE_MSG-1);
  if (FTP_FAILED (ftp_result))
    {
      FTP_ERROR ("Error while sending the RNTO command\n");
      return ftp_result;
    }
  repCode = getResponseCode (srvMsg);
  if (250 == repCode) // Rename worked
    {
      ftp_result = FTP_SUCCESS;
    }
  else if (550 == repCode) // Source file don't exist
    {
      FTP_DEBUG ("File %s doest not exist on FTP\n", origin);
      ftp_result = FTP_SAMESIZE;
    }
  else
    {
      FTP_ERROR ("Bad response from server (%d, expected 250 or 550)\n", repCode);
      ftp_result = FTP_FAIL;
    }
  return ftp_result;
}

_ftp_status
ftpCd (_ftp_t *ftp, const char *nextDir)
{
  _ftp_status ftp_result = FTP_FAIL;
  char buffer [256] = {0};
  char srvMsg [MAX_SIZE_MSG] = {0};
  if (NULL == nextDir)
    {
      FTP_ERROR ("nextDir must not be NULL\n");
      return FTP_FAIL;
    }
  if (NULL == ftp)
    {
      FTP_ERROR ("FTP not open\n");
      return FTP_FAIL;
    }
  snprintf (buffer, sizeof (buffer)-1, "CWD %s\r\n", nextDir);
  ftp_result = ftpTransfert (ftp, buffer, srvMsg, MAX_SIZE_MSG-1);
  if (FTP_FAILED (ftp_result))
    {
      FTP_ERROR ("Error while sending the CWD command\n");
      return ftp_result;
    }
  int repCode = getResponseCode (srvMsg);
  if (250 != repCode)
    {
      FTP_ERROR ("Bad response from server (%d, expected 250)\n", repCode);
      ftp_result = FTP_FAIL;
    }
  return ftp_result;
}

_ftp_status
ftpPwd (_ftp_t *ftp, char *workingDir, int wdLen)
{
  _ftp_status ftp_result = FTP_FAIL;
  char srvMsg [MAX_SIZE_MSG] = {0};
  if (NULL == workingDir)
    {
      FTP_ERROR ("workingDir must not be NULL\n");
      return FTP_FAIL;
    }
  if (NULL == ftp)
    {
      FTP_ERROR ("FTP not open\n");
      return FTP_FAIL;
    }
  ftp_result = ftpTransfert (ftp, "PWD\r\n\0", srvMsg, MAX_SIZE_MSG-1);
  if (FTP_FAILED (ftp_result))
    {
      FTP_ERROR ("Error while sending the CWD command\n");
      return ftp_result;
    }
  int repCode = getResponseCode (srvMsg);
  if (257 != repCode)
    {
      FTP_ERROR ("Bad response from server (%d, expected 257)\n", repCode);
      ftp_result = FTP_FAIL;
    }
  else
    {
      int pwdStartIndex = 0;
      int pwdEndIndex = 0;
      for (pwdStartIndex = 0; (pwdStartIndex < MAX_SIZE_MSG) && (srvMsg[pwdStartIndex] != '\"'); pwdStartIndex++);
      for (pwdEndIndex = pwdStartIndex+1; (pwdEndIndex < MAX_SIZE_MSG) && (srvMsg[pwdEndIndex] != '\"'); pwdEndIndex++);
      if (MAX_SIZE_MSG == pwdStartIndex ||
          MAX_SIZE_MSG == pwdEndIndex)
        {
          FTP_ERROR ("FTP Answer does not conains PWD\n");
          ftp_result = FTP_FAIL;
        }
      else
        {
          int srvLen = pwdEndIndex - (pwdStartIndex + 1);
          int totalLen = (srvLen < wdLen) ? srvLen : wdLen;
          strncpy (workingDir, &(srvMsg[pwdStartIndex+1]), totalLen);
          FTP_DEBUG ("PWD is %s\n", workingDir);
        }
    }
  return ftp_result;
}

_ftp_status
ftpMkdir (_ftp_t *ftp, const char *dirName)
{
  _ftp_status ftp_result = FTP_FAIL;
  char buffer [256] = {0};
  char srvMsg [MAX_SIZE_MSG] = {0};
  if (NULL == dirName)
    {
      FTP_ERROR ("dirName must not be NULL\n");
      return FTP_FAIL;
    }
  if (NULL == ftp)
    {
      FTP_ERROR ("FTP not open\n");
      return FTP_FAIL;
    }
  snprintf (buffer, sizeof (buffer)-1, "MKD %s\r\n", dirName);
  ftp_result = ftpTransfert (ftp, buffer, srvMsg, MAX_SIZE_MSG-1);
  if (FTP_FAILED (ftp_result))
    {
      FTP_ERROR ("Error while sending the MKD command\n");
      return ftp_result;
    }
  int repCode = getResponseCode (srvMsg);
  if (257 != repCode)
    {
      FTP_ERROR ("Bad response from server (%d, expected 257)\n", repCode);
      ftp_result = FTP_FAIL;
    }
  return ftp_result;
}

_ftp_status
ftpRmdir (_ftp_t *ftp, const char *dirName)
{
  _ftp_status ftp_result = FTP_FAIL;
  char buffer [256] = {0};
  char srvMsg [MAX_SIZE_MSG] = {0};
  if (NULL == dirName)
    {
      FTP_ERROR ("dirName must not be NULL\n");
      return FTP_FAIL;
    }
  if (NULL == ftp)
    {
      FTP_ERROR ("FTP not open\n");
      return FTP_FAIL;
    }
  snprintf (buffer, sizeof (buffer)-1, "RMD %s\r\n", dirName);
  ftp_result = ftpTransfert (ftp, buffer, srvMsg, MAX_SIZE_MSG-1);
  if (FTP_FAILED (ftp_result))
    {
      FTP_ERROR ("Error while sending the RMD command\n");
      return ftp_result;
    }
  int repCode = getResponseCode (srvMsg);
  if (250 == repCode) // Deleted
    {
      FTP_DEBUG ("Successfully deleted %s directory\n", dirName);
      ftp_result = FTP_SUCCESS;
    }
  else if (550 == repCode) // Didn't exist / not empty
    {
      FTP_DEBUG ("Did not delete directory %s : did not exist or was not empty\n", dirName);
      ftp_result = FTP_BADSIZE;
    }
  else
    {
      FTP_ERROR ("Bad response from server (%d, expected 250 or 550)\n", repCode);
      ftp_result = FTP_FAIL;
    }
  return ftp_result;
}

_ftp_status ftpAbort (_ftp_t *ftp)
{
  _ftp_status retVal = FTP_FAIL;
  if (NULL == ftp)
    {
      FTP_ERROR ("FTP not open\n");
    }
  else
    {
      if (1 == ftp->opInProgress)
        {
          ftp->abortCurrentOp = 1;
          retVal = FTP_SUCCESS;
        }
      else
        {
          retVal = FTP_SAMESIZE;
        }
    }
  return retVal;
}
