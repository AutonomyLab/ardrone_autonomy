/**
 *  \brief    Com Api for video sdk. Public definitions.
 *  \author   Sylvain Gaeremynck <sylvain.gaeremynck@parrot.fr>
 *  \version  3.0
 *  \date     16/03/2007
 *  \warning  Subject to completion
 */

#ifndef _VP_COM_INCLUDE_H_
#define _VP_COM_INCLUDE_H_

#include <VP_Os/vp_os_types.h>
#include <VP_Os/vp_os_signal.h>

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#if defined(USE_ELINUX)
#if  defined(BR2_PACKAGE_BCM4318_AP)
#     define AP
#    else
#     define STA
#    endif // BR2_PACKAGE_BCM4318_AP
#   endif // USE_ELINUX

#define VP_COM_NAME_MAXSIZE     32    /* String's max size for vp_com */
#define VP_COM_MAX_NUM_DEVICES  256   /* Max number of devices we can record when discovering devices */
#define VP_COM_MAX_PINCODE_SIZE 32    /* passkey/pincode max length */

#define VP_COM_THREAD_SERVER_STACK_SIZE       8192  /* vp_com_server thread stack size */
#define VP_COM_THREAD_NUM_MAX_CLIENTS         16    /* Max client vp_com_server thread can handle */
#define VP_COM_THREAD_LOCAL_BUFFER_SIZE   	  2048
#define VP_COM_THREAD_LOCAL_BUFFER_MAX_SIZE   4096
												/* Max size of vp_com_server's local buffer to bufferize incoming data */
                                                /* For instance this value is INTERNAL_BUFFER_SIZE/2 (see ATcodec impl) */

#ifdef USE_MINGW32

#define socklen_t int

#endif

struct _vp_com_socket_t;
struct in_addr;

/****************************************************
 * Definitions                                      *
 ****************************************************/

/**
 * \enum Types of wireless connection (Bluetooth or Wifi)
 */
typedef enum _VP_COM_WL_TYPE_
{
  VP_COM_BLUETOOTH,                                         /// Bluetooth
  VP_COM_WIFI,                                              /// Wifi
  VP_COM_SERIAL,
  VP_COM_WIRED
} VP_COM_WL_TYPE;

/**
 * \enum Types of socket (client or server)
 */
typedef enum _VP_COM_SOCKET_TYPE_
{
  VP_COM_CLIENT,                                            /// Client
  VP_COM_SERVER                                             /// Server
} VP_COM_SOCKET_TYPE;

/**
 * \enum Protocol for the socket to be opened
 */
typedef enum _VP_COM_SOCKET_PROTOCOL_
{
  VP_COM_RFCOMM,                                            /// Rfcomm (RS232 emulation available only in bluetooth mode)
  VP_COM_TCP,                                               /// TCP/IP
  VP_COM_UDP                                                /// UDP/IP
} VP_COM_SOCKET_PROTOCOL;

typedef enum _VP_COM_SOCKET_OPTIONS_
{
  VP_COM_NON_BLOCKING   = 1,
  VP_COM_NO_DELAY       = 2,
  VP_COM_MULTICAST_ON   = 4,
} VP_COM_SOCKET_OPTIONS;

typedef enum _VP_COM_WIFI_EVENTS_
{
  VP_COM_WIFI_EVENT_LINK      = 0,                          /// Link up or down
  VP_COM_WIFI_EVENT_BEACONS   = 1                           /// Beacons received or not
} VP_COM_WIFI_EVENTS;

typedef enum _VP_COM_SOCKET_STATE_ {
  VP_COM_SOCKET_SELECT_ENABLE = 0,
  VP_COM_SOCKET_SELECT_DISABLE  = 1
} VP_COM_SOCKET_STATE;

typedef enum _vp_com_autoip_t {
  VP_COM_AUTOIP_DISABLE = 0,
  VP_COM_AUTOIP_ENABLE  = 1,    // Use of autoip
} vp_com_autoip_t;

typedef enum _VP_COM_SOCKET_BLOCKING_OPTIONS {
  VP_COM_DEFAULT = 0,
  VP_COM_WAITALL = 1,
  VP_COM_DONTWAIT = 2,
} VP_COM_SOCKET_BLOCKING_OPTIONS;
     

typedef C_RESULT (*select_cb) (struct _vp_com_socket_t* server_socket, struct _vp_com_socket_t* client_socket, VP_COM_SOCKET_STATE state, Write write);

typedef struct _vp_com_config_t
{
  char        itfName[VP_COM_NAME_MAXSIZE];
} vp_com_config_t;

typedef struct _vp_com_wired_config_t
{
  char        itfName[VP_COM_NAME_MAXSIZE];                 /// eth0, ....
  char        localHost[VP_COM_NAME_MAXSIZE];               /// ip of the localhost
  char        netmask[VP_COM_NAME_MAXSIZE];                 /// subnetwork mask
  char        broadcast[VP_COM_NAME_MAXSIZE];               /// broadcast address
} vp_com_wired_config_t;

typedef struct _vp_com_wifi_config_t
{
  char        itfName[VP_COM_NAME_MAXSIZE];                 /// wl0, rausb0 ....
  char        localHost[VP_COM_NAME_MAXSIZE];               /// ip of the localhost
  char        netmask[VP_COM_NAME_MAXSIZE];                 /// subnetwork mask
  char        broadcast[VP_COM_NAME_MAXSIZE];               /// broadcast address
  char        gateway[VP_COM_NAME_MAXSIZE];                 /// ip of gateway
  char        server[VP_COM_NAME_MAXSIZE];                  /// ip of server

  uint32_t    infrastructure;                               /// if infrastructure == 1 then we need an access point
  uint32_t    secure;                                       /// if secure == 1 then we use the defined passkey
  char        passkey[VP_COM_MAX_PINCODE_SIZE];             /// passkey to use for futur connection
  char        country[4];                                   /// Wifi country code. See Broadcom BCM4318 SDIO Country Code

} vp_com_wifi_config_t;

typedef struct _vp_com_bluetooth_config_t
{
  char        itfName[VP_COM_NAME_MAXSIZE];                 /// bnep0, ....
  char        localHost[VP_COM_NAME_MAXSIZE];               /// ip of localhost
  char        netmask[VP_COM_NAME_MAXSIZE];                 /// subnetwork mask
  char        broadcast[VP_COM_NAME_MAXSIZE];               /// broadcast address
  char        gateway[VP_COM_NAME_MAXSIZE];                 /// ip of gateway
  char        server[VP_COM_NAME_MAXSIZE];                  /// ip of server

  uint32_t    secure;                                       /// if secure == 1 then we use the defined passkey
  char        passkey[VP_COM_MAX_PINCODE_SIZE];             /// passkey to use for futur connection
} vp_com_bluetooth_config_t;

#if defined(__LINUX__) || defined(__ELINUX__)
typedef struct _vp_com_serial_config_t
{
  char              itfName[VP_COM_NAME_MAXSIZE];           /// /dev/ser0, /dev/ttyS0, /dev/ttyUSB0, etc.
  vp_com_baudrate_t baudrate;                               /// baudrate
  vp_com_baudrate_t initial_baudrate;                       /// baudrate used to wait sync
  uint32_t          sync;                                   /// if sync == 1 then we wait synchronization data
  uint32_t          sync_done;                              /// if sync == 1 then we wait synchronization data
  uint32_t          blocking;                               /// if blocking == 0 then it is non-blocking mode
  char              sync_string[VP_COM_NAME_MAXSIZE];       /// if sync == 1 then sync_string is
                                                            ///   the string we sent to synchronise
} vp_com_serial_config_t;
#endif

/**
 * \struct Connection's definition
 */
typedef struct _vp_com_connection_t
{
  uint32_t    is_up;                                        /// == 1 if connection is up. 0 otherwise
} vp_com_connection_t;

typedef struct _vp_com_bluetooth_connection_t
{
  uint32_t    is_up;                                        /// == 1 if connection is up. 0 otherwise
  bdaddr_t    address;
} vp_com_bluetooth_connection_t;

typedef struct _vp_com_wifi_connection_t
{
  uint32_t    is_up;                                        /// == 1 if connection is up. 0 otherwise
  char        networkName[VP_COM_NAME_MAXSIZE];             /// ssid
} vp_com_wifi_connection_t;

typedef struct _vp_com_wired_connection_t
{
  uint32_t    is_up;                                        /// == 1 if connection is up. 0 otherwise
} vp_com_wired_connection_t;

typedef C_RESULT (*Read_from)  (void* s, int8_t* buffer, int32_t* size, uint32_t from_ip);


/**
 * \struct Socket's definition
 */
typedef struct _vp_com_socket_t
{
  VP_COM_SOCKET_TYPE        type;                             /// Socket type (Server or client)
  VP_COM_SOCKET_PROTOCOL    protocol;                         /// Protocol (Rfcomm, Tcp or Udp)
  VP_COM_SOCKET_BLOCKING_OPTIONS block;                       /// Blocking policy for Tcp/Udp sockets

  uint32_t                  scn;                              /// Channel used for the connection (only for a Rfcomm socket)
  uint32_t                  port;                             /// Port for the connection (only for ip based protocol)
  char                      serverHost[VP_COM_NAME_MAXSIZE];  /// ip of the serverhost (if client)
  uint32_t                  remotePort;                       /// port used by client when incoming connection opened
  select_cb                 select;                           /// callback when a (de)connection is received
  Read_from                 read;                             /// callback when data are received

  uint32_t                  is_multicast;                     /// tells if the socket is in multicast mode or not
  uint32_t                  multicast_base_addr;              /// base address used to compute multicast address to use

  /// Private data
  void*                     priv;                             /// socket number
  VP_COM_SOCKET_STATE       is_disable;                       /// tells if the socket is enable or not
  struct _vp_com_socket_t*  server;                           /// Param only used when socket is client
  uint32_t                  queue_length;                     /// only for server socket

} vp_com_socket_t;

/**
 * \typedef Prototypes of discovery callback functions
 */
typedef void (*vp_com_network_adapter_lookup_t)(const char* name);
typedef void (*vp_com_inquiry_t)(bdaddr_t* address, const uint8_t* name);

typedef struct _vp_com_t {

  VP_COM_WL_TYPE        type;         /// Mandatory! This field must be filled before any call to VP_Com functions

  bool_t                initialized;
  uint32_t              ref_count;
  vp_os_mutex_t         mutex;

  vp_com_config_t*      config;
  vp_com_connection_t*  connection;

  /// autoip
  vp_com_autoip_t       autoip;       /// Use zeroconf to configure interface (ip will be in range 169.254.x.y)
  void (*pick_ip)(struct in_addr *ip);

  /// callbacks
  C_RESULT (*init)(void);
  C_RESULT (*shutdown)(void);
  C_RESULT (*network_adapter_lookup)(vp_com_network_adapter_lookup_t callback);
  C_RESULT (*inquire)(const char* deviceName, vp_com_inquiry_t callback, uint32_t timeout);
  C_RESULT (*local_config)(vp_com_config_t* config);
  C_RESULT (*connect)(struct _vp_com_t* vp_com, vp_com_connection_t* connection, int32_t num_attempts);
  C_RESULT (*disconnect)(vp_com_config_t* config, vp_com_connection_t* connection);
  C_RESULT (*get_rssi)(vp_com_config_t* config, int32_t* rssi);
  C_RESULT (*wait_connections)(vp_com_connection_t** c, vp_com_socket_t* server, vp_com_socket_t* client, int queueLength);
  C_RESULT (*open)(vp_com_config_t* config, vp_com_connection_t* connection, vp_com_socket_t* socket, Read* read, Write* write);
  C_RESULT (*close)(vp_com_socket_t* socket);

} vp_com_t;


/****************************************************
 * Functions                                        *
 ****************************************************/

/**
 * \fn init com api
 * \param A com object with the first field set to the choosen protocol and all other fields set to 0
 * \return VP_SUCCESS or VP_FAILURE
 */
C_RESULT vp_com_init(vp_com_t* vp_com);

/**
 * \fn shutdown com api
 * \return VP_SUCCESS or VP_FAILURE
 */
C_RESULT vp_com_shutdown(vp_com_t* vp_com);

/**
 * \fn Configure ip allocation strategy - The prototype of this function could change
 * \param vp_com a com object
 * \param autoip the allocation strategy to use
 * \param pick_ip the ip generation function (used in VP_COM_AUTOIP_EX mode)
 * \return VP_SUCCESS or VP_FAILURE
 */
C_RESULT vp_com_ip_alloc(vp_com_t* vp_com, vp_com_autoip_t autoip, void (*pick_ip)(struct in_addr *ip) );

/**
 * \fn Configure the local network interface
 * \param Configuration parameters
 * \return VP_SUCCESS or VP_FAILURE
 */
C_RESULT vp_com_local_config(vp_com_t* vp_com, vp_com_config_t* config);

/**
 * \fn Connect to an access point
 * \param Connection parameters
 * \param Number of attempts to try connection
 * \return VP_SUCCESS or VP_FAILURE
 */
C_RESULT vp_com_connect(vp_com_t* vp_com, vp_com_connection_t* connection, uint32_t numAttempts);

/**
 * \fn Disconnect from an access point
 * \param Connection parameters
 * \return VP_SUCCESS or VP_FAILURE
 */
C_RESULT vp_com_disconnect(vp_com_t* vp_com);

/**
 * \fn Returns wifi link's quality
 * \param rssi signal quality is a percent. A value of 100 means we have an excellent connection.
 * \return VP_SUCCESS or VP_FAILURE
 */
C_RESULT vp_com_get_rssi(vp_com_t* vp_com, int32_t* rssi);

/**
 * \fn Open a socket
 * \param Socket parameters
 * \param Used to get a function pointer on a read compatible with the socket parameters (can be NULL)
 * \param Used to get a function pointer on a write compatible with the socket parameters (can be NULL)
 * \return VP_SUCCESS or VP_FAILURE
 */
C_RESULT vp_com_open(vp_com_t* vp_com, vp_com_socket_t* socket, Read* read, Write* write);

/**
 * \fn Close a socket
 * \param Socket parameters
 * \return VP_SUCCESS or VP_FAILURE
 */
C_RESULT vp_com_close(vp_com_t* vp_com, vp_com_socket_t* socket);

/**
 * \fn Set socket option
 * \param Socket parameters
 * \param options
 * \return VP_SUCCESS or VP_FAILURE
 */
C_RESULT vp_com_sockopt(vp_com_t* vp_com, vp_com_socket_t* socket, VP_COM_SOCKET_OPTIONS options);

/**
 * \fn Wait for incomming connections
 * \param Socket server parameters
 * \param Socket client parameters (filled by the function)
 * \param Queue length
 * \return VP_SUCCESS or VP_FAILURE
 */
C_RESULT vp_com_wait_connections(vp_com_t* vp_com, vp_com_socket_t* server, vp_com_socket_t* client, int32_t queueLength);



/****************************************************
 * Utility functions                                *
 ****************************************************/


/**
 * \fn Parse a string to create a bd address
 * \param [in] address (string format)
 * \param [out] address (bd format)
 * \return VP_SUCCESS or VP_FAILURE
 */
C_RESULT vp_com_str_to_address(const char* address, bdaddr_t* addr);

/**
 * \fn Transform a bdaddr from its binary format to string format
 * \param [in] address (bd format)
 * \param [out] address (string format)
 * \return VP_SUCCESS or VP_FAILURE
 */
C_RESULT vp_com_address_to_str(const bdaddr_t* addr, char* address);

/**
 * \fn Copy a bd address
 * \param [in] address
 * \param [out] address
 * \return VP_SUCCESS or VP_FAILURE
 */
C_RESULT vp_com_copy_address(const bdaddr_t* from, bdaddr_t* to);

/**
 * \fn Compare two bd addresses
 * \param [in] address1
 * \param [in] address2
 * \return VP_SUCCESS or VP_FAILURE
 */
C_RESULT vp_com_cmp_address(const bdaddr_t* bd1, const bdaddr_t* bd2);

/**
 * \fn Look for available network adapter
 * \param Callback function (see Definitions)
 * \return VP_SUCCESS or VP_FAILURE
 */
C_RESULT vp_com_network_adapter_lookup(vp_com_t* vp_com, vp_com_network_adapter_lookup_t callback);

/**
 * \fn Look for available network
 * \param device to use
 * \param Callback function (see Definitions)
 * \param timeout to stop the inquiry
 * \return VP_SUCCESS or VP_FAILURE
 */
C_RESULT vp_com_inquire(vp_com_t* vp_com, const char* deviceName, vp_com_inquiry_t callback, uint32_t timeout);

/****************************************************
 * Threads                                          *
 ****************************************************/

/**
 * Declaration of a thread that handles incomming connections in a generic way
 *
 */

#include <VP_Api/vp_api_thread_helper.h>

typedef struct _vp_com_server_thread_param_t {
  bool_t                run;

  vp_com_t*             com;
  vp_com_config_t*      config;
  vp_com_connection_t*  connection;

  uint32_t              num_servers;
  vp_com_socket_t*      servers;

  bool_t                timer_enable;
  int32_t               wait_sec;
  int32_t               wait_usec;
} vp_com_server_thread_param_t;

// This thread can be launched to handle incomming connections
// You must call vp_com_start_server to start this thread
PROTO_THREAD_ROUTINE_STACK( vp_com_server, thread_params, VP_COM_THREAD_SERVER_STACK_SIZE );

/**
 * Start thread server
 *
 */
C_RESULT vp_com_init_server(void); // This function must be called before starting thread server
C_RESULT vp_com_wait_for_server_up(void); // Wait until com layer is initialized
C_RESULT vp_com_timed_wait_for_server_up(uint32_t ms); // Wait some ms until com layer is initialized

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _VP_COM_INCLUDE_H_
