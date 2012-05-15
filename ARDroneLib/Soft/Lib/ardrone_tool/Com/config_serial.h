#ifndef _CONFIG_SERIAL_H_
#define _CONFIG_SERIAL_H_

#ifndef _CONFIG_COM_H_
#error "You cannot include config_serial.h. Please include config_com.h instead."
#endif


#include <VP_Com/vp_com.h>

/// Initializations for serial communication
/// Serial link 0 is used for navdata & ATCmd
/// Serial link 1 is used for video
/// Serial link 2 is used for communication with ADC

// Each tools must implement this functions
extern vp_com_t*             serial_com(void);
extern vp_com_config_t*      serial_config_0(void);
extern vp_com_config_t*      serial_config_1(void);
extern vp_com_config_t*      serial_config_2(void);
extern vp_com_connection_t*  serial_connection_0(void);
extern vp_com_connection_t*  serial_connection_1(void);
extern vp_com_connection_t*  serial_connection_2(void);

void serial_config_socket(vp_com_socket_t* socket, VP_COM_SOCKET_TYPE type);

#ifdef USE_NAVDATA_SERIAL
  #define COM_NAVDATA()             serial_com()
  #define COM_CONFIG_NAVDATA()      serial_config_0()
  #define COM_CONNECTION_NAVDATA()  serial_connection_0()
  #define COM_CONFIG_SOCKET_NAVDATA(socket, type, opt, serverhost)  serial_config_socket(socket, type)
#endif // USE_NAVDATA_SERIAL

#ifdef USE_AT_SERIAL
  #define COM_AT()                  serial_com()
  #define COM_CONFIG_AT()           serial_config_0()
  #define COM_CONNECTION_AT()       serial_connection_0()
  #define COM_CONFIG_SOCKET_AT(socket, type, opt, serverhost)  serial_config_socket(socket, type)
#endif // USE_NAVDATA_SERIAL

#ifdef USE_VIDEO_SERIAL
  #define COM_VIDEO()               serial_com()
  #define COM_CONFIG_VIDEO()        serial_config_1()
  #define COM_CONNECTION_VIDEO()    serial_connection_1()
  #define COM_CONFIG_SOCKET_VIDEO(socket, type, opt, serverhost)  serial_config_socket(socket, type)

  #define COM_RAW_CAPTURE()               serial_com()
  #define COM_CONFIG_RAW_CAPTURE()        serial_config_2()
  #define COM_CONNECTION_RAW_CAPTURE()    serial_connection_2()
  #define COM_CONFIG_SOCKET_RAW_CAPTURE(socket, type, opt, serverhost)  serial_config_socket(socket, type)
#endif // USE_VIDEO_SERIAL

#endif // _CONFIG_SERIAL_H_
