#ifndef _CONFIG_WIFI_H_
#define _CONFIG_WIFI_H_

#ifndef _CONFIG_COM_H_
#error "You cannot include config_wifi.h. Please include config_com.h instead."
#endif


#include <VP_Com/vp_com.h>

// Each tools must implement this functions
extern vp_com_t*             wifi_com(void);
extern vp_com_config_t*      wifi_config(void);
extern vp_com_connection_t*  wifi_connection(void);

void wifi_config_socket(vp_com_socket_t* socket, VP_COM_SOCKET_TYPE type, int32_t port, const char* serverhost);

#ifdef USE_NAVDATA_IP
  #define COM_NAVDATA()             wifi_com()
  #define COM_CONFIG_NAVDATA()      wifi_config()
  #define COM_CONNECTION_NAVDATA()  wifi_connection()
  #define COM_CONFIG_SOCKET_NAVDATA(socket, type, opt, serverhost)  wifi_config_socket(socket, type, opt, serverhost)
#endif // USE_NAVDATA_IP

#ifdef USE_AT_IP
  #define COM_AT()                  wifi_com()
  #define COM_CONFIG_AT()           wifi_config()
  #define COM_CONNECTION_AT()       wifi_connection()
  #define COM_CONFIG_SOCKET_AT(socket, type, opt, serverhost)  wifi_config_socket(socket, type, opt, serverhost)
#endif // USE_NAVDATA_IP

#ifdef USE_VIDEO_IP
  #define COM_VIDEO()               wifi_com()
  #define COM_CONFIG_VIDEO()        wifi_config()
  #define COM_CONNECTION_VIDEO()    wifi_connection()
  #define COM_CONFIG_SOCKET_VIDEO(socket, type, opt, serverhost)  wifi_config_socket(socket, type, opt, serverhost)

  #define COM_RAW_CAPTURE()               wifi_com()
  #define COM_CONFIG_RAW_CAPTURE()        wifi_config()
  #define COM_CONNECTION_RAW_CAPTURE()    wifi_connection()
  #define COM_CONFIG_SOCKET_RAW_CAPTURE(socket, type, opt, serverhost)  wifi_config_socket(socket, type, opt, serverhost)
#endif // USE_VIDEO_IP

#define COM_AUTH()               wifi_com()
#define COM_CONFIG_AUTH()        wifi_config()
#define COM_CONNECTION_AUTH()    wifi_connection()
#define COM_CONFIG_SOCKET_AUTH(socket, type, opt, serverhost)  wifi_config_socket(socket, type, opt, serverhost)

#define COM_CONTROL()                  wifi_com()
#define COM_CONFIG_CONTROL()           wifi_config()
#define COM_CONNECTION_CONTROL()       wifi_connection()
#define COM_CONFIG_SOCKET_CONTROL(socket, type, opt, serverhost)  wifi_config_socket(socket, type, opt, serverhost)

#define COM_REMOTE_CONSOLE()               wifi_com()
#define COM_CONFIG_REMOTE_CONSOLE()        wifi_config()
#define COM_CONNECTION_REMOTE_CONSOLE()    wifi_connection()
#define COM_CONFIG_SOCKET_REMOTE_CONSOLE(socket, type, opt, serverhost)  wifi_config_socket(socket, type, opt, serverhost)

#endif // _CONFIG_WIFI_H_
