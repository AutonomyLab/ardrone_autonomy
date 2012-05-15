#include <VP_Com/vp_com_error.h>
#include <VP_Com/vp_com_socket.h>

#include "vp_com_wifi.h"
#include "vp_com_config_itf.h"

#include <VP_Os/vp_os_malloc.h>
#include <VP_Os/vp_os_print.h>
#include <VP_Os/vp_os_delay.h>

#ifdef USE_IWLIB
#    include <iwlib.h>
#endif

#include <netdb.h>
#include <sys/socket.h>

#include <net/if.h>
#include <netinet/in.h>

#ifndef PROC_NET_WIRELESS
#define PROC_NET_WIRELESS "/proc/net/wireless"
#endif

C_RESULT vp_com_wf_init(void)
{
  return VP_COM_OK;
}

C_RESULT vp_com_wf_shutdown(void)
{
  return VP_COM_OK;
}

C_RESULT vp_com_wf_network_adapter_lookup(vp_com_network_adapter_lookup_t callback)
{
#ifdef USE_IWLIB
  char buff[1024];
	char* ret=NULL;

  FILE* fh = fopen( PROC_NET_WIRELESS, "r" );

  if(fh != NULL)
  {
    ret=fgets( buff, sizeof(buff), fh );

    ret=fgets( buff, sizeof(buff), fh );
    while( fgets( buff, sizeof(buff), fh ) )
    {
      char name[IFNAMSIZ + 1];
      char *s = buff; // start
      char *e = buff + strlen(buff); // end

      vp_os_memset( name, 0, IFNAMSIZ + 1 );

      while( *s == ' ') s++; // skip leading separator

      while( e != s )
      {
        if( *e == ':' )
          break;
        e--;
      }

      if( e != s )
      { // it's a valid name
        strncpy( name, s, e - s );
        callback( name );
      }
    }

    fclose( fh );

  }
#endif
  return VP_COM_OK;
}

C_RESULT vp_com_wf_inquire(const char* deviceName, vp_com_inquiry_t callback, uint32_t timeout)
{
  return VP_COM_OK;
}

C_RESULT vp_com_wf_local_config(vp_com_wifi_config_t* cfg)
{
  return vp_com_config_itf( cfg->itfName, cfg->localHost, cfg->broadcast, cfg->netmask ) < 0 ? VP_COM_ERROR : VP_COM_OK;
}

C_RESULT vp_com_wf_connect(vp_com_t* vp_com, vp_com_wifi_connection_t* connection, int32_t numAttempts)
{
  C_RESULT res = VP_COM_OK;
#ifdef USE_IWLIB
  int32_t wlsock;
  vp_com_wifi_config_t* config = (vp_com_wifi_config_t*)vp_com->config;
  wireless_config iwconf;

  wlsock = iw_sockets_open();

  res = ( wlsock < 0 ) ? VP_COM_ERROR : VP_COM_OK;
  VP_COM_CHECK( res );

  iw_get_basic_config( wlsock, config->itfName, &iwconf );

  iwconf.has_nwid = 0;
  iwconf.has_freq = 0;
  iwconf.has_key  = 0;

  iwconf.has_mode = 1;
  iwconf.mode = config->infrastructure ? IW_MODE_INFRA : IW_MODE_ADHOC;

  iwconf.has_essid = 1;
  iwconf.essid_on = 1;
  strncpy( &iwconf.essid[0], connection->networkName, IW_ESSID_MAX_SIZE+1 );
  
  res = iw_set_basic_config( wlsock, config->itfName, &iwconf ) < 0 ? C_FAIL : C_OK;

  if( SUCCEED(res) )
    PRINT(" OK!\n");
  else
    PRINT(" FAILED!\n");

  iw_sockets_close(wlsock);

#endif
  return res;
}

C_RESULT vp_com_wf_disconnect(vp_com_wifi_config_t* config, vp_com_wifi_connection_t* connection)
{
  C_RESULT res = VP_COM_OK;
#ifdef USE_IWLIB
  struct iwreq wrq;
  char essid[IW_ESSID_MAX_SIZE + 1];

  int wlsock = iw_sockets_open();

  vp_os_memset(&wrq, 0, sizeof(struct iwreq));

  strncpy(essid, connection->networkName, strlen(connection->networkName));
  wrq.u.essid.flags = 0;
  wrq.u.essid.pointer = (caddr_t) essid;
  wrq.u.essid.length = strlen(essid);
  if(iw_get_kernel_we_version() < 21)
    wrq.u.essid.length++; // Get version from kernel, device may not have range...

  res = ( iw_set_ext( wlsock, config->itfName, SIOCSIWESSID, &wrq) < 0 ) ? VP_COM_ERROR : VP_COM_OK;

  iw_sockets_close(wlsock);
#endif

  return res;
}

C_RESULT vp_com_wf_get_rssi(vp_com_wifi_config_t* cfg, int32_t* rssi)
{
#ifdef USE_IWLIB
  struct iwreq wrq;
  iwstats stats;
  iwrange range;

  int wlsock = iw_sockets_open();

  vp_os_memset(&wrq, 0, sizeof(struct iwreq));
  iw_get_stats(wlsock, cfg->itfName, &stats, &range, 1);
  iw_sockets_close(wlsock);

//   struct  iw_statistics
//   {
//     __u16           status;           // Status * - device dependent for now
//     struct iw_quality       qual;     // Quality of the link * (instant/mean/max)
//     struct iw_discarded     discard;  // Packet discarded counts
//     struct iw_missed        miss;     // Packet missed counts
//   };
//
//   struct iw_range
//   {
//     ...
//     //  Quality of link & SNR stuff */
//     //  Quality range (link, level, noise)
//     //  If the quality is absolute, it will be in the range [0 ; max_qual],
//     //  if the quality is dBm, it will be in the range [max_qual ; 0].
//     //  Don't forget that we use 8 bit arithmetics...
//     struct iw_quality       max_qual;       // Quality of the link
//     //  This should contain the average/typical values of the quality
//     //  indicator. This should be the threshold between a "good" and
//     //  a "bad" link (example : monitor going from green to orange).
//     //  Currently, user space apps like quality monitors don't have any
//     //  way to calibrate the measurement. With this, they can split
//     //  the range between 0 and max_qual in different quality level
//     //  (using a geometric subdivision centered on the average).
//     //  I expect that people doing the user space apps will feedback
//     //  us on which value we need to put in each driver...
//     struct iw_quality       avg_qual;       // Quality of the link
//     ...
//   };
//   struct  iw_quality
//   {
//     __u8            qual;           // link quality (%retries, SNR, %missed beacons or better...)
//     __u8            level;          // signal level (dBm)
//     __u8            noise;          // noise level (dBm)
//     __u8            updated;        // Flags to know if updated
//   };

  *rssi = stats.qual.qual;

#endif
  return VP_COM_OK;
}

C_RESULT vp_com_wf_wait_connections(vp_com_wifi_connection_t** c, vp_com_socket_t* server, vp_com_socket_t* client, int32_t queueLength)
{
  return vp_com_wait_socket(server, client, queueLength);
}

C_RESULT vp_com_wf_open(vp_com_wifi_config_t* config, vp_com_wifi_connection_t* connection, vp_com_socket_t* sck, Read* read, Write* write)
{
  return vp_com_open_socket(sck, read, write);
}

C_RESULT vp_com_wf_close(vp_com_socket_t* socket)
{
  return vp_com_close_socket(socket);
}
