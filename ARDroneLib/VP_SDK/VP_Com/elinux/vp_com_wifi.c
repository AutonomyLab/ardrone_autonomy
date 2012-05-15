#include <VP_Com/vp_com_error.h>
#include <VP_Com/vp_com_socket.h>

#include "vp_com_wifi.h"
#include "vp_com_config_itf.h"

#include <VP_Os/vp_os_malloc.h>
#include <VP_Os/vp_os_print.h>
#include <VP_Os/vp_os_delay.h>

#define PROC_NET_DEV "/proc/net/dev"

#include <netdb.h>
#include <sys/socket.h>
#include "vp_com_interface.h"
#include <unistd.h>

#ifdef USE_BROADCOM
#include "vp_com_wlc.h"
#endif

#ifdef USE_IWLIB
#include <iwlib.h>
#endif

#include <net/if.h>
#include <netinet/in.h>


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
  char buff[1024];

  FILE* fh = fopen( PROC_NET_DEV, "r" );

  if(fh != NULL)
  {
    // eat first two lines
    if( fgets( buff, sizeof(buff), fh ) && fgets( buff, sizeof(buff), fh ) )
    {
      while( fgets( buff, sizeof(buff), fh ) )
      {
        // Do parsing here and invoke callback if an interface matches our needs
      }
    }

    fclose( fh );
  }

  return VP_COM_OK;
}

C_RESULT vp_com_wf_inquire(const char* deviceName, vp_com_inquiry_t callback, uint32_t timeout)
{
  return VP_COM_OK;
}

#ifdef USE_BROADCOM
C_RESULT vp_com_wf_local_config(vp_com_wifi_config_t* cfg)
{
  C_RESULT res = VP_COM_ERROR;

#ifdef STA
//  wl_scan_results_t* scan_results;
//  wl_bss_info_t* bss; uint8_t* ptr;
//  int32_t i;
//  uint32_t channels_load[10], channel_load, next_channel_load;
#endif

  DEBUG_PRINT_SDK("vp_com_wf_local_config\n");

  vp_com_config_itf( cfg->itfName, cfg->localHost, cfg->broadcast, cfg->netmask );

    VP_WLC_BEGIN( cfg->itfName )

      if( vp_wlc_get_magic() == WLC_IOCTL_MAGIC )
      {
        vp_wlc_down();
        vp_wlc_set_country( cfg->country );
        vp_wlc_up();

        if( vp_wlc_get_radio() != VP_WLC_RADIO_ENABLE )
          vp_wlc_set_radio(VP_WLC_RADIO_ENABLE);

#ifdef STA
       /* for( i = 1; i < 10; i++ )
          channels_load[i] = 0;

        if( vp_wlc_scan() == 0 )
        {
          vp_com_wait_scan_complete();
          scan_results = vp_wlc_get_scan_results();

          PRINT("Wifi scan gave %d results\n", scan_results->count);

          ptr = (void*)&scan_results->bss_info[0];
          for(i=0; i < scan_results->count; i++)
          {
            int32_t index_min, index, index_max;

            bss = (wl_bss_info_t*) ptr;
            ptr = (uint8_t*)bss + bss->length;

            index     = bss->chanspec & WL_CHANSPEC_CHAN_MASK;
            if( index < 10 )
            {
              index_min = index-1;
              index_max = index+1;

              channels_load[index] += 5;
              if( index_min > 0 )
                channels_load[index_min] += 3;
              if( index_max < 10 )
                channels_load[index_max] += 3;
            }

            PRINT("\t-bss %s on channel %d\n", bss->SSID, bss->chanspec & WL_CHANSPEC_CHAN_MASK );
          }

          channels_load[0] = 1;
          channel_load = channels_load[1] + channels_load[2];

          for( i = 2; i < 9; i++ )
          {
            next_channel_load = channels_load[i-1] + channels_load[i] + channels_load[i+1];

            if( channel_load > next_channel_load )
            {
              channel_load = next_channel_load;
              channels_load[0] = i;
            }
          }

          next_channel_load = channels_load[8] + channels_load[9];
          if( channel_load > next_channel_load )
          {
            channel_load = next_channel_load;
            channels_load[0] = 9;
          }

          vp_wlc_set_channel( channels_load[0] );

          PRINT("Setting channel %d\n", channels_load[0]);

        }*/
#endif

        vp_wlc_set_infrastructure( (cfg->infrastructure == 1) ? VP_WLC_INFRASTRUCTURE : VP_WLC_ADHOC );
        vp_wlc_set_authentication( (cfg->secure == 1) ? VP_WLC_AUTH_SHARED : VP_WLC_AUTH_OPEN );

        res = VP_COM_OK;
      }
#ifdef STA
      vp_wlc_down();
#endif
    VP_WLC_END

  return res;
}
#endif

#ifdef USE_IWLIB
C_RESULT vp_com_wf_local_config(vp_com_wifi_config_t* cfg)
{
	C_RESULT res=C_FAIL;
	int wlsock = -1;
	int16_t key_flags = 0;
	wireless_config iwconf;

	// If network is secure, configure it now.
	if(cfg && cfg->secure==1)
	{
		vp_com_config_itf( cfg->itfName, cfg->localHost, cfg->broadcast, cfg->netmask );

		wlsock = iw_sockets_open();

		res = ( wlsock < 0 ) ? VP_COM_ERROR : VP_COM_OK;
		VP_COM_CHECK( res );

		iw_get_basic_config( wlsock, cfg->itfName, &iwconf );
		iwconf.has_nwid = 0;
		iwconf.has_freq = 0;
		iwconf.has_mode = 0;
		iwconf.has_key  = 0;

		// After watching Linux/kernel/linux/drivers/parrot/net/wireless/ar6000/ar6000/wireless.ext:1041
		// it appears we need to clear ssid before setting passkey
		//     if (ar->arSsidLen) {
		//         return -EIO;
		//     }
		// So we do set passkey in two passes, first clear ssid then set new passkey

		iwconf.has_essid = 1;
		iwconf.essid_on = 0;
		vp_os_memset( &iwconf.essid[0], 0, IW_ESSID_MAX_SIZE+1 );

		res = iw_set_basic_config( wlsock, cfg->itfName, &iwconf ) < 0 ? C_FAIL : C_OK;

		// Setup security
		iwconf.has_essid = 0;
		iwconf.has_key = 1;

		if(cfg->secure)
		{
			iwconf.key_size =  iw_in_key_full( wlsock, cfg->itfName, cfg->passkey, (char*)&iwconf.key, &key_flags );
			iwconf.key_flags  = IW_ENCODE_ENABLED | IW_ENCODE_RESTRICTED;
		}
		else
		{
			vp_os_memset(&iwconf.key[0], 0, IW_ENCODING_TOKEN_MAX);
			iwconf.key_flags  = IW_ENCODE_DISABLED;
		}

		res = iw_set_basic_config( wlsock, cfg->itfName, &iwconf ) < 0 ? C_FAIL : C_OK;

		iw_sockets_close(wlsock);
	}
	else
	{
		res=C_OK;
	}
	return res;
}
#else
#ifndef USE_BROADCOM
C_RESULT vp_com_wf_local_config(vp_com_wifi_config_t* cfg)
{
	return C_OK;
}
#endif
#endif

#ifdef USE_BROADCOM
C_RESULT vp_com_wf_connect(vp_com_t* vp_com, vp_com_wifi_connection_t* connection, int32_t numAttempts)
{
  C_RESULT res = VP_COM_ERROR;
  ///struct bootp* bootp_data = NULL;
  vp_com_wifi_config_t* config = (vp_com_wifi_config_t*) vp_com->config;

#ifdef STA
  int32_t   attempt;
  int32_t   ret;
  bdaddr_t  bdaddr;
#endif

  DEBUG_PRINT_SDK("vp_com_wf_connect\n");

  VP_WLC_BEGIN(config->itfName)
#ifdef STA
    vp_wlc_up();

    attempt = 0;

    do
    {
#endif
      vp_wlc_set_ssid(connection->networkName);
#ifdef STA
      vp_com_wait_set_ssid();
      attempt ++;
    }
    while( ((ret = vp_wlc_get_bssid(&bdaddr)) < 0) && attempt < 10 );

    if(ret >= 0)
    {
#endif

      res = VP_COM_OK;

#ifdef STA
      // Disable roaming
      vp_wlc_disable_roaming();
    }
#endif
  VP_WLC_END

  if( SUCCEED( res ) )
  {
#ifdef STA
    PRINT("STA - Joined essid %s\n", connection->networkName);
#else // AP
    PRINT("AP - Created essid %s\n", connection->networkName);
#endif // STA
  }
  else
  {
    DEBUG_PRINT_SDK("vp_com_wf_connect failed\n");
  }
  return res;
}
#endif

#ifdef USE_IWLIB
C_RESULT vp_com_wf_connect(vp_com_t* vp_com, vp_com_wifi_connection_t* connection, int32_t numAttempts)
{
  C_RESULT res;
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

	// No need to set the network name if it has not change.
	if(strcmp(&iwconf.essid[0], connection->networkName)!=0)
	{
		strncpy( &iwconf.essid[0], connection->networkName, IW_ESSID_MAX_SIZE+1 );

		res = iw_set_basic_config( wlsock, config->itfName, &iwconf ) < 0 ? C_FAIL : C_OK;
	}
	else
	{
		res=C_OK;
	}

  if( SUCCEED(res) )
    PRINT(" OK!\n");
  else
    PRINT(" FAILED!\n");

  iw_sockets_close(wlsock);

  return res;
}
#else
#ifndef USE_BROADCOM
C_RESULT vp_com_wf_connect(vp_com_t* vp_com, vp_com_wifi_connection_t* connection, int32_t numAttempts)
{
	return C_OK;
}
#endif
#endif

#ifdef USE_BROADCOM
C_RESULT vp_com_wf_disconnect(vp_com_wifi_config_t* config, vp_com_wifi_connection_t* connection)
{
  VP_WLC_BEGIN( config->itfName )
#ifdef STA
    vp_wlc_up();
    vp_wlc_set_ssid( NULL );
//    vp_com_wait_set_ssid();
#endif
    vp_wlc_down();
  VP_WLC_END

  return VP_COM_OK;
}
#endif

#ifdef USE_IWLIB
C_RESULT vp_com_wf_disconnect(vp_com_wifi_config_t* cfg, vp_com_wifi_connection_t* connection)
{
  C_RESULT res;
  int wlsock = -1;
  int16_t key_flags = 0;
  wireless_config iwconf;

  wlsock = iw_sockets_open();

  VP_COM_CHECK( ( wlsock < 0 ) ? VP_COM_ERROR : VP_COM_OK );

  vp_os_memset( &iwconf, 0, sizeof(iwconf) );

  iwconf.has_essid = 1;

  res = iw_set_basic_config( wlsock, cfg->itfName, &iwconf ) < 0 ? C_FAIL : C_OK;

  iw_sockets_close(wlsock);

  return res;
}
#else
#ifndef USE_BROADCOM
C_RESULT vp_com_wf_disconnect(vp_com_wifi_config_t* cfg, vp_com_wifi_connection_t* connection)
{
	return C_OK;
}
#endif
#endif

C_RESULT vp_com_wf_get_rssi(vp_com_wifi_config_t* cfg, int32_t* rssi)
{
  /*struct iwreq wrq;
  iwstats stats;
  iwrange range;

  int wlsock = iw_sockets_open();

  vp_os_memset(&wrq, 0, sizeof(struct iwreq));
  iw_get_stats(wlsock, cfg->itfName, &stats, &range, 1);
  iw_sockets_close(wlsock);*/

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

  //*rssi = stats.qual.qual;

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
