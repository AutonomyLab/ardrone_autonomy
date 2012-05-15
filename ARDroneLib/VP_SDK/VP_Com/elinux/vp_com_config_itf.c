#include <VP_Os/vp_os_malloc.h> 
#include <VP_Os/vp_os_print.h>

#include <VP_Com/linux/vp_com_config_itf.h>

#include <string.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <arpa/inet.h>
#include <unistd.h>

// taken from ifconfig
static int vp_com_set_flag( int skfd, const char *ifname, short flag )
{
  struct ifreq ifr;

  vp_os_memset( &ifr, 0, sizeof(struct ifreq) );
  strncpy( ifr.ifr_name, ifname, IFNAMSIZ );

  if( ioctl( skfd, SIOCGIFFLAGS, &ifr ) < 0 )
    return -1;

  strncpy( ifr.ifr_name, ifname, IFNAMSIZ );
  ifr.ifr_flags |= flag;

  if( ioctl(skfd, SIOCSIFFLAGS, &ifr ) < 0 )
    return -1;

  return 0;
}

static int vp_com_set_ip( int skfd, const char* ifname, int cmd, int ip )
{
  struct ifreq ifr;
  struct sockaddr_in* addr;

  vp_os_memset( &ifr, 0, sizeof(struct ifreq) );
  strncpy( ifr.ifr_name, ifname, IFNAMSIZ );

  // Check if an ip is already set
  if( ioctl( skfd, SIOCGIFADDR, &ifr ) != -1 )
  {
    addr = (struct sockaddr_in*) &ifr.ifr_addr;

    // Is this the same ip?
    if( addr->sin_family == AF_INET && ip == addr->sin_addr.s_addr )
      return 0;
/*
    // No so we try to delete it
    if( ioctl( skfd, SIOCDIFADDR, &ifr ) < 0 )
    {
      DEBUG_PRINT_SDK("Unable to delete old ip address - You should remove your interface %s\n", ifname);
      return -1;
    }
*/
  }

  vp_os_memset( &ifr, 0, sizeof(struct ifreq) );
  strncpy( ifr.ifr_name, ifname, IFNAMSIZ );

  addr = (struct sockaddr_in*) &ifr.ifr_addr;
  addr->sin_family      = AF_INET;
  addr->sin_addr.s_addr = ip;

  strncpy( ifr.ifr_name, ifname, IFNAMSIZ );

  if( ioctl( skfd, cmd, &ifr ) < 0 )
    return -1;

  return 0;
}


int vp_com_config_itf( const char* interface, const char * ip, const char* broadcast, const char* netmask )
{
  int ret = -1;

  int sck = socket( PF_INET, SOCK_DGRAM, 0 );

  if( sck < 0 )
    return -1;

  if( vp_com_set_ip( sck, interface, SIOCSIFADDR, inet_addr(ip) ) < 0 )
    goto vp_com_config_itf_end;

  if( vp_com_set_ip( sck, interface, SIOCSIFBRDADDR, inet_addr(broadcast) ) < 0 )
    goto vp_com_config_itf_end;

  if( vp_com_set_ip( sck, interface, SIOCSIFNETMASK, inet_addr(netmask) ) < 0 )
    goto vp_com_config_itf_end;

  if( vp_com_set_flag( sck, interface, IFF_UP | IFF_RUNNING ) < 0 )
    goto vp_com_config_itf_end;

  ret = 0;

vp_com_config_itf_end:

  close(sck);

  return ret;
}
