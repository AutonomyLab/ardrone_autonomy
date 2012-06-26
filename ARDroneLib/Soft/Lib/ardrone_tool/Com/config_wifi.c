#include <VP_Os/vp_os_malloc.h>

#include <config.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <ardrone_tool/Com/config_com.h>
#include <VP_Os/vp_os_print.h>

#include <VP_Com/vp_com_socket.h>
#include <VP_Com/vp_com.h>

#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <net/if.h>
#include <arpa/inet.h>
#include <errno.h>
#include <stdio.h>

#ifndef USE_ANDROID
#include <ifaddrs.h>
#else
#include <net/if_arp.h>
#endif

#define BUFFER_SIZE			1024
#define AUTH_MAX_NUMRETRIES	3
#define AUTH_MSG			"PARROT AUTH"
#define AUTH_MSG_OK			"PARROT AUTH OK"

vp_com_t* wifi_com(void)
{
  static vp_com_t com = {
    VP_COM_WIFI,
    FALSE,
    0,
#ifdef _WIN32
    { 0 },
#else // ! USE_MINGW32
    PTHREAD_MUTEX_INITIALIZER,
#endif // ! USE_MINGW32
    NULL,
    NULL,
    0,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL
  };

  return &com;
}

#define inaddrr(x) (*(struct in_addr *) &ifr->x[sizeof sa.sin_port])
#define IFRSIZE ((int)(size * sizeof (struct ifreq)))

C_RESULT wifi_server_auth(struct in_addr *addr)
{
	Read read = NULL;
	Write write = NULL;
	C_RESULT result = C_FAIL;
    int numretries = 1;
    uint8_t recvString[BUFFER_SIZE]; /* Buffer for received string */
    int recvStringLen;            /* Length of received string */
    struct timeval tv = {0, 500000};
    int on=1;
    const uint8_t msg[] = AUTH_MSG;
    struct in_addr to;
    struct in_addr from;

    vp_com_socket_t socket;

    // Initialize sending socket
    to.s_addr = htonl(WIFI_BROADCAST_ADDR);
	COM_CONFIG_SOCKET_AUTH(&socket, VP_COM_CLIENT, AUTH_PORT, inet_ntoa(to));
	socket.protocol = VP_COM_UDP;

	if(VP_FAILED(vp_com_init(COM_AUTH())))
	{
		printf("Failed to init Authentification\n");
		vp_com_shutdown( COM_AUTH() );
		return C_FAIL;
	}

	if(VP_FAILED(vp_com_open(COM_AUTH(), &socket, &read, &write)))
	{
		printf("Failed to open Authentification\n");
		vp_com_shutdown( COM_AUTH() );
		return C_FAIL;
	}

    if (setsockopt((int)socket.priv, SOL_SOCKET, SO_BROADCAST,(char *)&on,sizeof(on)) < 0)
    {
		printf("Failed to set socket option Authentification\n");
    	vp_com_close(COM_AUTH(), &socket);
		vp_com_shutdown( COM_AUTH() );
        return C_FAIL;
    }

    if (setsockopt((int)socket.priv, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0)
    {
		printf("Failed to set socket option Authentification\n");
    	vp_com_close(COM_AUTH(), &socket);
		vp_com_shutdown( COM_AUTH() );
        return C_FAIL;
    }

    do
    {
    	int len = strlen((char*)msg);
    	if(write != NULL)
    	{
    		if (VP_FAILED(write(&socket, msg, &len)))
			{
				vp_com_close(COM_AUTH(), &socket);
				vp_com_shutdown( COM_AUTH() );
				return C_FAIL;
			}
    	}

    	printf("Wait authentification\n");
    	do
        {
        	if(read != NULL)
        	{
             	recvStringLen = BUFFER_SIZE;
				if(VP_FAILED(read(&socket, recvString, &recvStringLen)))
				{
					vp_com_close(COM_AUTH(), &socket);
					vp_com_shutdown( COM_AUTH());
					return C_FAIL;
				}
        	}

        	recvString[recvStringLen] = '\0';
        }
        while((recvStringLen != 0) && (strcmp((char *)recvString, AUTH_MSG) == 0));
    }
    while((strcmp((char *)recvString, AUTH_MSG_OK) != 0) && (numretries++ < AUTH_MAX_NUMRETRIES));

    if(strcmp((char*)recvString, AUTH_MSG_OK) == 0)
    {
    	from.s_addr = socket.scn;
    	printf("Authentification ok from %s:%d\n", inet_ntoa(from), socket.port);
    	memcpy(addr, &from, sizeof(struct in_addr));
    	result = C_OK;
    }

    vp_com_close(COM_AUTH(), &socket);
	vp_com_shutdown( COM_AUTH() );

    return result;
}



#ifdef USE_ANDROID
vp_com_config_t* wifi_config(void)
{
	static vp_com_wifi_config_t config =
	{
		{ 0 },
		WIFI_MOBILE_IP,
		WIFI_NETMASK,
		WIFI_BROADCAST,
		WIFI_GATEWAY,
		WIFI_SERVER,
		WIFI_INFRASTRUCTURE,
		WIFI_SECURE,
		WIFI_PASSKEY,
		{ 0 },
	};

	unsigned char *u;
	int sockfd, size = 1;
	struct ifreq *ifr;
	struct ifconf ifc;
	struct sockaddr_in sa;

	if (0 > (sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP))) {
		fprintf(stderr, "Cannot open socket.\n");
		exit(EXIT_FAILURE);
	}

	ifc.ifc_len = IFRSIZE;
	ifc.ifc_req = NULL;

	do {
		++size;
		/* realloc buffer size until no overflow occurs */
		if (NULL == (ifc.ifc_req = vp_os_realloc(ifc.ifc_req, IFRSIZE))) {
			fprintf(stderr, "Out of memory.\n");
			exit(EXIT_FAILURE);
		}
		ifc.ifc_len = IFRSIZE;
		if (ioctl(sockfd, SIOCGIFCONF, &ifc)) {
			perror("ioctl SIOCFIFCONF");
			exit(EXIT_FAILURE);
		}
	} while (IFRSIZE <= ifc.ifc_len);

	ifr = ifc.ifc_req;
	for (; (char *) ifr < (char *) ifc.ifc_req + ifc.ifc_len; ++ifr) {

		if (ifr->ifr_addr.sa_data == (ifr + 1)->ifr_addr.sa_data) {
			continue; /* duplicate, skip it */
		}

		if (ioctl(sockfd, SIOCGIFFLAGS, ifr)) {
			continue; /* failed to get flags, skip it */
		}

		struct in_addr addr;
		addr = inaddrr(ifr_addr.sa_data);

		if ( (ntohl(addr.s_addr) & 0xFFFFFF00) == WIFI_BASE_ADDR )
		{
			inet_ntop(AF_INET, &addr, config.localHost, VP_COM_NAME_MAXSIZE);

			//INTERFACE
			memcpy(config.itfName, ifr->ifr_name, strlen(ifr->ifr_name));

			//IP ADDRESS
			addr.s_addr = htonl ( ntohl(addr.s_addr) - ( ( ( ntohl(addr.s_addr) & 0xFF ) - 1 ) % 5 ) );
			memcpy(config.server, inet_ntoa(addr), strlen(inet_ntoa(addr)));

			//NET MASK
			if (0 == ioctl(sockfd, SIOCGIFNETMASK, ifr) && strcmp(
					"255.255.255.255", inet_ntoa(inaddrr(ifr_addr.sa_data)))) {

				addr = inaddrr(ifr_addr.sa_data);
				inet_ntop(AF_INET, &addr, config.netmask, VP_COM_NAME_MAXSIZE);
			}

			//BROADCAST ADDRESS
			if (ifr->ifr_flags & IFF_BROADCAST) {
				if (0 == ioctl(sockfd, SIOCGIFBRDADDR, ifr) && strcmp("0.0.0.0",
						inet_ntoa(inaddrr(ifr_addr.sa_data)))) {

					addr = inaddrr(ifr_addr.sa_data);
					inet_ntop(AF_INET, &addr, config.broadcast, VP_COM_NAME_MAXSIZE);
				}
			}

			break;
		}
	}

	close(sockfd);
	return (vp_com_config_t*)&config;
}

#else
vp_com_config_t* wifi_config(void)
{
	static vp_com_wifi_config_t config =
	{
		{ 0 },
		WIFI_MOBILE_IP,
		WIFI_NETMASK,
		WIFI_BROADCAST,
		WIFI_GATEWAY,
		WIFI_SERVER,
		WIFI_INFRASTRUCTURE,
		WIFI_SECURE,
		WIFI_PASSKEY,
		{ 0 },
	};

	struct ifaddrs * ifAddrStructHead = NULL;
	struct ifaddrs * ifAddrStruct = NULL;
	struct in_addr tmpAddr;
	bool_t found = FALSE;

	if(strlen(config.itfName) == 0)
	{
		getifaddrs(&ifAddrStruct);
		ifAddrStructHead = ifAddrStruct;

		while (!found && (ifAddrStruct != NULL))
		{
			// Looking for WIFI interface's IP address corresponding to WIFI_BASE_ADDR
			if (ifAddrStruct->ifa_addr->sa_family == AF_INET)
			{
				tmpAddr = ((struct sockaddr_in *)ifAddrStruct->ifa_addr)->sin_addr;
				if ( (ntohl(tmpAddr.s_addr) & 0xFFFFFF00) == WIFI_BASE_ADDR )
				{
					inet_ntop(AF_INET, &tmpAddr, config.localHost, VP_COM_NAME_MAXSIZE);
					memcpy(config.itfName, ifAddrStruct->ifa_name, strlen(ifAddrStruct->ifa_name));
					if(VP_FAILED(wifi_server_auth(&tmpAddr)))
						tmpAddr.s_addr = htonl ( ntohl(tmpAddr.s_addr) - ( ( ( ntohl(tmpAddr.s_addr) & 0xFF ) - 1 ) % 5 ) );
					memcpy(config.server, inet_ntoa(tmpAddr), strlen(inet_ntoa(tmpAddr)));
					tmpAddr = ((struct sockaddr_in *)ifAddrStruct->ifa_netmask)->sin_addr;
					inet_ntop(AF_INET, &tmpAddr, config.netmask, VP_COM_NAME_MAXSIZE);
					tmpAddr = ((struct sockaddr_in *)ifAddrStruct->ifa_broadaddr)->sin_addr;
					inet_ntop(AF_INET, &tmpAddr, config.broadcast, VP_COM_NAME_MAXSIZE);
					found = TRUE;
				}
			}
			ifAddrStruct = ifAddrStruct->ifa_next;
		}

		if (ifAddrStructHead != NULL)
		{
		  freeifaddrs(ifAddrStructHead);
		}
	}

	return (vp_com_config_t*)&config;
}
#endif

vp_com_connection_t* wifi_connection(void)
{
  static vp_com_wifi_connection_t connection = {
    0,
    WIFI_NETWORK_NAME
  };

  return (vp_com_connection_t*) &connection;
}

void wifi_config_socket(vp_com_socket_t* socket, VP_COM_SOCKET_TYPE type, int32_t port, const char* serverhost)
{
  vp_os_memset(socket, 0, sizeof(vp_com_socket_t));

  socket->type           = type;
  socket->protocol       = VP_COM_TCP;
  socket->port           = port;

  if(serverhost && socket->type == VP_COM_CLIENT)
	{
    	strncpy(socket->serverHost, serverhost, VP_COM_NAME_MAXSIZE-1);
		socket->serverHost[VP_COM_NAME_MAXSIZE-1]='\0';
	}
}
