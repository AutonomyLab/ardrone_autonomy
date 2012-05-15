/*
 *
 *  BlueZ - Bluetooth protocol stack for Linux
 *
 *  Copyright (C) 2002-2003  Maxim Krasnyansky <maxk@qualcomm.com>
 *  Copyright (C) 2002-2006  Marcel Holtmann <marcel@holtmann.org>
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <stdio.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <signal.h>
#include <getopt.h>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/socket.h>

#include <netinet/in.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/l2cap.h>
#include <bluetooth/bnep.h>
#include <bluetooth/hci_lib.h>

#include <VP_Com/vp_com_error.h>
#include "bluez.h"

/**
 * From bnep.c BlueZ source file
 */

static int ctl;

/* Compatibility with old ioctls */
#define OLD_BNEPCONADD      1
#define OLD_BNEPCONDEL      2
#define OLD_BNEPGETCONLIST  3
#define OLD_BNEPGETCONINFO  4

static unsigned long bnepconnadd;
static unsigned long bnepconndel;
static unsigned long bnepgetconnlist;
static unsigned long bnepgetconninfo;

struct __service_16 { 
	uint16_t dst;
	uint16_t src;
} __attribute__ ((packed));

struct __service_32 { 
	uint16_t unused1;
	uint16_t dst;
	uint16_t unused2;
	uint16_t src;
} __attribute__ ((packed));

struct __service_128 { 
	uint16_t unused1;
	uint16_t dst;
	uint16_t unused2[8];
	uint16_t src;
	uint16_t unused3[7];
} __attribute__ ((packed));

C_RESULT bluez_init(void)
{
	ctl = socket(PF_BLUETOOTH, SOCK_RAW, BTPROTO_BNEP);
	if (ctl < 0) {
		perror("Failed to open control socket");
		return VP_COM_ERROR;
	}

	/* Temporary ioctl compatibility hack */
	{ 
		struct bnep_connlist_req req;
		struct bnep_conninfo ci[1];

		req.cnum = 1;
		req.ci   = ci;

		if (!ioctl(ctl, BNEPGETCONNLIST, &req)) {
			/* New ioctls */
			bnepconnadd     = BNEPCONNADD;
			bnepconndel     = BNEPCONNDEL;
			bnepgetconnlist = BNEPGETCONNLIST;
			bnepgetconninfo = BNEPGETCONNINFO;
		} else {
			/* Old ioctls */
			bnepconnadd     = OLD_BNEPCONADD;
			bnepconndel     = OLD_BNEPCONDEL;
			bnepgetconnlist = OLD_BNEPGETCONLIST;
			bnepgetconninfo = OLD_BNEPGETCONINFO;
		}
	}

	return VP_COM_OK;
}

int bluez_kill_all_connections(void)
{
  struct bnep_connlist_req req;
  struct bnep_conninfo ci[48];
  int i;

  req.cnum = 48;
  req.ci   = ci;
  if (ioctl(ctl, bnepgetconnlist, &req)) {
    perror("Failed to get connection list");
    return -1;
  }

  for (i=0; i < req.cnum; i++) {
    struct bnep_conndel_req req;
    memcpy(req.dst, ci[i].dst, ETH_ALEN);
    req.flags = 0;
    if (ioctl(ctl, bnepconndel, &req)) {
      perror("Failed to delete connection in connection list");
      return -1;
    }
  }

  return 0;
}

static int bnep_connadd(int sk, uint16_t role, char *dev)
{
  struct bnep_connadd_req req;

  strcpy( req.device, dev );
  req.sock = sk;
  req.role = role;
  if( ioctl( ctl, bnepconnadd, &req ) )
    return -1;

  strcpy( dev, req.device );

  return 0;
}

/* Create BNEP connection 
 * sk      - Connect L2CAP socket
 * role    - Local role
 * service - Remote service
 * dev     - Network device (contains actual dev name on return)
 */
int bnep_create_connection(int sk, uint16_t role, uint16_t svc, char *dev)
{
	struct bnep_setup_conn_req *req;
	struct bnep_control_rsp *rsp;
	struct __service_16 *s;
	unsigned char pkt[BNEP_MTU];
	int r;

	// Send request
	req = (void *) pkt;
	req->type = BNEP_CONTROL;
	req->ctrl = BNEP_SETUP_CONN_REQ;
	req->uuid_size = 2;	// 16bit UUID
	s = (void *) req->service;
	s->dst = htons(svc);
	s->src = htons(role);

	if (send(sk, pkt, sizeof(*req) + sizeof(*s), 0) < 0)
		return -1;

receive:
	// Get response
	r = recv(sk, pkt, BNEP_MTU, 0);
	if (r <= 0)
		return -1;

	errno = EPROTO;

	if (r < sizeof(*rsp))
		return -1;
	
	rsp = (void *) pkt;
	if (rsp->type != BNEP_CONTROL)
		return -1;

	if (rsp->ctrl != BNEP_SETUP_CONN_RSP)
		goto receive;

	r = ntohs(rsp->resp);

	switch (r) {
	case BNEP_SUCCESS:
		break;

	case BNEP_CONN_INVALID_DST:
	case BNEP_CONN_INVALID_SRC:
	case BNEP_CONN_INVALID_SVC:
		errno = EPROTO;
		return -1;

	case BNEP_CONN_NOT_ALLOWED:
		errno = EACCES;
		return -1;
	}

	return bnep_connadd( sk, role, dev );
}

/**
 * From pand.c BlueZ source file
 */

enum {
	NONE,
	SHOW,
	LISTEN,
	CONNECT,
	KILL
} modes;

static uint16_t role = BNEP_SVC_PANU;	/* Local role (ie service) */
static uint16_t service = BNEP_SVC_NAP;	/* Remote service */

//static int  detach = 0;
static int  persist;
//static int  use_sdp = 1;
//static int  use_cache;
//static int  auth;
//static int  secure;
//static int  master;
//static int  cleanup;
//static int  search_duration = 10;

/*TODO
static struct {
	int      valid;
	char     dst[40];
	bdaddr_t bdaddr;
} cache;
*/

static char netdev[16] = "bnep%d";
//static char *pidfile = NULL;
//static bdaddr_t src_addr = *BDADDR_ANY;
//static int src_dev = -1;
//static bdaddr_t connection;

volatile int terminate;

//static void do_kill(char *dst);
/*
static void run_devup(char *dev, char *dst, int sk, int nsk)
{
	execl("/sbin/ifconfig", "ifconfig", "bnep0", "192.168.2.2", NULL);
	//exit(1);
	exit(-1);
}
*/
/* Connect and initiate BNEP session
 * Returns:
 *   -1 - critical error (exit persist mode)
 *   1  - non critical error
 *   0  - success
 */
static int create_connection(bdaddr_t* src, bdaddr_t *bdaddr)
{
	struct l2cap_options l2o;
	struct sockaddr_l2 l2a;
//	struct hci_dev_info di;
	socklen_t olen;
	int sk, r = 0;

        // memcpy(&src_addr,src,sizeof(bdaddr_t));

	sk = socket(AF_BLUETOOTH, SOCK_SEQPACKET, BTPROTO_L2CAP);
	if (sk < 0) {
		syslog(LOG_ERR, "Cannot create L2CAP socket. %s(%d)",
				strerror(errno), errno);
		return -1;
	}

	/* Setup L2CAP options according to BNEP spec */
	memset(&l2o, 0, sizeof(l2o));
	olen = sizeof(l2o);
	getsockopt(sk, SOL_L2CAP, L2CAP_OPTIONS, &l2o, &olen);
	l2o.imtu = l2o.omtu = BNEP_MTU;
	setsockopt(sk, SOL_L2CAP, L2CAP_OPTIONS, &l2o, sizeof(l2o));

	memset(&l2a, 0, sizeof(l2a));
	l2a.l2_family = AF_BLUETOOTH;
	bacpy(&l2a.l2_bdaddr, src);

	if (bind(sk, (struct sockaddr *) &l2a, sizeof(l2a)))
		syslog(LOG_ERR, "Bind failed. %s(%d)", 
				strerror(errno), errno);

	memset(&l2a, 0, sizeof(l2a));
	l2a.l2_family = AF_BLUETOOTH;
	bacpy(&l2a.l2_bdaddr, bdaddr);
	l2a.l2_psm = htobs(BNEP_PSM);

	if (!connect(sk, (struct sockaddr *) &l2a, sizeof(l2a)) && 
			!bnep_create_connection(sk, role, service, netdev)) {

		syslog(LOG_INFO, "%s connected", netdev); return 0;
/*
		switch(fork())
		  {
		  case -1:
		    r = 1;
		    break;
		  case 0:
		    run_devup(netdev, dst, sk, -1);
		    return -1;
		    break;
		  default:
		    break;
		  }
*/
/* 		if (persist) { */
/* 			w4_hup(sk); */

/* 			if (terminate && cleanup) { */
/* 				syslog(LOG_INFO, "Disconnecting from %s.", dst); */
/* 				do_kill(dst); */
/* 			} */
/* 		} */

/* 		r = 0; */
	} else {
//		syslog(LOG_ERR, "Connect to %s failed. %s(%d)",dst, strerror(errno), errno);
		r = 1;
	}

/* 	close(sk); */

/* 	if (use_cache) { */
/* 		if (!r) { */
/* 			/\* Succesesful connection, validate cache *\/ */
/* 			strcpy(cache.dst, dst); */
/* 			bacpy(&cache.bdaddr, bdaddr); */
/* 			cache.valid = use_cache; */
/* 		} else */
/* 			cache.valid--; */
/* 	} */

	return r;
}

static int accept_connection(int sk, uint16_t role, char *dev)
{
	struct bnep_setup_conn_req *req;
	struct bnep_control_rsp *rsp;
	unsigned char pkt[BNEP_MTU];
	int r;

	r = recv(sk, pkt, BNEP_MTU, 0);
	if (r <= 0)
		return -1;

	errno = EPROTO;

	if (r < sizeof(*req))
		return -1;

	req = (void *) pkt;
	if (req->type != BNEP_CONTROL || req->ctrl != BNEP_SETUP_CONN_REQ)
		return -1;

	/* FIXME: Check role UUIDs */

	rsp = (void *) pkt;
	rsp->type = BNEP_CONTROL;
	rsp->ctrl = BNEP_SETUP_CONN_RSP;
	rsp->resp = htons(BNEP_SUCCESS);
	if (send(sk, rsp, sizeof(*rsp), 0) < 0)
		return -1;

	return bnep_connadd(sk, role, dev);
}

/* Search and connect
 * Returns:
 *   -1 - critical error (exit persist mode)
 *   1  - non critical error
 *   0  - success
 */
static int do_connect(bdaddr_t* src,bdaddr_t* dst)
{
  //	inquiry_info *ii;
	int reconnect = 0;
	//	int i, n;
	int r = 0;

	do {
		if (reconnect)
			sleep(persist);
		reconnect = 1;

		//TODO if (cache.valid > 0) {
			/* Use cached bdaddr */
			// r = create_connection(cache.dst, &cache.bdaddr);
                        r = create_connection(src,dst);
			if (r < 0) {
				terminate = 1;
				break;
			}
			continue;
		//TODO }

	} while (!terminate && persist);

	return r;
}

void sig_hup(int sig)
{
	return;
}

void sig_term(int sig)
{
	terminate = 1;
}

// src is ethernet address of local bluetooth device
// dst is ethernet address of distant bluetooth device
C_RESULT bluez_create_connection( bdaddr_t* src, bdaddr_t* dst )
{
  struct sigaction sa;

  /* Initialize signals */
  memset(&sa, 0, sizeof(sa));
  sa.sa_flags   = SA_NOCLDSTOP;
  sa.sa_handler = SIG_IGN;
  sigaction(SIGCHLD, &sa, NULL);
  sigaction(SIGPIPE, &sa, NULL);

  sa.sa_handler = sig_hup;
  sigaction(SIGHUP, &sa, NULL);

  sa.sa_handler = sig_term;
  sigaction(SIGTERM, &sa, NULL);
  sigaction(SIGINT,  &sa, NULL);

  if(do_connect(src,dst))
    return VP_COM_ERROR;

  return VP_COM_OK;
}

C_RESULT bluez_listen_connection(bdaddr_t* src_addr)
{
  struct l2cap_options l2o;
  struct sockaddr_l2 l2a;
  socklen_t olen;
  int sk, lm;
  socklen_t alen = sizeof(l2a);
  int nsk;

  *src_addr = *BDADDR_ANY;

  /* Create L2CAP socket and bind it to PSM BNEP */
  sk = socket(AF_BLUETOOTH, SOCK_SEQPACKET, BTPROTO_L2CAP);
  if (sk < 0) {
          syslog(LOG_ERR, "Cannot create L2CAP socket. %s(%d)",
                          strerror(errno), errno);
          return VP_COM_ERROR;
  }

  memset(&l2a, 0, sizeof(l2a));
  l2a.l2_family = AF_BLUETOOTH;
  bacpy(&l2a.l2_bdaddr, src_addr);
  l2a.l2_psm = htobs(BNEP_PSM);

  if (bind(sk, (struct sockaddr *) &l2a, sizeof(l2a))) {
          syslog(LOG_ERR, "Bind failed. %s(%d)", strerror(errno), errno);
          return VP_COM_ERROR;
  }

  /* Setup L2CAP options according to BNEP spec */
  memset(&l2o, 0, sizeof(l2o));
  olen = sizeof(l2o);
  if (getsockopt(sk, SOL_L2CAP, L2CAP_OPTIONS, &l2o, &olen) < 0) {
          syslog(LOG_ERR, "Failed to get L2CAP options. %s(%d)",
                          strerror(errno), errno);
          return VP_COM_ERROR;
  }

  l2o.imtu = l2o.omtu = BNEP_MTU;
  if (setsockopt(sk, SOL_L2CAP, L2CAP_OPTIONS, &l2o, sizeof(l2o)) < 0) {
          syslog(LOG_ERR, "Failed to set L2CAP options. %s(%d)",
                          strerror(errno), errno);
          return VP_COM_ERROR;
  }

  /* Set link mode */
  lm = 0;
  lm |= L2CAP_LM_MASTER;
  lm |= L2CAP_LM_AUTH;
  lm |= L2CAP_LM_ENCRYPT;
  lm |= L2CAP_LM_SECURE;

  if (lm && setsockopt(sk, SOL_L2CAP, L2CAP_LM, &lm, sizeof(lm)) < 0) {
          syslog(LOG_ERR, "Failed to set link mode. %s(%d)", strerror(errno), errno);
          return VP_COM_ERROR;
  }

  listen(sk,1);

  nsk = accept(sk, (struct sockaddr *) &l2a, &alen);
  if(nsk < 0)
    return VP_COM_ERROR;

  if(accept_connection(nsk, role, netdev) != 0)
  {
    close(nsk);
    return VP_COM_ERROR;
  }

  bacpy(src_addr, &l2a.l2_bdaddr);

  return VP_COM_OK;
}

