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

#ifndef _COMBLUEZ_H_
#define _COMBLUEZ_H_

struct bdaddr_t;

// Create a connection between src (i.e. local adapter) and dst (i.e. remote adapter)
C_RESULT bluez_init(void);
C_RESULT bluez_create_connection(bdaddr_t* src, bdaddr_t* dst);
C_RESULT bluez_listen_connection(bdaddr_t* src_addr);
C_RESULT bluez_kill_all_connections();

#endif // _COMBLUEZ_H_
