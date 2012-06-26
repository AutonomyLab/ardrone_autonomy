/*
 * vp_os_ltt.h
 *
 *  Created on: Sep 15, 2009
 *      Author: pierre
 */
//#define LTT_TRACE
#ifndef VP_OS_LTT_H_
#define VP_OS_LTT_H_

#if defined(LTT_TRACE) && defined (USE_ELINUX) && !defined(DEBUG_MODE)
#include <ltt-user.h>
extern int fd_ltt;
# define LTT_START()            if (fd_ltt == -1)fd_ltt = trace_open();
# define LTT_STOP()             trace_close(fd_ltt);fd_ltt=-1;
# define LTT_WRITEF(...)        if (fd_ltt!=-1)trace_writef(fd_ltt,__VA_ARGS__);
# define LTT_SET_ID(x)          if (fd_ltt!=-1)trace_start_id(fd_ltt,x);
# define LTT_CLEAR_ID(x)        if (fd_ltt!=-1)trace_stop_id(fd_ltt,x);
#else
# define LTT_START()            do{}while(0)
# define LTT_STOP()             do{}while(0)
# define LTT_WRITEF(...)        do{}while(0)
# define LTT_SET_ID(...)        do{}while(0)
# define LTT_CLEAR_ID(...)        do{}while(0)
#endif
#endif
