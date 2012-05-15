#ifndef _SYSLOG_H_
#define _SYSLOG_H_

#define LOG_PID 0x01
#define LOG_CONS 0x02
#define LOG_PERROR 0x20
#define openlog(A,B,C) (void)(A); (void)(B)
#define syslog(A,B,C)
#define closelog()

#endif // _SYSLOG_H_
