#ifndef _PARROTOS_CONFIG_H_
#define _PARROTOS_CONFIG_H_

#ifdef DEBUG_MODE
#define POSIX_DEBUG
#endif // DEBUG_MODE

/** PAL_STACKSIZE range from 4096 to 65536 **/
#define PAL_STACKSIZE_DEFAULT 6144

#define PAL_STACKSIZE PAL_STACKSIZE_DEFAULT

/** Trace pal functions execution **/
#define PAL_TRACE_ALARM_VAL     0
#define PAL_TRACE_COND_VAL      0
#define PAL_TRACE_FLAG_VAL      0
#define PAL_TRACE_HWALARM_VAL   0
#define PAL_TRACE_MBOX_VAL      0
#define PAL_TRACE_MUTEX_VAL     0
#define PAL_TRACE_SEM_VAL       0
#define PAL_TRACE_SYS_VAL       0
#define PAL_TRACE_THREAD_VAL    0
#define PAL_TRACE_TIME_VAL      0
#define PAL_TRACE_UART_VAL      0
#define PAL_TRACE_GPIO_VAL      0

#endif // _PARROTOS_CONFIG_H_
