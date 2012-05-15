/**
 *  \brief    OS Api for video sdk. Public definitions.
 *  \author   Aurelien Morelle <aurelien.morelle@parrot.fr>
 *  \author   Sylvain Gaeremynck <sylvain.gaeremynck@parrot.fr>
 *  \version  2.0
 *  \date     2006/12/15
 */

#ifndef _SIGNAL_INCLUDE_OS_DEP_
#define _SIGNAL_INCLUDE_OS_DEP_

/* 
IMPORTANT - select here whether to use condition variables from
			the Windows SDK (Vista and later) or from the 'Pthread for Win32' library (XP and earlier).
*/
#define USE_WINDOWS_CONDITION_VARIABLES
//#define USE_PTHREAD_FOR_WIN32



#include <VP_Os/vp_os.h>




#if defined USE_WINDOWS_CONDITION_VARIABLES
	/* CONDITION_VARIABLE only work on VISTA/SEVEN/Server 2008 */
	typedef CRITICAL_SECTION vp_os_mutex_t;

	typedef struct
	{
	  CONDITION_VARIABLE  cond;
	  vp_os_mutex_t       *mutex;
	}
	vp_os_cond_t;
	

#elif defined USE_PTHREAD_FOR_WIN32
		#include <pthread.h>
	
		typedef pthread_mutex_t vp_os_mutex_t;

		typedef struct _vp_os_cond_t_
		{
		  pthread_cond_t  cond;
		  vp_os_mutex_t     *mutex;
		}
		vp_os_cond_t;

#else

	typedef CRITICAL_SECTION vp_os_cond_t;
	typedef CRITICAL_SECTION vp_os_mutex_t;

#endif


#endif // ! _SIGNAL_INCLUDE_OS_DEP_

