#ifndef _ARDRONE_NAVDATA_CLIENT_H_
#define _ARDRONE_NAVDATA_CLIENT_H_

#include <VP_Os/vp_os_types.h>
#include <VP_Api/vp_api_thread_helper.h>

#include <ardrone_api.h>
#include <ardrone_tool/Control/ardrone_navdata_control.h>
#include <ardrone_tool/Navdata/ardrone_general_navdata.h>
#include <config.h>

#define NAVDATA_MAX_RETRIES	5

// Facility to declare a set of navdata handler
// Handler to resume control thread is mandatory
#define BEGIN_NAVDATA_HANDLER_TABLE \
  ardrone_navdata_handler_t ardrone_navdata_handler_table[] = {

#define END_NAVDATA_HANDLER_TABLE					\
  { ardrone_general_navdata_init, ardrone_general_navdata_process, ardrone_general_navdata_release, NULL }, \
  { ardrone_navdata_control_init, ardrone_navdata_control_process, ardrone_navdata_control_release, NULL }, \
  { NULL, NULL, NULL, NULL }						\
};

#define NAVDATA_HANDLER_TABLE_ENTRY( init, process, release, init_data_ptr ) \
  { (ardrone_navdata_handler_init_t)init, process, release, init_data_ptr },

typedef C_RESULT (*ardrone_navdata_handler_init_t)( void* data );
typedef C_RESULT (*ardrone_navdata_handler_process_t)( const navdata_unpacked_t* const navdata );
typedef C_RESULT (*ardrone_navdata_handler_release_t)( void );

typedef struct _ardrone_navdata_handler_t {
  ardrone_navdata_handler_init_t    init;
  ardrone_navdata_handler_process_t process;
  ardrone_navdata_handler_release_t release;

  void*                             data; // Data used during initialization
} ardrone_navdata_handler_t;

typedef enum
{
	NAVDATA_BOOTSTRAP = 0,
	NAVDATA_DEMO,
	NAVDATA_FULL
} navdata_mode_t;

extern ardrone_navdata_handler_t ardrone_navdata_handler_table[] WEAK;

uint32_t ardrone_navdata_client_get_num_retries(void);
C_RESULT ardrone_navdata_client_init(void);
C_RESULT ardrone_navdata_client_suspend(void);
C_RESULT ardrone_navdata_client_resume(void);
C_RESULT ardrone_navdata_client_shutdown(void);
C_RESULT ardrone_navdata_open_server(void);

PROTO_THREAD_ROUTINE( navdata_update , nomParams );

#endif // _ARDRONE_NAVDATA_H_

