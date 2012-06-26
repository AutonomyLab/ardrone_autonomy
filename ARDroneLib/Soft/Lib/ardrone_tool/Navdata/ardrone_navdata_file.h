#ifndef _ARDRONE_NAVDATA_FILE_H_
#define _ARDRONE_NAVDATA_FILE_H_

#include <stdio.h>
#include <ardrone_api.h>

#include <ardrone_tool/Navdata/ardrone_navdata_client.h>

extern FILE* navdata_file;

typedef void (*ardrone_navdata_custom_print_header)(FILE *navdata_file);
typedef void (*ardrone_navdata_custom_print)(FILE *navdata_file);

typedef struct _ardrone_navdata_file_data_
{
	const char *filename;
	ardrone_navdata_custom_print_header print_header;
	ardrone_navdata_custom_print 		print;
} ardrone_navdata_file_data;

// For this handler, data is the path where the file will be created
// If data is NULL then the file is created in current directory
C_RESULT ardrone_navdata_file_init( void* data );
C_RESULT ardrone_navdata_file_process( const navdata_unpacked_t* const navdata );
C_RESULT ardrone_navdata_file_release( void );

#endif // _ARDRONE_NAVDATA_FILE_H_
