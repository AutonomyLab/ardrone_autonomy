#ifndef _ARDRONE_NAVDATA_FILE_H_
#define _ARDRONE_NAVDATA_FILE_H_

#include <stdio.h>
#include <ardrone_api.h>

#include <ardrone_tool/Navdata/ardrone_navdata_client.h>

extern FILE* navdata_file;

// For this handler, data is the path where the file will be created
// If data is NULL then the file is created in current directory
C_RESULT ardrone_navdata_file_init( void* data );
C_RESULT ardrone_navdata_file_process( const navdata_unpacked_t* const navdata );
C_RESULT ardrone_navdata_file_release( void );

#endif // _ARDRONE_NAVDATA_FILE_H_
