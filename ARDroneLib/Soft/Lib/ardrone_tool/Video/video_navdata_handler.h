#ifndef _VIDEO_NAVDATA_HANDLER_H_
#define _VIDEO_NAVDATA_HANDLER_H_

#include <ardrone_tool/Navdata/ardrone_navdata_client.h>

// Handler that reset video connexion when there is a timeout in navdata connexion

extern uint32_t hdvideo_remaining_frames;
extern uint32_t hdvideo_remaining_kilobytes;
extern uint32_t hdvideo_maximum_kilobytes;
extern float hdvideo_fifo_fill_percentage; // range 0.0 - 100.0
extern float hdvideo_retrieving_progress; // -1.0 when not retreiving, else range 0.0-1.0

C_RESULT video_navdata_handler_init( void* data );
C_RESULT video_navdata_handler_process( const navdata_unpacked_t* const navdata );
C_RESULT video_navdata_handler_release( void );

void startRetreiving (void);
void endRetreiving (void);

#endif // _VIDEO_NAVDATA_HANDLER_H_
