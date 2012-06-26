#ifndef _ARDRONE_ACADEMY_NAVDATA_H_
#define _ARDRONE_ACADEMY_NAVDATA_H_

#include <ardrone_tool/Navdata/ardrone_navdata_client.h>
#include <VLIB/video_codec.h>

C_RESULT ardrone_academy_navdata_init( void* data );
C_RESULT ardrone_academy_navdata_process( const navdata_unpacked_t* const navdata );
C_RESULT ardrone_academy_navdata_release( void );

FLYING_STATE ardrone_academy_navdata_get_flying_state(const navdata_unpacked_t* data);
bool_t ardrone_academy_navdata_takeoff(void);
bool_t ardrone_academy_navdata_record(void);
bool_t ardrone_academy_navdata_screenshot(void);
bool_t ardrone_academy_navdata_emergency(void);
bool_t ardrone_academy_navdata_get_configuration(void);
bool_t ardrone_academy_navdata_get_takeoff_state(void);
bool_t ardrone_academy_navdata_get_record_ready(void);
bool_t ardrone_academy_navdata_get_camera_state(void);
bool_t ardrone_academy_navdata_get_usb_state(void);
bool_t ardrone_academy_navdata_get_emergency_state(void);
uint32_t ardrone_academy_navdata_get_remaining_usb_time(void);
bool_t ardrone_academy_navdata_check_usb_record_status(void);
bool_t ardrone_academy_navdata_check_takeoff_cancelled(void);
void ardrone_academy_navdata_set_wifi_record_codec(codec_type_t newCodec);


#endif //! _ARDRONE_ACADEMY_NAVDATA_H_
