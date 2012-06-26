#ifndef _ARDRONE_DATE_H_
#define _ARDRONE_DATE_H_

#include <ardrone_api.h>

#define ARDRONE_DATE_MAXSIZE		32
#define ARDRONE_FILE_DATE_FORMAT	"%Y%m%d_%H%M%S"

void ardrone_time2date(time_t time, const char *format, char *date);
void ardrone_date2time(char *date, const char *format, time_t *time);

#endif /* _ARDRONE_DATE_H_ */
