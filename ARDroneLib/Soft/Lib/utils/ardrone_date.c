#define _XOPEN_SOURCE
#include <time.h>
#include <utils/ardrone_date.h>
#include <stdio.h>

void ardrone_time2date(time_t time, const char *format, char *date)
{
  if(date)
    {
      struct tm *tm = localtime(&time);
      strcpy(date, ARDRONE_DEFAULT_DATE);
      if(tm != NULL)
        strftime(date, ARDRONE_DATE_MAXSIZE, format ? format : ARDRONE_FILE_DATE_FORMAT, tm);
    }
}

void ardrone_date2time(char *date, const char *format, time_t *time)
{
  struct tm tm;

  if(date != NULL)
    {
      *time = 0;
      if(strptime(date, (NULL != format) ? format : ARDRONE_FILE_DATE_FORMAT, &tm) != NULL)
        *time = mktime(&tm);
    }
}
