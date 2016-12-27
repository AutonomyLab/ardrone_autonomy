/*
Parker Conroy
ARLab @ University of Utah
Nov 2012

This code controls the leds of the ardrone via the ardrone_autonomy package
according to this:
*/

/*
# 0 : BLINK_GREEN_RED
# 1 : BLINK_GREEN
# 2 : BLINK_RED
# 3 : BLINK_ORANGE
# 4 : SNAKE_GREEN_RED
# 5 : FIRE
# 6 : STANDARD
# 7 : RED
# 8 : GREEN
# 9 : RED_SNAKE
# 10: BLANK
# 11: LEFT_GREEN_RIGHT_RED
# 12: LEFT_RED_RIGHT_GREEN
# 13: BLINK_STANDARD
uint8 type

# In Hz
float32 freq

# In Seconds
uint8 duration 

*/


#include <cstdlib>
#include "ros/ros.h"

int main(int argc, char **argv)
{
	
using namespace std;
  
system( "rosservice  call /ardrone/setledanimation 5 2 5" );
}
