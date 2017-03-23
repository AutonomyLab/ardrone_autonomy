Header header

# Navdata including the ARDrone 2 specifica sensors
# (magnetometer, barometer)

# 0 means no battery, 100 means full battery
float32 batteryPercent

# 0: Unknown, 1: Init, 2: Landed, 3: Flying, 4: Hovering, 5: Test
# 6: Taking off, 7: Goto Fix Point, 8: Landing, 9: Looping
# Note: 3,7 seems to discriminate type of flying (isFly = 3 | 7)
uint32 state

int32 magX
int32 magY
int32 magZ

# pressure sensor
int32 pressure

# apparently, there was a temperature sensor added as well.
int32 temp

# wind sensing...
float32 wind_speed
float32 wind_angle
float32 wind_comp_angle

# left/right tilt in degrees (rotation about the X axis)
float32 rotX

# forward/backward tilt in degrees (rotation about the Y axis)
float32 rotY

# orientation in degrees (rotation about the Z axis)
float32 rotZ

# estimated altitude (cm)
int32 altd

# linear velocity (mm/sec)
float32 vx

# linear velocity (mm/sec)
float32 vy

# linear velocity (mm/sec)
float32 vz

#linear accelerations (unit: g)
float32 ax
float32 ay
float32 az

#motor commands (unit 0 to 255)
uint8 motor1
uint8 motor2
uint8 motor3
uint8 motor4

#Tags in Vision Detectoion
uint32 tags_count
uint32[] tags_type
uint32[] tags_xc
uint32[] tags_yc
uint32[] tags_width
uint32[] tags_height
float32[] tags_orientation
float32[] tags_distance

#time stamp
float32 tm
