Header header

# 0 means no battery, 100 means full battery
float32 batteryPercent

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

