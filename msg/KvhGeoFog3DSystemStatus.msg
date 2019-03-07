############################################################################
# System Status message for the KVH GEO FOG 3D
#
# header.stamp specifies the ROS time for this measurement. Satellite
# time is reported as unix_time_s and unix_time_us.
#
# All fields with units of measure are accompanied by suffixes listed here.
# These fields mirror exactly what comes out of the KVH INS, and thus
# may or may not conform to ROS REP 103.
#
# Suffixes used in this file:
#  s         - seconds
#  us        - microseconds
#  rad       - radians
#  m         - meters
#  mps       - meters per second
#  g         - g-force
#  rad_per_s - radians per second
############################################################################

Header header

#
# System status bitmask, as reported by the KVH GEO FOG 3D.
# Each bit is a boolean representing a specific subsystem.
# Bits:
#  0  - System Failure
#  1  - Accelerometer sensor failure
#  2  - Gyroscope sensor failure
#  3  - Magnetometer sensor failure
#  4  - Pressure sensor failure
#  5  - GNSS failure
#  6  - Accelerometer over range
#  7  - Gyroscope over range
#  8  - Magnetometer over range
#  9  - Pressure over range
#  10 - Minimum temperature alarm
#  11 - Maximum temperature alarm
#  12 - Low voltage alarm
#  13 - High voltage alarm
#  14 - GNSS antenna disconnected
#  15 - Data output overflow alarm
#
uint16 system_status

uint16 filter_status

uint32 unix_time_s

uint32 unix_time_us

float64 latitude_rad

float64 longitude_rad

float64 height_m

float32 absolute_velocity_north_mps

float32 absolute_velocity_east_mps

float32 absolute_velocity_down_mps

#
# If mounted to the vehicle rigidly with the suggested orientation from KVH
# (X points forward, Y points right, Z points down), then body_acceleration_x
# is the acceleration front to back, with positive (+) values meaning vehicle
# acceleration forward.
#
float32 body_acceleration_x_mps

#
# If mounted to the vehicle rigidly with the suggested orientation from KVH
# (X points forward, Y points right, Z points down), then body_acceleration_y
# is the acceleration sideways, with positive (+) values meaning vehicle
# accelerating to the right.
#
float32 body_acceleration_y_mps

#
# If mounted to the vehicle rigidly with the suggested orientation from KVH
# (X points forward, Y points right, Z points down), then body_acceleration_z
# is the acceleration up and down, with positive (+) values meaning vehicle
# acceleration downward.
#
float32 body_acceleration_z_mps

float32 g_force_g

float32 roll_rad

float32 pitch_rad

float32 heading_rad

float32 angular_velocity_x_rad_per_s

float32 angular_velocity_y_rad_per_s

float32 angular_velocity_z_rad_per_s

float32 latitude_stddev_m

float32 longitude_stddev_m

float32 height_stddev_m

THROUGH THE MIP "Insurance Collaboration"

Building on work that IIHS has done, building a relationship. They've done a series of surveys and road tests. We've started re-designing the survey, did it here, now we're setting up a set of focus groups with their input. Also trying to integrate it into PARTS. Driver experience, usage, unexpected behaviors.
