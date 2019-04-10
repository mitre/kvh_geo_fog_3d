###############################################
# Satellites message for the KVH GEO FOG 3D satellites packet
###############################################

#
# ROS message header.
# header.stamp specifies the ROS time for this measurement. Satellite
# time is reported as unix_time_s and unix_time_us.
#
Header header

#
# TODO: Enter comment on what hdop is
#
float32 hdop

#
# TODO: Enter comment on what vdop is
#
float32 vdop

#
# Number of total satellites seen
#
uint8 gps_satellites

#
# Number of glonass satellites seen
#
uint8 glonass_satellites

#
# Number of beidou satellites seen
#
uint8 beidou_satellites

#
# Number of galileo satellites seen
#
uint8 galileo_satellites

#
# Number of sbas satellites seen
#
uint8 sbas_satellites