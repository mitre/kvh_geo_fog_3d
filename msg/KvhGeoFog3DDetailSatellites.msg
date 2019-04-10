##############################
# Detailed satellites message for KVG GEO FOG 3D detailed satellites packet
##############################

Header header 

#
# Integer enumeration of values
# Values:
#   0 - Unknown
#   1 - GPS
#   2 - GLONASS
#   3 - BeiDou
#   4 - GALILEO
#   5 - SBAS
#   6 - QZSS
#   7 - Starfire
#   8 - Omnistar 
#
uint8 satellite_system

#
# TODO: Lookup PRN and use as comment
#
uint8 satellite_number

#
# Each bit represents a different satellite frequency
# Bits:
#   0 - L1 C/A
#   1 - L1 C
#   2 - L1 P
#   3 - L1 M 
#   4 - L2 C 
#   5 - L2 P 
#   6 - L2 M 
#   7 - L5
#
int8 satellite_frequencies

#
# TODO: Look up elevation degree and use as comment
#
uint8 elevation_deg

#
# TODO: Lookup azimuth and use as comment
#
uint16 azimuth_deg

#
# TODO: Lookup snr and use as comment
#
uint8 snr