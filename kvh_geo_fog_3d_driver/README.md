# kvh-geo-fog-3d-driver

Driver for the KVH GEO FOG 3D inertial navigation systems. Connects over serial
to the KVH GEO FOG 3D device and publishes out both KVH-specific and generic
ROS-ified messages, as well as performs basic transforms to convert data to
the ROS-conformant conventions.

Initial testing at The MITRE Corporation was done on the "Dual" model, which
provides dual-antenna heading solutions. However, the API should match between
the dual and non-dual models.

For detailed information on the KVH GEO FOG 3D functionality, consult the 
technical reference manual.

Product pages:

[KVH GEO FOG 3D](https://www.kvh.com/admin/products/gyros-imus-inss/ins/geo-fog-3d/commercial-geo-fog-3d)

[KVH GEO FOG 3D Dual](https://www.kvh.com/admin/products/gyros-imus-inss/ins/geo-fog-3d-dual/commercial-geo-fog-3d-dual)

# ROS API

## kvh_geo_fog_3d_driver_node

The main driver node.

### Published Topics

#### KVH-specific messages

All of these messages are defined as part of the kvh_geo_fog_3d_msgs package.

- `~<node_name>/kvh_system_state` (kvh_geo_fog_3d_msgs/KvhGeoFog3DSystemState)
- `<node_name>/kvh_satellites` (kvh_geo_fog_3d_msgs/KvhGeoFog3DSatellites)
- `<node_name>/kvh_detailed_satellites` (kvh_geo_fog_3d_msgs/KvhGeoFog3DDetailSatellites)
- `<node_name>/kvh_local_magnetics` (kvh_geo_fog_3d_msgs/KvhGeoFog3DLocalMagneticField)
- `<node_name>/kvh_utm_position` (kvh_geo_fog_3d_msgs/KvhGeoFog3DUTMPosition)
- `<node_name>/kvh_ecef_pos` (kvh_geo_fog_3d_msgs/KvhGeoFog3DECEFPos)
- `<node_name>/kvh_north_seeking_status` (kvh_geo_fog_3d_msgs/KvhGeoFog3DNorthSeekingInitStatus)
- `<node_name>/kvh_odometer_state` (kvh_geo_fog_3d_msgs/KvhGeoFog3DSatellites)
- `<node_name>/kvh_raw_sensors` (kvh_geo_fog_3d_msgs/KvhGeoFog3DRawSensors)
- `<node_name>/kvh_raw_gnss` (kvh_geo_fog_3d_msgs/KvhGeoFog3DRawGNSS)

#### Conventional ROS messages

Not all of these messages conform to ROS REP-105 or REP-145. See the
"ROS Conformance and Conventions" section below for more details.

Generally speaking, anything with a "_flu" or a "_enu" suffix conforms to ROS
REPs.

All angles are measured in radians unless suffixed with "_deg".

NavSatFix latitude/longitude measured in degrees (see message definition).

- `~<node_name>/imu/data_raw_frd` (sensor_msgs/Imu)
- `~<node_name>/imu/data_raw_flu` (sensor_msgs/Imu) *REP-105, REP-145 compliant*
- `~<node_name>/imu/data_ned` (sensor_msgs/Imu)
- `~<node_name>/imu/data_enu` (sensor_msgs/Imu) *REP-105, REP-145 compliant*
- `~<node_name>/imu/rpy_ned` (geometry_msgs/Vector3Stamped)
- `~<node_name>/imu/rpy_ned_deg` (geometry_msgs/Vector3Stamped)
- `~<node_name>/imu/rpy_enu` (geometry_msgs/Vector3Stamped)
- `~<node_name>/imu/rpy_enu_deg` (geometry_msgs/Vector3Stamped)
- `~<node_name>/gps/fix` (sensor_msgs/NavSatFix) *Filtered GNSS location*
- `~<node_name>/gps/raw_fix` (sensor_msgs/NavSatFix) *Raw GNSS location*
- `~<node_name>/gps/mag` (sensor_msgs/MagneticField)
- `~<node_name>/gps/utm_ned` (nav_msgs/Odometry)
- `~<node_name>/gps/utm_enu` (nav_msgs/Odometry) *REP-105 compliant*
- `~<node_name>/odom/wheel_encoder` (nav_msgs/Odometry)
- `~<node_name>imu/raw_sensor_frd` (nav_msgs/Odometry)
- `~<node_name>/odom/raw_sensor_flu` (nav_msgs/Odometry) *REP-105 compliant*

### Parameters

- `~<node_name>/port` (string, default: /dev/ttyUSB0)
- `~<node_name>/baud` (int, default: 115200)
- `~<node_name>/debug` (bool, default: false)
- `~<node_name>/filterVehicleType` (string, default: 2 (car, see spatial_packets.h))
- `~<node_name>/atmosphericAltitudeEnabled` (bool, default: true)
- `~<node_name>/velocityHeadingEnabled` (bool, default: false)
- `~<node_name>/reversingDetectionEnabled` (bool, default: true)
- `~<node_name>/motionAnalysisEnabled` (bool, default: true)

# Setup

## Installation

TODO

## Serial Port Configuration

By default, the serial port is owned by root.dialout. To run this driver
without root privileges, you must add your user to the dialout group.

Beware that some COM ports are limited to 115,200, which is not enough
bandwidth to handle large amounts of KVH data. Some have higher speeds enabled
by multiplying baud rates in userspace by some value, e.g. x8. The FTDI debug
cable that comes with the KVH does not have this limitation.

## Quick Start Using the Driver

Below is some sample code that shows how you can leverage this Kvh Driver to get data.

```cpp
    typedef std::pair<packet_id_e, int> freqPair;

    kvh::KvhPacketRequest packetRequest{
        freqPair(packet_id_utm_position, 50),
        freqPair(packet_id_system_state, 50),
        ... // Any additional packets
    };

    // Set connected port
    std::string kvhPort("/dev/ttyUSB0");

    kvhDriver.Init(kvhPort, packetRequest);

    system_state_packet_t sysPacket;

    while(1)
    {
        kvhDriver.Once(); // Check for new published packets

        if (kvhDriver.PacketIsUpdated(packet_id_system_state))
        {
            kvhDriver.GetPacket(packet_id_system_state, sysPacket);

            ... // Use System State Packet
        }
        usleep(10000); // Usually will want to sleep for some amount of time
    }

```

# ROS conformance and Conventions

This driver attempts to conform to ROS Enhancement Proposals (REPs) when appropriate.

## REP 105 - Coordinate Frames for Mobile Platforms

Link: https://www.ros.org/reps/rep-0105.html

Summary:
* Data will be published by default in sensor conventions (FRD/NED) with the suffixes _frd and _ned
* Data will also be published with FLU/ENU topics, with the suffixes _flu and _enu

This sensor, like many navigation sensors, uses the somewhat standard Front-Right-Down
and North-East-Down (FRD/NED) coordinate frame conventions. ROS opts to use the standards
Front-Left-Up and East-North-Up (FLU/ENU) frames.

All data will be published in the sensor-standard formats (FRD/NED) and suffixed with "_frd"
and "_ned". The driver will also publish FLU/ENU topics to conform with ROS REP 105.
These topics will have the suffix "_flu" and "_enu".

## REP 145 - Conventions for IMU Sensor Drivers

Link: http://www.ros.org/reps/rep-0145.html

Summary:
* The driver will not perform additional off-sensor processing, other than coordinate frame transforms.
* The raw data will be published in two frames: imu_link_frd and imu_link_flu.
* imu/data_raw_frd and imu/data_raw_flu will be data without orientation data, i.e. pure measurement data
* imu/data_ned and imu/data_enu will be data with orientation data.
* Orientation data is north-oriented.
* In the orientation messages, the twist component (i.e. accelerations and velocities) are given with respect
to the child_frame_id and the pose component (i.e. position and orientation) are given with respect to the
frame_id.

This sensor fuses orientation data into its inertial solution, using GPS-baseline heading calculations,
gyro-compassing, and course-over-ground calculations, among other methods. To conform with ROS REP 145,
we will publish the raw inertial data, stripped of orientation, in both FRD and FLU frames.

We will deviate from REP 145 by explicitly terminating topics with _frd and _flu. There will be no
standard imu/data_raw and imu/data labelled topics. The end user should select the topic with the
frame convention they'd like to consume.

For frame_id strings, we will deviate from REP 103 and use the following:
* imu_link_frd and imu_link_flu - These frames will be used for raw data.
* utm_ned and utm_enu - These frames are necessary because the orientation data is north-oriented.

## Bearing Convention

This sensor produces bearing measurements with respect to true north. The other two forms of north are
magnetic north and grid north. Grid north differs very slightly from true north, and is dependent upon
where in the UTM grid your sensor is located. Magnetic north differs significantly from true north, and
is dependent upon the fluctuation of the magnetic field.

## Orientation Convention

The orientations presented by the sensor are either in ENU or NED conventions. Practically, this means
a sign change in pitch and yaw between the two conventions. Yaw is aligned with true north.

Because of a limitation in how ROS sensor_msgs/Imu reports data, it can be unclear when viewing the
message specifications the frame respective of which orientation is reported. This ambiguity is caused
by not having a separate frame_id for orientation (which is measured w.r.t. a relatively fixed point) versus
rates and accelerations (which are measured w.r.t. the body frame), and is not present in nav_msgs/Odometry
which separates these two frames. Orientation data within sensor_msgs/Imu conforms to the same conventions
as nav_msgs/Odometry, and is globally fixed in the NED and ENU frames.

# Contributing

Below are instructions on how to complete several common tasks that may be required depending on your uses.

## Adding a new packet type
1. Add packet to each set/map currently in *src/kvh_driver/kvh_global_vars.cpp*. 

For the most part, you should be able to follow the predefined pattern, but you will be adding the packet_id to the supportedPackets_, the packet size to packetSize_, and the string literal of the packet type to packetTypeStr_. Note, this is essentially registering your desired packet. **Developer Note: The packetSize_ and packetTypeStr_ could be inferred from the packet_id, except for the fact that some packets exist that were not properly implemented in kvh's api, and which we have had to extend. We have chosen to modify here, instead of modifying their api for our purposes.**

2. Add packet-specific decoding function to **DecodePacket** function in *src/kvh_driver/decode_packets.cpp* 

You should be able to just follow the pattern shown by the previously implemented packets, but will be included here for comprehensiveness.

```cpp
    case packet_id:
        packet_type_t packet;
        if (decode_packet_type(&packet, _anPacket) == 0)
        {
            packetStorage_.UpdatePacket(packet_type_id, packet);
            packetStorage_.SetPacketUpdated(packet_type_id, true);
        }
```

3. Add packet to initialisation mapping in the **Init** function of *src/kvh_driver/packet_storage.cpp*

The packet storage is initialised with the requested packets at the beginning. You must add a line similar to the following to make sure the initialization succeeds.

```cpp
    case packet_type_id:
      packetMap_[packet_type_id] = std::make_pair(false, std::make_shared<packet_type_struct_t>());
      break;
```

4. Add to the `KvhPacketRequest` that you are sending upon creation of the driver.

# Packaging

For information on packaging and releasing this package at MITRE, see PACKAGING.md

# Limitations
Here is just a list of some currently know limitations of the current architecture

1. Inability to change packets without restarting entire driver. This could potentially be added by creating a function in the **kvh::Driver** class for sending the packet request packet, and then re-initialising/recreating the packet storage object with the new requested packets.

2. Added complexity due to make changes to struct outside of API classes. Many of the steps shown in the *Adding a new packet type* section could be done automatically if we did not need to account for the possibility of extended structs. Currently the only example we have is the utm_fix struct type which extends the utm_position packet the kvh api has. We did this since we did not want to change their API in any way, but there may be a time the added complexity is not worth it. 


