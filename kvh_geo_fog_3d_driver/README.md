# kvh-geo-fog-3d-driver

Driver for the KVH GEO FOG 3D inertial navigation system.

# Installation

TODO

# Serial Port Configuration

By default, the serial port is owned by root.dialout. To run this driver
without root privileges, you must add your user to the dialout group.

Beware that some COM ports are limited to 115,200, which is not enough
bandwidth to handle large amounts of KVH data. Some have higher speeds enabled
by multiplying baud rates in userspace by some value, e.g. x8. The FTDI debug
cable that comes with the KVH does not have this limitation.

# Packaging

For information on packaging and releasing this package at MITRE, see PACKAGING.md

# ROS conformance

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

# Bearing Convention

This sensor produces bearing measurements with respect to true north. The other two forms of north are
magnetic north and grid north. Grid north differs very slightly from true north, and is dependent upon
where in the UTM grid your sensor is located. Magnetic north differs significantly from true north, and
is dependent upon the fluctuation of the magnetic field.

# Orientation Convention

The orientations presented by the sensor are either in ENU or NED conventions. Practically, this means
a sign change in pitch and yaw between the two conventions. Yaw is aligned with true north.

Because of a limitation in how ROS sensor_msgs/Imu reports data, it can be unclear when viewing the
message specifications the frame respective of which orientation is reported. This ambiguity is caused
by not having a separate frame_id for orientation (which is measured w.r.t. a relatively fixed point) versus
rates and accelerations (which are measured w.r.t. the body frame), and is not present in nav_msgs/Odometry
which separates these two frames. Orientation data within sensor_msgs/Imu conforms to the same conventions
as nav_msgs/Odometry, and is globally fixed in the NED and ENU frames.