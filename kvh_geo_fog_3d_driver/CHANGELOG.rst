^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package kvh_geo_fog_3d_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.0 (2019-09-27)
-----------
* Merge branch 'raw_packets' into 'master'
  Filter Options and Magnetometer
  See merge request DART/kvh_geo_fog_3d!14
* Filter Options and Magnetometer
* Defaulting to turning off velocity heading, since KVH recommended it.
* Implemented filter options packet configuration.
* Added navsatfix and imu messages for the raw data to make graphing easier. Fixed some problems with getting raw packets.
* Added raw gnss and sensor messages to those output by the driver.
* Merge branch 'odom_packet' into 'master'
  Odom packet
  See merge request DART/kvh_geo_fog_3d!13
* Fixed launch file syntax and printing error.
* Added ability to customize initialization options.
* Uncommenting build deb stuff.
* Set wheel encoder frame id to baselink. Updated baud setter to have different values for each of the ports.
* Added odom state message and message publishers to kvh driver node.
* Added odometer state packet to the kvh driver. Next will be implementing messages.
* Merge branch 'utm_test' into 'master'
  Utm test
  See merge request DART/kvh_geo_fog_3d!12
* Added utm tests.
* Need fix for utm in packet storage.
  Merge branch 'cpp_check_fix' into utm_test
* Fixing packet storage utm problem.
* Fixed utm struct problem.
* Merge branch 'cpp_check_fix' into 'master'
  Fixes for cpp check warnings.
  See merge request DART/kvh_geo_fog_3d!11
* Fixes for cpp check warnings.
* Moving and fixing the release script.
* Contributors: Bostic, Trevor R, LaCelle, Zachary

1.1.0 (2019-08-13)
-----------
* Moving msgs into their own package
* Updating package.xml and CMakeLists.txt to handle the new packaging scheme
* Fixes of tfs for orientation
* Various fixes to covariances, especially when we temporarily lose communications
* Updating the IMU data publishing to match ROS schemes
* Fixing a bug in the TF for GPS
* Many frame ID changes
* Variable baud rates
* Adding an autobaud node
* Bug fixes in the UTM and Fix packets
* More ublox custom messages

1.0.0 (2019-04-25)
-----------
* Initial release of the ROS KVH GEO FOG 3D driver package, reading basic state and information packets.
