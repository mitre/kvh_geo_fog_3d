^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package kvh_geo_fog_3d_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.0 (2019-09-27)
-----------
* Filter Options and Magnetometer
* Defaulting to turning off velocity heading, since KVH recommended it for the Dual.
* Implemented filter options packet configuration.
* Added navsatfix and imu messages for the raw data to make graphing easier.
* Fixed some problems with getting raw packets.
* Added raw gnss and sensor messages to those output by the driver.
* Fixed launch file syntax and printing error.
* Added ability to customize initialization options.
* Set wheel encoder frame_id to base_link.
* Updated baud setter to have different values for each of the ports.
* Added odom state message and message publishers to kvh driver node.
* Added odometer state packet to the kvh driver. Next will be implementing messages.
* Added utm tests.
* Fixing packet storage utm problem.
* Fixed utm struct problem.
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
