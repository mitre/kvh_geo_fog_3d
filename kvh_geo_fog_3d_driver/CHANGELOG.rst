^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package kvh_geo_fog_3d_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2019-08-13)
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
