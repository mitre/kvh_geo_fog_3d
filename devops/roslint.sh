#!/bin/bash

# CD to script directory
cd "${0%/*}"
cd ..

echo "Script assumes you've already build everything!"

ROSLINT_DIR=roslint_output

if [ ! -d ${ROSLINT_DIR} ]; then
    mkdir -p ${ROSLINT_DIR}
fi

# Have to use no-deps, otherwise it tries to build dependent packages with the roslint requirement, and if
# they don't have it (e.g. msgs package) they will fail.
# This means that dependent packages (e.g. msgs) must already be built, however!

catkin build kvh_geo_fog_3d_driver --no-deps --make-args roslint >& ${ROSLINT_DIR}/kvh_geo_fog_3d_driver.txt
catkin build kvh_geo_fog_3d_rviz --no-deps --make-args roslint >& ${ROSLINT_DIR}/kvh_geo_fog_3d_rviz.txt

echo "Finished with Ros linter!"
