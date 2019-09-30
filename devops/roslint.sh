#!/bin/bash

# CD to script directory
pushd "${0%/*}/.."  >& /dev/null

echo "Script assumes you've already build everything and"
echo " have sourced your workspace's setup.bash!"

ROSLINT_DIR=$(pwd)/roslint_output

if [ ! -d ${ROSLINT_DIR} ]; then
    mkdir -p ${ROSLINT_DIR}
else
    rm ${ROSLINT_DIR}/*
fi

# Have to use no-deps, otherwise it tries to build dependent packages with the roslint requirement, and if
# they don't have it (e.g. msgs package) they will fail.
# This means that dependent packages (e.g. msgs) must already be built, however!

# Get our sourced WS
WORKSPACE_ROOT=""
IFS=':'
read -ra CATKIN_WSES <<< "${CMAKE_PREFIX_PATH}"
IFS=' '
for ws in ${CATKIN_WSES}
do
    if [[ ${ws} != *"/opt/ros/"* ]]; then
        # Remove /devel from string to get root
        WORKSPACE_ROOT=${ws//\/devel/}
    fi
done

pushd ${WORKSPACE_ROOT}/build

pushd kvh_geo_fog_3d_driver >& /dev/null
make roslint >& ${ROSLINT_DIR}/kvh_geo_fog_3d_driver.txt
popd >& /dev/null
pushd kvh_geo_fog_3d_rviz >& /dev/null
make roslint >& ${ROSLINT_DIR}/kvh_geo_fog_3d_rviz.txt
popd >& /dev/null

popd >& /dev/null
popd >& /dev/null

echo "Finished with Ros linter!"
