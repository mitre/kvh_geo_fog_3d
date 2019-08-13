#!/bin/bash

if [ ! -d debian ]; then
    echo "This assumes you've already run bloom-generate, and also"
    echo "that you're running it from the package root directory, i.e."
    echo "you should see a debian/ directory."
    exit 1
fi

# This string tells our script that we've already fixed this file
FLAG_STRING="# FIXED BY MITRE BUILD SCRIPTS"

##
# The lines containing local .cmake files
##

#This finds the location of the catkin_ws directory, which we will use as the path for the devel/ space
parentname="$(dirname "$(pwd)")"
while [ "$(basename "${parentname}")" != "catkin_ws" ]; do
    parentname="$(dirname "${parentname}")"
    if [ "${parentname}" == "/" ]; then
        echo "Couldn't find catkin_ws directory! Unfortunately, this script requires this naming convention."
        exit 1
    fi
done

# The actual lines to add
CMAKE_CUSTOM_PACKAGES_LINES=\\t\\t-Dkvh_geo_fog_3d_msgs_DIR=\"${parentname}/devel/share/kvh_geo_fog_3d_msgs/cmake\"

##

# The last non-blank line of the file
line=`awk '/./{line=$0} END{print line}' debian/rules`

# Make sure we haven't already fixed this file
if [ "${line}" == "${FLAG_STRING}" ]; then
    echo "Aleady fixed this rules file!"
    exit 0
else
    # Append the \ character to the line
    sed -i -e '/CMAKE_PREFIX_PATH=.*/s/$/ \\/g' debian/rules
    # Append our msgs line
    sed -i -e "/CMAKE_PREFIX_PATH=.*/a\\${CMAKE_CUSTOM_PACKAGES_LINES}" debian/rules
    # Append our flag so we know we fixed it
    echo "" >> debian/rules
    echo "${FLAG_STRING}" >> debian/rules
fi

