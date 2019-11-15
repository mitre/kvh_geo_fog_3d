#!/bin/bash

# Requires:
# cppcheck (1.38)
#

# Get project name
if [ $# -ne 1 ]; then
    echo "Usage:"
    echo " $0 <project name>"
    echo " (project name should be the directory you've cloned)"
    exit 1
fi
# Overall project name
PROJECT_NAME=$1

# Get script directory
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")

# Import functions.sh
. ${SCRIPT_DIR}/functions.sh

PACKAGE_DIRS=()
PACKAGE_NAMES=()
find_ros_packages

echo "DIRS:"
echo ${PACKAGE_DIRS[*]}
echo "NAMES:"
echo ${PACKAGE_NAMES[*]}

if [ "${#PACKAGE_DIRS[@]}" = "0" ]; then
    echo "Failed to find any packages, exiting..."
    exit 0
fi

# Get our workspace root
WORKSPACE_ROOT=""
get_workspace_root

# Arguments:
# 1: cppcheck output directory (usually the project name)
# 2: package name
# 3: space-separated list of cpp files
run_cppcheck() {
    if [ ! -d ${1} ]; then
	mkdir -p ${1}
    fi
    CPPCHECK_OUT=${1}/${2}.cppcheck

    cppcheck --enable=warning,style,performance,portability --language=c++ --platform=unix64 --std=c++11 -I kvh_geo_fog_3d_driver/include/ -I kvh_geo_fog_3d_rviz/include/ --xml --xml-version=2 ${3} 2> "${CPPCHECK_OUT}"
}

# Directory in which to store all of our suggested changes to files from clang_format
CPPCHECK_DIR=cppcheck_output

PROJECT_ROOT=${WORKSPACE_ROOT}/src/${PROJECT_NAME}
# If this is a simple project (i.e. one package within the whole project) then
# PACKAGE_NAME will be blank
for i in "${!PACKAGE_DIRS[@]}"; do
    dir=${PACKAGE_DIRS[$i]}
    package=${PACKAGE_NAMES[$i]}
    if [ -d ${dir}/src ]; then
        #PACKAGE_SOURCE_PATHS=${PROJECT_ROOT}/${dir}/src/*.cpp
	# Enable globstar and use globbing, instead of trying to for loop over
	# find results. That approach broke on whitespace
	shopt -s globstar
	for f in **/*.cpp; do
	    PACKAGE_SOURCE_PATHS+=$(realpath ${f})
	    PACKAGE_SOURCE_PATHS+=" "
	done

	echo "cppcheck on ${PACKAGE_SOURCE_PATHS}..."
	run_cppcheck "${CPPCHECK_DIR}" "${package}" "${PACKAGE_SOURCE_PATHS}"
    else
        echo "WARNING: Package ${package} doesn't have a source directory, skipping..."
    fi
done
