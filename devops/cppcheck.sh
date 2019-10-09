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

# Helper to read package.xml
read_dom ()
{
    ORIGINAL_IFS=${IFS}
    IFS=\>
    read -d \< ENTITY CONTENT
    local ret=$?
    TAG_NAME=${ENTITY%% *}
    ATTRIBUTES=${ENTITY#* }
    IFS=${ORIGINAL_IFS}
    return $ret
}

PACKAGE_DIRS=()
PACKAGE_NAMES=()
# Check toplevel for package
if [ -f "package.xml" ]; then
    echo "Found package.xml in top-level directory"
    while read_dom; do
	if [ "${ENTITY}" = "name" ]; then
	    PACKAGE_NAMES+=(${CONTENT})
	    PACKAGE_DIRS+=(".")
	    break
	fi
    done < package.xml
fi
for DIR in *; do
    if [[ -d "${DIR}" && ! -L "${DIR}" ]]; then
	if [ -f ${DIR}/package.xml ]; then
	    echo "Found package.xml in ${DIR}"
	    while read_dom; do
		if [ "${ENTITY}" = "name" ]; then
		    PACKAGE_NAMES+=(${CONTENT})
		    PACKAGE_DIRS+=(${DIR})
		    break
		fi
	    done < ${DIR}/package.xml
	fi
    fi
done
echo "DIRS:"
echo ${PACKAGE_DIRS[*]}
echo "NAMES:"
echo ${PACKAGE_NAMES[*]}

if [ "${#PACKAGE_DIRS[@]}" = "0" ]; then
    echo "Failed to find any packages, exiting..."
    exit 0
fi


# Arguments:
run_cppcheck() {
    if [ ! -d ${1} ]; then
	mkdir -p ${1}
    fi
    CPPCHECK_OUT=${2}.cppcheck

    touch ${1}/${CPPCHECK_OUT}
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
        PACKAGE_SOURCE_PATHS=${PROJECT_ROOT}/${dir}/src/*.cpp
        BUILD_PATH=${WORKSPACE_ROOT}/build/${package}/

	echo "cppcheck on ${PACKAGE_SOURCE_PATHS}..."
	run_cppcheck "${CPPCHECK_DIR}" "${package}"
    else
        echo "WARNING: Package ${package} doesn't have a source directory, skipping..."
    fi
done
