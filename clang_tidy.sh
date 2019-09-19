#!/bin/bash

# Required:
#  clang (>=3.8)
#  clang-tidy (>=3.8)
#  parallel
#  clang-tidy-to-xml.py installed to /usr/local/bin (from https://github.com/PSPDFKit-labs/clang-tidy-to-junit)
#
# Also requires you to compile your code with the flag -DCMAKE_EXPORT_COMPILE_COMMANDS=1,
# which will write the compile_commands.json to the build directory of the catkin project.

# Checks to run using clang-tidy
CLANG_TIDY_CHECKS=clang-analyzer-core*,clang-analyzer-cplusplus,clang-analyzer-llvm*,clang-analyzer,nullability*,clang-analyzer-security*,clang-analyzer-unix*,readability-braces-around-statements,modernize-pass-by-value,modernize-use-nullptr,misc-inefficient-algorithm
# Overall project name
PROJECT_NAME=kvh_geo_fog_3d
# Bash array of packages within this project
PACKAGE_NAME=($(ls -d ${PROJECT_NAME}*/ | tr " " "\n" | sed 's:/*$::'))
# OLD WAY
#PACKAGE_NAME=(kvh_geo_fog_3d_driver kvh_geo_fog_3d_msgs kvh_geo_fog_3d_rviz kvh_geo_fog_3d)

# Arguments:
# $1 : String of source paths for the package, usually with regex. Example: "src/*.cpp"
# $2 : Build directory path, containing compile_commands.json
# $3 : Path to strip off of results. Usually the project root.
# $4 : Prefix to put on the clangtidy xml results file. Usually the package name.
run_clang_tidy() {
    TMP_CLANGTIDY=.tmp_clangtidy
    parallel -m clang-tidy -p ${2} {} --checks=${CLANG_TIDY_CHECKS} ::: ${1} > ${TMP_CLANGTIDY}
    
    # Re-format into JUnit
    cat ${TMP_CLANGTIDY} | clang-tidy-to-junit.py ${3}/ > ${4}_clangtidy.xml

    # Remove temp file
    rm ${TMP_CLANGTIDY}    
}

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

PROJECT_ROOT=${WORKSPACE_ROOT}/src/${PROJECT_NAME}
# If this is a simple project (i.e. one package within the whole project) then
# PACKAGE_NAME will be blank
if [ -z "${PACKAGE_NAME}" ]; then
    if [ -d ${package}/src ]; then
        PACKAGE_SOURCE_PATHS=${PROJECT_ROOT}/src/*.cpp
        BUILD_PATH=${WORKSPACE_ROOT}/build/${PROJECT_NAME}/
    
        run_clang_tidy "${PACKAGE_SOURCE_PATHS}" "${BUILD_PATH}" "${PROJECT_ROOT}" "${PROJECT_NAME}"
    else
        echo "WARNING: Project doesn't have a source directory, skipping..."
    fi
else
    for package in ${PACKAGE_NAME[@]}; do
        echo "Entering package ${package}..."
        if [ -d ${package}/src ]; then
            PACKAGE_SOURCE_PATHS=${package}/src/*.cpp
            BUILD_PATH=${WORKSPACE_ROOT}/build/${package}

            run_clang_tidy "${PACKAGE_SOURCE_PATHS}" "${BUILD_PATH}" "${PROJECT_ROOT}" "${package}"
        else
            echo "WARNING: Package doesn't have a source directory, skipping..."
        fi
    done
fi