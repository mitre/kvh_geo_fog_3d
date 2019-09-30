#!/bin/bash

# Required:
#  clang (>=3.8)
#  clang-tidy (>=3.8)
#  parallel
#  clang-tidy-to-xml.py installed to /usr/local/bin (from https://github.com/PSPDFKit-labs/clang-tidy-to-junit)
#
# Also requires you to compile your code with the flag -DCMAKE_EXPORT_COMPILE_COMMANDS=1,
# which will write the compile_commands.json to the build directory of the catkin project.

# If we don't have a local .clang_format, get it from the repo
GOT_CLANG_FORMAT=0
if [ ! -f .clang_format ]; then
    echo "Pulling .clang_format for ROS from the MITRE repos..."
    wget --no-check-certificate https://gitlab.mitre.org/DART-release/roscpp_code_format/raw/mitre/.clang-format -O .clang_format
    GOT_CLANG_FORMAT=1
fi

# Checks to run using clang-tidy
CLANG_TIDY_CHECKS=clang-analyzer-core*,clang-analyzer-cplusplus,clang-analyzer-llvm*,clang-analyzer,nullability*,clang-analyzer-security*,clang-analyzer-unix*,readability-braces-around-statements,modernize-pass-by-value,modernize-use-nullptr,misc-inefficient-algorithm
# Overall project name
PROJECT_NAME=kvh_geo_fog_3d
# Bash array of packages within this project
PACKAGE_NAME=($(ls -d ${PROJECT_NAME}*/ | tr " " "\n" | sed 's:/*$::'))
# OLD WAY
#PACKAGE_NAME=(kvh_geo_fog_3d_driver kvh_geo_fog_3d_msgs kvh_geo_fog_3d_rviz kvh_geo_fog_3d)
# Directory in which to store all of our suggested changes to files from clang_format
CLANG_FORMAT_DIR=clang_format_output
CLANGTIDY_DIR=clangtidy

# Arguments:
# $1 : String of source paths for the package, usually with regex. Example: "src/*.cpp"
# $2 : Build directory path, containing compile_commands.json
# $3 : Path to strip off of results. Usually the project root.
# $4 : Prefix to put on the clangtidy xml results file. Usually the package name.
# $5 : Output directory (relative path)
run_clang_tidy() {
    if [ ! -d ${5} ]; then
	mkdir -p ${5}
    fi
    TMP_CLANGTIDY=.tmp_clangtidy
    parallel -m clang-tidy -p ${2} {} --checks=${CLANG_TIDY_CHECKS} ::: ${1} > ${TMP_CLANGTIDY}
    
    # Re-format into JUnit and put into collection directory
    cat ${TMP_CLANGTIDY} | clang-tidy-to-junit.py ${3}/ > ${5}/${4}_clangtidy.xml

    # Remove temp file
    rm ${TMP_CLANGTIDY}    
}

# Arguments:
# $1 : String of source paths for the package, usually with regex. Example: "src/*.cpp"
# $2 : Output directory, usually some prefix (e.g. clang_format_output)/package_name.
run_clang_format() {
    if [ ! -d ${2} ]; then
        mkdir -p ${2}
    fi
    tmp_cpp=.tmpcpp
    for file in ${1}; do
        echo ${file}
        clang-format ${file} > ${tmp_cpp}
        diff -u ${file} ${tmp_cpp} > ${2}/$(basename ${file}).diff
        rm ${tmp_cpp}
    done
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

        run_clang_tidy "${PACKAGE_SOURCE_PATHS}" "${BUILD_PATH}" "${PROJECT_ROOT}" "${PROJECT_NAME}" "${CLANGTIDY_DIR}"
    else
        echo "WARNING: Project doesn't have a source directory, skipping..."
    fi
else
    for package in ${PACKAGE_NAME[@]}; do
        echo "Entering package ${package}..."
        if [ -d ${package}/src ]; then
            PACKAGE_SOURCE_PATHS=${package}/src/*.cpp
            BUILD_PATH=${WORKSPACE_ROOT}/build/${package}

            run_clang_tidy "${PACKAGE_SOURCE_PATHS}" "${BUILD_PATH}" "${PROJECT_ROOT}" "${package}" "${CLANGTIDY_DIR}"
            run_clang_format "${PACKAGE_SOURCE_PATHS}" "${CLANG_FORMAT_DIR}/${package}"
        else
            echo "WARNING: Package doesn't have a source directory, skipping..."
        fi
    done
fi

if [ "${GOT_CLANG_FORMAT}" -eq "1" ]; then
    echo "Removing fetched .clang_format..."
    rm .clang_format
fi

echo "Packaging up clang_format results..."
tar -czf clang_format_${PROJECT_NAME}.tar.gz ${CLANG_FORMAT_DIR}
rm -rf ${CLANG_FORMAT_DIR}

echo "Done! See the ${CLANGTIDY_DIR}/*_clangtidy.xml files and the outputs tar'd under ${CLANG_FORMAT_DIR}.tar.gz"
