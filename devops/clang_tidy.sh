#!/bin/bash

# Required:
#  clang (>=3.8)
#  clang-tidy (>=3.8)
#  parallel
#  clang-tidy-to-xml.py installed to /usr/local/bin (from https://github.com/PSPDFKit-labs/clang-tidy-to-junit)
#
# Also requires you to compile your code with the flag -DCMAKE_EXPORT_COMPILE_COMMANDS=1,
# which will write the compile_commands.json to the build directory of the catkin project.

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

# If we don't have a local .clang_format, get it from the repo
GOT_CLANG_FORMAT=0
if [ ! -f .clang_format ]; then
    echo "Pulling .clang_format for ROS from the MITRE repos..."
    wget --no-check-certificate https://gitlab.mitre.org/DART-release/roscpp_code_format/raw/mitre/.clang-format -O .clang_format
    GOT_CLANG_FORMAT=1
fi

# Checks to run using clang-tidy
CLANG_TIDY_CHECKS=clang-analyzer-core*,clang-analyzer-cplusplus,clang-analyzer-llvm*,clang-analyzer,nullability*,clang-analyzer-security*,clang-analyzer-unix*,readability-braces-around-statements,modernize-pass-by-value,modernize-use-nullptr,misc-inefficient-algorithm
# Bash array of packages within this project
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
    CLANGTIDY_OUT=${4}.clangtidy
    parallel -m clang-tidy -p ${2} {} --checks=${CLANG_TIDY_CHECKS} ::: ${1} > ${5}/${CLANGTIDY_OUT}
    
    # Re-format into JUnit and put into collection directory
    cat ${5}/${CLANGTIDY_OUT} | clang-tidy-to-junit.py ${3}/ > ${5}/${4}_clangtidy_junit.xml
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
        echo "clang-format on ${file}..."
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
for i in "${!PACKAGE_DIRS[@]}"; do
    dir=${PACKAGE_DIRS[$i]}
    package=${PACKAGE_NAMES[$i]}
    if [ -d ${dir}/src ]; then
        PACKAGE_SOURCE_PATHS=${PROJECT_ROOT}/${dir}/src/*.cpp
        BUILD_PATH=${WORKSPACE_ROOT}/build/${package}/

	echo "clang-tidy and clang-format on ${PACKAGE_SOURCE_PATHS}..."
	run_clang_tidy "${PACKAGE_SOURCE_PATHS}" "${BUILD_PATH}" "${PROJECT_ROOT}" "${package}" "${CLANGTIDY_DIR}" >& /dev/null
	run_clang_format "${PACKAGE_SOURCE_PATHS}" "${CLANG_FORMAT_DIR}/${package}" >& /dev/null
    else
        echo "WARNING: Project doesn't have a source directory, skipping..."
    fi
done

if [ "${GOT_CLANG_FORMAT}" -eq "1" ]; then
    echo "Removing fetched .clang_format..."
    rm .clang_format
fi

echo "Packaging up clang_format results..."
tar -czf clang_format_${PROJECT_NAME}.tar.gz ${CLANG_FORMAT_DIR}
rm -rf ${CLANG_FORMAT_DIR}

echo "Done! See the ${CLANGTIDY_DIR}/*_clangtidy.xml files and the outputs tar'd under ${CLANG_FORMAT_DIR}.tar.gz"
