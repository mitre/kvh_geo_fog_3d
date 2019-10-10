# Helper to read XML files
# Handles tags, but not attributes well
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

# Requires PACKAGE_NAMES and PACKAGE_DIRS arrays to be declared
find_ros_packages ()
{
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
}

# Requires local devel setup.bash to be sourced
get_workspace_root ()
{
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
}
