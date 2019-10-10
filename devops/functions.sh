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
