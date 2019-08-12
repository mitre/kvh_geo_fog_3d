#!/bin/bash

pushd "${0%/*}" >& /dev/null

echo "Cleaning out Debian build files from all sub-packages..."

for dir in */
do
    # Remove trailing slash
    dir=${dir%*/}
    if [ -f "${dir}/clean_deb.sh" ]; then
        echo "Cleaning ${dir}..."
        pushd ${dir} >& /dev/null
        ./clean_deb.sh
        popd >& /dev/null
    fi
done

echo "Removing remaining debian files..."
rm -f ros-*-kvh-geo-fog-3d*.deb

echo "Done!"

popd >& /dev/null
