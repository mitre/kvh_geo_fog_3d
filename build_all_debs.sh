#!/bin/bash

pushd "${0%/*}" >& /dev/null

. clean_all_debs.sh

echo "Building all Debians for sub-packages..."

for dir in */
do
    # Remove trailing slash
    dir=${dir%*/}
    if [ -f "${dir}/build_deb.sh" ]; then
        pushd ${dir}
        ./build_deb.sh
        popd >& /dev/null
    fi
done

echo "Done!"

popd >& /dev/null
