#!/bin/bash

echo "Cleaning out Debian build files..."

pushd "${0%/*}" >& /dev/null

rm -rf debian/
rm -rf obj-x86_64-linux-gnu/
rm -f ../ros-kinetic-kvh-geo-fog-3d-msgs*.deb

echo "Done!"

popd >& /dev/null
