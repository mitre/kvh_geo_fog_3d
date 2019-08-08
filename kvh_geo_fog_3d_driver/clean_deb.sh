#!/bin/bash

echo "Cleaning out Debian build files..."

rm -rf debian/
rm -rf obj-x86_64-linux-gnu/
rm -f ../ros-kinetic-kvh-geo-fog-3d-driver*.deb

echo "Done!"
