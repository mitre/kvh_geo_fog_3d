#!/bin/bash

. clean_deb.sh

echo "Building Debian..."
bloom-generate rosdebian --os-name ubuntu --os-version xenial --ros-distro kinetic
fakeroot debian/rules binary

echo "Done!"