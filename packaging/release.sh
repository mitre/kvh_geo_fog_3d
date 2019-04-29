#!/bin/bash

echo "This script will release a new version of your software!"
echo "It requires that you've already run catkin_generate_changelog and catkin_prepare_release."
echo "If you are unfamiliar with python-bloom or are unsure, do not proceed."
echo "Continue? (enter 1 for Yes, 2 for No)"
select yn in "Yes" "No"; do
    case $yn in
        Yes ) break;;
        No ) exit;;
    esac
done

echo "OK, proceeding!"

FIRST_RELEASE=0
read -r -p "Is this your first release? [y/N] " response
response=${response,,}    # tolower
if [[ "$response" =~ ^(yes|y)$ ]]; then
   FIRST_RELEASE=1
fi

# Get into the base directory of the package, assuming this script is located
# one directory deep
pushd "${0%/*}" >& /dev/null
pushd ..

export ROSDISTRO_INDEX_URL=https://gitlab.mitre.org/dart-release/rosdistro/raw/master/index-v4.yaml
export PYTHONHTTPSVERIFY=0
if [ ${FIRST_RELEASE} -eq 0 ]; then
    bloom-release --rosdistro kinetic --track kinetic kvh_geo_fog_3d_driver --no-web
else
    bloom-release --rosdistro kinetic --track kinetic kvh_geo_fog_3d_driver --no-web --edit
fi

# Get back to normal
unset ROSDISTRO_INDEX_URL
unset PYTHONHTTPSVERIFY
popd
popd

echo "Complete!"
