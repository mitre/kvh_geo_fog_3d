FROM artifacts.mitre.org:8200/untrusted-mitrewide-overwritable/dart/dart_gitlab_runner_kinetic:latest
MAINTAINER Zach LaCelle <zlacelle@mitre.org>

# catkin and ROS really like bash over sh
SHELL ["/bin/bash", "-c"]

ARG catkin_ws=kvh_geo_fog_3d_ws

RUN mkdir -p ${catkin_ws}/src
COPY [".", "${catkin_ws}/src"]
RUN source /opt/ros/kinetic/setup.bash && \
    pushd ${catkin_ws} && \
    catkin build -DCMAKE_BUILD_TYPE=Release --verbose && \
    popd

ADD docker/ros_entrypoint.sh /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]