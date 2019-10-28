FROM artifacts.mitre.org:8200/untrusted-mitrewide-overwritable/dart/dart_gitlab_runner_melodic:latest
MAINTAINER Zach LaCelle <zlacelle@mitre.org>

# catkin and ROS really like bash over sh
SHELL ["/bin/bash", "-c"]

ARG catkin_ws=catkin_ws

RUN mkdir -p ${catkin_ws}/src
COPY --chown=build:root [".", "${catkin_ws}/src"]
RUN source /opt/ros/melodic/setup.bash && \
    pushd ${catkin_ws} && \
    catkin build -DCMAKE_BUILD_TYPE=Release && \
    popd

ADD docker/ros_entrypoint.sh /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]