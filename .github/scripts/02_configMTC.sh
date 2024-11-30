#!/bin/sh
mkdir /home/ws/addon_repos && \
vcs import /home/ws/addon_repos < /home/ws/.github/dependencies.repo && \
rosdep update && \
rosdep install --from-paths /home/ws --ignore-src -y
# colcon build --paths /home/ws/addon_repos/*
# colcon build --paths /home/ws/addon_repos/mtc/* /home/ws/addon_repos/* --cmake-args -DCMAKE_BUILD_TYPE=Release
# source /home/ws/install/setup.bash
# colcon build --base-paths /home/ws