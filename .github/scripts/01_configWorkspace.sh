#!/bin/sh
sudo update-ca-certificates && \
cat /home/ws/.github/to_bash.txt >> ~/.bashrc && \
sudo chown -R ros2-dev /home/ && \
source /opt/ros/humble/setup.sh && \
source /usr/share/colcon_cd/function/colcon_cd.sh  && \
export _colcon_cd_root=/opt/ros/humble/ && \
export LIBGL_ALWAYS_INDIRECT=0