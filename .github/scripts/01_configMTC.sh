mkdir /home/ros2-dev/addon_repos /home/ros2-dev/addon_repos/build /home/ros2-dev/addon_repos/install /home/ros2-dev/addon_repos/log && \
vcs import /home/ros2-dev/addon_repos < /home/ros2-dev/ws/.github/dependencies.repo && \
colcon build --base-paths /home/ros2-dev/addon_repos --paths /home/ros2-dev/addon_repo/* && \
source /home/ros2-dev/addon_repos/install/setup.bash && \
colcon build --base-paths /home/ws