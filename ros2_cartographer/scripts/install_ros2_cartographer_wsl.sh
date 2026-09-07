#!/bin/bash
set -e

echo "=== 1. Setting up locales ==="
apt-get update
apt-get install -y locales curl gnupg lsb-release
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

echo "=== 2. Adding ROS 2 repository ==="
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list

echo "=== 3. Installing ROS 2 Humble Desktop & Cartographer ==="
apt-get update
DEBIAN_FRONTEND=noninteractive apt-get install -y \
  ros-humble-desktop \
  ros-humble-cartographer \
  ros-humble-cartographer-ros \
  ros-humble-nav2-map-server \
  ros-humble-tf2-tools \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool

echo "=== 4. Setting up Environment for user gold ==="
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  rosdep init || true
fi
su - gold -c "rosdep update || true"

if ! grep -q "/opt/ros/humble/setup.bash" /home/gold/.bashrc; then
  echo "source /opt/ros/humble/setup.bash" >> /home/gold/.bashrc
fi

echo "=== Installation Completed Successfully ==="
