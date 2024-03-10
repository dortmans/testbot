#!/usr/bin/env bash

set -e

BASHRC=$HOME/.bashrc
ROS_DISTRO=${ROS_DISTRO:-humble}
WORKSPACE="$HOME/ros2_ws"

function detect_and_setup_ros {
    if [[ -d /opt/ros/$ROS_DISTRO ]]; then
        source /opt/ros/$ROS_DISTRO/setup.bash
        if ! grep -q "^source /opt/ros/$ROS_DISTRO/setup.bash" $BASHRC; then
            echo -e "source /opt/ros/$ROS_DISTRO/setup.bash" >> $BASHRC
        fi
        echo "Detected and setup ROS distro '$ROS_DISTRO'."
    else
        echo "Please install ROS before running this script."
    fi
}

function install_robot_upstart {
    sudo apt install -y ros-$ROS_DISTRO-robot-upstart
}

function install_zenoh_bridge {
    echo "deb [trusted=yes] https://download.eclipse.org/zenoh/debian-repo/ /" \
        | sudo tee -a /etc/apt/sources.list > /dev/null
    sudo apt update
    sudo apt install -y zenoh-bridge-ros2dds
}

function install_dummy_robot {
    sudo apt install -y ros-humble-dummy-robot-bringup
}

function install_image_tools {
    sudo apt install -y ros-humble-image-tools
}

function install_testbot_required_packages {
    install_dummy_robot
    install_image_tools
    install_robot_upstart
    install_zenoh_bridge
}

function install_testbot_packages {
    mkdir -p $WORKSPACE/src
    cd $WORKSPACE/src
    git clone https://github.com/dortmans/testbot.git
    cd $WORKSPACE
    rosdep install --from-paths src -y --ignore-src
    colcon build --symlink-install
    source install/setup.bash
    if ! grep -q "^source $WORKSPACE/install/setup.bash" $BASHRC; then
        echo -e "source $WORKSPACE/install/setup.bash" >> $BASHRC
    fi
}

function testbot_setup {
    ros2 run testbot_setup testbot_setup.sh
}

######################


echo "Installing 'testbot' ..."

detect_and_setup_ros

install_testbot_required_packages

install_testbot_packages

testbot_setup

echo
echo "Installation of the robot software completed."
echo "Reboot your robot computer to automatically start the robot software."
echo

