#!/usr/bin/env bash

set -e

PACKAGE=testbot_setup
PACKAGE_PREFIX_PATH=`ros2 pkg prefix $PACKAGE`

function install_configuration_files {
    ETC=$PACKAGE_PREFIX_PATH/share/$PACKAGE/etc
    sudo cp -a $ETC/. /etc/
}

function setup_robot_upstart {
    # Create robot_upstart job
    ros2 run robot_upstart install \
    testbot_bringup/launch/testbot.launch.py \
    --job testbot \
    --symlink
}

######################

install_configuration_files

setup_robot_upstart


