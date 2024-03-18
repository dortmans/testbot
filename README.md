# Testbot

Testbot is an example software stack for a robot that:
- automatically launches at boot using [robot_upstart](https://github.com/clearpathrobotics/robot_upstart/tree/foxy-devel)
- can be inspected and controlled remotely via Wifi using [zenoh-bridge-ros2dds](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds)

## Installation

We assume that your robot computer is running Ubuntu 22.04 LTS server with ROS 2 Humble installed.

To install the Testbot robot software run following command on the robot computer:
```
wget -qO- https://raw.githubusercontent.com/dortmans/testbot/main/testbot_install.sh | bash
```

## Usage

This software stack is automatically started at boottime of the robot computer.

## Remote monitoring and control

To remotely monitor and control the Testbot software install and run the [testbot_desktop](https://github.com/dortmans/testbot_desktop) package on your laptop.