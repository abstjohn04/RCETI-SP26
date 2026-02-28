# RCETI

Robotic Control of Endotracheal Tube Insertion

## Overview

R.C.E.T.I is a ROS 2 autonomous intubation device. The system utilizes a multi axis rigid linear actuator frame to position the device at the patient's airway, and a 4 tendon continuum robotic tube to safely navigate the throat.

This project assumes the user is controlling the robot remotely via a Raspberry Pi and is using Ubuntu 22.04 LTS.

### Requirements

* Ubuntu 22.04
* ROS 2
* Raspberry Pi

### File Structure

#### The workspace is divided into specific ROS 2 packages handling different levels of the robotic stack

**rceti_support**: Contains the URDF, Xarco, and 3D STIL meshes defining the rigid physical frame of the robot.

**rceti_continuum**: The core mathematical engine. It calculates the inverse kinematics (tendon lengths).

**rceti_controller**: The hardware interface running on the Raspberry Pi.

**rceti_keyboard**: A node allowing for manual control of the four independend continuum tendons via keyboard inputs

**rceti_deployment**: The launch files that tie the simulation, hardware, and math nodes together.

## Installation

See [INSTALL.md](INSTALL.md) to install ROS 2 and setup the environment on the Ubuntu host machine.

## Network Setup

### Setting Up Static IP on Ubuntu Host

1. Open Settings > Network.
2. Click on Wired Connection (the Ethernet interface).
3. Click Settings (⚙️).
4. Go to the IPv4 tab.
5. Select Manual and set:

    * Address: 192.168.2.1
    * Netmask: 255.255.255.0
    * Gateway: Leave blank

6. Click Apply and disconnect/reconnect Ethernet.

### Setting Up ROS to use static IP

Run these commands in your Ubuntu 22.04 terminal to bind ROS 2 traffic to the Ethernet interface. This will allow communication with the Raspberry Pi.

`echo "export ROS_DOMAIN_ID=7" >> ~/.bashrc ` \
`echo "export ROS_IP=$(hostname -I | awk '{print $1}')" >> ~/.bashrc` \
`echo "export ROS_HOSTNAME=$(hostname -I | awk '{print $1}')" >> ~/.bashrc` \
`source ~/.bashrc` \

## License & Acknowledgements

This project is licensed under the Apache License 2.0.

Original Work: This repository is a derivative work based on the original [RCETI robotics project](https://github.com/bturner86239/RCETI/graphs/contributors). For a list of original authors, please visit the contributors page of the original repository at `https://github.com/bturner86239/RCETI/graphs/contributors`.

Modifications: In 2026, CSE 2.3 modified the source code, hardware architecture, and simulation environment to implement an independent 4-servo perpendicular continuum drive.
