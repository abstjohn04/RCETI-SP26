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

See [NETWORK.md](NETWORK.md) to access the Raspberry Pi.

## License & Acknowledgements

This project is licensed under the Apache License 2.0.

Original Work: This repository is a derivative work based on the original [RCETI robotics project](https://github.com/bturner86239/RCETI/graphs/contributors). For a list of original authors, please visit the contributors page of the original repository at `https://github.com/bturner86239/RCETI/graphs/contributors`.

Modifications: In 2026, CSE 2.3 modified the source code, hardware architecture, and simulation environment to implement an independent 4-servo perpendicular continuum drive.
