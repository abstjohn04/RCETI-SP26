# RCETI

Robotic Control of Endotracheal Tube Insertion

## Overview

R.C.E.T.I is a ROS 2 autonomous intubation device. The system utilizes a multi axis rigid linear actuator frame to position the device at the patient's airway, and a 4 tendon continuum robotic tube to safely navigate the throat.

This project assumes the user is controlling the robot remotely via a Raspberry Pi and is using Ubuntu 22.04 LTS.

### Github Branches

**RCETI is currently divided into two branches.**  

The "main" branch contains the most recent stable code that has been tested on the physical robot. It contains updated and more precise servo movements. It does not contain the machine learning algorithms or the updated simulation environment.  

The "development" branch contains the latest code. It has not been tested on the physical robot. It contains the machine learning node and a more polished simulation environment and math engine.

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

**rceti_vision**: The input node for the machine learning algorithms.

## Installation

See [INSTALL.md](INSTALL.md) to install ROS 2 and setup the environment on the Ubuntu host machine.

## Network Setup

See [NETWORK.md](NETWORK.md) to access the Raspberry Pi.

## Usage

Before launching any ROS node, the user must first source the environment and build.

`colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON`

`source install/setup.bash`

Afterwards, the full network can be launched.

### Commands

`colcon build --packages-select package_name` - Builds a specific node

`ros2 launch rceti_deployment rceti_deployment.launch.xml` - Runs the whole robot system with machine learning disabled. Manual keyboard controls enabled.

`ros2 launch rceti_deployment rceti_deployment.launch.xml use_ai:=true` - Runs the whole robot system with machine learning enabled. Launches mock camera and vision bridge. Manual keyboard controls disabled. UNAVAILABLE IF USING MAIN BRANCH.

`ros2 launch package_name package_launch_file` - Launches a package's launch file (see names of launch files in codebase).

`ros2 run package_name package_run_file` - Runs a specific eligible file in a node (see names of files in codebase).

### RViz simulation tool

This tool launches with the rceti_continuum node. It fetches the modeled meshes from rceti_support and simulates them according to the urdf files (See invididual nodes' .urdf and .xacro files for more details on what is launched and created).  

To disable the Transform frames (all the green, red and blue lines) in the simulation, uncheck the "TF" button in the left settings screen.

### Moving the robot

If you are using the machine learning node with the mock camera, it will automatically move for you!  

To manually move the robot, the rceti_keyboard node must be launched. This will launch a separate terminal which outputs coordinates.  

To adjust the position of the servo mount, use A and D to move left and right, and W and S to move up and down.  
Use L and P to adjust the angle of the servo mount.

To move the tube, use the left and right arrow keys to adjust the X coordinate (yaw) and the up and down arrow keys to adjust the Y coordinate (pitch).

## License & Acknowledgements

This project is licensed under the Apache License 2.0.

Original Work: This repository is a derivative work based on the original [RCETI robotics project](https://github.com/bturner86239/RCETI/graphs/contributors). For a list of original authors, please visit the contributors page of the original repository at `https://github.com/bturner86239/RCETI/graphs/contributors`.

Modifications: In 2026, CSE 2.3 modified the source code, hardware architecture, and simulation environment to implement an independent 4-servo perpendicular continuum drive.
