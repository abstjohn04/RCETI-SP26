# RCETI System Setup & Installation

## Setup Ubuntu Host System

First, we need to ensure the Ubuntu host machine is up to date and has the basic tools required to download the robotics software.

1. Open a new Terminal window (Ctrl + Alt + T).
2. Enter the following command and press enter:  
`sudo apt update && sudo apt upgrade -y`  
*(Note: If prompted with "A new version of Ubuntu is available. Would you like to upgrade?", select **"Do Not Upgrade"**. The robot specifically requires Ubuntu 22.04)*
3. Restart the Ubuntu host machine

## Install ROS 2

RCETI runs on ROS 2 Humble. Run the following commands one by one in your terminal to install the core robotic libraies.

1. Add the ROS 2 Repository
    * `sudo apt install software-properties-common`
    * `sudo add-apt-repository universe`
    * `sudo apt update && sudo apt install curl -y`
    * `export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')`
2. Install the Core Software
    * `curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"`
    * `sudo dpkg -i /tmp/ros2-apt-source.deb`
    * `sudo apt update && sudo apt install ros-humble-desktop -y && sudo apt install ros-humble-moveit -y && sudo apt install python3-colcon-common-extensions -y && sudo apt install python3-rosdep2 -y`
3. Restart the Ubuntu host machine.

## Set up the ROS environment

Next we will setup the ROS environment. Enter the following commands in your terminal one by one.

1. Source the setup files:  
    `source /opt/ros/humble/setup.bash`
2. If sourcing the files works, make it run automatically  
    `echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc`
3. Close the current terminal and open a new one (Ctrl + Alt + T)
4. Create a new folder named “rceti_ws” with a subfolder named “src” in your home directory to store the code:  
    ``cd && mkdir -p rceti_ws/src``

## Download & Build the RCETI Code

Now that the robotics software is installed, we will download the RCETI specific code. Once again, follow the steps and enter the commands in your terminal.

1. Install git:  
    `sudo apt update && sudo apt install git -y`
2. Clone the repo to rceti/src (make sure to include the dot at the end of the command!):  
    `cd ~/rceti/src && git clone https://github.com/abstjohn04/RCETI-SP26 .`
3. Enter the rceti_ws/ directory and install dependencies:  
    * `cd ~/rceti_ws`
    * `rosdep update`
    * `rosdep install -i --from-path src --rosdistro humble -y`
    * `colcon build`
4. Restart the Ubuntu host machine (CRUCIAL!)

## Running the Robot System

To Start the RCETI robot, enter the following command:  
    `cd ~/rceti_ws/ && source install/setup.bash && ros2 launch rceti_deployment rceti_deployment.launch.xml`

If the installation and launch was successful, four things will open on your screen:

1. **The 3D Simulation (RViz)**: The 3D live model of the robot.
2. **The Control Terminal**: A new terminal will pop up allowing you to use your WASD and Arrow keys to control the physical robot
3. **The Data Terminal**: A window displaying the real time math calculations and the coordinates.
4. **The System Log**: The original terminal where you typed the command will begin streaming the system health status.

To close the application, navigate to the System Log terminal window and press "Ctrcl + C". Close the other windows manually if needed
