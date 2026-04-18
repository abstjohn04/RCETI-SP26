# RCETI Network Setup

## Setting Up Static IP on Ubuntu Host

1. Open Settings > Network.
2. Click on Wired Connection (the Ethernet interface).
3. Click Settings (⚙️).
4. Go to the IPv4 tab.
5. Select Manual and set:

    * Address: 192.168.2.1
    * Netmask: 255.255.255.0
    * Gateway: Leave blank

6. Click Apply and disconnect/reconnect Ethernet.

## Setting Up ROS to use static IP

Run these commands in your Ubuntu 22.04 terminal to bind ROS 2 traffic to the Ethernet interface. This will allow communication with the Raspberry Pi.

`echo "export ROS_DOMAIN_ID=7" >> ~/.bashrc ` \
`echo "export ROS_IP=$(hostname -I | awk '{print $1}')" >> ~/.bashrc` \
`echo "export ROS_HOSTNAME=$(hostname -I | awk '{print $1}')" >> ~/.bashrc` \
`source ~/.bashrc` \