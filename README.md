# UDP Communication ROS Package
This package was designed as a generic package to assist with interfacing ROS with the UDP transport layer. It provides ROS Services for creating sockets, sending and receiving UDP data.

**Supported platforms/releases**:

| Platform                                                   | ROS Release                                                    |
| ---------------------------------------------------------- | -------------------------------------------------------------- |
| [Ubuntu 18.04 Bionic](http://releases.ubuntu.com/18.04.4/) | [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) |


There are two ways of utlizing this package:
- using the provided launch file and launching from the command line
  * ```roslaunch udp_com udp_com.launch```
- integrating into a custom project via source code

In either case, the udp_com package provides ROS services for interfacing with the UDP transport layer:
- **create_socket_service**: creates a UDP socket and creates an instance to receive/send data
- **send_service**: creates a ROS service to send data to a specific socket

Please [review the source code documentation](https://github.conti.de/pages/ADAS-LIDAR/udp_com/html/index.html) for more details on how the project is structured.

### Getting started
For anyone already familiar with ROS, please see how this package is [utilized and implemented in the hfl_driver package](https://github.conti.de/ADAS-LIDAR/hfl_ros).

First, make sure your using the targeted platform and releases listed above.

Go ahead and clone this repo:
```
git clone https://github.conti.de/ADAS-LIDAR/udp_com
```

