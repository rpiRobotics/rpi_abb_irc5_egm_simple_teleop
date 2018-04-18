# rpi_abb_irc5_egm_simple_teleop
ROS package providing simple teleoperation of an ABB IRB 6640 180-255 robot using the IRC5 EGM interface.

**WARNING: EXPERIMENTAL PACKAGE. USE AT YOUR OWN (SIGNIFICANT) RISK!**

Expects an Xbox 360 wired controller. Mapping is as follows:

* X - Left Stick Left/Right
* Y - Left Stick Up/Down
* Z - Right Stick Up/Down
* Yaw - Right Stick Left/Right
* Pitch - D-Pad Left/Right
* Roll - D-Pad Up/Down

The EGM device remote addresswill need to be set to the IP address of the computer running rpi_abb_irc5_egm_simple_teleop. In Robot Studio, this can be found under Controller -> Configuration Editor -> Communication -> Transmission Protocol -> UCdevice .

The file rapid/EGM_slave.mod contains a basic rapid module that will allow the robot to be commanded from the ROS machine.

License: BSD


