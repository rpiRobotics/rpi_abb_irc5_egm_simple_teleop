# rpi_abb_irc5_egm_simple_teleop
ROS package providing simple teleoperation of an ABB IRB 6640 180-255 robot using the IRC5 EGM interface. This branch has the additional capability of using MoveIt! path planning. When the gamepad RB button is held, general 6DOF teleoperation is enabled. To use a MoveIt plan, first generate a path using RViz or other method, and "Plan and Execute" the path. This resulting trajectory be sent to this teleop controller. Hold the A button and use the Left Stick up and down to move the robot through the path. Left stick Up will advance in time, Down will retard the time. Once the path has been completed, it will send a success message back to MoveIt. Currently the path execution will fail occasionally due to an excessive error between desired and real positions during path execution. This problem is being investigated.

**WARNING: EXPERIMENTAL PACKAGE. USE AT YOUR OWN (SIGNIFICANT) RISK!**

Expects an Xbox 360 wired controller. 

For geveral 6DOF teleoperation, hold RB. Mapping is as follows:

* X - Left Stick Left/Right
* Y - Left Stick Up/Down
* Z - Right Stick Up/Down
* Yaw - Right Stick Left/Right
* Pitch - D-Pad Left/Right
* Roll - D-Pad Up/Down

For MoveIt path following, hold the A button. Mapping for motion is as follows:

* Advance Time: Left Stick Up
* Retard Time: Left Stick Down

The EGM device remote addresswill need to be set to the IP address of the computer running rpi_abb_irc5_egm_simple_teleop. In Robot Studio, this can be found under Controller -> Configuration Editor -> Communication -> Transmission Protocol -> UCdevice .

The file rapid/EGM_slave.mod contains a basic rapid module that will allow the robot to be commanded from the ROS machine.

Start command: `roslaunch rpi_abb_irc5_egm_simple_teleop rpi_abb_irc5_egm_simple_teleop.launch`

License: BSD


