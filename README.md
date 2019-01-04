### Environment Dependence

We use `Ubuntu 16.04` and ROS distribution `kinetic`

To install ROS:

`sudo apt install ros-kinetic-desktop-full`

We need ROS-Gazebo interface and controller packages:

`sudo apt install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control ros-kinetic-ros-controllers` 

### Launch the Simulation

Firstly, copy the directory `exoskeleton` to your `catkin_ws/src`

Launch the exoskeleton model:

`roslaunch exoskeleton world.launch paused:=true`

Launch the `rqt_gui` for control and tuning parameter:

`roslaunch exoskeleton rqt.launch`

upper half for publish joint command.
lower half for reconfigure joint controller PID parameters.
(Default is {p: 100, i: 0.01, d: 10} configured in `config/control.yaml`)

### ROS Topic 

Totally seven links:

`waist_link` `LU_link` `RU_link` `LD_link` `RD_link` `LF_link` `RF_link`

Totally six joints:

 `LU_joint` `RU_joint` `LD_joint` `RD_joint` `LF_joint` `RF_joint`

*L for left, R for right, U for up, D for down, F for foot*
*e.g. LU_link => left up link(left thigh)*

Joint position controller:

```
/exoskeleton/LU_position_controller/command
```

Link IMU plug-in topic:

`/imu/LINKNAME`

### Mechanical Parameter

See in `urdf/robot.xacro`
It's a very simple model.