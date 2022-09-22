# PX4 command

## Introduction
PX4 command sent via terminal (based on mavlink), for whom wants to control vehicle by themselves, e.g. setting points.

## Todo
- [x] Mode Change
- [x] Set Command 
- [x] Send Command to PX4

## Required Packages
```
PX4-Autopilot
Mavros
```

## Installation
```bash
# Create Catkin Workspace in home
mkdir -p ~/px4_ws/src 
cd catkin_ws/src
catkin_init_workspace
# Clone this Repo
git clone https://github.com/Lovely-XPP/PX4_cmd.git
# make
cd ..
catkin_make
# Add Source for the project
"source ~/px4_ws/devel/setup.bash" >> ~/.bashrc
# update terminal
source ~/.bashrc
```

## Run Simulation
```bash
bash ~/px4_ws/src/px4_cmd/src/sh/sitl_gazebo_iris.sh
```

## About Offboard Mode
If you have problem for changing mode to Offboard Mode, please check the offical instruction:

> Note for Offboard Mode
> - This mode requires position or pose/attitude information - e.g. GPS, optical flow, visual-inertial odometry, mocap, etc.
> - RC control is disabled except to change modes.
> - The vehicle must be armed before this mode can be engaged.
> - The vehicle must be already be receiving a stream of target setpoints (>2Hz) before this mode can be engaged.
> - The vehicle will exit the mode if target setpoints are not received at a rate of > 2Hz.
> - Not all coordinate frames and field values allowed by MAVLink are supported.

More details in [https://docs.px4.io/main/en/flight_modes/offboard.html](https://docs.px4.io/main/en/flight_modes/offboard.html).

## Credits
- PX4_command
- [PX4_Guide](https://docs.px4.io/main)