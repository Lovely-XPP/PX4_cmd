# PX4 command

## Introduction
PX4 command sent via terminal (based on mavlink), for whom wants to control vehicle by themselves, e.g. setting points.

## Todo
- [x] Mode Change
- [ ] Send Desire Position/Attitude/Accelerate for Offboard

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
