# ROS Source Packages

This directory contains the source code for the ROS packages used in the Project Thiot.

## Packages

- **px4_control**: The main package for drone control, waypoint navigation, and CWS algorithm integration.

## Quick Start (Simulation)

### Terminal 1: Launch Firmware

```bash
cd PX4-Autopilot/
bash single_quadrotor_mavros
```

### Terminal 2: Connect Control

```bash
rosrun px4_control drone_test.py
```

---

For full installation and high-level architecture, please refer to the [Root README](../README.md).
