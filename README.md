# 🛸 Nanodrones-IIT-B: Project TH-IoT & IoE Office 🚁

[![ROS](https://img.shields.io/badge/ROS-Noetic/Melodic-blue.svg)](http://wiki.ros.org/ROS)
[![PX4](https://img.shields.io/badge/PX4-Autopilot-green.svg)](https://px4.io/)
[![Collaboration](https://img.shields.io/badge/Collaboration-Gyanesh%20&%20Eshaan-orange.svg)](#collaborators)

This repository contains the software stack for autonomous drone control and path optimization, developed as a collaborative research project between **Gyanesh Samanta** and **Eshaan Bhardwaj**. The project focuses on intelligent waypoint navigation and optimized multi-target visiting using advanced heuristics.

---

## 🌟 Key Features

- **Autonomous Waypoint Navigation**: Seamless integration with PX4 Firmware and MAVROS.
- **Path Optimization (CWS Algorithm)**: Implementation of the **Clarke & Wright Savings** heuristic for solving Vehicle Routing Problems (VRP) in 3D space.
- **Simulation Suite**: Robust support for **GAZEBO** and **jMAVsim** situational environments.
- **Real-World Localization**: Integration with **Vicon Motion Capture** systems for high-precision indoor localization.
- **Intelligent Landing**: Advanced protocols for safe and precise landing at target coordinates.

---

## 🛠 Project Structure

- `src/px4_control`: Core ROS package for drone control logic.
- `src/px4_control/scripts/cws`: Implementation of the Clarke & Wright Savings algorithm.
- `src/px4_control/launch`: Launch files for simulation and real-world deployment.
- `build/` & `devel/`: Catkin workspace artifacts.

---

## 🚀 Getting Started

### 1. Prerequisites

Ensure you have a ROS environment (Melodic/Noetic) set up on Ubuntu.

```bash
# Install MAVROS and dependencies
sudo apt-get install ros-$ROS_DISTRO-mavros ros-$ROS_DISTRO-mavros-extras
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash install_geographiclib_datasets.sh
```

### 2. PX4 Autopilot Setup

```bash
git clone https://github.com/PX4/Firmware.git --recursive
cd Firmware
make px4_sitl_default
```

### 3. Build the Workspace

```bash
mkdir -p ~/catkin_ws/src
# Clone this repository into src/
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

---

## 💻 Usage

### 📊 Simulation (jMAVsim/Gazebo)

1. **Launch Simulator**:
   ```bash
   cd ~/Firmware
   make px4_sitl_default jmavsim
   ```
2. **Connect MAVROS**:
   ```bash
   roslaunch px4_control px4_sim.launch
   ```
3. **Execute Control Script**:
   ```bash
   rosrun px4_control drone_test.py
   ```

### 📍 Real Drone (Vicon)

1. **Launch Localization**:
   ```bash
   roslaunch px4_control drone.launch gcs_url:=udp://@GROUND_STATION_IP
   ```
2. **Run Test**:
   ```bash
   rosrun px4_control drone_test.py
   ```

---

## 🤝 Collaborators

This project is a joint effort between:

- **Gyanesh Samanta** ([@Gyanesh-Samanta](https://github.com/GyaneshSamanta))
- **Eshaan Bhardwaj** ([@Eshaan-B](https://github.com/Eshaan-B)) - _Original Repository Architect_

Originally forked and extended from [Eshaan-B/thiot](https://github.com/Eshaan-B/thiot).

---

## 📜 License

This project is for research purposes. See `LICENSE` for details (if applicable).

---
