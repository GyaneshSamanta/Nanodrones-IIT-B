<div align="center">
  <img src="https://upload.wikimedia.org/wikipedia/commons/thumb/8/87/Crazyflie_2.0_quadrotor.jpg/640px-Crazyflie_2.0_quadrotor.jpg" width="60%" alt="Nanodrone" />

# Nanodrones — IIT Bombay (TH-IoT)

**Autonomous waypoint navigation and 3D path-optimized routing for nanodrones, built at IIT Bombay's IoE Office.**

[![ROS](https://img.shields.io/badge/ROS-Noetic%20%7C%20Melodic-22314e.svg)](http://wiki.ros.org/ROS)
[![PX4](https://img.shields.io/badge/PX4-Autopilot-3cb371.svg)](https://px4.io/)
[![Lab](https://img.shields.io/badge/Lab-IIT%20Bombay%20IoE-orange.svg)](#)

</div>

---

## About

|        |                                                                                              |
| ------ | -------------------------------------------------------------------------------------------- |
| Who    | [Gyanesh Samanta](https://github.com/GyaneshSamanta) and [Eshaan Bhardwaj](https://github.com/Eshaan-B). |
| What   | A ROS / PX4 / MAVROS stack that flies a nanodrone through optimized 3D waypoint sequences.    |
| When   | 2023 — research project at the **IoE (Internet of Everything) Office, IIT Bombay**.           |
| Where  | Indoor flight arena instrumented with **Vicon** motion capture; simulated in Gazebo + jMAVsim. |
| Why    | Indoor multi-target nanodrone routing is a real Vehicle Routing Problem in 3D — and fun.       |

## The Story

The IoE Office at IIT Bombay runs a Vicon-instrumented indoor arena where nanodrones can fly under sub-centimeter localization. The brief was simple to state, hard to satisfy: given a list of 3D targets the drone must visit, find a near-optimal order and fly it autonomously, in simulation first and on the physical bird second.

The order-finding half is a **Vehicle Routing Problem in 3D**. Brute-force is fine for a handful of points and useless past that. We picked the **Clarke & Wright Savings (CWS)** heuristic — a classic VRP construction algorithm that builds a route by greedily merging the most "savings" pairs — and adapted it for 3D Euclidean distances. Implementation lives in `src/px4_control/scripts/cws/`.

The flying half is **PX4 + MAVROS + ROS**. We wrote control logic that consumes the CWS-ordered waypoints, hands them to PX4 via MAVROS setpoints, and supervises the takeoff → traverse → land lifecycle. The same script runs against jMAVsim and Gazebo for simulation, and against a real PX4-equipped nanodrone listening to Vicon poses on the physical arena. Landing protocol got its own special attention: a precise landing routine you can trust on a fragile platform.

The repository is a Catkin workspace; `src/px4_control` is the only ROS package, and the `build/` and `devel/` artefacts ship with it for reference.

## Gallery

In-flight footage and arena photos are not in-repo. Architecture-wise:

- **CWS routing** — `src/px4_control/scripts/cws/` (heuristic + 3D distance helpers).
- **Flight controller** — `src/px4_control/scripts/drone_test.py`.
- **MAVROS test harness** — `src/px4_control/scripts/mavros_test_common.py`.

---

## Tech Stack

- **ROS** (Melodic / Noetic) on Ubuntu
- **PX4 Autopilot** (SITL + real firmware)
- **MAVROS** for ROS ↔ MAVLink bridging
- **Gazebo** + **jMAVsim** for simulation
- **Vicon Motion Capture** for indoor localization
- **Python** for control + the CWS implementation
- **Catkin** build system

## Repo Structure

```
thiot/
├── src/
│   └── px4_control/
│       ├── CMakeLists.txt
│       ├── package.xml
│       └── scripts/
│           ├── drone_test.py        # Top-level flight script
│           ├── mavros_test_common.py # MAVROS state-machine helpers
│           └── cws/                 # Clarke & Wright Savings (3D VRP)
├── build/                            # Catkin build artefacts
├── devel/                            # Catkin devel space
└── .catkin_workspace
```

## Getting Started

**Prereqs:** Ubuntu with ROS Noetic (or Melodic).

```bash
# MAVROS + datasets
sudo apt-get install ros-$ROS_DISTRO-mavros ros-$ROS_DISTRO-mavros-extras
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash install_geographiclib_datasets.sh

# PX4 firmware
git clone https://github.com/PX4/Firmware.git --recursive
cd Firmware && make px4_sitl_default

# Build this workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src && git clone https://github.com/GyaneshSamanta/thiot.git
cd ~/catkin_ws && catkin_make
source devel/setup.bash
```

**Simulate:**

```bash
cd ~/Firmware && make px4_sitl_default jmavsim
roslaunch px4_control px4_sim.launch
rosrun px4_control drone_test.py
```

**Real drone (Vicon):**

```bash
roslaunch px4_control drone.launch gcs_url:=udp://@GROUND_STATION_IP
rosrun px4_control drone_test.py
```

## Contributing

This is a research project. Issues / PRs welcome — particularly anything that swaps CWS for a stronger metaheuristic (LKH, OR-Tools, ALNS) on the 3D VRP.

## License

Research use. See `LICENSE` if present; otherwise treat as all-rights-reserved pending publication.

## Credits

- [Gyanesh Samanta](https://github.com/GyaneshSamanta)
- [Eshaan Bhardwaj](https://github.com/Eshaan-B) — original repository architect
- Hosted by the **IoE Office, IIT Bombay** (Vicon arena + nanodrone hardware).
- Foundations: PX4 Autopilot, MAVROS, ROS, Clarke & Wright (1964).
