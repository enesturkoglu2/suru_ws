# 🦅 ROS 2 Autonomous Drone Swarm (Leader-Follower)

![ROS 2](https://img.shields.io/badge/ROS2-Humble-red) ![PX4](https://img.shields.io/badge/PX4-Autopilot-blue) ![Gazebo](https://img.shields.io/badge/Simulation-Gazebo-orange) ![Python](https://img.shields.io/badge/Language-Python-yellow)

## 🚀 Project Overview
This project implements an autonomous multi-UAV (Unmanned Aerial Vehicle) swarm simulation using **ROS 2 Humble**, **MAVROS**, and **PX4 Autopilot** in Gazebo Classic.

The system features a **Leader-Follower architecture** where:
* **UAV0 (Leader):** Follows an independent trajectory (e.g., vertical oscillation).
* **UAV1 (Follower):** Tracks the leader's position in real-time and performs complex formation flights (Orbit/Satellite Mode) autonomously.

This repository specifically addresses and solves common **MAVROS multi-drone connection issues**, including "Ghost Topics" and Namespace conflicts in WSL 2 environments.

## 🎥 Demo (Simulation)
*(Buraya projenin çalışırkenki kısa bir GIF'ini veya ekran görüntüsünü eklersen harika olur)*

## ✨ Key Features
* **Multi-Vehicle Simulation:** Simultaneous control of 3 drones (iris models) in Gazebo.
* **Orbit Formation Logic:** The follower drone calculates a dynamic circular path around the moving leader using trigonometric functions.
* **Robust MAVROS Configuration:** * Custom launch file handling for multiple UDP ports (14541, 14542).
    * **"Ghost Topic" Fix:** Solved data stream issues using `target_component_id: 0` and CycloneDDS.
* **QGroundControl Integration:** Full telemetry support via UDP Bind Mode.

## 🛠️ Tech Stack & Requirements
* **OS:** Ubuntu 22.04 (LTS) / Windows WSL 2
* **Middleware:** ROS 2 Humble
* **Flight Control:** PX4 Autopilot (SITL)
* **Communication:** MAVROS & MAVLink
* **Simulation:** Gazebo Classic 11
* **DDS Implementation:** Eclipse CycloneDDS

## ⚙️ Installation

1.  **Clone the Repository**
    ```bash
    mkdir -p ~/suru_ws/src
    cd ~/suru_ws/src
    git clone [https://github.com/KULLANICI_ADIN/REPO_ISMI.git](https://github.com/KULLANICI_ADIN/REPO_ISMI.git)
    ```

2.  **Build the Package**
    ```bash
    cd ~/suru_ws
    colcon build --packages-select suru_yonetimi_paketi --symlink-install
    source install/setup.bash
    ```

## 🚀 How to Run (The Golden Sequence)

To avoid connection issues, follow this strict startup sequence:

**1. Start Simulation (Multi-Drone)**
Initialize 3 drones in Gazebo (Ports: 14540, 14541, 14542).
```bash
cd ~/PX4-Autopilot
Tools/simulation/gazebo-classic/sitl_multiple_run.sh -m iris -n 3 -w baylands
