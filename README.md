# 🤖 Fetch Robot – Latency-Aware Teleoperation (Real Robot Setup)

This repository enables **teleoperation of the real Fetch mobile manipulator** under latency constraints using a custom **low-level admittance controller**. The system models the robot as a virtual mass-damper and processes human intent as external forces to generate smooth and safe motion under delay.

---

## 📦 Repository Setup

### 1️⃣ Clone the Repository

```bash
git clone https://github.com/ChinmayAmrutkar/Fetch-Robot-Latency-Teleop-Real.git
cd Fetch-Robot-Latency-Teleop-Real
```

### 2️⃣ Create Workspace & Build

```bash
cd ~/fetch_latencey_real_ws
catkin_make
source devel/setup.bash
```

---

## ⚙️ Real Robot Launch Instructions

### 🌐 1. Set ROS Master to Fetch Robot

In **every terminal**, first set the ROS Master URI:

```bash
export ROS_MASTER_URI=http://fetch1090:11311
```

> Replace `fetch1090` with your robot’s actual hostname or IP.

---

### 📷 2. Start Delayed Camera Node

```bash
rosrun fetch_latency_real delayed_camera.py


```

- Subscribes to `/head_camera/rgb/image_raw`
- Publishes to `/head_camera/rgb/image_raw_delayed`
- Adds artificial delay to simulate laggy video feeds

---

### 🧠 3. Start Admittance Controller

```bash
rosrun fetch_latency_real admittance_controller.py

```

- Implements virtual **mass-damper** dynamics  
- Converts operator intent (`/intent_vel`) into velocity commands  
- Publishes to `/cmd_vel` topic

---

### 🎮 4. Start Teleoperation Node

```bash
rosrun fetch_latency_real teleop.py
```

- Publishes velocity intent commands to `/intent_vel`
- Keyboard-based control (WASD layout)

---

## ✅ System Requirements

- **Real Fetch Robot** with network access
- ROS Melodic on both robot and control workstation
- SSH/network configured for remote ROS master connection

---

## 🧠 Author

**Chinmay Amrutkar**  
M.S. Robotics and Autonomous Systems – AI  
Arizona State University  
GitHub: [@ChinmayAmrutkar](https://github.com/ChinmayAmrutkar)

---

## 📘 License

This project is licensed under the **MIT License**.
```
