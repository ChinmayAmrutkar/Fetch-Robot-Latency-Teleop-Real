# 🤖 Fetch Robot – Latency-Aware Teleoperation & Navigation

This repository enables **teleoperation of the real Fetch mobile manipulator** under latency constraints and provides the tools for **autonomous navigation** using the ROS stack. The teleoperation system models the robot as a virtual mass-damper to generate smooth motion under delay, while the navigation system uses AMCL for localization and `move_base` for path planning.

---

## 📦 Repository Setup

### 1️⃣ Clone the Repository

```bash
git clone https://github.com/ChinmayAmrutkar/Fetch-Robot-Latency-Teleop-Real.git
cd Fetch-Robot-Latency-Teleop-Real
```

### 2️⃣ Create Workspace & Build

```bash
# Assuming your workspace is named fetch_latencey_real_ws
cd ~/fetch_latencey_real_ws
catkin_make
source devel/setup.bash
```

---

## 🗺️ Mapping and Autonomous Navigation

Before you can navigate autonomously, you must first create a map of your environment. This is a two-phase process.

### **Phase 1: Building a Map**

In this phase, you will drive the robot to create a 2D map of your lab or workspace.

1.  **Set ROS Master:** In a new terminal, set the ROS Master to your Fetch robot.
    ```bash
    export ROS_MASTER_URI=http://fetch1090:11311
    ```

2.  **Launch GMapping:** Run the custom mapping launch file. This starts the robot drivers, the `gmapping` SLAM algorithm, and RViz with the correct configuration.
    ```bash
    roslaunch fetch_latency_real gmapping_real_robot.launch
    ```

3.  **Teleoperate to Map:** In a **second terminal** (with the ROS Master set), start your teleoperation node.
    ```bash
    rosrun fetch_latency_real teleop.py
    ```
    Drive the robot **slowly** around the entire environment. To get a good map, ensure you cover all areas and return to previously visited locations to "close the loops."

4.  **Save the Map:** Once you are satisfied with the map in RViz, open a **third terminal** (with the ROS Master set) and save the map to your project's map directory.
    ```bash
    rosrun map_server map_saver -f ~/chinmay/Fetch-Robot-Latency-Teleop-Real/src/fetch_latency_real/maps/my_lab_map
    ```
    This creates `my_lab_map.yaml` and `my_lab_map.pgm`. You can now `Ctrl+C` all terminals.

### **Phase 2: Localization & Navigation**

Now, use the map you just created to localize the robot and send it autonomous goals.

1.  **Launch Navigation:** On the robot, launch the navigation stack. You must provide the **full, absolute path** to the map file as it exists on the robot's filesystem.
    ```bash
    # Replace the path with the actual location of your map on the robot's filesystem
    roslaunch fetch_latency_real localize_real_robot.launch map_file:=/home/fetchuser/chinmay/Fetch-Robot-Latency-Teleop-Real/src/fetch_latency_real/maps/my_lab_map.yaml
    ```
    > **Important:** The `map_server` runs on the robot, so the path must be valid from its perspective. Using `~` in the path can be unreliable as it resolves to the home directory of the user running the command. Always use the full path (e.g., `/home/fetchuser/...`) to avoid "file not found" errors.

2.  **Localize the Robot:** In RViz, the robot will be lost initially.
    * Click the **"2D Pose Estimate"** button in the top toolbar.
    * Click and drag an arrow on the map that matches the robot's actual starting position and orientation in the real world.

3.  **Send a Navigation Goal:**
    * **⚠️ Be prepared to E-Stop the robot!**
    * Click the **"2D Nav Goal"** button in the top toolbar.
    * Click and drag an arrow on the map to set a destination. The robot will begin moving autonomously.

### **🔬 Phase 3: Performance Evaluation against Motion Capture (Mocap)**

To quantitatively measure the accuracy of AMCL, you can compare its pose estimate against a high-precision motion capture system (e.g., Vicon, OptiTrack).

1.  **Hardware Setup:**
    * Place reflective markers on your Fetch robot in a rigid, asymmetric pattern that the mocap system can see and track.
    * In your mocap software (e.g., Vicon Tracker, OptiTrack Motive), create a rigid body for the Fetch robot using these markers. Name this body something memorable, like `fetch`.

2.  **Software Setup:**
    * You need a ROS driver for your mocap system. A very common one is `vrpn_client_ros`. If you don't have it, install it:
        ```bash
        sudo apt-get install ros-melodic-vrpn-client-ros
        ```
3.  **Run the Experiment:**
    * **Terminal 1:** Launch the mocap ROS node. The exact launch file depends on your system, but for `vrpn_client_ros` it's typically:
        ```bash
        roslaunch vrpn_client_ros sample.launch server:=<IP_OF_MOCAP_PC>
        ```
    * **Terminal 2:** Launch your localization file from Phase 2.
        ```bash
        roslaunch fetch_latency_real localize_real_robot.launch map_file:=/path/to/your/map.yaml
        ```
    * **Terminal 3:** Run your new logging script.
        ```bash
        rosrun fetch_latency_real log_mocap_comparison.py
        ```
    * **Terminal 4:** Drive the robot around using your `teleop.py` node.
    * When finished, `Ctrl+C` all the terminals. The comparison data will be saved in `amcl_vs_mocap_log.csv` in your home directory.

---

## ⚙️ Manual Teleoperation (Latency-Aware)

If you only want to use the latency-aware teleoperation without navigation, follow these steps.

### 🌐 1. Set ROS Master to Fetch Robot

In **every terminal**, first set the ROS Master URI:

```bash
export ROS_MASTER_URI=http://fetch1090:11311
```
> Replace `fetch1090` with your robot’s actual hostname or IP.

### 📷 2. Start Delayed Camera Node

```bash
rosrun fetch_latency_real delayed_camera.py
```
- Subscribes to `/head_camera/rgb/image_raw`
- Publishes to `/head_camera/rgb/image_raw_delayed`
- Adds artificial delay to simulate laggy video feeds

### 🧠 3. Start Admittance Controller

```bash
rosrun fetch_latency_real admittance_controller.py
```
- Implements virtual **mass-damper** dynamics
- Converts operator intent (`/intent_vel`) into velocity commands
- Publishes to `/cmd_vel` topic

### 🎮 4. Start Teleoperation Node

```bash
rosrun fetch_latency_real teleop.py
```
- Publishes velocity intent commands to `/intent_vel`
- Keyboard-based control (WASD layout)

---

## ✅ System Requirements

-   **Real Fetch Robot** with network access
-   ROS Melodic on both robot and control workstation
-   SSH/network configured for remote ROS master connection

---

## 🧠 Author

**Chinmay Amrutkar**
M.S. Robotics and Autonomous Systems – AI
Arizona State University
GitHub: [@ChinmayAmrutkar](https://github.com/ChinmayAmrutkar)

---

## 📘 License

This project is licensed under the **MIT License**.
