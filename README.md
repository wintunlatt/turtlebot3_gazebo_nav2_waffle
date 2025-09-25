# TurtleBot3 Gazebo Nav2 Waffle + Multi-Goal Navigation

This repository extends the official [TurtleBot3 simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations) with a custom ROS 2 package **`tb3_multi_goal`** that enables **multi-goal navigation directly from RViz clicks**.

## 📂 Repository Structure

```
turtlebot3_gazebo_nav2_waffle/
 ├── README.md
 ├── tb3_multi_goal/          # ✅ Custom package for multi-goal navigation
 └── turtlebot3_simulations/  # Submodule: official TurtleBot3 simulations (ROBOTIS)
```

- `tb3_multi_goal/` → your custom ROS 2 package  
- `turtlebot3_simulations/` → tracked as a **git submodule** (official upstream)  

---

## 🚀 Setup Instructions

### 1. Clone this repo (with submodules)
```bash
git clone https://github.com/wintunlatt/turtlebot3_gazebo_nav2_waffle.git
cd turtlebot3_gazebo_nav2_waffle
git submodule update --init --recursive
```

### 2. Build the workspace
From your workspace root (e.g., `~/ws_ros2`):
```bash
colcon build --symlink-install
source install/setup.bash
```

### 3. Run Simulation + Navigation + Multi-Goal

**Step A: Launch Gazebo world**
```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

**Step B: Launch Nav2**
```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True
```

**Step C: Run Multi-Goal Navigation Node**
```bash
ros2 run tb3_multi_goal multi_goal_from_click_execute
```

Now:
- Use the **2D Goal Pose tool** in RViz to click multiple goal positions.  
- Press **Enter** in the terminal to execute them sequentially.  

---

## 🛠 Requirements
- ROS 2 Humble (recommended)  
- `gazebo_ros` packages  
- `nav2` stack  
- TurtleBot3 simulation dependencies  

Make sure you have sourced ROS 2 and TurtleBot3 setup scripts:
```bash
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash
```

---

## 📌 Notes
- The `tb3_multi_goal` package is self-contained and works with the official TurtleBot3 simulation packages.  
- The submodule ensures you always have the correct upstream TurtleBot3 simulation code.  

---

## 🙌 Credits
- [ROBOTIS TurtleBot3](https://github.com/ROBOTIS-GIT/turtlebot3)  
- Multi-goal extension: [Win Tun Latt](https://github.com/wintunlatt)  





