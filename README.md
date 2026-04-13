# 🚀 ROS2 Reactive Navigation Controller (V4)

⚠️ **Important Note**

This controller is an educational implementation and **does NOT provide fully reliable navigation**.

While it performs well in simple scenarios, it can still:

* Collide with obstacles
* Get stuck in complex environments
* Fail in tight or symmetric situations

These limitations are intentional and highlight the boundaries of **pure reactive control**.

---

## 🧠 Purpose of This Project

This project is designed to demonstrate:

* Why simple obstacle avoidance is not sufficient for real-world robotics
* The challenges of reactive navigation
* The need for advanced systems like **ROS2 Nav2**

Rather than being a production-ready solution, this controller is a **learning tool**.

---

## 🧠 Motivation

This project was developed to deeply understand:

* Why simple obstacle avoidance is not enough
* The limitations of reactive control systems
* How real navigation stacks (like Nav2) are structured

Instead of directly using Nav2, this controller rebuilds core ideas from scratch.

---

## ⚙️ Features

### ✅ Sensor Processing

* Subscribes to `/scan` (LIDAR)
* Filters invalid values
* Computes:

  * Front distance (median + min)
  * Left / Right sectors

### ✅ Goal Tracking

* Uses `/odom` to track robot position
* Computes:

  * Distance to goal
  * Heading error (`yaw_error`)
* Moves toward a defined `(goal_x, goal_y)`

### ✅ State Machine

The controller uses a structured state machine:

* `GO_TO_GOAL`
* `AVOID_LEFT`
* `AVOID_RIGHT`
* `BACK_UP` (recovery)
* `GOAL_REACHED`

---

### ✅ Obstacle Avoidance (Improved)

Unlike naive reactive systems, this controller:

* Chooses a side (`LEFT` or `RIGHT`) once
* Commits to that direction
* Attempts to reduce oscillation (left-right loop)

---

### ✅ Recovery Behavior (V4)

When the robot gets too close to an obstacle:

* Switches to `BACK_UP`
* Moves backward for a short duration
* Returns to previous avoidance direction

This helps reduce:

* Getting stuck
* Immediate collisions

---

### ✅ Safety Layer

* Uses **front_min** for emergency proximity detection
* Uses **front_median** for general behavior
* Introduces thresholds:

  * `front_stop_threshold`
  * `front_avoid_threshold`
  * `front_clear_threshold`

---

### ✅ Speed Control

* Dynamic linear velocity scaling:

  * Slows down near goal
  * Slows down with large heading error
* Separate speeds for:

  * Normal motion
  * Avoidance
  * Recovery

---

## 📊 Architecture Overview

This controller mimics a simplified navigation stack:

| This Project     | Equivalent in Nav2 |
| ---------------- | ------------------ |
| LIDAR processing | Costmap            |
| Goal tracking    | Global planner     |
| Avoidance logic  | Local planner      |
| State machine    | Behavior Tree      |
| BACK_UP recovery | Recovery behaviors |

---

## 🚧 Known Limitations

* No global path planning
* No map awareness
* Can:

  * Collide with obstacles
  * Get stuck in loops
  * Fail in narrow spaces
  * Struggle when goal is behind obstacles

These limitations are **intentional and educational**.

---

## 🧪 Tested With

* ROS2 Jazzy
* Ubuntu 24.04
* Gazebo simulation
* TurtleBot3

---

## ▶️ How to Run

1. Build workspace:

```bash
colcon build
source install/setup.bash
```

2. Launch simulation:

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

3. Run controller:

```bash
ros2 run <your_package_name> controller_v4
```

---

## 🎯 Learning Outcomes

By building this controller, you will understand:

* Reactive vs planned navigation
* Why obstacle avoidance alone is insufficient
* The importance of:

  * Memory (state)
  * Recovery behaviors
  * Stability (hysteresis)

---

## 🔜 Next Step

This project naturally leads to:

👉 **ROS2 Nav2 Navigation Stack**

---

## 👨‍💻 Author

Developed as part of a hands-on ROS2 learning journey toward becoming a robotics/navigation engineer.
