# 🐭 ESP32 Micromouse – Flood Fill Maze Solver

## 📌 Overview

This project implements a **Micromouse robot** using an ESP32 microcontroller that autonomously navigates and solves a maze using the **Flood Fill algorithm**.

The robot explores the maze using sensors, builds a map in real time, computes the shortest path to the center, and optimizes its route over multiple runs.

---

## ⚙️ Hardware Components

* ESP32 Microcontroller
* ESP32 30-pin Expansion Board (Shield)
* MPU6050 (Gyroscope + Accelerometer)
* DC Motors with Encoders ×2
* L298N (L928N) H-Bridge Motor Driver
* VL53L0X LiDAR Sensors ×2 (Left & Right)
* IR Sensor (Front wall detection)
* Caster Balls ×2

---

## 🔌 Pin Configuration

| Component     | ESP32 Pin |
| ------------- | --------- |
| Motor IN1     | 25        |
| Motor IN2     | 26        |
| Motor IN3     | 27        |
| Motor IN4     | 14        |
| Left Encoder  | 32        |
| Right Encoder | 4         |
| LiDAR XSHUT 1 | 17        |
| LiDAR XSHUT 2 | 18        |
| IR Sensor     | 19        |
| I2C SDA       | 21        |
| I2C SCL       | 22        |

---

## 🧠 Algorithm

### Flood Fill Algorithm

* The maze is represented as an **8×8 grid**
* Each cell stores:

  * Walls (N, E, S, W)
  * Cost (distance to goal)
* The robot:

  1. Explores the maze using sensors
  2. Updates walls dynamically
  3. Recomputes shortest paths using **BFS (Flood Fill)**
  4. Moves toward the lowest-cost neighbor

---

## 🤖 Robot Behavior

### Exploration Phase

* Uses:

  * IR sensor → front wall
  * LiDAR → left/right walls
* Updates maze map in real-time
* Recomputes path continuously

### Optimization Phase

* Once the optimal path is stable:

  * Stops using sensors
  * Runs fastest path

---

## ⚡ Motion Control

* **Yaw correction** using MPU6050
* **PID-like control** for:

  * Straight movement
  * Precise turning
* Encoder-based distance tracking

---

## 📏 Key Parameters

* Wheel diameter: 46 mm
* Cell size: 190 mm
* Encoder resolution: 203 ticks/rev
* Base speed: 50 PWM
* Turn tolerance: ±0.5°

---

## 📂 Code Structure

* **Motor Control** → PWM + direction logic
* **Sensors API** → IR + LiDAR readings
* **IMU Handling** → angle tracking & calibration
* **Flood Fill** → BFS-based pathfinding
* **Stack/Queue** → path tracking & backtracking

---

## 🚀 How It Works

1. Robot starts at (0,0)
2. Moves forward while sensing walls
3. Updates internal maze representation
4. Runs Flood Fill to compute shortest path
5. Reaches center goal
6. Returns to start
7. Repeats until optimal path is found

---

## 📸 Demo

*(Add your video or images here)*

---

## 🛠️ How to Run

1. Open project in Arduino IDE / PlatformIO
2. Install required libraries:

   * Adafruit MPU6050
   * Adafruit VL53L0X
3. Upload code to ESP32
4. Place robot in maze and power on

---

## 📈 Future Improvements

* Faster path optimization
* Better PID tuning
* Diagonal movement
* Map visualization interface

---

## 👨‍💻 Author

**Sameer**
Computer Engineering Student

---

## ⭐ Notes

This project combines:

* Embedded Systems
* Robotics
* Algorithms (Flood Fill / BFS)
* Real-time control

---

> Built as part of a Micromouse robotics project using ESP32 🚀
