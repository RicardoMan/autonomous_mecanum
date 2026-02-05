# Mecanum Robot with micro-ROS

Building an autonomous omnidirectional robot from scratch using micro-ROS, ROS2 Humble, and an Arduino Portenta H7 microcontroller.

## Project Goal

Create a fully autonomous mecanum wheel robot capable of omnidirectional movement, controlled wirelessly via ROS2, with eventual SLAM and autonomous navigation capabilities.

**Key Feature:** No GPU, no Raspberry Pi — just efficient embedded control on a microcontroller.

---

## Project Progress

| Step | Description | Status |
|------|-------------|--------|
| 1 | Wireless Control (micro-ROS + WiFi) | Complete |
| 2 | Encoder Odometry | Complete |
| 3 | IMU Integration | In Progress |
| 4 | RViz Visualization | Pending |
| 5 | Sensor Fusion (EKF) | Pending |
| 6 | SLAM | Pending |
| 7 | Autonomous Navigation (Nav2) | Pending |

---

## Hardware

### Components

| Component | Model | Quantity |
|-----------|-------|----------|
| Microcontroller | Arduino Portenta H7 | 1 |
| Carrier Board | Portenta Mid Carrier (ASX00055) | 1 |
| Motor Controllers | RoboClaw 2x15A | 2 |
| Motors | goBILDA 5203 Yellow Jacket (19.2:1, 312 RPM) | 4 |
| Encoders | Built-in quadrature (2150 CPR) | 4 |
| Wheels | Mecanum 96mm | 4 |
| Batteries | Matrix 12V 3000mAh NiMH | 2 |
| IMU | SparkFun ISM330DHCX 6DOF | 1 |

### Robot Dimensions

```
        FRONT
   [FL]-------[FR]
    |           |
    |<--275mm-->|
    |           |
    |   315mm   |
    |     |     |
   [RL]-------[RR]
        REAR

Wheel Diameter: 96mm
```

### Wiring

```
Portenta H7          RoboClaw #1 (0x80 - Front)
-----------          -------------------------
TX1  --------------> S1: I/O
RX1  <-------------- S2: I/O
GND  --------------> S1: GND

Portenta H7          RoboClaw #2 (0x81 - Rear)
-----------          ------------------------
TX3  --------------> S1: I/O
RX3  <-------------- S2: I/O
GND  --------------> S1: GND
```

### Motor Assignment

| RoboClaw | Address | M1 | M2 |
|----------|---------|----|----|
| RC1 (Front) | 0x80 | Front Right | Front Left |
| RC2 (Rear) | 0x81 | Rear Right | Rear Left |

---

## Software

### Requirements

- Host PC: Ubuntu 22.04 with ROS2 Humble
- Microcontroller: PlatformIO with Arduino framework
- Docker: For micro-ROS agent

### WiFi Configuration

| Parameter | Value |
|-----------|-------|
| SSID | PortentaROS |
| Password | microros123 |
| Agent IP | 10.42.0.1 |
| UDP Port | 8888 |

### ROS2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| /cmd_vel | geometry_msgs/Twist | Velocity commands (subscriber) |
| /odom | nav_msgs/Odometry | Robot odometry (publisher) |

---

## Quick Start

### 1. Clone Repository

```bash
git clone https://github.com/YOUR_USERNAME/mecanum-robot.git
cd mecanum-robot
```

### 2. Flash Portenta

```bash
cd portenta_firmware
pio run -t upload
```

### 3. Run on Host PC

Terminal 1 - Activate Hotspot:
```bash
nmcli connection up Hotspot
```

Terminal 2 - Start micro-ROS Agent:
```bash
docker run -it --rm --net=host microros/micro-ros-agent:humble udp4 --port 8888 -v4
```

Terminal 3 - Teleop Control:
```bash
cd ~/Desktop
source /opt/ros/humble/setup.bash
python3 teleop_qweasdzxc.py
```

Terminal 4 - View Odometry:
```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /odom
```

### 4. Reset Portenta

Press the RESET button on the Portenta board after the agent is running.

---

## Keyboard Controls

```
       Q              W              E
  (diagonal)      (forward)     (diagonal)

       A              S              D
  (lateral)        (STOP)       (lateral)

       Z              X              C
  (diagonal)      (backward)    (diagonal)

       R = rotate left
       T = rotate right
       
       Ctrl+C = exit
```

---

## Repository Structure

```
mecanum-robot/
├── README.md
├── docs/
│   ├── Step1_Wireless_Control.pdf
│   ├── Step2_Odometry_Backup.pdf
│   └── Mecanum_Robot_Technical_Documentation.pdf
├── portenta_firmware/
│   ├── platformio.ini
│   └── src/
│       └── main.cpp
├── host_scripts/
│   └── teleop_qweasdzxc.py
└── models/
    └── (3D models coming soon)
```

---

## Mecanum Kinematics

### Forward Kinematics (Wheel to Robot Velocity)

```
Vx = (V_fl + V_fr + V_rl + V_rr) / 4
Vy = (-V_fl + V_fr + V_rl - V_rr) / 4
w  = (-V_fl + V_fr - V_rl + V_rr) / (4 * L)
```

### Inverse Kinematics (Robot to Wheel Velocity)

```
V_fl = (Vy - Vx - L*w) / r
V_fr = (Vy + Vx + L*w) / r
V_rl = (Vy + Vx - L*w) / r
V_rr = (Vy - Vx + L*w) / r

Where:
  r = wheel radius = 0.048m
  L = (Lx + Ly) / 2 = 0.295m
```

---

## Odometry

Encoder-based odometry using 4-wheel inverse kinematics:

- Update Rate: 20 Hz
- Encoder Resolution: 2150 CPR
- Published Frame: odom to base_link

Known Limitations:
- Mecanum wheels can slip laterally, causing drift
- IMU fusion (Step 3) will improve accuracy

---

## Roadmap

- [x] Step 1: Wireless control via micro-ROS
- [x] Step 2: Encoder-based odometry
- [ ] Step 3: IMU integration (SparkFun ISM330DHCX)
- [ ] Step 4: RViz visualization with 3D model
- [ ] Step 5: Sensor fusion with EKF
- [ ] Step 6: SLAM implementation
- [ ] Step 7: Autonomous navigation with Nav2

---

## Contact

Ricardo Manriquez
https://www.linkedin.com/in/rman1988/

---

## Acknowledgments

- micro-ROS for embedded ROS2 support
- RoboClaw for motor controller documentation
- goBILDA for quality motors and mecanum wheels
- ROS2 community for excellent documentation
