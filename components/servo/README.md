# Robotic Arm Configuration & Calibration Guide

This document outlines the servo pin connections, calibration limits (pulse widths in microseconds ($\mu\text{s}$)), and preset joint configurations for the robotic arm.

---

## 1. Pin / Bus Connections

| Joint / Component | Description | Channel / Bus ID |
| :--- | :--- | :---: |
| **Base** | Main rotation base | `0` |
| **Shoulder** | 1st joint from base | `1` |
| **Elbow** | 2nd joint from base | `2` |
| **Gripper** | End-effector claw | `13` *(3rd from end)* |
| **Wrist Rotate** | Head motor / roll orientation | `14` *(2nd from end)* |
| **Wrist Pitch** | Motor beside head / elevation | `15` *(Last bus)* |

---

## 2. Motor Calibration & Motion Limits

> **Note:** All pulse values are measured in microseconds ($\mu\text{s}$).

### Base Motor (Channel 0)
* **Extreme Left:** $400\ \mu\text{s}$
* **$90^\circ$ Left:** $450\ \mu\text{s}$
* **Middle / Center:** $1800\text{--}1900\ \mu\text{s}$ *(last calibrated: $1900\ \mu\text{s}$)*
* **Extreme Right:** $3250\ \mu\text{s}$
* **$90^\circ$ Right:** *Not physically possible*

### Shoulder Motor (Channel 1)
*1st joint from bottom. Higher value moves arm higher / backwards.*
* **Extreme Down:** $1250\ \mu\text{s}$
* **$90^\circ$ Position:** $1800\ \mu\text{s}$
* **Extreme Up / Backwards:** $2500\ \mu\text{s}$

### Elbow Motor (Channel 2)
*2nd joint from bottom.*
* **Extreme Up:** $1000\text{--}1100\ \mu\text{s}$
* **$90^\circ$ Position:** $1900\text{--}2100\ \mu\text{s}$
* **Extreme Down:** $2600\ \mu\text{s}$

### Wrist Pitch Motor (Channel 15)
*Motor beside head. Higher value makes it straighter.*
* **Extreme Down:** $450\ \mu\text{s}$
* **$90^\circ$ Position:** $700\ \mu\text{s}$
* **Extreme Up / Straight:** $1900\ \mu\text{s}$

### Wrist Rotate / Head Motor (Channel 14)
*Closest to gripper. Higher value turns right.*
* **Centered / Flat:** $400\ \mu\text{s}$
* **Extreme Left Turn:** $450\ \mu\text{s}$
* **Extreme Right Turn:** $1450\ \mu\text{s}$

### Gripper Motor (Channel 13)
*Smaller values open the claw.*
* **Fully Open:** $1400\ \mu\text{s}$
* **Fully Closed:** $2800\ \mu\text{s}$

---

## 3. Preset Poses & Configurations

### Summary Comparison Table

| Joint | Channel | Home ($\mu\text{s}$) | Scan Items ($\mu\text{s}$) | Shelf Scan ($\mu\text{s}$) |
| :--- | :---: | :---: | :---: | :---: |
| **Base** | 0 | 1450 | 2900 | 550 |
| **Shoulder** | 1 | 2000 | 1700 | 1850 |
| **Elbow** | 2 | 1800 | 1500 | 1900 |
| **Wrist Pitch** | 15 | 700 | 800 | 1600 |
| **Wrist Rotate** | 14 | 400 | 400 | 400 |
| **Gripper** | 13 | 1450 *(Open)* | 1450 *(Open)* | 1450 *(Open)* |

---

### Detailed Pose Descriptions

#### Home Position
Standard resting and initialization pose.
* **Base:** $1450\ \mu\text{s}$
* **Shoulder:** $2000\ \mu\text{s}$
* **Elbow:** $1800\ \mu\text{s}$
* **Wrist Pitch:** $700\ \mu\text{s}$
* **Wrist Rotate:** $400\ \mu\text{s}$
* **Gripper:** $1450\ \mu\text{s}$ *(Open)*

#### Scan Items Position
Forward/low angled pose for scanning surface items.
* **Base:** $2900\ \mu\text{s}$
* **Shoulder:** $1700\ \mu\text{s}$
* **Elbow:** $1500\ \mu\text{s}$
* **Wrist Pitch:** $800\ \mu\text{s}$
* **Wrist Rotate:** $400\ \mu\text{s}$
* **Gripper:** $1450\ \mu\text{s}$ *(Open)*

#### Shelf Scan Position
Elevated/angled pose for inventory or shelf inspection.
* **Base:** $550\ \mu\text{s}$
* **Shoulder:** $1850\ \mu\text{s}$
* **Elbow:** $1900\ \mu\text{s}$
* **Wrist Pitch:** $1600\ \mu\text{s}$
* **Wrist Rotate:** $400\ \mu\text{s}$
* **Gripper:** $1450\ \mu\text{s}$ *(Open)*