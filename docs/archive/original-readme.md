# 🦾 SCARA Robot | Precision 4-DOF Manipulator
**Autonomous Pick-and-Place System | ROS2 & Embedded Control**

A custom-engineered SCARA (Selective Compliance Articulated Robot Arm) designed for high-speed, repeatable automation. This project integrates mechanical design, kinematic theory, and real-time embedded firmware into a functional production prototype.

---

## 🚀 Technical Highlights
* **Kinematic Control:** Implemented **Forward and Inverse Kinematics** for deterministic end-effector positioning in Cartesian space.
* **Motion Planning:** Utilized **PID control** and S-curve acceleration profiles to minimize mechanical jerk and improve repeatability.
* **Digital Twin:** Developed a high-fidelity **URDF model** for simulation in **Gazebo** and visualization in **RViz2**.
* **Embedded Architecture:** Bare-metal firmware (C++) managing stepper motor coordination and real-time sensor feedback.

---

## 🛠 Tech Stack
| Category | Tools & Technologies |
| :--- | :--- |
| **Robotics** | ROS2 (Humble/Foxy), Gazebo, RViz2, MoveIt! |
| **Embedded** | C++, STM32/Arduino, I2C, SPI, AccelStepper |
| **Mechanical** | **SolidWorks**, Bambu Lab X1C (3D Printing), GT2 Pulleys |
| **Control** | Python, Forward/Inverse Kinematics, PID Tuning |

---

## 📸 Media & Performance
### System in Action
![SCARA Robot Arm](Images/SCARA-Robot-Arm.gif)
*Caption: Autonomous pick-and-place cycle demonstrating trajectory repeatability within ±0.5mm.*

---

## 📂 Project Structure
* `/CAD`: SolidWorks assembly files and STL components.
* `/Firmware`: Embedded C++ source code for the microcontroller.
* `/ROS2_WS`: Robot descriptions (URDF), launch files, and simulation nodes.
* `/Scripts`: Python-based GUI and Kinematic solver scripts.

---

## 📈 Future Iterations
- [ ] **Computer Vision:** Integrating OpenCV for autonomous object detection and color sorting.
- [ ] **Structural Rigidity:** Replacing PLA components with Carbon Fiber reinforced filaments for higher payload capacity.
- [ ] **Edge Computing:** Deploying processing nodes on an SBC (Raspberry Pi/Jetson Nano) for decentralized control.

---

### 🔗 Portfolio & Contact
Developed by **Matthew Garcia** [← Back to Portfolio](https://matthew-garcia-portfolio.vercel.app)
