# RMC-TRACER Line-Following Robot Simulation (Webots – C++)

*Based on the real robot RMC-TRACER of @takumi_fc3s*

---

## Project Overview

This project implements an **line-following robot controller** written in **C++ for Webots**, targeting a line-Following Robot differential drive robot (RMC-TRACER).

The controller includes sensor calibration and PID-based steering, designed to preserve stability while it followin the line.

![World Overview](docs/images/Webots/RMC-TRACER-V2_1.png)

---

## Current Implementation Status

### Fully implemented features
- Automatic sensor calibration
- FSM-Finite state machine control
- PID-Adaptative based line following (*New!*)

---

## 3D Model Design Basis

This project is based on the real RMC-TRACER robot by @takumi_fc3s. I used the original 3D design of the robot's body model, including the chassis, battery holder, and cover. I modeled the hardware in Blender—the motors, PCBs, and their components—taking care to keep the model mesh as simple as possible while still capturing the details of the real robot.

![World Overview](docs/images/Renders/Blender/Blender01.png)
*Optimized 3D design mesh.*

![World Overview](docs/images/Renders/Blender/Blender02.png)
*3D design formed with separate objects with colors and textures.*

![World Overview](docs/images/Renders/Blender/Blender03.png)
*Complete 3D model.*

![World Overview](docs/images/Renders/Blender/Blender04.png)
*3D model hardware (internal parts).*

## Webots World Structure

The simulation is built around a complete Webots world that combines **functional separation** and **visual realism**.
Each component has a specific responsibility, allowing clean interaction between control logic, visualization, and timing.

### World Components

- **WorldInfo**
  Defines global simulation parameters such as time step, physics, and world behavior.

- **Viewpoint**
  Controls the main 3D camera parameters used for visualization.

- **TexturedBackground**
  Provides background textures and lighting conditions for the environment.

- **Sport Center (Solid)**
  A 3D sports center model used as the main environment backdrop.

- **Furnished Group**
  A set of desks, chairs, and computers that enrich the indoor scene and provide a realistic context.

```
World
├── Environment (Sport Center, Furniture)
├── Visualization (Follower_cam Robot)
├── Timing & Events (Start_goal Robot)
└── Line Follower Control (RMC-TRACER Robot)
```

---

### Robots in the World

- **RMC-TRACER Robot**
  The line follower robot controlled.
  It performs:
  - Sensor calibration
  - Pause
  - Line follower tracer 

- **Follower_cam Robot**
  A lightweight robot equipped with a follow camera.
  Its controller keeps the RMC-TRACER robot centered in view while applying a smooth oscillatory motion, improving visual tracking and presentation.

---

## ⚠️ Model & Physical Limitations

The robot model does **not include a suction or downforce system**.

### As a result:
- Abrupt acceleration may cause the front of the robot to lift
- Sudden speed changes can reduce sensor contact and stability
- High-speed entry into tight curves may lead to track loss

### For this reason, the controller applies:

- Speed limits
- PID adaptative limits

The purpose of these limitations is to accurately reflect the behavior of the real robot.

---

## Performance Summary
```
├── Track length:         12.50 m
├── Radius of curves:      0.15 m
├── Average speed:         0.50 m/s
```
Performance depends on simulation parameters and host machine capabilities.

---

## Code Architecture

The controller is fully modular and split into multiple files:

```
├── RMC_Tracer_Ctrl.cpp    // State machine and orchestration (Main)
├── Config.hpp             // Global configuration & constants
├── State.*                // Robot states
├── Sensors.*              // Sensor handling & normalization
├── PID.*                  // PID controller
├── SpeedCtrl.*            // Motor and speed management
├── Sound.*                // Sound indicator
├── UI.*                   // 3D visualization (OLED screen)
├── Graph.*                // Graphics processing
```

![Structure](docs/images/Webots/Controller_Map_V2.png)

---

## How to Use

1. Clone the repository
2. Open the world in Webots
3. Compile the RMC_Tracer_Ctrl.cpp robot controller -> [Go to How to compile *.cpp controllers](#how-to-compile-cpp-controllers)
4. Compile the Follow_Cam001.cpp robot controller   -> [Go to How to compile *.cpp controllers](#how-to-compile-cpp-controllers)
5. Run the simulation
6. Observe: calibration → pause → line follower run

```
git clone https://github.com/DrakerDG/RMC-TRACER
```

No manual tuning is required for basic operation.

---

## 💻 Hardware and software equipment used
```
- Host: Victus by HP Laptop 16-d0xxx
- CPU: 11th Gen Intel i7-11800H (16) @ 4.600GHz
- GPU: NVIDIA GeForce RTX 3060 Mobile / Max-Q
- Memory: 31727MiB 
- OS: Ubuntu 24.04.3 LTS x86_64
- Simulator: Webots
- 3D design: Blender
- Textures: Inkscape & Gimp
```
---

## Feedback & Contributions

Feedback is welcome in the following areas:
- Control logic improvements
- PID tuning strategies
- Performance on different machines
- Code structure and maintainability
- Webots-specific optimizations

Please refer to **CONTRIBUTING.md** before submitting changes.

---

## Screenshots & Visualization

![World Overview](docs/images/Webots/RMC-TRACER-V2_2.png)
*Webots world overview showing the sports center environment, track layout, and RMC-TRACER Robot during simulation.*

![Line Follower Robot - 3D View](docs/images/Webots/RMC-TRACER-V2_3.png)
*RMC_Tracer_Ctrl following the track using PID control.*

![Real-Time UI Overlay](docs/images/Webots/RMC-TRACER-V2_4.png)
*Real-time 3D display showing timer, elapsed, state, robot speed and sensor states during execution.*

![Simulation Environment Detail](docs/images/Webots/RMC-TRACER-V2_5.png)
*Indoor sports center environment with furnished elements used to provide a realistic simulation context.*

---

## Project Intent

The primary goals of this repository are:
- To explore control strategies in simulation
- To study line-following performance under realistic physical constraints
- To evaluate PID behavior and speed profiling
- To provide a reference implementation for experimentation and learning

This project exists as a supportive and complementary effort to the RMC-TRACER robot ecosystem, offering:
- A simulation-based environment
- A modular C++ controller architecture
- Reproducible experiments without hardware risk

---

## Relationship to RMC-TRACER robot owner

- All credit for the robot concept belongs to the RMC-TRACER robot and its contributors (@takumi_fc3s)
- This repository aims to add value through simulation, not ownership
- Improvements developed here are intended to be shared openly

Contributors are encouraged to respect the spirit of open collaboration and attribution.

---

## Philosophy

This repository prioritizes:
- Learning over optimization-at-all-costs
- Transparency over abstraction
- Stability and realism over raw speed

Any contribution should align with these principles.

## Final note
“If this work helps improve understanding or development of the RMC-TRACER robot on the line follower running, it has fulfilled its purpose.”

---

## External Sources

- **Blender**
  https://www.blender.org
  
- **Webots Simulator**
  https://cyberbotics.com

- **takumi**
  https://x.com/takumi_fc3s

- **DrakeDG**
  https://x.com/draker_dg
  www.youtube.com/@DrakerDG

---

## Appendix

### How to compile *.cpp controllers

![Build cpp](docs/images/Webots/Build/build01.png)
*Open file RMC_Tracer_Ctrl.cpp*

![Build cpp](docs/images/Webots/Build/build02.png)
*Start the build process (Build the current project)*

![Build cpp](docs/images/Webots/Build/build03.png)
*Confirmation of successful build!*

![Build cpp](docs/images/Webots/Build/build04.png)
*Open file Follow_Cam001.cpp*

![Build cpp](docs/images/Webots/Build/build05.png)
*Start the build process (Build the current project)*

![Build cpp](docs/images/Webots/Build/build06.png)
*Confirmation of successful build!*

---

## Bonus

### 3D model renders
- **Renders**

[![Renders](docs/images/Renders/Renders.png)](docs/images/Renders/Wallpaper)
[*Preview*](docs/images/Renders/Wallpaper)
