Autonomous Navigation in Abandoned Mines
This project was developed for the Guidance and Navigation course at the Polytechnic University of Madrid, as part of the Master's in Automation and Robotics. It integrates key concepts such as control systems, Kalman filtering, odometry, sensor fusion, and path planning to enable a robot to autonomously navigate and explore abandoned mine environments.

🚀 Project Overview
The goal of this project is to simulate and implement an autonomous navigation system for a robot (named "Marvin") operating in a complex, obstacle-rich mine environment. The system uses:

Path Planning with RRT* (Rapidly-exploring Random Tree Star)

Sensor Fusion via an Extended Kalman Filter (EKF)

Reactive and PID Control for obstacle avoidance and trajectory tracking

Odometry and Landmark-based Localization

The simulation is built using Apollo, a robotics simulation environment developed by the Universidad Politécnica de Madrid.

📁 Project Structure
text
.
├── MINA_ABANDONADA.xml          # Mine environment definition (map, obstacles, landmarks)
├── Ejectuable_Mina.m            # Main executable script
├── generar_mapa.m               # Map generation utility
├── Kalman_EXT.m                 # Extended Kalman Filter implementation
├── Cont_PD.m                    # PID controller for trajectory tracking
├── Cont_Reactivo.m              # Reactive control for obstacle avoidance
├── Sensores_Inc.m               # Sensor calibration and variance calculation
├── transformToMap.m             # Coordinate transformation between map systems
├── Odometria_Inc.m              # Odometry calibration and error analysis
├── PlanificadorRRT.m            # RRT* path planner
└── README.md
🛠️ Dependencies
MATLAB (tested on R2021a or later)

Apollo Simulation Framework (provided by UPM)

Image Processing Toolbox (for map handling)

Robotics System Toolbox (for path planning)

🧭 How to Use
1. Set Up the Environment
Ensure Apollo is installed and properly configured in MATLAB.

Load the mine environment using MINA_ABANDONADA.xml.

2. Run the Main Script
Execute the main script to start the simulation:

matlab
Ejectuable_Mina
This script:

Initializes the robot at a starting position.

Plans a path from start to goal using RRT*.

Uses sensor data and Kalman filtering for localization.

Applies PID and reactive control to navigate safely.

Stops when the goal is reached or after a timeout.

3. Customize Start and Goal
Modify the following lines in Ejectuable_Mina.m to set custom start and goal positions:

matlab
inicio = [0, 0];          % Start coordinates
objetivo = [36, 0];       % Goal coordinates
📊 Key Features
Path Planning (PlanificadorRRT.m)
Uses RRT* to find a collision-free path.

Accounts for the robot's turning radius.

Visualizes the tree expansion and final path.

Localization (Kalman_EXT.m)
Fuses odometry and landmark (beacon) data.

Corrects drift using an Extended Kalman Filter.

Control Systems
Cont_PD.m: PID controller for smooth path tracking.

Cont_Reactivo.m: Reactive controller for obstacle avoidance using ultrasonic sensors.

Calibration
Sensores_Inc.m: Calibrates laser and ultrasonic sensors.

Odometria_Inc.m: Calibrates odometry and computes error variances.

Map Handling
generar_mapa.m: Generates a 2D occupancy grid from the XML environment.

transformToMap.m: Converts between Apollo and MATLAB coordinate systems.

🧪 Simulation Outputs
Real-time visualization of the robot's path.

Estimated vs. real pose from the Kalman filter.

Control outputs (linear and angular velocities).

Success/failure status upon reaching the goal.

📌 Notes
The project assumes the Apollo framework is correctly installed and linked in MATLAB.

Sensor noise parameters and control gains are tuned for the provided mine environment.

The RRT* planner may require adjustment of parameters (maxIterations, MaxConnectionDistance) for different environments.

👥 Authors
This project was developed as part of the Master's in Automation and Robotics at the Polytechnic University of Madrid.

📜 License
This project is for academic purposes. Please acknowledge the authors if used or referenced.

For any questions or issues, feel free to open an issue or contact the contributors.
