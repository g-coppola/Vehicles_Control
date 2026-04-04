# Vehicles Control: Autonomous Driving & Quadcopter Dynamics

This repository contains the simulation projects developed for the **Vehicles Control** course (Academic Year 2025/2026). The repository is divided into two main modules:
* **Module 1**: Quadcopter Dynamics and Control Strategies (PID, Gain Scheduling, and Robust Control).
* **Module 2**: Robust Control Implementation ($\mathcal{H}_{2}$ vs $\mathcal{H}_{\infty}$) for Autonomous Vehicles in Urban Scenarios.

---

## 🚁 Module 1: Quadcopter Dynamics and Control Strategies

### 📌 Project Overview
This module focuses on the modeling, stabilization, and control of a Quadcopter (UAV) using MATLAB and Simulink. The project explores the non-linear dynamics of the drone and implements a progression of control architectures to achieve stable flight and accurate trajectory tracking. A significant focus is evaluating controller performance under parametric uncertainties, specifically addressing sudden mass variations (e.g., payload pickup or drop), by moving from classical PID architectures to advanced Gain Scheduling and Robust Control strategies.

### ✨ Key Features
* **Quadcopter Dynamic Modeling**: Implementation of the full non-linear 6-DOF (Degrees of Freedom) dynamics of the drone.
* **Open-Loop Analysis**: Evaluation of the unforced system dynamics and stability through free response simulations.
* **Classical Control**: Implementation of basic feedback loops and PID controllers for attitude and altitude stabilization.
* **Advanced & Adaptive Control**: 
    * **State-Feedback Control**: Full state regulation for dynamic stability.
    * **Gain Scheduling**: Adaptive controller parameters to maintain performance across different operating points or mass configurations.
* **Robust Control**: Synthesis of a robust controller designed to maintain stability and performance bounds despite significant mass changes during flight.

### 📂 Module 1 Structure
* **Main Scripts & Data:**
    * `Main.m`: The master script that sequentially executes the entire project. It loads parameters, synthesizes controllers, and runs the Simulink models.
    * `Control.m`: Script dedicated to the synthesis of the state-feedback control matrices.
    * `RobustControl.m`: Script for designing the robust controller and computing optimal robust gains.
    * `MassChange.mat`: Data file containing the simulation parameters and time-series for the payload variation scenario.
    * `Report_VCMod1.pdf`: Detailed technical report documenting the mathematical modeling, control theory, and simulation results.
* **Simulink Models:** `QuadCopter.slx`, `Free_Response.slx`, `PID_Control.slx`, `FeedbackControl.slx`, `GainScheduling.slx`.

### 📊 Results Summary
The comparative study demonstrates how different control strategies handle non-linearities and disturbances:
* The **PID** and standard **State-Feedback** controllers achieve good nominal performance but struggle with aggressive transient errors when the drone's mass suddenly changes.
* The **Gain Scheduling** approach improves adaptability by adjusting gains based on the estimated mass.
* The **Robust Controller** provides the highest degree of reliability, guaranteeing stability and minimizing tracking degradation even during the worst-case mass variation scenarios.

---

## 🚘 Module 2: Robust Control Implementation for Autonomous Vehicles

### 📌 Project Overview
This module contains a simulation study of an autonomous vehicle navigating a complex urban environment using MATLAB's Automated Driving Toolbox. The ego vehicle autonomously detects lane markings, traffic signals, and obstacles to execute dynamic maneuvers such as steering, stopping, and speed regulation. To ensure robust trajectory tracking and stability under unmodeled dynamics and external disturbances, two robust control strategies are implemented and compared: **$\mathcal{H}_{2}$** and **$\mathcal{H}_{\infty}$**. The vehicle dynamics are captured using a Linear Parameter-Varying (LPV) framework, scheduled by the longitudinal velocity, to adapt to varying urban driving conditions.

### ✨ Key Features
* **Urban Scenario Simulation**: Includes intersections, cross traffic, pedestrian crossings, mandatory turns, and obstacle avoidance maneuvers.
* **LPV System Modeling**: A simplified front-steering kinematic bicycle model is linearized and scheduled across six velocity sub-intervals (from 0 to 5 m/s) using sigmoid weighting functions to ensure smooth transitions.
* **Robust Control Synthesis**: State-feedback controllers with augmented yaw error dynamics to minimize tracking errors.
    * **$\mathcal{H}_{2}$ Control**: Solved via LMI formulation to minimize the energy of the closed-loop system response to stochastic disturbances.
    * **$\mathcal{H}_{\infty}$ Control**: Solved via LMI formulation to minimize the worst-case effect of disturbances.
* **Performance Comparison**: Comprehensive analysis of trajectory tracking precision, control effort, and disturbance rejection between the two controllers.
* **3D Visualization**: Photorealistic 3D environment simulation utilizing the Automated Driving Toolbox.

### 📂 Module 2 Structure
* `Main.m`: The primary script to synthesize the LPV controllers, run the Simulink simulations, and generate comparison plots.
* `ScenarioBuild.m` / `get_Data.m`: Scripts used to construct the driving scenario, define the ego vehicle and non-ego actors, and extract reference trajectory data.
* `AutomatedVehicle.slx` / `VehicleModel.slx`: Simulink models for simulating the non-linear vehicle dynamics and the closed-loop control system.
* `AugmentedState.m`: Function to compute the augmented state-space matrices ($A_{a}$, $B_{a}$, etc.) incorporating the yaw tracking error dynamics.
* `H_2.m` / `Hinfinity.m`: Functions defining and solving the Linear Matrix Inequalities (LMIs) to obtain the optimal controller gains.
* `ScenarioH2.m` / `Simulation_H2.m` / `Simulation_Scenario.m`: Scripts to run and visualize the final 3D chase plot simulations based on the controller outputs.

### 📊 Results Summary
The study highlights a clear trade-off between tracking precision and robustness:
* **$\mathcal{H}_{2}$ Controller**: Demonstrates superior tracking accuracy and negligible steady-state error, making it ideal for tight turns and obstacle avoidance. However, it is sensitive to stochastic noise, resulting in high-frequency steering oscillations.
* **$\mathcal{H}_{\infty}$ Controller**: Provides a smoother, damped steering response with excellent noise rejection, but at the cost of larger transient tracking errors.

---

## 🛠️ Prerequisites
To successfully run the simulations for both modules, you will need the following installed:
* **MATLAB & Simulink** (Developed on R2024a)
* **Automated Driving Toolbox** (Required for Module 2)
* **Control System Toolbox**
* **Robust Control Toolbox**
* **YALMIP** (for LMI formulation in both modules)
* **SeDuMi** or **LMILab** (or another compatible solver configured in YALMIP `sdpsettings`)

## 🚀 How to Run

### Running Module 1 (Quadcopter)
1.  Navigate to the Module 1 directory.
2.  Open MATLAB and run `Main.m`. 
3.  The script is designed to run the entire simulation workflow automatically. It will load parameters, run the specific Simulink model for each control strategy, and generate the corresponding plots.
4.  The execution will pause after finishing each section. Keep an eye on the MATLAB Command Window and **press any key** when prompted to proceed to the next control typology.

### Running Module 2 (Autonomous Vehicle)
1.  Navigate to the Module 2 directory.
2.  **Generate Scenario Data**: Run `get_Data.m`. This will build the scenario, extract the reference data, and save it into `SimulationData.mat`.
3.  **Run the Main Simulation**: Open and run `Main.m`. 
    * The script will first compute the $\mathcal{H}_{2}$ gains, simulate the system, and pause.
    * Follow the command window prompts to continue synthesizing the $\mathcal{H}_{\infty}$ controller and generating the final comparative plots.
4.  **Run 3D Visualization**: After running the main simulation, you can run `Simulation_H2.m` (or `Simulation_Scenario.m`) to view the photorealistic 3D chase plot of the vehicle navigating the scenario.

---

## ✍️ Author
**Giuseppe Coppola**
*Vehicles Control Module 1 & 2*
Academic Year 2025/2026
