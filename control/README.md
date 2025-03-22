# Control Systems
Submodule of McMaster Exoskeleton Embedded Software, handling feedback control loop.

This repository contains the **general state controller** and **torque controller**.

## Setup

To run this project, you will need:

### 1. Required Software
- **C++ Compiler:** `g++` 
- **Python:** `python3` (for graphing)

### 2. Install Python Dependencies
Before running the analysis, install the required Python libraries:

```bash
pip3 install matplotlib pandas
```

---

## How to Run the Code

### 1. Clone the Repository
```bash
git clone <repo-url>
cd <repo-folder>/control
```

### 2. Compile & Execute with One Command
```bash
make run_plot
```
This will:
- **Compile the general state controller**
- **Run the FSM**
- **Log joint state and torque data**
- **Generate plots automatically**

### 3. Navigate Through the State Controller
The program starts in the Idle State. Follow these steps:

**Switch to Active State:** 
   - When prompted, enter `1` to activate the controller.
   - This will simulate joint motion and compute torques.

**Exit the Program:**
   - When prompted, enter `3` to stop execution.

---

## Understanding the Output
After exiting, the system will generate 3 plots:

1. Hip Joint Data (Angle, Velocity, Acceleration, Torque)
2. Knee Joint Data (Angle, Velocity, Acceleration, Torque)
3. Torque Component Breakdown (Mass × Acceleration, Coriolis, Gravity, Total Torque)

As an initial test case, the hip and knee joint angles follow a sinusoidal trajectory.

TODO: Find out what AI model for hip and knee joint angles will be.
