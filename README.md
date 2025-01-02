# Exoskeleton-Embedded
Main Repo for McMaster Exoskeleton Embedded Software

## Modules

### Sensor Input Integration
Submodule responsible for interfacing with sensors placed on the exoskeleton, and either storing them for training purposes, or using them to predict user gait in real-time

### Control Systems / Loop
Submodule responsible for integrating the gait prediction from the model and setting the new setpoint for each joint in the system in a regulated smooth manner.

### Motor Control
Submodule responsible for taking the new setpoint of the system and controlling the motors into the desired position

