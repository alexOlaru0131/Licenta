# Bachelor Thesis - Autonomous robot with probabilistic trajectory and object avoidance

<img src="02. Imagini/WhatsApp Image 2025-11-17 at 21.57.48_a81f4383.jpg" alt="Photo from the simulator">

<p align="center">
<a href=""><strong>Documentation</strong> (under development)</a>
</p>    

# Project description
This is my Bachelor's Thesis project, Autonomous robot with probabilistic trajectory and object avoidance.

For this project I developed a four-wheeled robot equipped with:
- <a href="https://docs.arducam.com/Raspberry-Pi-Camera/Tof-camera/TOF-Camera/#specifications">ToF camera</a>
- Raspberry Pi 5
- STM32 NUCLEO L452RE-P
- 2x L298N Motor Driver
- 4x Motor Encoders
- 4x 3-6V DC Motor

# Vision algorithm
The ToF camera feedback is 240×180 is filtered using PAVA algorithm (isotonic regression) effectively performing noise reduction and spatial downsampling. Selected points until the clearance margin are extracted and interpolated for improved approximation.

The 2D feedback is then transformed into a 3D plane and used for further processing.

# Model-in-the-Loop
An approximate Simulink model of the plant was developed to support controller design and testing. Model Predictive Control (MPC) and Reinforcement Learning (RL) algorithms are applied on this model to improve control performance and evaluate real-time execution constraints.

# Simulation environment
The simulation environment was developed in Unity Engine trying to mimic realistic constraints of the robot and test the control.

# STM32 NUCLEO L452RE-P development board
The board uses CMSIS-OS (a FreeRTOS-based operating system) for task scheduling. It receives a message via UART, decodes it, and controls the motors using PWM. For precise and accurate movement, distance values are measured and control algorithms are applied.

# Used technologies
- C#
- C/C++
- Python
- Reinforcement Learning
- Model Predictive Control
- CMSIS-OS (FreeRTOS)
- Computer Vision
- Control Theory

