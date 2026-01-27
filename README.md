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

Reinforcement Learning Algorithm used: <a href="https://spinningup.openai.com/en/latest/algorithms/ppo.html"> Proximal Policy Optimization</a>

# Vision algorithm
The ToF camera feedback is reduced from 240×180 to 48×36 by applying a 5×5 moving average filter, effectively performing noise reduction and spatial downsampling. Breadth-First Search (BFS) is applied to the path from the lowest median point to the maximum-distance point. Selected points along the path are extracted and interpolated for improved approximation.

The 2D feedback is then transformed into a 3D plane and used for further processing.

# Simulation environment
The simulation environment was developed in Unity Engine trying to mimic realistic constraints of the robot. The Proximal Policy Optimization model is trained here for precise path walking and object avoidance.

# Raspberry Pi 5 development board
The board runs on Raspbian, processes the input from the ToF camera and sends the data as input for Proximal Policy Optimization model. All of this is wrapped with Model Predictive Control. The final output is sent to the STM32 board through UART for further interpretation.

# STM32 NUCLEO L452RE-P development board
The board uses CMSIS-OS (a FreeRTOS-based operating system) for task scheduling. It receives a 16-bit message via UART, decodes it, and controls the motors using PWM. For precise and accurate movement, distance values are measured and control algorithms are applied.

# Used technologies
- C#
- C/C++
- Python
- Deep Reinforcement Learning
- CMSIS-OS (FreeRTOS)
- Computer Vision
- Control Theory
