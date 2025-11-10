# 3D-Robotic-Arm-Simulation-MatLab

This project demonstrates the creation and simulation of a 3D robotic arm in MATLAB using the RigidBodyTree framework from the Robotics System Toolbox.
The robot consists of multiple rigid bodies connected by fixed and revolute joints, and it uses inverse kinematics to trace a circular trajectory in 3D space.
______________________________________________________________________________________________________________________________________________________________
🚀 Features

- Builds a 4-link robotic arm using rigidBodyTree
- Defines fixed & revolute joints and transforms
- Adds an end-effector frame for task control
- Uses inverse kinematics to compute joint angles
- Animates the robot tracing a circular path


📂 Project Structure

Component	          Description

Rigid Body Tree	    Defines robot links & joints
End-Effector	      Tool frame at robot tip
Inverse Kinematics  Computes joint configurations
Animation	          Plots robot movement over time

Example Output

The robot traces a circular path in 3D space by solving IK for each point:

center = [0.3 0.4 0];
radius = 0.15;

▶️ Requirements

MATLAB (R2020+ recommended)

Robotics System Toolbox
