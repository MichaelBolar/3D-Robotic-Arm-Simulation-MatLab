# 3D-Robotic-Arm-Simulation-MatLab

This project demonstrates the creation and simulation of robotic arms in MATLAB using the RigidBodyTree framework from the Robotics System Toolbox. The repository includes both **2D** and **3D** implementations of a 6-DOF robotic arm that uses inverse kinematics to trace circular trajectories.

## 🚀 Features

* **Two simulation modes**: 2D planar and 3D spatial robot motion
* Builds a 6-link robotic arm using `rigidBodyTree`
* Defines revolute joints with custom homogeneous transformations
* Adds an end-effector frame (`toolTip`) for task control
* Uses inverse kinematics to compute joint configurations
* Animates the robot tracing circular paths with real-time visualization

## 📂 Project Structure

| Component | Description |
|-----------|-------------|
| **ThreeDRobotArmSimulation.m** | 3D robot arm with vertical motion (traces circle in Y-Z plane) |
| **TwoDRobotArmSimulation.m** | 2D planar robot arm (traces circle in X-Y plane) |
| Rigid Body Tree | Defines 6 robot links & revolute joints |
| End-Effector | Tool frame at robot tip for trajectory control |
| Inverse Kinematics | Computes joint configurations for desired positions |
| Animation | Real-time visualization at 15 fps |

## 🤖 Robot Specifications

**3D Configuration:**
- 6 revolute joints with varying link offsets
- Circular trajectory: `center = [0, 0.4, 0.3]`, `radius = 0.1`
- Motion in 3D space with orthographic projection

**2D Configuration:**
- 6 revolute joints optimized for planar motion
- Circular trajectory: `center = [0.565, 0.32, 0]`, `radius = 0.13`
- Motion constrained to X-Y plane

Both configurations use a 6-DOF serial manipulator with a fixed end-effector frame.

## 🎯 Example Output

The robot traces a circular path by solving inverse kinematics for 51 points along the trajectory. The IK solver uses weighted constraints `[0, 0, 0, 1, 1, 0]` to prioritize position over orientation.

**3D Simulation:**
```matlab
center = [0 0.4 0.3]; 
radius = 0.1;
```

**2D Simulation:**
```matlab
center = [0.565 0.32 0]; 
radius = 0.13;
```

## ▶️ Requirements

- **MATLAB** (R2020+ recommended)
- **Robotics System Toolbox**

## 🔧 Usage

1. Clone the repository
2. Open MATLAB and navigate to the project directory
3. Run either simulation:
   ```matlab
   % For 3D simulation
   run('ThreeDRobotArmSimulation.m')
   
   % For 2D simulation (uncomment animation code first)
   run('TwoDRobotArmSimulation.m')
   ```

**Note:** The 2D simulation has its animation code commented out. Uncomment the loop at the end of the file to see the animation.

## 📝 Key Concepts

- **Homogeneous Transformations**: Used to define link positions and orientations
- **Inverse Kinematics**: Solves for joint angles given desired end-effector position
- **Rate Control**: Ensures smooth animation at specified frame rate
