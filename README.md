# Sliding Mode Controller for Controlling a Quadrotor

## Description
This project implements a sliding mode controller (SMC) to achieve robust trajectory tracking for a quadrotor UAV, specifically the Crazyflie 2.0 model. 
The controller leverages SMC principles to handle nonlinear dynamics and disturbances, ensuring stable flight along predefined 3D trajectories. 
The repository includes Python scripts for control and visualization, a detailed report, and a simulation GIF demonstrating the quadrotor’s performance in Gazebo.

## Demonstration
![SlidingMode_quad](https://github.com/user-attachments/assets/610eb022-a821-4bb2-8182-2ec2864f37d1)

## Installation
To set up and run the project, follow these steps:
1. Clone the repository:
```
git clone https://github.com/kshitijSharma2204/Robust_Trajectory_Tracking_for_Quadrotor_Using_Sliding_Mode_Control_Law.git
cd Robust_Trajectory_Tracking_for_Quadrotor_Using_Sliding_Mode_Control_Law
```
2. Set up a Python environment (optional but recommended):
```
python3 -m venv venv
source venv/bin/activate
```
3. Install Python dependencies:
```
pip install numpy matplotlib
```
4. Install ROS Noetic (for Gazebo simulation):
  - Follow the official ROS Noetic installation guide for Ubuntu 20.04
  - Set up a catkin workspace:
   ```
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws/src
   catkin_init_workspace
   cd ..
   catkin_make
   source devel/setup.bash
   ```
5. Install Crazyflie Gazebo package:
```
sudo apt install ros-noetic-crazyflie-gazebo
```

## Usage
1. Launch the Gazebo simulation:
```
roslaunch crazyflie_gazebo crazyflie2.launch
```
2. Run the controller:
  - In a new terminal, source the workspace and run:
  ```
  source ~/catkin_ws/devel/setup.bash
  python3 quad_control.py
  ```
3. Visualize the trajectory:
  - After simulation, generate a 3D plot of the trajectory:
  ```
  python3 visualize.py
  ```
  - The plot is saved as trajectory.png.

## File Overview
  - `quad_control.py`: Implements the SMC, subscribes to odometry data, computes control inputs, and publishes motor speeds to the Crazyflie in Gazebo.
  - `visualize.py`: Generates a 3D plot of the quadrotor’s trajectory from logged data (log.pkl).
  - `Final_Report.pdf`: Details the theoretical background, trajectory design (polynomial-based), SMC derivation, and simulation results.
  - `SlidingMode_quad.gif`: Visual demonstration of the quadrotor tracking the desired trajectory in Gazebo.

## Project Details
  - Trajectory Design: Desired trajectories are 5th-order polynomials for smooth position, velocity, and acceleration profiles.
  - Controller: SMC tracks desired roll, pitch, yaw, and altitude, converting position errors into motor commands via an allocation matrix.
  - Tuning: Parameters (e.g., Kp=20, Kd=-3.5, lamda1=0.5, etc.) are optimized for convergence to the desired path.
  - Simulation: Tested in Gazebo with ROS Noetic, using the Crazyflie 2.0 model.
