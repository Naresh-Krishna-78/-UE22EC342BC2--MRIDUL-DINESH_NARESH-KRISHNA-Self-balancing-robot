# -UE22EC342BC2--MRIDUL-DINESH_NARESH-KRISHNA-Self-balancing-robot
This project is a self-balancing two-wheeled robot implemented in two parallel approaches:
    Hardware Implementation (using Arduino, motors, and MPU6050)
    Software Simulation (using ROS 2 Jazzy and Gazebo)

It was developed as part of the Robotics Systems course (UE22EC342BC2) by Mridul Dinesh and Naresh Krishna.
Hardware Implementation (Arduino)

The hardware version of the self-balancing robot is built using:
    Arduino UNO
    MPU6050 (Accelerometer + Gyroscope)
    L298N motor driver
    2 DC motors
    Battery supply 

Key Features:
    The MPU6050 reads real-time orientation data (acceleration and angular velocity).
    The Arduino processes this data to determine tilt.
    Based on the tilt angle, it drives the motors forward or backward to maintain balance.
    Basic motor speed and direction control is demonstrated via sample code.

Example Functionality:
    Reads sensor data from the MPU6050 and prints it over serial.
    Controls the motor direction and speed via H-bridge driver (L298N).
    Can be extended with PID control for full balancing.

Software Implementation (ROS 2 + Gazebo)

The software version simulates the same robot in a virtual environment using:
    ROS 2 Jazzy
    Gazebo (gz) simulator
    URDF for robot modeling
    IMU and joint plugins
    PID controller configuration
    Custom ROS 2 nodes

Key Features:
    A full 3D model of the robot is created using URDF and simulated in Gazebo.
    IMU data is simulated and used to mimic real-world balance dynamics.
    A PID controller node reads the tilt and corrects the wheel velocities.
    All components are modular and follow ROS 2 architecture (launch files, nodes, config).

Purpose of Dual Implementation
By developing both hardware and software versions of the self-balancing robot, this project demonstrates:
    Practical understanding of real-world sensors, motors, and microcontrollers.
    Parallel understanding of simulation tools, control algorithms, and ROS middleware.
    Comparison between physical implementation limitations vs simulation flexibility.

Repository Structure
    hardware/ – Arduino code for MPU6050 and motor control
    src/project55/ – ROS 2 packages including URDF, launch files, controller configs, and nodes
    README.md – Overview of the project
