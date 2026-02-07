# mote_ros

## Introduction
`mote_ros` is designed to develop a shared common operating picture (COP) for the cooperative and autonomous execution of multi-vehicle missions, integrating both aerial vehicles (AVs) and ground vehicles (GVs). This repository focuses on creating interconnected systems that enhance air-ground cooperative autonomy through shared world models, mission definitions, and dynamic environmental awareness.

## Project Objectives

### World Model and Mission Definition
- Develop a comprehensive world model containing structural and semantic information about the environment.
- Overlay a mission definition that is intricately linked with the world model to form the foundational layer of the COP.

### Inter-Vehicle Map Information Exchange
- Implement mechanisms for ground vehicle to ground vehicle (GV-GV) data exchange to enhance map accuracy and facilitate better local and global planning strategies.

### Trajectory Estimation and Information Transfer
- Enable aerial vehicles (AVs) to communicate moving object trajectory estimation (MOTE) data to ground vehicles (GVs).
- Allow GVs to dynamically replan their routes based on real-time data about obstacles, objects of interest, and potential threats from non-fleet entities.

## Installation

```bash
# Clone the repository
git clone git@github.com:isabelmoore/mote_ros.git
# Navigate to the project directory
cd mote_ros


# [Add additional setup/installation commands here]

# 1. Create the Conda Environment
conda env create -f environment.yml

# 2. Activate the Environment
conda activate mote_ros


## Localization & Drift Correction

This project tackles the problem of **Yaw Drift** in dead-reckoning navigation. Low-cost IMUs often introduce a constant or slowly varying bias to the angular velocity reading, causing the estimated trajectory to curl away from the true path.

We implement two advanced sensor fusion approaches to estimate and cancel this bias in real-time.

### 1. Extended Kalman Filter (EKF)
The EKF treats the system as a state estimation problem where **Yaw Bias** is a hidden variable.
- **State Vector**: `[x, y, velocity, yaw, yaw_bias]`
- **Prediction**: The model predicts angular change using `(omega_measured - yaw_bias)`.
- **Update**: When position measurements (e.g., GPS) don't match the prediction, the filter adjusts the `yaw_bias` to minimize the error.

**Run the Demo:**
```bash
python3 ekf_localization.py
```

### 2. Particle Filter (Monte Carlo Localization)
To run independently:
```bash 
docker-compose run --rm mote_ros /bin/bash -c "python3 src/AGC-MOTE-Monte-Carlo/scripts/analyze_bag_data.py && python3 src/AGC-MOTE-Monte-Carlo/scripts/animate_trajectory.py"
```



To run the bag file analysis:
```bash
docker-compose run --rm mote_ros /bin/bash -c "source devel/setup.bash && roslaunch mote_ros run_both_nodes.launch"
```


To run the visualization node:
```bash
docker-compose run --rm mote_ros /bin/bash -c "source devel/setup.bash && roslaunch mote_ros rviz.launch"
```