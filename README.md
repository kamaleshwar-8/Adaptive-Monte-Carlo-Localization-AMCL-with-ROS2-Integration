# Adaptive Monte Carlo Localization (AMCL) with ROS2 Integration

## Overview
This MATLAB script implements Adaptive Monte Carlo Localization using particle filters with ROS2 integration for robot localization.

## Features
- Adaptive particle count
- ROS2 integration
- Real-time visualization
- Landmark-based localization

## Parameters
- Initial particles: 100
- Time step: 0.1 seconds
- Particle limits: 100-1000
- Linear velocity: 1 m/s
- Angular velocity: 0.1 rad/s
- Process noise (Q)
- Measurement noise (R)

## Components
### Particle Filter
- Prediction step
- Measurement update
- Adaptive resampling
- Particle spread monitoring

### ROS2 Integration
- Node: "/matlab_node"
- Publisher: '/drone/position'
- Message type: geometry_msgs/Point

## Algorithm Steps
1. Particle prediction
2. Weight update based on landmarks
3. Resampling
4. Adaptive particle count adjustment
5. Position estimation and publishing

## Visualization
- Real-time particle display
- Landmark positions
- Estimated position
- Bounded visualization area

## Usage
1. Configure ROS2 environment
2. Set desired parameters
3. Run script
4. Monitor localization performance

## Dependencies
- MATLAB ROS2 Toolbox
- Statistics and Machine Learning Toolbox

## Output


https://github.com/user-attachments/assets/303e20df-59dc-402f-ae6a-7d5ed9b7134b

