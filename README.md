# ROS2-car (WIP) (0% Complete)

[README_ELI5](./README_ELI5.md)

## Goal

- Simultaneous Localization and Mapping (SLAM)
- Autonomous drifting capabilities

## Hardware Components

- **Desktop with ROS2**
  - Image recognition
  - SLAM
  - Planning
  - Miscellaneous computationally heavy processing
- **Raspberry Pi for Remote Control**
  - Remote control operation
  - Sensor to ROS publishing
  - Low-level, high-frequency control (VDC) (could be offloaded to ESP32)
- **Vehicle**
  - TT02 chassis for the autonomous car
  - Battery for car
  - Power bank for powering onboard electronics

## Targets

- Validation in simulation
  - Simulation for an AWD vehicle with a differential and drifting capabilities.
- Control development
  - PID Control
  - Machine Learning
- Deployment IRL
  - TT02 chassis

## Contributing

We welcome contributions from the community! If you have suggestions, improvements, or bug fixes, please feel free to submit a pull request or open an issue.
