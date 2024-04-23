# AWS DeepRacer Joystick 

## Overview

This repository contains the necessary components to enable joystick controls for AWS DeepRacer using a custom joystick package. The project includes modifications to ROS nodes and specific configurations for the Logitech Dual Action controller.

## Getting Started

This package consists of three main parts:

1. **node.py** - Contains the code necessary to interface with the official joy_node provided by ROS.
2. **logitech_dual_action.yaml** - Specifies the joystick configuration including type, topic names, and axes for the teleoperation and deadman buttons.
3. **setup.py** and **package.xml** - Include the standard setup and configuration files for the AWS DeepRacer package.

### Prerequisites

Here's how to use the `deepracer_joy` package:

1. **Stop the core service:**
   ```bash
   systemctl stop deepracer-core
   ```

2. **Restart the core service:**
   ```bash
   systemctl restart deepracer-core
   ```

3. **Source the necessary environment scripts:**
   ```bash
   source /opt/ros/foxy/setup.bash
   source /opt/intel/openvino_2021/bin/setupvars.sh
   source ~/deepracer_ws/install/setup.bash
   ```

4. **Run the official Joy node:**
   ```bash
   ros2 run joy joy_node
   ```

5. **In another terminal, repeat steps 1-3 and then run the custom joystick node:**
   ```bash
   ros2 run deepracer_joy_node.py
   ```

## Structure

The package structure is outlined as follows:

- **deepracer_joy/**
  - **config/**
    - logitech_dual_action.yaml
  - **deepracer_joy/**
    - __init__.py
    - deepracer_joy.py
  - package.xml
  - setup.py

---

For further information and updates, please refer to the official AWS DeepRacer documentation and ROS community resources.
```

This markdown file is ready to be used in your GitHub repository. It's structured to provide a clear overview and easy navigation for users looking to integrate joystick controls into their AWS DeepRacer projects.
