# AWS DeepRacer Joystick Readme

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
