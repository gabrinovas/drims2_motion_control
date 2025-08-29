# DRIMS2 Motion Control

This repository is part of the **DRIMS2 Summer School** and provides a motion control stack based on ROS 2 and MoveIt. It includes behavior tree integration, motion command interfaces, and an action server for handling motion requests.

## Repository Structure

This repository contains the following ROS 2 packages:

- **`drims2_msgs`**  
  Contains the ROS 2 interface definitions for motion commands. Key interfaces include:
  - Actions:
    - `MoveToPose`
    - `MoveToJoint`
  - Services:
    - `AttachObject`
    - `DetachObject`
    - `DiceIdentification`


- **`drims2_motion_server`**  
  Implements the motion **action/service server** that:
  - Receives `MoveToPose` and `MoveToJoint` requests.
  - Handles `AttachObject` and `DetachObject` service calls.
  - Interfaces with **MoveIt** to plan and execute robot motions.

  🔗 [Documentation](https://cnr-stiima-iras.github.io/drims2_motion_control/index.html)

- **`drims2_behavior_tree`**  
  Integrates a **Behavior Tree engine** to control task execution. This package contains:
  - Custom **leaf nodes** for motion commands.
  - Logic for behavior tree loading and execution.

## Getting Started

### Clone the Repository

```bash
mkdir -p ~/projects/drims2_ws/src
cd ~/projects/drims2_ws/src
git clone https://github.com/CNR-STIIMA-IRAS/drims2_motion_control.git
vcs import < drims2_motion_control/dependencies.repos
```
