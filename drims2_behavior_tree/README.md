# Drims2 Behavior Tree

The `drims2_behavior_tree` package provides custom Behavior Tree (BT) nodes for controlling robot motions in the **DRIMS2 Motion Control** stack.  
It builds upon the BehaviorTree.ROS2/.CPP library and ROS 2 action interfaces to enable modular and flexible behavior orchestration.

## Overview

This package integrates motion-related actions into behavior trees, offering ready-to-use leaf nodes that interact with ROS 2 action servers such as motion planners and gripper controllers.

Each node can be loaded as a plugin and used in XML-based behavior tree definitions.

## Available Behavior Tree Nodes

The following custom BT **action nodes** are available:

| Node Name         | Input Ports                                                                 | Description                                                |
|-------------------|------------------------------------------------------------------------------|------------------------------------------------------------|
| `GripperCommand`  | `position` (double), `max_effort` (double)                                  | Sends a gripper command with target position and max effort. |
| `MoveToJoint`     | `joint_target` (vector\<double>)                                             | Moves the robot to a specified joint configuration.         |
| `MoveToPose`      | `pose_target` (PoseStamped) **or**<br>`frame_id` (string), `position` (vec3), `orientation` (vec4) | Moves the robot to a target pose. Accepts a direct pose_target as `PoseStamped.msg` or decomposed components. |
