---
id: chapter6
title: ROS 2 Navigation Stack Fundamentals
sidebar_position: 6
---

## Learning Objectives

*   Understand the purpose and high-level architecture of the ROS 2 Navigation Stack (Nav2).
*   Learn about the key components of Nav2, including the planner, controller, and recovery behaviors.
*   Configure and launch Nav2 for a simulated robot.

## Key Concepts

*   **Nav2**: The second generation of the ROS Navigation Stack, designed for autonomous navigation.
*   **Global Planner**: Computes a long-term path from the robot's current position to a goal.
*   **Local Planner**: Computes short-term velocity commands to follow the global path while avoiding obstacles.
*   **Costmap**: A 2D or 3D map representing the environment, used for navigation.
*   **AMCL (Adaptive Monte Carlo Localization)**: A probabilistic localization system for a robot moving in 2D.
*   **Behavior Trees**: Used in Nav2 to orchestrate the complex logic of navigation.

## Practical Tasks

1.  **Install Nav2**: Install the ROS 2 Navigation Stack packages.
2.  **Configure Nav2**: Create a `nav2_params.yaml` file to configure the planners, controllers, and other Nav2 components.
3.  **Create a Launch File**: Write a ROS 2 launch file to bring up the entire navigation system, including AMCL, Nav2, and the robot model.
4.  **Send a Navigation Goal**: Use RViz or a simple Python script to send a navigation goal to Nav2 and watch the robot navigate.

## Diagrams

*   **Navigation Stack Components**: A diagram showing the main components of Nav2 and how they interact.
*   **Global vs. Local Planning**: A diagram illustrating the difference between the global path and the local path adjustments.

## Code Templates

### Nav2 Configuration Snippet (`nav2_params.yaml`)

```yaml
# examples/module1/chapter6-nav2-fundamentals/nav2_params.yaml

amcl:
  ros__parameters:
    use_sim_time: True
    alpha1: 0.2
    alpha2: 0.2
    # ... other AMCL parameters

bt_navigator:
  ros__parameters:
    use_sim_time: True
    bt_xml_filename: "navigate_w_replanning_and_recovery.xml"

# ... other Nav2 component configurations
```
