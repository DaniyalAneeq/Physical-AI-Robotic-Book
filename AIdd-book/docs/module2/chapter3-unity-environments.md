---
id: chapter3-unity-environments
title: 'Environment Building in Unity: Scenes and Robots'
sidebar_position: 3
---

## Learning Objectives

-   Set up a new Unity project specifically for robotics simulation using the 3D (URP) template.
-   Install and configure the Unity Robotics Hub packages.
-   Understand the structure of a Unity Scene and the role of GameObjects and Components.
-   Import a robot from a URDF file and correctly configure its physics using Articulation Bodies.
-   Create a simple C# script to interact with the simulated environment.

## Key Concepts

-   **Unity Editor**: The primary interface for creating and managing all aspects of a Unity project, from scenes and assets to code and physics.
-   **Scene**: A container for all the GameObjects that make up a level or a simulation environment.
-   **GameObject**: The fundamental object in Unity. Everything in a scene, from a robot link to a light source, is a GameObject.
-   **Component**: GameObjects are containers. Their functionality is defined by the Components attached to them, such as `Transform` (position/rotation), `MeshRenderer` (visual appearance), or `ArticulationBody` (physics).
-   **Universal Render Pipeline (URP)**: A prebuilt Scriptable Render Pipeline by Unity. It provides a good balance between performance and high-quality, customizable graphics.
-   **Unity Robotics Hub**: A collection of official Unity packages that provide tools and utilities for robotics simulation, including the URDF Importer and ROS-TCP-Connector for ROS 2 communication.
-   **Articulation Body**: A specialized physics component designed for creating realistic joints and articulations, which is ideal for robotics. It provides more stable and accurate physics for chain-like structures than standard Rigidbody components.

## Deep Theoretical Explanation

### The GameObject-Component Model

Unity's architecture is built on the GameObject-Component model. This is a powerful design pattern where GameObjects are simple entities, and their behavior is defined by a collection of modular Components. For example, a robot link would be a GameObject with the following components:
-   `Transform`: Defines its position, rotation, and scale in the scene.
-   `MeshFilter`: Holds the 3D mesh data for the link's visual appearance.
-   `MeshRenderer`: Renders the mesh using a specific material (color/texture).
-   `ArticulationBody`: Defines its physics properties and its connection to a parent link.

This approach is highly flexible and allows for complex behaviors to be built by combining simple, reusable components.

### URDF Importer and Articulation Bodies

When you import a URDF file using the Unity Robotics Hub, the importer automatically performs several crucial steps:
1.  **Parsing**: It reads the URDF file and understands the link and joint hierarchy.
2.  **GameObject Creation**: It creates a hierarchy of GameObjects in the Unity scene to match the URDF's `<link>` structure.
3.  **Articulation Body Setup**: For each non-fixed joint, it adds an `ArticulationBody` component to the corresponding GameObject and configures it based on the joint type (`revolute`, `prismatic`, etc.), limits, and dynamics from the URDF. The root link of the articulation chain is also given an `ArticulationBody` component, but it's configured as the root.

This automatic setup provides a robust and physically accurate representation of the robot, ready for simulation and control.

## Practical Tasks

### 1. Setting up the Unity Project

1.  Open the Unity Hub and create a new project.
2.  Select the **3D (URP)** template. This provides a good starting point with modern rendering features.
3.  Give your project a name (e.g., `RoboticsSimulations`) and click "Create Project".

### 2. Installing the Unity Robotics Hub

1.  In the Unity Editor, go to `Window > Package Manager`.
2.  Click the `+` icon in the top-left and select "Add package from git URL...".
3.  Enter the following URL to install the ROS TCP Connector: `com.unity.robotics.ros-tcp-connector`
4.  Repeat the process to install the URDF Importer: `com.unity.robotics.urdf-importer`

### 3. Importing a URDF

1.  Create an `Assets/URDF` folder in your project.
2.  Copy your robot's URDF file and all its associated mesh files into this new folder.
3.  Right-click the URDF file in the Unity Project window and select "Import Robot from URDF".
4.  A configuration window will appear. You can usually leave the settings as default and click "Import".
5.  Drag the newly created robot prefab from the `Assets` folder into your scene.

## Diagrams

*   **Unity Scene Hierarchy**: A screenshot or diagram showing the hierarchy of GameObjects for an imported robot in the Unity Hierarchy window. It will show the parent-child relationships of the links.
*   **Component-Based Architecture**: A diagram illustrating a single GameObject ("RobotLink") with several components attached to it, such as `Transform`, `MeshRenderer`, and `ArticulationBody`, with lines pointing to each component explaining its function.

## Code Templates

### C# Script for Basic Environment Interaction

This script can be attached to any GameObject in the scene (e.g., a "GameController" object) to allow simple interaction with the environment.

```csharp
// examples/module2/chapter3-unity-environments/EnvironmentController.cs
using UnityEngine;

public class EnvironmentController : MonoBehaviour
{
    // Drag your scene's Directional Light here in the Inspector
    public Light directionalLight;

    void Update()
    {
        // Toggle the main light on and off when the 'L' key is pressed.
        if (Input.GetKeyDown(KeyCode.L))
        {
            if (directionalLight != null)
            {
                directionalLight.enabled = !directionalLight.enabled;
            }
        }
    }
}
```