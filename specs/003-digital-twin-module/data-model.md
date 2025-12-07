# Data Model: Module 2: The Digital Twin (Gazebo & Unity)

This document outlines the key data entities for Module 2.

## Entities

### DigitalTwin

Represents the core concept of a digital twin.

-   **Attributes**:
    -   `name`: The name of the digital twin.
    -   `description`: A brief description of the digital twin.
-   **Relationships**:
    -   Has one `RobotModel`.
    -   Runs in one or more `SimulationEnvironment`s.

### SimulationEnvironment

Represents the simulation environment.

-   **Attributes**:
    -   `name`: The name of the environment (e.g., "Gazebo", "Unity").
    -   `version`: The version of the simulation environment.
-   **Relationships**:
    -   Contains multiple `RobotModel`s.

### RobotModel

Represents the robot model.

-   **Attributes**:
    -   `name`: The name of the robot model.
    -   `format`: The format of the model file (e.g., "SDF", "URDF").
    -   `filePath`: The path to the model file.
-   **Relationships**:
    -   Has many `Sensor`s.
    -   Has `PhysicsProperties`.

### Sensor

Represents a sensor on the robot.

-   **Attributes**:
    -   `name`: The name of the sensor.
    -   `type`: The type of sensor (e.g., "LiDAR", "Depth Camera", "IMU").
    -   `noiseModel`: The noise model for the sensor.
-   **Relationships**:
    -   Belongs to a `RobotModel`.

### PhysicsProperties

Represents the physics properties of a robot model.

-   **Attributes**:
    -   `gravity`: The gravity vector.
    -   `friction`: The friction coefficient.
    -   `inertia`: The inertia tensor.
-   **Relationships**:
    -   Belongs to a `RobotModel`.
