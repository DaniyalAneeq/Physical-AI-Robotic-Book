# Research for Module 4: Vision-Language-Action (VLA)

This document addresses the "NEEDS CLARIFICATION" items identified in the `plan.md`.

## 1. Plan Refinement in VLA Pipelines

**Decision**: The plan refinement will be handled by a feedback loop where the state of the environment, as perceived by the vision system, is fed back into the LLM planner.

**Rationale**:
The initial plan is generated based on the initial voice command. However, the environment is dynamic.
- **Continuous Perception**: The robot's vision system will continuously monitor the environment to detect changes, such as object locations.
- **State Update**: When a discrepancy is detected between the expected state and the actual state, the robot's world model is updated.
- **Re-planning**: The updated world model and the original goal are fed back to the LLM planner to generate a new, refined plan. For example, if an object is not at the expected location, the robot can re-prompt the LLM with "The object is not at the expected location. The current view shows [new view description]. Generate a new plan to find the object."

**Alternatives considered**:
- **Hardcoded recovery behaviors**: For example, if an object is not found, perform a pre-defined search pattern. This is less flexible than using the LLM for re-planning and doesn't leverage the cognitive capabilities of the VLA model.

## 2. Testing Framework

**Decision**: `pytest` will be used as the testing framework for the Python code.

**Rationale**:
- **Industry Standard**: `pytest` is a widely adopted, mature, and feature-rich testing framework for Python.
- **Simplicity**: It has a simple and clean syntax for writing tests.
- **Extensibility**: `pytest` has a rich ecosystem of plugins, including `pytest-ros` for ROS 2 integration testing, which will be valuable for this project.
- **Good Fit**: It is well-suited for testing the individual Python scripts and the ROS 2 nodes that will be developed for this module.

**Alternatives considered**:
- **unittest**: This is part of the Python standard library, but `pytest` offers a more concise syntax and more powerful features.
- **Nose2**: Another popular testing framework, but `pytest` has a larger community and more extensive plugin support.
