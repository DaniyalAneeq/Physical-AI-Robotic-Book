# Feature Specification: Module 4: Vision-Language-Action (VLA)

**Feature Branch**: `001-vla-module`
**Created**: 2025-12-06
**Status**: Draft
**Input**: User description: "You are generating the spec.md for Module 4 of my Spec-Kit Plus project. The constitution and previous module specifications are already finalized, so maintain strict structural consistency with earlier modules. Create a complete, hierarchical, production-ready Module 4 specification for the Docusaurus-based textbook “Physical AI & Humanoid Robotics”. This specification must follow precisely the same structure, formatting, and level of detail used in Module 1, Module 2, and Module 3. Module Title Module 4: Vision-Language-Action (VLA) Scope This module covers: Convergence of LLMs and Robotics Voice-to-Action pipelines using OpenAI Whisper Vision-Language-Action reasoning Cognitive planning: transforming natural-language instructions into ROS 2 action sequences Grounded perception using multimodal models Integration with previous modules (ROS 2 → Digital Twin → AI Brain → VLA Layer) Your Required Output (same structure as previous modules): 1. Full Module Structure Provide at least 5 fully developed chapters. Each chapter must include: Learning objectives Key concepts Practical hands-on tasks Diagrams (described only) Sample code templates, including: Whisper API usage ROS 2 nodes for voice command pipelines LLM prompting for action planning ROS 2 action graphs from VLA reasoning Basic multimodal perception integration Make sure chapters build progressively: Fundamentals of VLA Whisper voice input pipeline LLM-based cognitive planning Vision-grounded action understanding Integration pipeline for a humanoid robot 2. Content Requirements For each chapter, include technically accurate, verifiable content related to: Voice command recognition with Whisper Prompt engineering for robotic action planning Translating “Clean the room” → structured ROS 2 actions Vision-grounded object recognition LLM-based sequence generation and task decomposition Safety constraints and execution monitoring ROS 2 integration for humanoid behaviors Include code snippets using: Python rclpy Whisper API LLM-based planners ROS 2 action servers and clients 3. Docusaurus Mapping Generate deterministic folder paths under: AIdd-book/docs/module-4/ Example (mandatory): AIdd-book/docs/module-4/01-introduction-to-vla.md AIdd-book/docs/module-4/02-whisper-voice-to-action.md AIdd-book/docs/module-4/03-llm-cognitive-planning.md AIdd-book/docs/module-4/04-vision-grounded-action.md AIdd-book/docs/module-4/05-integrated-vla-humanoid-pipeline.md Match the naming style of Modules 1–3. 4. Module-Level Capstone Definition Define the module-specific final project leading toward the book-wide “Autonomous Humanoid” capstone. Describe a project where the humanoid robot: Receives a voice command (Whisper) Uses an LLM planner to break it into actions Performs vision-based object identification Uses ROS 2 for navigation, manipulation, and execution Runs end-to-end inside the digital twin environment Specify: Objectives, inputs, outputs Evaluation criteria Required datasets or simulated environments 5. Accuracy Requirements Use only verifiable, publicly documented APIs for Whisper, ROS 2, and LLM reasoning. Maintain the deterministic structure required for /sp.plan → /sp.tasks → /sp.implement. Follow the identical formal style of earlier specs (critical). Produce the final result as a clean, well-structured spec.md for Module 4."

## Module Title

Module 4: Vision-Language-Action (VLA)

## Scope

This module covers:

*   Convergence of LLMs and Robotics
*   Voice-to-Action pipelines using OpenAI Whisper
*   Vision-Language-Action reasoning
*   Cognitive planning: transforming natural-language instructions into ROS 2 action sequences
*   Grounded perception using multimodal models
*   Integration with previous modules (ROS 2 → Digital Twin → AI Brain → VLA Layer)

## Full Module Structure

This module comprises the following chapters, building progressively from fundamentals to integrated systems:

### Chapter 1: Introduction to Vision-Language-Action (VLA)

*   **Learning objectives**: Understand the core concepts of VLA, its significance in modern robotics, and the convergence of LLMs with robotic systems.
*   **Key concepts**: VLA definition, history, current challenges, LLMs in robotics, multimodal perception.
*   **Practical hands-on tasks**: Setting up a basic Python environment for VLA experiments.
*   **Diagrams**: Conceptual diagram of a VLA pipeline in robotics.
*   **Sample code templates**:
    ```python
    # Basic Python setup for VLA
    import os
    print("VLA environment ready.")
    ```
*   **Docusaurus Mapping**: `AIdd-book/docs/module-4/01-introduction-to-vla.md`

### Chapter 2: Whisper Voice-to-Action Pipeline

*   **Learning objectives**: Implement a voice command recognition system using OpenAI Whisper and integrate it into a ROS 2 framework for basic robotic actions.
*   **Key concepts**: OpenAI Whisper API, speech-to-text processing, ROS 2 `rclpy` publishers/subscribers, voice command parsing.
*   **Practical hands-on tasks**: Creating a ROS 2 node that listens for voice commands via Whisper and publishes corresponding simple action commands.
*   **Diagrams**: Data flow diagram for Whisper integration into ROS 2.
*   **Sample code templates**:
    ```python
    # ROS 2 node for Whisper voice command processing
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    # Assuming openai and audio processing libraries are installed
    # import openai
    # import sounddevice as sd

    class WhisperVoiceCommandNode(Node):
        def __init__(self):
            super().__init__('whisper_voice_command_node')
            self.publisher_ = self.create_publisher(String, 'voice_commands', 10)
            self.get_logger().info('Whisper Voice Command Node started.')

        def process_audio_and_publish(self, audio_data):
            # Placeholder for Whisper API call
            # response = openai.Audio.transcribe("whisper-1", audio_data)
            # command_text = response["text"]
            command_text = "move forward" # Simulated command
            msg = String()
            msg.data = command_text
            self.publisher_.publish(msg)
            self.get_logger().info(f'Published: "{command_text}"')

    def main(args=None):
        rclpy.init(args=args)
        node = WhisperVoiceCommandNode()
        # In a real scenario, audio would be continuously captured and processed
        node.process_audio_and_publish(None)
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```
*   **Docusaurus Mapping**: `AIdd-book/docs/module-4/02-whisper-voice-to-action.md`

### Chapter 3: LLM-based Cognitive Planning for Robotics

*   **Learning objectives**: Develop strategies for using LLMs to transform natural language instructions into structured, executable ROS 2 action sequences for robots.
*   **Key concepts**: Prompt engineering for task decomposition, action graph generation, LLM as a planner, state representation, PDDL (brief intro for context).
*   **Practical hands-on tasks**: Designing prompts to break down complex commands ("Clean the room") into sequential ROS 2 actions; implementing a Python script to interact with an LLM API and generate action plans.
*   **Diagrams**: Flowchart of LLM-based cognitive planning process.
*   **Sample code templates**:
    ```python
    # LLM prompting for action planning
    import json
    # Assuming an LLM API client is available
    # from llm_api_client import LLMClient

    def get_llm_action_plan(natural_language_instruction):
        # Placeholder for LLM API call
        # client = LLMClient()
        # prompt = f"Given the instruction '{natural_language_instruction}', generate a sequence of ROS 2 actions in JSON format. Each action should have 'name' and 'parameters'. Example: {{'actions': [{{'name': 'navigate', 'parameters': {{'target_location': 'kitchen'}}}}]}}"
        # response = client.generate(prompt)
        # return json.loads(response.text)

        # Simulated LLM response
        if "clean the room" in natural_language_instruction.lower():
            return {
                "actions": [
                    {"name": "navigate_to_location", "parameters": {"location": "living_room"}},
                    {"name": "identify_objects", "parameters": {"objects": ["trash", "clutter"]}},
                    {"name": "pick_up_object", "parameters": {"object_type": "trash"}},
                    {"name": "deposit_object", "parameters": {"container": "trash_bin"}},
                    {"name": "pick_up_object", "parameters": {"object_type": "clutter"}},
                    {"name": "place_object", "parameters": {"location": "shelf"}},
                    {"name": "navigate_to_location", "parameters": {"location": "bedroom"}},
                    # ... more actions
                ]
            }
        return {"actions": []}

    if __name__ == '__main__':
        instruction = "Clean the room by picking up trash and putting away clutter."
        plan = get_llm_action_plan(instruction)
        print(json.dumps(plan, indent=2))
    ```
    ```python
    # ROS 2 action graph from VLA reasoning
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from action_interfaces.action import Navigate, PickObject # Custom action interfaces

    class RobotActionExecutor(Node):
        def __init__(self):
            super().__init__('robot_action_executor')
            self.nav_action_client = ActionClient(self, Navigate, 'navigate')
            self.pick_action_client = ActionClient(self, PickObject, 'pick_object')
            self.get_logger().info('Robot Action Executor started.')

        def execute_plan(self, action_plan):
            for action in action_plan["actions"]:
                self.get_logger().info(f"Executing action: {action['name']} with params {action['parameters']}")
                if action["name"] == "navigate_to_location":
                    goal_msg = Navigate.Goal()
                    goal_msg.location = action["parameters"]["location"]
                    self.nav_action_client.wait_for_server()
                    future = self.nav_action_client.send_goal_async(goal_msg)
                    rclpy.spin_until_future_complete(self, future)
                    result = future.result().get_result()
                    self.get_logger().info(f"Navigation result: {result.success}")
                # Add logic for other actions like pick_up_object, etc.

    def main(args=None):
        rclpy.init(args=args)
        node = RobotActionExecutor()
        # Example plan from LLM
        example_plan = {
            "actions": [
                {"name": "navigate_to_location", "parameters": {"location": "living_room"}},
                # ... more actions
            ]
        }
        node.execute_plan(example_plan)
        node.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```
*   **Docusaurus Mapping**: `AIdd-book/docs/module-4/03-llm-cognitive-planning.md`

### Chapter 4: Vision-Grounded Action Understanding

*   **Learning objectives**: Integrate multimodal perception models for grounded object recognition and contextual understanding, enabling robots to act based on visual information.
*   **Key concepts**: Multimodal LLMs, vision transformers, object detection APIs (e.g., OpenCV with pre-trained models, specific LLM-vision integration), semantic mapping, grounding language to visual entities.
*   **Practical hands-on tasks**: Using a simple vision model (e.g., pre-trained YOLO or a basic LLM vision API) to identify objects in a simulated environment; integrating visual feedback into the LLM planning loop.
*   **Diagrams**: Architecture for vision-grounded action reasoning.
*   **Sample code templates**:
    ```python
    # Basic multimodal perception integration
    import cv2
    import numpy as np
    # Assuming a pre-trained object detection model (e.g., via OpenCV's DNN module)
    # or an LLM vision API client

    def detect_objects_with_vision(image_path):
        image = cv2.imread(image_path)
        if image is None:
            print(f"Error: Could not load image from {image_path}")
            return []

        # Placeholder for object detection using OpenCV or LLM Vision API
        # For simplicity, simulate detection
        detected_objects = [
            {"label": "cup", "bbox": (100, 100, 50, 70)}, # x, y, width, height
            {"label": "book", "bbox": (250, 200, 80, 120)},
        ]
        return detected_objects

    if __name__ == '__main__':
        # Example usage with a dummy image
        dummy_image_path = "path/to/simulated_robot_view.png" # Replace with actual path
        # Create a dummy image for testing
        dummy_image = np.zeros((400, 600, 3), dtype=np.uint8)
        cv2.imwrite(dummy_image_path, dummy_image)

        objects = detect_objects_with_vision(dummy_image_path)
        print("Detected objects:", objects)
    ```
*   **Docusaurus Mapping**: `AIdd-book/docs/module-4/04-vision-grounded-action.md`

### Chapter 5: Integrated VLA Humanoid Pipeline

*   **Learning objectives**: Build a complete end-to-end VLA pipeline for a humanoid robot, integrating voice input, cognitive planning, vision-grounded perception, and ROS 2 execution.
*   **Key concepts**: System integration, error handling, safety constraints, execution monitoring, digital twin environment (Gazebo/Isaac Sim), full-stack robotic control.
*   **Practical hands-on tasks**: Developing a comprehensive Python script that orchestrates the entire VLA workflow in a simulated humanoid robot environment.
*   **Diagrams**: Full VLA architecture for a humanoid robot.
*   **Sample code templates**:
    ```python
    # Integrated VLA pipeline for a humanoid robot
    # Orchestrates Whisper, LLM planning, Vision, and ROS 2 actions

    import rclpy
    from whisper_ros_node import WhisperVoiceCommandNode
    from llm_planner import get_llm_action_plan
    from vision_grounding import detect_objects_with_vision
    from robot_action_executor import RobotActionExecutor
    # from digital_twin_interface import DigitalTwinInterface # For interacting with Gazebo/Isaac Sim

    def main_vla_pipeline(args=None):
        rclpy.init(args=args)

        # Initialize components
        whisper_node = WhisperVoiceCommandNode()
        robot_executor_node = RobotActionExecutor()
        # digital_twin = DigitalTwinInterface()

        # Simulate voice command
        voice_command = "Go to the kitchen and pick up the red apple."
        print(f"Received voice command: {voice_command}")

        # LLM Cognitive Planning
        action_plan = get_llm_action_plan(voice_command)
        print(f"Generated action plan: {action_plan}")

        # Vision-Grounded Perception (simulated)
        # image_from_robot_cam = digital_twin.get_camer-image()
        detected_objects = detect_objects_with_vision("path/to/simulated_robot_view.png") # Placeholder
        print(f"Detected objects: {detected_objects}")

        # Refine plan based on vision (e.g., confirm object location) - [NEEDS CLARIFICATION: how plan refinement occurs]

        # Execute actions
        robot_executor_node.execute_plan(action_plan)

        whisper_node.destroy_node()
        robot_executor_node.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main_vla_pipeline()
    ```
*   **Docusaurus Mapping**: `AIdd-book/docs/module-4/05-integrated-vla-humanoid-pipeline.md`

## Content Requirements

Each chapter will include technically accurate, verifiable content related to:

*   **Voice command recognition with Whisper**: Detailed explanation of Whisper API usage, audio input processing, and transcription accuracy considerations.
*   **Prompt engineering for robotic action planning**: Best practices for designing LLM prompts to elicit structured, reliable action sequences, including zero-shot and few-shot examples.
*   **Translating “Clean the room” → structured ROS 2 actions**: Step-by-step methodology for converting high-level natural language goals into a series of granular, executable ROS 2 actions, considering state, pre-conditions, and post-conditions.
*   **Vision-grounded object recognition**: Integration of visual perception outputs (e.g., object bounding boxes, semantic segmentation) with LLM reasoning to "ground" language commands to physical entities.
*   **LLM-based sequence generation and task decomposition**: Techniques for LLMs to break down complex tasks into sub-tasks and generate appropriate action sequences.
*   **Safety constraints and execution monitoring**: Strategies for incorporating safety protocols into LLM-generated plans and monitoring robot execution for deviations or unsafe states.
*   **ROS 2 integration for humanoid behaviors**: Practical examples of using `rclpy` to interface with ROS 2 action servers and clients for navigation, manipulation, and other humanoid robot behaviors.

Code snippets will utilize: Python, `rclpy`, Whisper API, LLM-based planners, ROS 2 action servers and clients.

## Docusaurus Mapping

All module content will be mapped to deterministic folder paths under `AIdd-book/docs/module-4/` as follows:

*   `AIdd-book/docs/module-4/01-introduction-to-vla.md`
*   `AIdd-book/docs/module-4/02-whisper-voice-to-action.md`
*   `AIdd-book/docs/module-4/03-llm-cognitive-planning.md`
*   `AIdd-book/docs/module-4/04-vision-grounded-action.md`
*   `AIdd-book/docs/module-4/05-integrated-vla-humanoid-pipeline.md`

## Module-Level Capstone Definition: Autonomous Humanoid VLA Agent

This module's capstone project focuses on building an autonomous humanoid robot agent that integrates Vision-Language-Action capabilities. This project directly contributes to the book-wide "Autonomous Humanoid" capstone.

**Project Description**:
The humanoid robot is tasked with performing a series of household chores or maintenance tasks within a simulated environment (e.g., a virtual apartment in Gazebo or Isaac Sim). The robot will receive high-level voice commands from a human operator, process these commands using an LLM for cognitive planning, utilize its vision system to identify and locate objects, and execute the necessary navigation, manipulation, and interaction sequences via ROS 2.

**Objectives**:

*   Develop a robust voice-to-action interface using OpenAI Whisper.
*   Implement an LLM-based planner capable of decomposing complex natural language instructions into a sequence of ROS 2 actions.
*   Integrate a vision system for grounded object recognition and localization within the simulated environment.
*   Orchestrate the entire VLA pipeline for end-to-end task execution by a humanoid robot.
*   Demonstrate safe and effective interaction with the environment based on verbal instructions and visual feedback.

**Inputs**:

*   Natural language voice commands (e.g., "Please tidy up the living room," "Fetch me the blue book from the shelf").
*   Simulated environment sensor data (camera feeds, depth sensors, proprioceptive feedback).
*   Pre-defined ROS 2 action interfaces for humanoid navigation, manipulation, and object interaction.

**Outputs**:

*   Successful execution of multi-step tasks by the simulated humanoid robot.
*   Log of executed ROS 2 actions and state changes.
*   Visualizations of robot actions and perceived environment within the digital twin.
*   Feedback to the user (simulated) on task progress or completion.

**Evaluation Criteria**:

*   **Task Completion Rate**: Percentage of voice commands successfully executed from start to finish.
*   **Action Plan Fidelity**: Accuracy of LLM-generated action sequences in mapping to the intended natural language command.
*   **Grounded Perception Accuracy**: Precision and recall of object identification and localization.
*   **Robustness to Ambiguity**: Robot's ability to handle slightly ambiguous commands or environmental variations (qualitative assessment).
*   **Safety Compliance**: Adherence to defined safety zones and avoidance of collisions within the simulated environment.
*   **Efficiency**: Time taken to complete tasks from command reception to final execution.

**Required Datasets or Simulated Environments**:

*   **Simulated Environment**: Gazebo or NVIDIA Isaac Sim, configured with a humanoid robot model (e.g., a modified URDF/SDF model) and a structured indoor environment (e.g., an apartment scene with various objects).
*   **Object Datasets**: A small dataset of common household objects (e.g., cups, books, remotes) with 3D models and corresponding textures for simulation, and potentially annotated images for training/testing vision models if not using purely pre-trained models.
*   **Pre-trained Models**: OpenAI Whisper ASR model, a suitable LLM (e.g., GPT-4, Claude 3, or a fine-tuned open-source model), and a pre-trained object detection model (e.g., YOLO, DETR).

## Accuracy Requirements

*   All content will reference verifiable, publicly documented APIs for Whisper, ROS 2, and LLM reasoning.
*   The specification will maintain the deterministic structure required for the subsequent `/sp.plan` → `/sp.tasks` → `/sp.implement` workflow.
*   The style and level of formality will be identical to Modules 1–3 of the textbook.

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Voice-Controlled Object Retrieval (Priority: P1)

A student, acting as an operator, verbally instructs the simulated humanoid robot to fetch a specific object (e.g., "Robot, bring me the blue cup from the table."). The robot processes the voice command, plans the navigation and manipulation sequence, uses its vision to locate the object, navigates to it, picks it up, and brings it to the operator's designated return location.

**Why this priority**: This is a fundamental VLA task demonstrating core integration of all module components: voice input, LLM planning, vision grounding, and ROS 2 execution.

**Independent Test**: The student can fully test this by issuing a voice command, observing the robot's actions in the simulator, and verifying that the correct object is retrieved. This delivers immediate, tangible value in showcasing VLA capabilities.

**Acceptance Scenarios**:

1.  **Given** the robot is in a known starting position and the target object is visible/locatable in the environment, **When** the operator issues a clear voice command to fetch it, **Then** the robot accurately identifies the command, generates a valid plan, navigates to the object, picks it up, and returns it to the specified drop-off point.
2.  **Given** the robot receives an ambiguous object description (e.g., "fetch the book" when multiple books are present), **When** the LLM planner identifies the ambiguity, **Then** the robot (or a simulated prompt) asks for clarification (e.g., "Which book? The red one or the large one?").

---

### User Story 2 - Task-Level Room Tidying (Priority: P2)

A student instructs the simulated humanoid robot to "Tidy up the living room." The robot uses its LLM planner to decompose this high-level goal into sub-tasks (e.g., identify clutter, pick up trash, place items on shelves), continuously leveraging vision for object identification and localization, and executing these sub-tasks via ROS 2.

**Why this priority**: This demonstrates complex cognitive planning and multi-step execution, building upon basic object retrieval.

**Independent Test**: The student can test by setting up a cluttered simulated room, issuing the command, and verifying that the robot successfully categorizes and organizes multiple items according to a logical plan.

**Acceptance Scenarios**:

1.  **Given** a simulated living room with several misplaced items (trash, books, toys), **When** the operator issues the command "Tidy up the living room," **Then** the robot generates a sequence of actions to clear the floor, place books on shelves, and deposit trash in a bin, and successfully executes these actions.
2.  **Given** an item is out of reach or cannot be identified by vision, **When** the robot encounters this situation, **Then** it either attempts an alternative approach or reports the unresolvable item to the operator (simulated).

---

### User Story 3 - Safety-Constrained Navigation (Priority: P3)

A student defines virtual "no-go" zones or fragile areas in the simulated environment. The robot, while executing VLA tasks, dynamically adjusts its navigation and manipulation plans to avoid these zones and prevent damage, demonstrating safety awareness.

**Why this priority**: This addresses critical safety and robustness aspects of robotics, essential for real-world deployment.

**Independent Test**: The student can test by creating a scenario where a planned path or manipulation action would violate a safety constraint and verifying the robot's ability to avoid the constraint or abort the unsafe action.

**Acceptance Scenarios**:

1.  **Given** the robot is planning to navigate to an object located behind a designated "fragile area," **When** the LLM planner considers the safety constraints, **Then** the robot either finds an alternative, safe path or reports that the object is inaccessible due to safety restrictions.
2.  **Given** the robot is holding an object near a collision-sensitive area, **When** an unexpected obstacle appears, **Then** the robot immediately pauses its action, re-evaluates, and performs a safe maneuver to avoid collision or damage.

---

### Edge Cases

*   What happens when the voice command is unintelligible or incomplete? The system should request clarification or indicate failure to understand.
*   How does the system handle dynamic changes in the environment (e.g., an object is moved after detection but before manipulation)? The system should re-perceive and re-plan.
*   What if an LLM-generated plan leads to an physically impossible or unsafe action? The execution monitoring system must detect and prevent such actions, requesting re-planning.
*   How does the robot respond if it loses track of a target object after initial detection? The robot should initiate a search behavior or report loss of tracking.

## Requirements *(mandatory)*

### Functional Requirements

*   **FR-001**: The system MUST accurately transcribe natural language voice commands using the OpenAI Whisper API.
*   **FR-002**: The system MUST process transcribed voice commands to extract high-level task goals and relevant parameters.
*   **FR-003**: The system MUST use an LLM-based planner to decompose high-level task goals into a sequence of structured, executable ROS 2 actions.
*   **FR-004**: The system MUST integrate with a vision system to perform grounded object recognition and localization within the simulated environment.
*   **FR-005**: The system MUST map identified objects and their locations to the LLM's world model for planning.
*   **FR-006**: The system MUST utilize ROS 2 action servers and clients for executing humanoid navigation, manipulation, and interaction behaviors.
*   **FR-007**: The system MUST incorporate safety constraints into LLM planning and execution monitoring to prevent unsafe robot actions.
*   **FR-008**: The system MUST provide simulated feedback on task progress, completion, or encountered issues.
*   **FR-009**: The system MUST support dynamic re-planning in response to environmental changes or execution failures.

### Key Entities *(include if feature involves data)*

*   **VoiceCommand**: Raw audio input, transcribed text, parsed intent, parameters.
*   **ActionPlan**: A sequence of structured ROS 2 actions, each with a name and associated parameters.
*   **RobotState**: Current pose, joint states, manipulated objects.
*   **EnvironmentMap**: Semantic representation of the simulated environment, including object locations and safety zones.
*   **DetectedObject**: Object label, bounding box/pose, confidence score from vision system.
*   **ROS2Action**: A specific robot behavior (e.g., `Navigate`, `PickObject`), defined by its interface and executed via ROS 2.

## Success Criteria *(mandatory)*

### Measurable Outcomes

*   **SC-001**: **Voice Command Transcription Accuracy**: 95% of voice commands MUST be accurately transcribed by Whisper, leading to correct intent parsing.
*   **SC-002**: **Task Completion Rate**: 80% of multi-step VLA tasks (e.g., "Tidy up the living room") MUST be completed successfully by the simulated robot in less than 5 minutes.
*   **SC-003**: **Object Retrieval Success Rate**: 90% of object retrieval commands (e.g., "Fetch the red apple") MUST result in the robot successfully picking up the correct object and delivering it.
*   **SC-004**: **Safety Compliance**: The robot MUST NOT enter any defined "no-go" zones or execute actions leading to simulated collisions in 100% of test cases.
*   **SC-005**: **Planning Robustness**: The LLM-based planner MUST generate a valid and executable plan for 90% of novel natural language instructions, demonstrating effective task decomposition.
*   **SC-006**: **Vision Grounding Accuracy**: 90% of visually identifiable objects referenced in commands MUST be correctly localized by the vision system with an average positional error of less than 5cm in the simulator.
*   **SC-007**: **User Satisfaction**: Students (simulated) can successfully implement and debug the VLA pipeline, achieving a high confidence level in their understanding of VLA concepts as measured by practical task performance. (Qualitative, instructor-assessed)
