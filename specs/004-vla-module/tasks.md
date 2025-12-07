# Tasks: Module 4: Vision-Language-Action (VLA)

**Input**: Design documents from `/specs/004-vla-module/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: Test tasks are included as `pytest` was selected in the research phase.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- Code examples: `AIdd-book/code-examples/module-4/`
- Documentation: `AIdd-book/docs/module-4/`

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure for the new module.

- [x] T001 Create code example directory structure in `AIdd-book/code-examples/module-4/` for chapters 1-5.
- [x] T002 [P] Create placeholder documentation files for chapters 1-5 in `AIdd-book/docs/module-4/`.
- [x] T003 Initialize `AIdd-book/code-examples/module-4/requirements.txt` with `rclpy`, `openai`, `opencv-python`, `sounddevice`, `numpy`, `pytest`.
- [x] T004 Create `.env.template` in `AIdd-book/code-examples/module-4/` for `OPENAI_API_KEY`.

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented.

- [x] T005 Implement a reusable, base ROS 2 node class in `AIdd-book/code-examples/module-4/common/base_node.py`.
- [x] T006 Implement a shared logger utility in `AIdd-book/code-examples/module-4/common/logger.py`.
- [ ] T007 Define custom ROS 2 message and action interfaces (e.g., for `ActionPlan`, `DetectedObject`) in a new ROS 2 package `vla_interfaces`.

**Checkpoint**: Foundation ready - user story implementation can now begin.

---

## Phase 3: User Story 1 - Voice-Controlled Object Retrieval (Priority: P1) 🎯 MVP

**Goal**: A student can verbally command the robot to fetch a specific object, and the robot will execute the task.

**Independent Test**: Issue a voice command like "Robot, bring me the blue cup from the table." and verify the robot retrieves the correct object in the simulator.

### Tests for User Story 1 ⚠️

> **NOTE: Write these tests FIRST, ensure they FAIL before implementation**

- [ ] T008 [P] [US1] Unit test for `WhisperVoiceCommandNode` to verify it transcribes and publishes messages in `AIdd-book/code-examples/module-4/chapter2_whisper/test_whisper_node.py`.
- [x] T009 [P] [US1] Unit test for `llm_planner.py` to verify it generates a valid action plan from a text command in `AIdd-book/code-examples/module-4/chapter3_llm_planning/test_llm_planner.py`.

### Implementation for User Story 1

- [ ] T010 [US1] Implement `WhisperVoiceCommandNode` in `AIdd-book/code-examples/module-4/chapter2_whisper/whisper_node.py`.
- [x] T011 [US1] Implement the LLM planning script in `AIdd-book/code-examples/module-4/chapter3_llm_planning/llm_planner.py`.
- [ ] T012 [P] [US1] Implement the vision node for object detection in `AIdd-book/code-examples/module-4/chapter4_vision/vision_node.py`.
- [ ] T013 [US1] Implement the `RobotActionExecutor` node in `AIdd-book/code-examples/module-4/chapter3_llm_planning/action_executor.py`.
- [ ] T014 [US1] Create a launch file to integrate all US1 nodes for the object retrieval task in `AIdd-book/code-examples/module-4/chapter5_integration/object_retrieval_launch.py`.
- [x] T015 [US1] Write the documentation for Chapter 1, 2 and 4 in `AIdd-book/docs/module-4/`.

**Checkpoint**: User Story 1 should be functional and testable independently.

---

## Phase 4: User Story 2 - Task-Level Room Tidying (Priority: P2)

**Goal**: A student can issue a high-level command to tidy a room, and the robot will decompose and execute the task.

**Independent Test**: Set up a cluttered simulated room, issue the command "Tidy up the living room," and verify the robot organizes the items.

### Implementation for User Story 2

- [x] T016 [US2] Enhance `llm_planner.py` in `AIdd-book/code-examples/module-4/chapter3_llm_planning/` to support multi-step task decomposition.
- [ ] T017 [US2] Enhance `vision_node.py` in `AIdd-book/code-examples/module-4/chapter4_vision/` to classify different types of objects (e.g., trash, clutter).
- [ ] T018 [US2] Add new manipulation actions (e.g., `place_object`) to `AIdd-book/code-examples/module-4/common/robot_actions.py`.
- [ ] T019 [US2] Create a launch file for the room tidying task in `AIdd-book/code-examples/module-4/chapter5_integration/room_tidying_launch.py`.
- [x] T020 [US2] Write the documentation for Chapter 3 in `AIdd-book/docs/module-4/03-llm-cognitive-planning.md`.

**Checkpoint**: User Stories 1 AND 2 should both work independently.

---

## Phase 5: User Story 3 - Safety-Constrained Navigation (Priority: P3)

**Goal**: The robot can perform its tasks while respecting predefined "no-go" zones.

**Independent Test**: Define a "no-go" zone, place an object behind it, and verify the robot either finds a safe path or reports the object as inaccessible.

### Implementation for User Story 3

- [ ] T021 [US3] Implement a `safety_node.py` in `AIdd-book/code-examples/module-4/chapter5_integration/` to publish safety zone information.
- [ ] T022 [US3] Modify `llm_planner.py` to incorporate safety constraints into plan generation.
- [ ] T023 [US3] Modify `action_executor.py` to include an execution monitoring component that can halt the robot.

**Checkpoint**: All user stories should now be independently functional.

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories.

- [x] T024 [P] Add detailed docstrings and comments to all Python files in `AIdd-book/code-examples/module-4/`.
- [x] T025 Finalize the content for all chapter documentation in `AIdd-book/docs/module-4/`.
- [ ] T026 [P] Create comprehensive unit tests for all utilities and nodes in `AIdd-book/code-examples/module-4/`.
- [x] T027 Validate that the `quickstart.md` guide is accurate and complete.

---

## Dependencies & Execution Order

- **Phase 1 (Setup)** must complete before all other phases.
- **Phase 2 (Foundational)** depends on Phase 1 and blocks all user stories.
- **User Stories (Phases 3-5)** can start after Phase 2 is complete. They can be developed in parallel.
- **Phase 6 (Polish)** begins after all desired user stories are complete.

## Implementation Strategy

The recommended strategy is **Incremental Delivery**:
1. Complete Phase 1 & 2.
2. Complete Phase 3 (US1) to deliver the MVP.
3. Complete Phase 4 (US2) and test independently.
4. Complete Phase 5 (US3) and test independently.
5. Finalize the module in Phase 6.
