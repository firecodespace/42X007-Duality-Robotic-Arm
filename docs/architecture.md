# System Architecture – 42X007 Duality Robotic Arm

## Overview
The 42X007 Duality Robotic Arm System is a three-layer autonomous control pipeline combining a Large Language Model (LLM), a tool-based planning system, and a ROS2-controlled UR3 robotic arm inside the FalconSim environment.

The purpose is to create a generalizable, intelligent robotic manipulation framework where natural language tasks are translated into precise, structured physical actions.

---

## Architecture Layers

### 1. LLM Planning Layer (Gemini 2.5 / 3.0)
- Receives human task instructions.
- Has access to a defined set of tools.
- Generates a multi-step **JSON plan**.
- Ensures tool parameters comply with schemas.
- Behaves as a high-level planner, *not* a controller.

### 2. Tools & Planning API Layer
- Provides the LLM with robot abilities (tools).
- Defines:
  - Tool names
  - Descriptions
  - Parameters
  - Preconditions & postconditions
- Produces a validated action sequence that the ROS executor can run.

### 3. ROS2 Execution Layer (UR3 Controller)
- Executes actions chronologically.
- Uses:
  - Inverse Kinematics (IK)
  - Forward Kinematics (FK)
  - Gripper control
  - World-state queries
- Monitors robot state and ensures stability between actions.

---

## Data Flow Diagram

```
Task Text
    ↓
LLM Planner (Gemini)
    ↓ JSON Plan
Planner → Validator → Executor
    ↓
ROS2 Node (ur3_controller_node)
    ↓
UR3 Robot in FalconSim
    ↓
State Feedback → Executor → LLM (Optional)
```

---

## Key System Goals
1. Clear separation of planning vs. control.
2. Deterministic, reproducible behavior.
3. Generalizable toolset for many tasks.
4. High-quality prompt engineering to guide LLM behavior.
5. High reliability across repeated runs.

---

## Components in Repo

| Layer | Folder | Responsibility |
|-------|--------|----------------|
| LLM Planner | `ur3_controller/gemini_interface.py` | Prompting, tool schemas, plan generation |
| Tools API | `defined_tools.py`, `tool_config.yaml` | Public robot API to LLM |
| Execution | `tool_executor.py`, `ur3_controller_node.py` | Running actions in ROS |
| Utilities | `utils/` | IK, validation, ROS helpers |
| Docs | `docs/` | Full technical reference |

This architecture ensures a scalable, clean robotics AI system ready for production-grade extensibility.
