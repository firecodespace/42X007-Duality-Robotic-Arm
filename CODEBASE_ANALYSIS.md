# 42X007 Duality Robotic Arm – Complete Codebase Analysis

## Executive Summary
This is a **three-layer LLM-controlled robotics pipeline** that transforms natural language tasks into precise robotic arm actions. The system integrates Gemini AI for planning, a tool-based abstraction layer, and ROS2 for UR3 robot control within the FalconSim environment.

---

## System Architecture

### 🏗️ Three Core Layers

```
┌─────────────────────────────────────────┐
│   LLM Planning Layer (Gemini 2.5/3.0)   │
│  - Receives natural language task       │
│  - Generates JSON action plan           │
│  - Uses tool schemas for guidance       │
└────────────────────┬────────────────────┘
                     ↓
┌─────────────────────────────────────────┐
│    Tools & Planning API Layer           │
│  - 11 defined tools with schemas        │
│  - Plan validation & execution          │
│  - Logging & state management           │
└────────────────────┬────────────────────┘
                     ↓
┌─────────────────────────────────────────┐
│   ROS2 Execution Layer (UR3)            │
│  - Robot state subscribers              │
│  - IK/FK/Gripper command publishers     │
│  - Real-time feedback monitoring        │
│  - FalconSim integration                │
└─────────────────────────────────────────┘
```

---

## Data Flow

```
User Task (Text)
    ↓
Planner.plan_task()
    ├─→ Retrieve world state
    ├─→ GeminiClient.generate_plan()
    │   ├─→ Build prompt with task + tools + world state
    │   └─→ Send to Gemini API (returns JSON plan)
    ├─→ Parse JSON output
    ├─→ Validate plan (tools exist, params correct)
    └─→ Log plan to /logs/llm_plans/
         ↓
ToolExecutor.execute_plan()
    ├─→ For each ToolCall in plan:
    │   ├─→ Route to appropriate _tool_method()
    │   ├─→ Send ROS command via UR3ROSInterface
    │   ├─→ Monitor robot state
    │   └─→ Log execution result
    └─→ Report success/failure
         ↓
Robot performs action in FalconSim
```

---

## Component Breakdown

### 1. **LLM Planning Layer** (`gemini_interface.py`)

**Purpose**: Generate robot action plans from natural language.

**Key Class**: `GeminiClient`
- `generate_plan(task_text, tool_specs, world_state)` → JSON string
- Builds comprehensive prompt with:
  - System role (planning assistant)
  - Tool descriptions and parameters
  - Output format rules (strict JSON only)
  - World state context
  - Few-shot examples

**Prompt Components**:
```
1. System Role: "You are an AI planning assistant for UR3 robot"
2. Tools Section: Lists all 11 available tools with descriptions
3. Output Format: Strict JSON array requirement
4. Planning Rules: Safety guidelines, use get_world_state when uncertain
5. World State Snippet: Current object/gripper/robot positions
6. Few-shot Example: Task → JSON plan mapping
7. Final Task: User's natural language instruction
```

**Current Implementation**:
- Prompts are fully constructed and logged
- Returns stub JSON plan (TODO: replace with real Gemini API call on VM)
- Temperature: 0.0 (deterministic)
- Max tokens: 1024

---

### 2. **Planner Layer** (`planner.py`)

**Purpose**: Orchestrate LLM invocation, parse results, validate plans.

**Key Class**: `Planner`
- Manages tool specifications
- Calls GeminiClient for planning
- Validates generated plans
- Logs plans to disk

**Main Method**: `plan_task(task_text, world_state)`

**Validation Steps**:
1. Check all tool names exist in tool_map
2. Verify required parameters are provided
3. Match parameter types to specification
4. Log plan with timestamp and task info

**Output**: List of `ToolCall` objects (tool name + arguments)

---

### 3. **Tools Definition** (`defined_tools.py`)

**Purpose**: Define available robot capabilities as tools.

**Key Classes**:
- `ToolParamSpec`: Describes a single parameter (type, required flag, enum options)
- `ToolSpec`: Describes a complete tool (name, description, params dict)

**Tool Loading**:
- Reads from `/config/tool_config.yaml`
- Parsed into `TOOL_SPECS` global list
- Accessed via `get_tool_specs()`

---

### 4. **Tool Executor Layer** (`tool_executor.py`)

**Purpose**: Execute validated plans on the actual robot.

**Key Class**: `ToolExecutor`
- Receives `ToolCall` list from Planner
- Routes each call to appropriate handler
- Sends commands via ROS2 interface
- Handles both primitive and composite tools

**Execution Flow**:
```python
execute_plan(plan: List[ToolCall])
    └─→ For each ToolCall:
        ├─→ execute_step(call)
        │   └─→ Route by tool name:
        │       ├─→ _move_to_position()
        │       ├─→ _move_to_named_pose()
        │       ├─→ _move_above_object()
        │       ├─→ _move_relative()
        │       ├─→ _open_gripper() / _close_gripper()
        │       ├─→ _pick_object() [composite]
        │       ├─→ _place_object_at() [composite]
        │       ├─→ _press_button() [composite]
        │       ├─→ _get_world_state()
        │       └─→ _wait_until_still()
        └─→ Log result / break on error
```

---

### 5. **ROS2 Interface Layer** (`utils/ros_utils.py`)

**Purpose**: Bridge between Python tool layer and ROS2/FalconSim.

**Key Class**: `UR3ROSInterface` (extends `rclpy.node.Node`)

**Responsibilities**:
- Subscribe to robot state topics
- Maintain thread-safe state cache
- Provide Python API for commands
- Monitor motion completion

**Subscriptions** (TODO on VM):
- `/ur3/ee_pose` → end-effector position/orientation
- `/ur3/object_poses` → positions of all scene objects
- `/ur3/held_item` → name of currently held object
- `/ur3/is_moving` → robot movement status

**Publishers/Clients** (TODO on VM):
- `/ur3/set_ik` → inverse kinematics command
- `/ur3/set_fk` → forward kinematics command
- `/ur3/set_gripper` → gripper open/close command

**Key Methods**:
- `get_world_state()` → Dict with current robot/scene state
- `move_to_position_ik(x, y, z, ...)` → IK movement
- `move_to_named_pose(pose_name)` → Predefined poses
- `move_relative(dx, dy, dz)` → Relative offset movement
- `set_gripper(open_gripper)` → Gripper control
- `wait_until_still(timeout_s)` → Motion completion blocking

---

## Tool Specifications (11 Total)

### Movement Tools
| Tool | Parameters | Purpose |
|------|-----------|---------|
| `move_to_position` | x, y, z, speed? | Absolute 3D position via IK |
| `move_to_named_pose` | pose_name | Predefined safe poses (home, drop_zone) |
| `move_above_object` | object_name, dz | Position above object + offset |
| `move_relative` | dx, dy, dz | Cartesian offset movement |

### Gripper Tools
| Tool | Parameters | Purpose |
|------|-----------|---------|
| `open_gripper` | (none) | Fully open gripper |
| `close_gripper` | (none) | Fully close gripper |

### Manipulation Tools
| Tool | Parameters | Purpose |
|------|-----------|---------|
| `pick_object` | object_name | Approach → close → lift sequence |
| `place_object_at` | target_type, x?, y?, z?, pose_name? | Move to position → open gripper |
| `press_button` | button_name | Approach → tap → retract sequence |

### State Tools
| Tool | Parameters | Purpose |
|------|-----------|---------|
| `get_world_state` | (none) | Query current robot/object state |
| `wait_until_still` | timeout_s | Block until motion completes |

---

## Configuration System

### `/config/tool_config.yaml`
- YAML definition of all 11 tools
- Each tool specifies:
  - name, description
  - parameter types (float, string, bool)
  - required vs optional flags
  - enum values for restricted parameters

### `/config/model_config.yaml`
- (Currently identical to tool_config.yaml)
- Can be extended for LLM model selection

### `/config/scenario_config.yaml`
- (Currently empty - for future scenario definitions)

### `ur3_controller/config.py`
- Python configuration constants
- Path definitions (BASE_DIR, PROJECT_ROOT, CONFIG_DIR)
- LLM settings (DEFAULT_MODEL_NAME, LLM_TEMPERATURE, LLM_MAX_TOKENS)
- Timeout defaults

---

## Logging System

### Logging Utilities (`utils/logging_utils.py`)
- `setup_logger(name, log_file?, level?)` creates consistent loggers
- All modules use: `logger = setup_logger(__name__)`
- Logs to both console (INFO) and optional file
- Timestamp format: `[YYYY-MM-DD HH:MM:SS] [module] [level] message`

### Log Directories
- `/logs/llm_plans/` → Saved JSON plans (plan_*.json)
- `/logs/run_logs/` → Execution logs

---

## Local Development (`ur3_controller_node.py`)

**Current Implementation**: Simple Python harness for local testing

```python
class UR3ControllerApp:
    def __init__(self):
        self.planner = Planner()
        self.executor = ToolExecutor()  # TODO: requires ROS interface
    
    def run_task(self, task_text, world_state=None):
        plan = self.planner.plan_task(task_text, world_state)
        self.executor.execute_plan(plan)
```

**Windows Local Dev**:
- Prompt building works offline
- Returns stub JSON plan
- No actual robot movement
- Ready to test prompt engineering

**VM Deployment**:
- Replace GeminiClient.generate_plan() with real API call
- Initialize UR3ROSInterface with actual subscriptions/publishers
- Launch as ROS2 node
- Real robot + FalconSim integration

---

## Execution Flow (Step-by-Step Example)

### Example: "Pick up the red cube and place it in the drop zone"

**Step 1: Prompt Building** (`gemini_interface.py`)
```
Prompt includes:
- Task: "Pick up the red cube and place it in the drop zone"
- Available tools: All 11 tools with full schemas
- World state: {"objects": {"red_cube": {...}}, "ee_pose": {...}}
- Constraints: Use minimum steps, get_world_state if uncertain, etc.
```

**Step 2: LLM Planning** (`planner.py`)
```python
raw_plan = llm_client.generate_plan(task, tools, world_state)
# Returns (currently stub):
[
  {"tool": "get_world_state", "args": {}},
  {"tool": "pick_object", "args": {"object_name": "red_cube"}},
  {"tool": "move_to_named_pose", "args": {"pose_name": "drop_zone"}},
  {"tool": "place_object_at", "args": {"target_type": "named_pose", "pose_name": "drop_zone"}}
]
```

**Step 3: Validation** (`planner.py`)
```
✓ Tool "get_world_state" exists
✓ Tool "pick_object" exists, has required param "object_name"
✓ Tool "move_to_named_pose" exists, has required param "pose_name"
✓ Tool "place_object_at" exists, params match schema
✓ Plan is valid
→ Log to /logs/llm_plans/plan_Pick_up_the_red_cube_and_place_it_in_the_drop_zone.json
```

**Step 4: Execution** (`tool_executor.py`)
```
Step 0: execute_step(ToolCall("get_world_state", {}))
  → ROS interface polls: ee_pose, object_poses, held_item, is_moving
  → Logs: "World state: {...}"

Step 1: execute_step(ToolCall("pick_object", {"object_name": "red_cube"}))
  → _pick_object("red_cube"):
    1. _move_above_object("red_cube", 0.10)
       └─ ROS sends IK command
    2. _wait_until_still(5.0)
       └─ Polls /ur3/is_moving until false or timeout
    3. _move_relative(0, 0, -0.10)
       └─ ROS sends relative offset command
    4. _wait_until_still(5.0)
    5. _close_gripper()
       └─ ROS sends gripper close command
    6. _wait_until_still(5.0)
    7. _move_relative(0, 0, 0.10)
       └─ Lift cube

Step 2: execute_step(ToolCall("move_to_named_pose", {"pose_name": "drop_zone"}))
  → ROS sends FK command to drop_zone pose

Step 3: execute_step(ToolCall("place_object_at", {...}))
  → _place_object_at(...):
    1. _move_to_named_pose("drop_zone")
    2. _wait_until_still(5.0)
    3. _open_gripper()
    4. _wait_until_still(5.0)

✓ Plan executed successfully
```

---

## Key Design Principles

### 1. **Separation of Concerns**
- **Planner**: Decides WHAT to do (LLM logic)
- **Validator**: Ensures CORRECTNESS (schema compliance)
- **Executor**: Handles HOW to do it (ROS commands)
- **ROS Interface**: Bridges Python ↔ Robot

### 2. **Deterministic Planning**
- Temperature = 0.0 (no randomness)
- Schema-based validation (no guessing)
- Explicit tool constraints in prompt
- Reproducible across runs

### 3. **Safety & Robustness**
- Waits until robot is still after each action
- Validates entire plan before execution
- Graceful error handling (logs and breaks)
- Thread-safe state management (locks in ROS interface)

### 4. **Generalizability**
- Tool-based abstraction (LLM doesn't know joint angles)
- Scalable tool set (11 carefully chosen tools)
- Clean Python API (easy to add new tools)
- Configurable via YAML

### 5. **Production-Ready Structure**
- Consistent logging across all modules
- Comprehensive error messages
- Modular, testable components
- Clear separation of local dev vs VM deployment

---

## Code Statistics

| Component | File | Lines | Purpose |
|-----------|------|-------|---------|
| LLM Interface | `gemini_interface.py` | 213 | Prompt building & LLM planning |
| Planner | `planner.py` | 70 | Orchestration, validation, logging |
| Tools Definition | `defined_tools.py` | 40 | Tool schema loading from YAML |
| Executor | `tool_executor.py` | 250 | Tool call routing & ROS execution |
| ROS Interface | `utils/ros_utils.py` | 202 | ROS2 state management & commands |
| Configuration | `config.py` | 25 | Path & constant definitions |
| Logging | `utils/logging_utils.py` | 25 | Centralized logging setup |
| **Total** | | **~825** | |

---

## TODOs for VM Deployment

### 1. **Gemini API Integration** (`gemini_interface.py`)
```python
# TODO: Replace stub with real API call
response = gemini_api.generate(
    model=self.model_name,
    prompt=prompt,
    temperature=self.temperature,
    max_tokens=self.max_tokens,
)
return response.text
```

### 2. **ROS2 Subscriptions** (`utils/ros_utils.py`)
```python
# Replace TODOs with actual topic names + message types
self.ee_pose_sub = self.create_subscription(PoseStamped, "/ur3/ee_pose", ...)
self.object_poses_sub = self.create_subscription(ObjectPoses, "/ur3/object_poses", ...)
self.held_item_sub = self.create_subscription(HeldItem, "/ur3/held_item", ...)
self.is_moving_sub = self.create_subscription(Bool, "/ur3/is_moving", ...)
```

### 3. **ROS2 Command Publishers** (`utils/ros_utils.py`)
```python
self.ik_pub = self.create_publisher(IKRequest, "/ur3/set_ik", qos)
self.fk_pub = self.create_publisher(FKRequest, "/ur3/set_fk", qos)
self.gripper_pub = self.create_publisher(GripperCommand, "/ur3/set_gripper", qos)
```

### 4. **ToolExecutor Initialization**
```python
# Initialize with actual ROS interface
ros_interface = UR3ROSInterface()
executor = ToolExecutor(ros_interface)
```

### 5. **ROS2 Node Launch**
- Wrap UR3ControllerApp as ROS2 action server
- Accept task goals via ROS action interface
- Return plan execution results

---

## Testing Strategy

### Local Windows Testing
1. Build prompts offline
2. Verify prompt structure
3. Test plan parsing/validation
4. Simulate tool sequences

### VM Integration Testing
1. Connect to FalconSim environment
2. Test ROS subscriptions (state polling)
3. Test ROS publishers (command sending)
4. Verify Gemini API integration
5. Run end-to-end task sequences

### Benchmarking (See `docs/experiments.md`)
- Compare Gemini 2.5 vs 3.0
- Measure plan correctness across scenarios
- Track execution time
- Validate output determinism (runs 10x, compare plans)

---

## Quick Reference

### Running Locally (Windows)
```bash
cd ros2_ws/src/ur3_controller
python -m ur3_controller.ur3_controller_node
# Will execute: "Move to home and get the world state."
```

### Running on VM
```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build
source install/setup.bash
run_ur3  # Custom command from workspace setup
```

### Debugging
- Check logs in `/logs/llm_plans/` for generated plans
- Check logs in `/logs/run_logs/` for execution traces
- Enable DEBUG logging by setting `level=logging.DEBUG` in logger setup

---

## File Tree Reference

```
42X007-Duality-Robotic-Arm/
├── README.md
├── config/
│   ├── model_config.yaml
│   ├── scenario_config.yaml
│   └── tool_config.yaml          # Tool definitions
├── docs/
│   ├── architecture.md
│   ├── control_flow.md
│   ├── experiments.md
│   ├── llm_integration.md
│   ├── prompt_strategy.md
│   ├── ros_integration.md
│   └── tools_design.md
├── logs/
│   ├── llm_plans/                # Saved JSON plans
│   └── run_logs/
└── ros2_ws/src/ur3_controller/
    └── ur3_controller/
        ├── __init__.py
        ├── config.py             # Paths & constants
        ├── defined_tools.py      # Tool spec loader
        ├── gemini_interface.py   # LLM prompt builder
        ├── planner.py            # Orchestrator
        ├── tool_executor.py      # Tool executor
        ├── ur3_controller_node.py # Main app
        └── utils/
            ├── __init__.py
            ├── ik_utils.py       # (Empty - TODO)
            ├── logging_utils.py  # Logging setup
            ├── ros_utils.py      # ROS2 interface
            └── validation.py     # (Empty - TODO)
```

---

## Summary

This is a **well-architected LLM-robotics system** with:
- ✅ Clear three-layer separation (Planning → Validation → Execution)
- ✅ Schema-driven tool abstraction (11 carefully designed tools)
- ✅ Deterministic prompt engineering (temperature 0.0)
- ✅ Production-grade logging & configuration
- ✅ Thread-safe ROS2 integration
- ✅ Ready for Gemini API integration on VM

The codebase is **modular, testable, and extensible**, providing a solid foundation for autonomous robotic manipulation tasks guided by natural language input.
