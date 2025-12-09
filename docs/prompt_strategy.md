# Prompt Strategy – 42X007 Duality Robotic Arm

## Objective
Enable Gemini to generate deterministic, safe, and valid JSON action plans for the UR3 robot.

---

## Prompt Components

### 1. System Role
- Gemini acts as a robotic manipulation planner.
- It must choose only from allowed tools.
- No free-form text; JSON only.

### 2. Tools Schema
- Each tool has:
  - name
  - description
  - parameter schema
- Must match exactly.

### 3. Output Format
Strict:

```
[
  {"tool": "tool_name", "args": { ... }},
  ...
]
```

---

## Constraints Embedded in Prompt
- No loops.
- No conditionals.
- No guessing object names.
- Use get_world_state when uncertain.
- Choose minimum necessary steps.
- Always place object safely.

---

## Few-Shot Examples
Critically important.

Examples include:
- Simple pick task
- Move + place task
- Multi-step manipulation

---

## Reliability Techniques
- Temperature = 0.0
- Schema-based validation
- Description-level rules ("Do not move through objects")
- Fail-safe behaviors

A well-crafted prompt ensures stability across runs.
