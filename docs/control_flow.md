# Control Flow – Execution Logic

## Overview
The system converts a text task → LLM plan → validated plan → executed actions.

---

## 1. Input Stage
- User provides task.
- World state retrieved.

---

## 2. Planning Stage
- Gemini receives:
  - Task
  - Tools
  - World snapshot
- Generates JSON plan.

---

## 3. Validation Stage
`planner.py` checks:
- Tool names exist.
- Parameter types match.
- No unsafe actions.

---

## 4. Execution Stage
`tool_executor.py` executes each step:
1. Log start of action.
2. Run action via ROS.
3. Poll robot state.
4. Confirm completion.
5. Log result.

If any tool fails → abort plan gracefully.

---

## 5. Feedback Stage
- Logs saved.
- LLM plan stored.
- Optional: re-planning.
