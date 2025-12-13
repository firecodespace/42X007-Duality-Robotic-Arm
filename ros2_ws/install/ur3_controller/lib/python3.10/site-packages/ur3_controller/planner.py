import json
import time

class Planner:
    def __init__(self, tools):
        self.tools = tools
        self.system_instructions = self._build_system_instructions()

    def _build_system_instructions(self):
        """Build optimized system instructions for Gemini."""
        return """You are a robot control planner. Generate a JSON array of tool calls to accomplish the task.

CRITICAL RULES:
1. Output ONLY a valid JSON array. No markdown, no explanations, no extra text.
2. Each element must be: {"tool": "name", "parameters": {"param": value}}
3. For pick + move tasks, use exactly 2 steps: pick_object, then place_object_relative
4. place_object_relative MUST have all THREE params: dx, dy, dz (use 0 for no movement)
5. Coordinate system:
   - X: positive=forward, negative=backward
   - Y: positive=right, negative=left
   - Z: positive=up, negative=down
6. Always use exact object names from available_objects list

EXAMPLE: "Pick red cube and move 30cm left"
[
  {"tool": "pick_object", "parameters": {"object_name": "red cube"}},
  {"tool": "place_object_relative", "parameters": {"dx": 0, "dy": -30, "dz": 0}}
]

CRITICAL: Do NOT invent tools. Do NOT skip parameters. Always return valid JSON array."""

    def plan(self, task_text, world_state):
        """Generate action plan using Gemini."""
        from .gemini_interface import plan_actions
        
        start = time.time()
        
        try:
            plan = plan_actions(
                task_text,
                self.tools,
                system_instructions=self.system_instructions,
                world_state=world_state
            )

            if not isinstance(plan, list):
                raise ValueError(f"Expected list, got {type(plan)}")

            # Normalize and validate each step
            for i, step in enumerate(plan):
                if not isinstance(step, dict):
                    raise ValueError(f"Step {i} must be dict")

                if "tool" not in step:
                    raise ValueError(f"Step {i} missing 'tool'")

                # Convert "parameters" -> "args" for executor
                if "parameters" in step:
                    step["args"] = step.pop("parameters")
                elif "args" not in step:
                    step["args"] = {}

                # Validate critical tools
                if step.get("tool") == "place_object_relative":
                    args = step["args"]
                    for param in ["dx", "dy", "dz"]:
                        if param not in args:
                            args[param] = 0.0

            duration = time.time() - start
            print(f"[PLAN] Generated {len(plan)} steps in {duration:.2f}s")
            return plan

        except Exception as e:
            print(f"[PLAN ERROR] {str(e)}")
            raise RuntimeError(f"Planning failed: {str(e)}")
