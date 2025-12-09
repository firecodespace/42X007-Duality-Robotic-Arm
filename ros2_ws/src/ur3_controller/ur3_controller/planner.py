from dataclasses import dataclass
from typing import Any, Dict, List
import json

from .defined_tools import ToolSpec, get_tool_specs
from .utils.logging_utils import setup_logger
from .config import LLM_PLAN_LOG_DIR
from .gemini_interface import GeminiClient

logger = setup_logger(__name__)

@dataclass
class ToolCall:
    tool: str
    args: Dict[str, Any]

class Planner:
    def __init__(self, tool_specs: List[ToolSpec] | None = None) -> None:
        self.tool_specs = tool_specs or get_tool_specs()
        self.tool_map: Dict[str, ToolSpec] = {t.name: t for t in self.tool_specs}
        self.llm_client = GeminiClient()

    def plan_task(self, task_text: str, world_state: Dict[str, Any] | None = None) -> List[ToolCall]:
        logger.info(f"Planning task: {task_text!r}")
        # Ask LLM for a raw JSON plan string
        raw_plan = self.llm_client.generate_plan(
            task_text=task_text,
            tool_specs=self.tool_specs,
            world_state=world_state or {},
        )
        logger.debug(f"Raw plan from LLM: {raw_plan}")

        plan_json = self._parse_raw_plan(raw_plan)
        tool_calls = [ToolCall(tool=step["tool"], args=step.get("args", {})) for step in plan_json]

        self._validate_plan(tool_calls)
        self._log_plan(task_text, tool_calls)

        return tool_calls

    def _parse_raw_plan(self, raw_plan: str) -> List[Dict[str, Any]]:
        try:
            data = json.loads(raw_plan)
            if not isinstance(data, list):
                raise ValueError("Top-level LLM output must be a JSON array.")
            return data
        except Exception as e:
            logger.error(f"Failed to parse LLM plan JSON: {e}")
            raise

    def _validate_plan(self, plan: List[ToolCall]) -> None:
        for idx, call in enumerate(plan):
            if call.tool not in self.tool_map:
                raise ValueError(f"Unknown tool in plan at step {idx}: {call.tool!r}")
            spec = self.tool_map[call.tool]
            # Check params
            for param_name, param_spec in spec.params.items():
                if param_spec.required and param_name not in call.args:
                    raise ValueError(f"Missing required param '{param_name}' for tool '{call.tool}' at step {idx}")
            # Extra params allowed, but you could restrict them here

        logger.info(f"Validated plan with {len(plan)} tool calls.")

    def _log_plan(self, task_text: str, plan: List[ToolCall]) -> None:
        LLM_PLAN_LOG_DIR.mkdir(parents=True, exist_ok=True)
        safe_task_name = task_text.replace(" ", "_")[:50]
        out_file = LLM_PLAN_LOG_DIR / f"plan_{safe_task_name}.json"
        data = {
            "task": task_text,
            "plan": [dict(tool=tc.tool, args=tc.args) for tc in plan],
        }
        with open(out_file, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)
        logger.info(f"Saved LLM plan to {out_file}")
