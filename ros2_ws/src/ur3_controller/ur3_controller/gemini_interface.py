from typing import Any, Dict, List
import textwrap

from .defined_tools import ToolSpec
from .utils.logging_utils import setup_logger
from .config import DEFAULT_MODEL_NAME, LLM_TEMPERATURE, LLM_MAX_TOKENS

logger = setup_logger(__name__)


def _format_tool_for_prompt(tool: ToolSpec) -> str:
    """Format a single tool specification into a readable block for the LLM."""
    lines = [f"- name: {tool.name}", f"  description: {tool.description}", "  params:"]
    if not tool.params:
        lines.append("    (none)")
    else:
        for pname, pspec in tool.params.items():
            enum_text = f", enum={pspec.enum}" if pspec.enum else ""
            req_text = "required" if pspec.required else "optional"
            lines.append(f"    - {pname}: type={pspec.type}, {req_text}{enum_text}")
    return "\n".join(lines)


def _format_tools_section(tool_specs: List[ToolSpec]) -> str:
    blocks = [_format_tool_for_prompt(t) for t in tool_specs]
    return "\n\n".join(blocks)


def _format_world_state_snippet(world_state: Dict[str, Any]) -> str:
    """
    Convert world_state dict into a compact text snippet.
    Keep it simple and robust; we don't assume exact schema here.
    """
    if not world_state:
        return "World state not provided."

    # You can customize this once you know exact keys from FalconSim.
    lines = []
    objects = world_state.get("objects") or world_state.get("object_poses")
    ee_pose = world_state.get("ee_pose")
    held_item = world_state.get("held_item")
    is_moving = world_state.get("is_moving")

    if objects:
        lines.append("Objects:")
        try:
            for name, pose in objects.items():
                lines.append(f"  - {name}: {pose}")
        except Exception:
            lines.append(f"  (raw objects data) {objects}")

    if ee_pose is not None:
        lines.append(f"End-effector pose: {ee_pose}")
    if held_item is not None:
        lines.append(f"Held item: {held_item}")
    if is_moving is not None:
        lines.append(f"Robot moving: {is_moving}")

    if not lines:
        return f"Raw world state: {world_state}"

    return "\n".join(lines)


def _build_prompt(
    task_text: str,
    tool_specs: List[ToolSpec],
    world_state: Dict[str, Any],
) -> str:
    """
    Build the full prompt for the LLM:
    - System role
    - Tools description
    - Output format rules
    - Optional world state
    - Task instruction
    - (Can include few-shot examples)
    """
    tools_section = _format_tools_section(tool_specs)
    world_state_snippet = _format_world_state_snippet(world_state)

    # Few-shot example: very simple scenario
    few_shot_example = textwrap.dedent(
        """
        Example:

        Task: "Move the arm to the home pose, then query the world state."

        Expected JSON plan:
        [
          {"tool": "move_to_named_pose", "args": {"pose_name": "home"}},
          {"tool": "get_world_state", "args": {}}
        ]
        """
    ).strip()

    prompt = f"""
    You are an AI planning assistant that controls a UR3 robotic arm in the FalconSim environment.

    You do NOT control motors or physics directly.
    Instead, you can only call a fixed set of tools.
    Your job is to choose which tools to call, in what order, and with what parameters,
    to complete the user's task safely and efficiently.

    TOOLS AVAILABLE
    ---------------
    The following tools are available to you:

    {tools_section}

    OUTPUT FORMAT (STRICT)
    ----------------------
    You MUST output ONLY a JSON array of tool call objects.
    Each element of the array must have:
      - "tool": the exact name of the tool (string)
      - "args": an object containing parameters for that tool

    No additional keys are allowed.
    No comments or explanations are allowed.
    Do NOT wrap the JSON in backticks.
    Do NOT output any text before or after the JSON.

    Example of valid output:
    [
      {{"tool": "move_to_named_pose", "args": {{"pose_name": "home"}}}},
      {{"tool": "get_world_state", "args": {{}}}}
    ]

    If the task cannot be solved with the available tools,
    return an empty array: [].

    PLANNING RULES
    --------------
    - Use the minimum number of tool calls necessary to safely complete the task.
    - When you are unsure about object positions or what is being held,
      first call "get_world_state".
    - After large movements, call "wait_until_still" before interacting with objects.
    - Prefer higher-level tools (e.g., "pick_object", "place_object_at")
      when they achieve the goal simply.
    - Never invent tool names or parameters that are not listed above.
    - Do not assume the existence of objects that are not in the world state.

    WORLD STATE SNIPPET (MAY BE PARTIAL)
    ------------------------------------
    {world_state_snippet}

    {few_shot_example}

    NOW PLAN
    --------
    The user's task is:

    "{task_text}"

    Return ONLY the JSON array of tool calls, and nothing else.
    """.strip()

    return prompt


class GeminiClient:
    """
    LLM client wrapper.

    On Windows (local dev):
      - We build and log the full prompt.
      - We return a stub JSON plan so the pipeline runs.

    On the VM:
      - You will replace generate_plan() internals to call
        the real Gemini API using this prompt.
    """

    def __init__(self, model_name: str = DEFAULT_MODEL_NAME) -> None:
        self.model_name = model_name
        self.temperature = LLM_TEMPERATURE
        self.max_tokens = LLM_MAX_TOKENS

    def generate_plan(
        self,
        task_text: str,
        tool_specs: List[ToolSpec],
        world_state: Dict[str, Any],
    ) -> str:
        """
        Build the planning prompt and, for now, return a stub plan.

        On the VM:
          - Use the constructed prompt in a real Gemini API call.
        """
        prompt = _build_prompt(task_text, tool_specs, world_state)
        logger.info("Constructed LLM prompt for planning.")
        logger.debug(f"LLM Prompt:\n{prompt}")

        # TODO (VM): Replace this stub with a real Gemini API call.
        # Pseudocode:
        # response = gemini_api.generate(
        #     model=self.model_name,
        #     prompt=prompt,
        #     temperature=self.temperature,
        #     max_tokens=self.max_tokens,
        # )
        # return response.text

        logger.warning("GeminiClient.generate_plan is using a stubbed implementation.")
        # Keep the stub EXAMPLE aligned with the few-shot example
        return """
        [
          {"tool": "move_to_named_pose", "args": {"pose_name": "home"}},
          {"tool": "get_world_state", "args": {}}
        ]
        """
