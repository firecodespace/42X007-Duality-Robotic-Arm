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
    
    Note: Prompt is intentionally flexible to allow LLM to adapt to various scenarios.
    """
    tools_section = _format_tools_section(tool_specs)
    world_state_snippet = _format_world_state_snippet(world_state)

    # Few-shot examples for common tasks (not task-specific)
    few_shot_examples = textwrap.dedent(
        """
        Example 1: Simple movement
        Task: "Move the arm to the home pose and get the world state."
        Expected JSON plan:
        [
          {"tool": "move_to_named_pose", "args": {"pose_name": "home"}},
          {"tool": "get_world_state", "args": {}}
        ]

        Example 2: Pick and place
        Task: "Pick up the red cube and place it at the drop zone."
        Expected JSON plan:
        [
          {"tool": "get_world_state", "args": {}},
          {"tool": "pick_object", "args": {"object_name": "red_cube"}},
          {"tool": "move_to_named_pose", "args": {"pose_name": "drop_zone"}},
          {"tool": "place_object_at", "args": {"target_type": "named_pose", "pose_name": "drop_zone"}}
        ]

        Example 3: Multi-step manipulation
        Task: "Get the current state, then press the button if it exists."
        Expected JSON plan:
        [
          {"tool": "get_world_state", "args": {}},
          {"tool": "press_button", "args": {"button_name": "button"}}
        ]
        """
    ).strip()

    prompt = f"""
    You are an AI planning assistant for a UR3 robotic arm in the FalconSim environment.

    Your job is to decompose a user task into a sequence of tool calls to execute it safely.

    TOOLS AVAILABLE
    ---------------
    You have access to these tools:

    {tools_section}

    IMPORTANT GUIDELINES
    --------------------
    1. ALWAYS prefer high-level tools (pick_object, place_object_at, press_button) when applicable.
    2. Use get_world_state when uncertain about object positions or current state.
    3. Call wait_until_still after each movement before next action.
    4. Use move_to_position for precise locations, move_to_named_pose for predefined poses.
    5. For button presses, use press_button directly.
    6. For multiple objects, handle them sequentially.
    7. If an object is not in the world state, adapt the plan (use available objects instead).
    8. Minimize the number of tool calls - be efficient.

    OUTPUT FORMAT (STRICT JSON)
    ---------------------------
    Return ONLY a JSON array of tool calls. No comments, no text, no markdown.
    Each element must have:
      - "tool": the tool name (string)
      - "args": parameters object

    Valid example:
    [
      {{"tool": "move_to_named_pose", "args": {{"pose_name": "home"}}}},
      {{"tool": "get_world_state", "args": {{}}}}
    ]

    If the task cannot be completed with available tools, return an empty array: []

    CURRENT WORLD STATE
    -------------------
    {world_state_snippet}

    EXAMPLES (for reference only - adapt to your specific task)
    -----------------------------------------------------------
    {few_shot_examples}

    YOUR TASK
    ---------
    Plan the following task:
    "{task_text}"

    Output ONLY the JSON array, nothing else.
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
