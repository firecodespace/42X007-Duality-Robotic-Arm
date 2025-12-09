from typing import Any, Dict, List
from .defined_tools import ToolSpec
from .utils.logging_utils import setup_logger

logger = setup_logger(__name__)

class GeminiClient:
    """
    Stub LLM client for local development.
    On the VM, replace generate_plan() with real Gemini API integration.
    """

    def __init__(self, model_name: str = "gemini-2.5") -> None:
        self.model_name = model_name

    def generate_plan(
        self,
        task_text: str,
        tool_specs: List[ToolSpec],
        world_state: Dict[str, Any],
    ) -> str:
        """
        Return a JSON string of an array of tool calls.
        For now, returns a trivial hard-coded plan so the rest of
        the system can be developed and tested.
        """
        logger.warning("GeminiClient.generate_plan is using a stubbed implementation.")
        # Example stub: move home and query world state
        return """
        [
          {"tool": "move_to_named_pose", "args": {"pose_name": "home"}},
          {"tool": "get_world_state", "args": {}}
        ]
        """
