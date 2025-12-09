from typing import Any, Dict
from .planner import Planner
from .tool_executor import ToolExecutor
from .utils.logging_utils import setup_logger

logger = setup_logger(__name__)

class UR3ControllerApp:
    """
    High-level orchestrator for planning and executing tasks.
    For local dev, this is a simple Python class.
    On the VM, this logic will be embedded in a ROS2 node.
    """

    def __init__(self) -> None:
        self.planner = Planner()
        self.executor = ToolExecutor()

    def run_task(self, task_text: str, world_state: Dict[str, Any] | None = None) -> None:
        logger.info(f"Running task: {task_text!r}")
        plan = self.planner.plan_task(task_text=task_text, world_state=world_state or {})
        self.executor.execute_plan(plan)

if __name__ == "__main__":
    # Simple local test harness
    app = UR3ControllerApp()
    app.run_task("Move to home and get the world state.")
