from typing import Any, Dict
from .planner import Planner
from .tool_executor import ToolExecutor
from .task_runner import TaskRunner
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
        self.executor = ToolExecutor(ros_interface=None)  # TODO: Initialize with ROS interface
        self.task_runner = TaskRunner()

    def run_task(self, task_text: str, world_state: Dict[str, Any] | None = None) -> None:
        """Run a single task by description."""
        logger.info(f"Running task: {task_text!r}")
        plan = self.planner.plan_task(task_text=task_text, world_state=world_state or {})
        success, summary = self.executor.execute_plan(plan)
        logger.info(summary)

    def run_predefined_task(self, task_id: int, world_state: Dict[str, Any] | None = None) -> None:
        """Run a predefined task by ID."""
        success, summary, result = self.task_runner.run_task(task_id, world_state)
        logger.info(summary)

    def run_all_predefined_tasks(self, world_state: Dict[str, Any] | None = None) -> None:
        """Run all 6 predefined tasks."""
        results = self.task_runner.run_all_tasks(world_state or {})
        self.task_runner.save_results()

if __name__ == "__main__":
    # Simple local test harness
    app = UR3ControllerApp()
    
    # List available tasks
    logger.info("Available tasks:")
    for task in app.task_runner.list_tasks():
        logger.info(f"  {task}")
    
    # Option 1: Run a single task
    logger.info("\nRunning task 1...")
    app.run_predefined_task(task_id=1)
    
    # Option 2: Run all tasks (commented out by default)
    # logger.info("\nRunning all tasks...")
    # app.run_all_predefined_tasks()
    
    # Option 3: Run custom task
    # app.run_task("Move to home and get the world state.")
