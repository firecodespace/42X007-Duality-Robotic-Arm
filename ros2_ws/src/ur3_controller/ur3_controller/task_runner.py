"""
Task runner for 6 standard robotic manipulation tasks.

Provides a simple interface to execute predefined tasks with graceful error handling.
"""

from typing import Dict, Any, List, Tuple
import json
from pathlib import Path
import yaml

from .planner import Planner
from .tool_executor import ToolExecutor
from .utils.logging_utils import setup_logger
from .config import CONFIG_DIR, LLM_PLAN_LOG_DIR

logger = setup_logger(__name__)


class Task:
    """Represents a single task to execute."""
    
    def __init__(self, task_id: int, name: str, description: str, instructions: str):
        self.id = task_id
        self.name = name
        self.description = description
        self.instructions = instructions
    
    def __repr__(self):
        return f"Task {self.id}: {self.name} - {self.description}"


class TaskRunner:
    """
    Executes predefined robotic manipulation tasks.
    
    Features:
    - Load tasks from configuration
    - Execute with minimal setup
    - Graceful error handling
    - Comprehensive logging
    """
    
    def __init__(self):
        self.planner = Planner()
        self.executor = ToolExecutor(ros_interface=None)  # TODO: Initialize with ROS interface
        self.tasks: List[Task] = []
        self.results: Dict[int, Dict[str, Any]] = {}
        self._load_tasks()
    
    def _load_tasks(self) -> None:
        """Load task definitions from configuration."""
        tasks_config_path = CONFIG_DIR / "tasks_config.yaml"
        
        if not tasks_config_path.exists():
            logger.warning(f"Tasks configuration not found at {tasks_config_path}")
            # Create minimal default tasks
            self._create_default_tasks()
            return
        
        try:
            with open(tasks_config_path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f)
            
            for task_data in data.get("tasks", []):
                task = Task(
                    task_id=task_data["id"],
                    name=task_data["name"],
                    description=task_data["description"],
                    instructions=task_data["instructions"]
                )
                self.tasks.append(task)
            
            logger.info(f"Loaded {len(self.tasks)} tasks from configuration")
        except Exception as e:
            logger.error(f"Failed to load tasks configuration: {e}")
            self._create_default_tasks()
    
    def _create_default_tasks(self) -> None:
        """Create minimal default tasks if configuration not available."""
        default_tasks = [
            Task(1, "pick_and_place_basic", "Pick and place basic object",
                 "Pick up the red_cube and place it at the drop_zone"),
            Task(2, "press_button", "Press a button sequence",
                 "Move to home position, then press the button"),
            Task(3, "position_object", "Position object precisely",
                 "Pick the blue_sphere and move it to a safe location"),
            Task(4, "rearrange_objects", "Rearrange multiple objects",
                 "Pick up the green_block and place it next to the red_cube"),
            Task(5, "sequential_manipulation", "Execute sequential picks",
                 "Pick red_cube, move to drop_zone, then return to home"),
            Task(6, "adaptive_placement", "Adapt to available objects",
                 "Get world state, pick any available object, place at home"),
        ]
        self.tasks = default_tasks
        logger.info(f"Using {len(self.tasks)} default tasks")
    
    def list_tasks(self) -> List[Task]:
        """List all available tasks."""
        return self.tasks
    
    def get_task(self, task_id: int) -> Task | None:
        """Get a specific task by ID."""
        for task in self.tasks:
            if task.id == task_id:
                return task
        return None
    
    def run_task(
        self,
        task_id: int,
        world_state: Dict[str, Any] | None = None
    ) -> Tuple[bool, str, Dict[str, Any]]:
        """
        Execute a single task.
        
        Args:
            task_id: Task ID to execute
            world_state: Optional world state snapshot
        
        Returns:
            (success: bool, summary: str, result_dict: dict)
        """
        task = self.get_task(task_id)
        if not task:
            msg = f"Task {task_id} not found"
            logger.error(msg)
            return False, msg, {}
        
        logger.info(f"\n{'='*60}")
        logger.info(f"Starting {task}")
        logger.info(f"{'='*60}")
        
        try:
            # Step 1: Plan
            logger.info(f"Planning: {task.instructions}")
            plan = self.planner.plan_task(
                task_text=task.instructions,
                world_state=world_state or {}
            )
            
            logger.info(f"Generated plan with {len(plan)} steps")
            for idx, step in enumerate(plan, 1):
                logger.debug(f"  Step {idx}: {step.tool}({step.args})")
            
            # Step 2: Execute
            logger.info("Executing plan...")
            success, exec_summary = self.executor.execute_plan(plan)
            
            # Step 3: Log results
            result = {
                "task_id": task_id,
                "task_name": task.name,
                "success": success,
                "plan_size": len(plan),
                "execution_summary": exec_summary,
                "failed_tools": self.executor.failed_tools,
            }
            self.results[task_id] = result
            
            logger.info(f"Task {task_id} completed: {exec_summary}")
            logger.info(f"{'='*60}\n")
            
            return success, exec_summary, result
        
        except Exception as e:
            error_msg = f"Task {task_id} execution failed: {e}"
            logger.error(error_msg)
            result = {
                "task_id": task_id,
                "task_name": task.name,
                "success": False,
                "error": str(e),
            }
            self.results[task_id] = result
            return False, error_msg, result
    
    def run_all_tasks(
        self,
        world_state: Dict[str, Any] | None = None,
        skip_on_failure: bool = False
    ) -> Dict[str, Any]:
        """
        Execute all tasks sequentially.
        
        Args:
            world_state: Optional world state snapshot
            skip_on_failure: Skip remaining tasks if one fails
        
        Returns:
            Summary dictionary with all results
        """
        logger.info(f"\n\n{'#'*60}")
        logger.info(f"# RUNNING ALL {len(self.tasks)} TASKS")
        logger.info(f"{'#'*60}\n")
        
        results_summary = {
            "total_tasks": len(self.tasks),
            "completed": 0,
            "successful": 0,
            "failed": 0,
            "task_results": []
        }
        
        for task in self.tasks:
            success, summary, result = self.run_task(task.id, world_state)
            results_summary["task_results"].append(result)
            results_summary["completed"] += 1
            
            if success:
                results_summary["successful"] += 1
            else:
                results_summary["failed"] += 1
                if skip_on_failure:
                    logger.warning("Skipping remaining tasks due to failure")
                    break
        
        # Log final summary
        self._log_final_summary(results_summary)
        return results_summary
    
    def _log_final_summary(self, summary: Dict[str, Any]) -> None:
        """Log a summary of all task executions."""
        success_rate = (summary["successful"] / summary["total_tasks"] * 100) \
            if summary["total_tasks"] > 0 else 0
        
        logger.info(f"\n{'='*60}")
        logger.info(f"FINAL SUMMARY")
        logger.info(f"{'='*60}")
        logger.info(f"Total tasks: {summary['total_tasks']}")
        logger.info(f"Completed: {summary['completed']}")
        logger.info(f"Successful: {summary['successful']}")
        logger.info(f"Failed: {summary['failed']}")
        logger.info(f"Success rate: {success_rate:.1f}%")
        
        if summary["failed"] > 0:
            logger.warning("Failed tasks:")
            for result in summary["task_results"]:
                if not result["success"]:
                    logger.warning(f"  - Task {result['task_id']}: {result.get('error', 'Unknown error')}")
        
        logger.info(f"{'='*60}\n")
    
    def save_results(self, output_path: Path | None = None) -> Path:
        """Save execution results to JSON file."""
        if output_path is None:
            LLM_PLAN_LOG_DIR.mkdir(parents=True, exist_ok=True)
            output_path = LLM_PLAN_LOG_DIR / "task_execution_results.json"
        
        with open(output_path, "w", encoding="utf-8") as f:
            json.dump(self.results, f, indent=2, default=str)
        
        logger.info(f"Results saved to {output_path}")
        return output_path
