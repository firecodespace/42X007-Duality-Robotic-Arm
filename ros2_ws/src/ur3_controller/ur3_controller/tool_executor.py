from __future__ import annotations

from typing import Any, Dict, List, Optional, Tuple
import time

from .planner import ToolCall
from .defined_tools import ToolSpec, get_tool_specs
from .utils.logging_utils import setup_logger
from .utils.ros_utils import UR3ROSInterface
from .error_handler import GracefulErrorHandler

logger = setup_logger(__name__)


class ExecutionResult:
    """Track execution status of a tool call."""
    def __init__(self, tool_name: str, success: bool, error_msg: str = "", attempt: int = 1):
        self.tool_name = tool_name
        self.success = success
        self.error_msg = error_msg
        self.attempt = attempt
        self.timestamp = time.time()

    def __repr__(self):
        status = "✓" if self.success else "✗"
        return f"{status} {self.tool_name} (attempt {self.attempt}): {self.error_msg or 'success'}"


class ToolExecutor:
    """
    High-level executor of tool calls using the UR3ROSInterface.

    Each public tool method translates into one or more ROS-level
    commands on the UR3ROSInterface.
    
    Features:
    - Graceful error handling with retries
    - Execution tracking and logging
    - Fallback strategies for common failures
    """

    def __init__(self, ros_interface: UR3ROSInterface, tool_specs: List[ToolSpec] | None = None) -> None:
        self.ros = ros_interface
        self.tool_specs = tool_specs or get_tool_specs()
        self.tool_map: Dict[str, ToolSpec] = {t.name: t for t in self.tool_specs}
        self.execution_log: List[ExecutionResult] = []
        self.error_handler = GracefulErrorHandler()
        self.max_retries = 2
        self.failed_tools: List[str] = []

    def execute_plan(self, plan: List[ToolCall]) -> Tuple[bool, str]:
        """
        Execute plan with graceful degradation.
        
        Returns:
            (success: bool, summary: str)
        """
        logger.info(f"Executing plan with {len(plan)} steps.")
        self.execution_log.clear()
        self.failed_tools.clear()
        
        completed_steps = 0
        for idx, call in enumerate(plan):
            logger.info(f"[Step {idx+1}/{len(plan)}] {call.tool}({call.args})")
            success = self._execute_with_retry(call)
            
            if success:
                completed_steps += 1
            else:
                logger.warning(f"Tool '{call.tool}' failed, continuing with next steps...")
                self.failed_tools.append(call.tool)
        
        summary = self._generate_summary(len(plan), completed_steps)
        return completed_steps > 0, summary

    def _execute_with_retry(self, call: ToolCall, attempt: int = 1) -> bool:
        """Execute a single tool call with retry logic and graceful fallback."""
        try:
            self.execute_step(call)
            result = ExecutionResult(call.tool, success=True, attempt=attempt)
            self.execution_log.append(result)
            logger.debug(f"Tool '{call.tool}' succeeded on attempt {attempt}")
            return True
        except Exception as e:
            error_msg = str(e)
            result = ExecutionResult(call.tool, success=False, error_msg=error_msg, attempt=attempt)
            self.execution_log.append(result)
            
            # Use error handler to decide on recovery strategy
            handling_info = self.error_handler.handle_tool_failure(
                tool_name=call.tool,
                error=e,
                attempt=attempt,
                max_retries=self.max_retries
            )
            
            if handling_info["action"] == "retry":
                logger.info(f"Retrying '{call.tool}'...")
                time.sleep(0.5)
                return self._execute_with_retry(call, attempt=attempt + 1)
            elif handling_info["action"] == "skip":
                logger.warning(f"Skipping '{call.tool}' due to fallback strategy")
                self.failed_tools.append(call.tool)
                return False
            elif handling_info["action"] == "fallback":
                logger.warning(f"Fallback strategy applied for '{call.tool}', continuing...")
                self.failed_tools.append(call.tool)
                return False
            else:  # abort
                logger.error(f"Critical failure in '{call.tool}', cannot continue")
                self.failed_tools.append(call.tool)
                return False

    def _generate_summary(self, total_steps: int, completed_steps: int) -> str:
        """Generate execution summary."""
        success_rate = (completed_steps / total_steps * 100) if total_steps > 0 else 0
        summary = f"Execution Summary: {completed_steps}/{total_steps} steps completed ({success_rate:.1f}%)"
        
        if self.failed_tools:
            summary += f"\nFailed tools: {', '.join(self.failed_tools)}"
        
        return summary

    def execute_step(self, call: ToolCall) -> None:
        tool_name = call.tool
        args = call.args

        if tool_name == "move_to_position":
            self._move_to_position(**args)
        elif tool_name == "move_to_named_pose":
            self._move_to_named_pose(**args)
        elif tool_name == "move_above_object":
            self._move_above_object(**args)
        elif tool_name == "move_relative":
            self._move_relative(**args)
        elif tool_name == "open_gripper":
            self._open_gripper()
        elif tool_name == "close_gripper":
            self._close_gripper()
        elif tool_name == "pick_object":
            self._pick_object(**args)
        elif tool_name == "place_object_at":
            self._place_object_at(**args)
        elif tool_name == "press_button":
            self._press_button(**args)
        elif tool_name == "get_world_state":
            self._get_world_state()
        elif tool_name == "wait_until_still":
            self._wait_until_still(**args)
        else:
            logger.error(f"Unknown tool: {tool_name}")

    # ==========================================================
    # Primitive tools
    # ==========================================================

    def _move_to_position(self, x: float, y: float, z: float, speed: float | None = None) -> None:
        logger.info(f"move_to_position to ({x}, {y}, {z}) speed={speed}")
        # You can decide how to handle orientation (roll/pitch/yaw) here or via config
        self.ros.move_to_position_ik(x=x, y=y, z=z, roll=None, pitch=None, yaw=None, speed=speed)

    def _move_to_named_pose(self, pose_name: str) -> None:
        logger.info(f"move_to_named_pose: {pose_name}")
        self.ros.move_to_named_pose(pose_name)

    def _move_above_object(self, object_name: str, dz: float) -> None:
        logger.info(f"move_above_object: {object_name}, dz={dz}")
        ws = self.ros.get_world_state()
        object_poses = ws.get("object_poses") or {}
        if object_name not in object_poses:
            logger.warning(f"Object {object_name!r} not found in world state.")
            return

        pose = object_poses[object_name]
        # TODO: Extract xyz from pose appropriately
        # Example (if pose has position.x/y/z):
        # x = pose.position.x
        # y = pose.position.y
        # z = pose.position.z + dz

        x = 0.0
        y = 0.0
        z = dz
        self.ros.move_to_position_ik(x, y, z, roll=None, pitch=None, yaw=None, speed=None)

    def _move_relative(self, dx: float, dy: float, dz: float) -> None:
        logger.info(f"move_relative by ({dx}, {dy}, {dz})")
        self.ros.move_relative(dx, dy, dz)

    def _open_gripper(self) -> None:
        logger.info("open_gripper")
        self.ros.set_gripper(open_gripper=True)

    def _close_gripper(self) -> None:
        logger.info("close_gripper")
        self.ros.set_gripper(open_gripper=False)

    # ==========================================================
    # Composite tools
    # ==========================================================

    def _pick_object(self, object_name: str) -> None:
        logger.info(f"pick_object: {object_name}")
        # Strategy:
        # 1. move_above_object(object_name, dz)
        # 2. move_relative(0, 0, -dz_down)
        # 3. close_gripper()
        # 4. move_relative(0, 0, dz_up)
        dz_above = 0.10
        dz_down = 0.10
        dz_up = 0.10

        try:
            self._move_above_object(object_name=object_name, dz=dz_above)
            self._wait_until_still(timeout_s=5.0)

            self._move_relative(dx=0.0, dy=0.0, dz=-dz_down)
            self._wait_until_still(timeout_s=5.0)

            self._close_gripper()
            self._wait_until_still(timeout_s=5.0)

            self._move_relative(dx=0.0, dy=0.0, dz=dz_up)
            self._wait_until_still(timeout_s=5.0)
            
            logger.info(f"Successfully picked {object_name}")
        except Exception as e:
            logger.warning(f"Pick operation for {object_name} encountered issue: {e}")
            # Attempt to recover by opening gripper and moving up
            try:
                self._open_gripper()
                self._move_relative(dx=0.0, dy=0.0, dz=0.15)
                self._wait_until_still(timeout_s=3.0)
            except:
                pass
            raise

    def _place_object_at(
        self,
        target_type: str,
        x: float | None = None,
        y: float | None = None,
        z: float | None = None,
        pose_name: str | None = None,
    ) -> None:
        logger.info(f"place_object_at: target_type={target_type}, pos=({x},{y},{z}), pose_name={pose_name}")
        if target_type == "position":
            if x is None or y is None or z is None:
                logger.error("place_object_at(target_type='position') requires x, y, z.")
                return
            self._move_to_position(x, y, z, speed=None)
        elif target_type == "named_pose":
            if pose_name is None:
                logger.error("place_object_at(target_type='named_pose') requires pose_name.")
                return
            self._move_to_named_pose(pose_name)
        else:
            logger.error(f"Unknown target_type for place_object_at: {target_type}")
            return

        self._wait_until_still(timeout_s=5.0)
        self._open_gripper()
        self._wait_until_still(timeout_s=5.0)

    def _press_button(self, button_name: str) -> None:
        logger.info(f"press_button: {button_name}")
        # Implementation similar to pick, but with a tap instead of grasp
        dz_above = 0.10
        dz_down = 0.05

        self._move_above_object(object_name=button_name, dz=dz_above)
        self._wait_until_still(timeout_s=5.0)

        self._move_relative(dx=0.0, dy=0.0, dz=-dz_down)
        self._wait_until_still(timeout_s=5.0)

        # Lift back
        self._move_relative(dx=0.0, dy=0.0, dz=dz_down)
        self._wait_until_still(timeout_s=5.0)

    def _get_world_state(self) -> None:
        ws = self.ros.get_world_state()
        logger.info(f"World state: {ws}")

    def _wait_until_still(self, timeout_s: float) -> None:
        logger.info(f"wait_until_still: timeout_s={timeout_s}")
        self.ros.wait_until_still(timeout_s=timeout_s)
