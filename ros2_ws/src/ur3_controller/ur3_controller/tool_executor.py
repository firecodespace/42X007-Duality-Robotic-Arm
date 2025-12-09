from typing import Any, Dict, List
from .planner import ToolCall
from .defined_tools import ToolSpec, get_tool_specs
from .utils.logging_utils import setup_logger

logger = setup_logger(__name__)

class ToolExecutor:
    def __init__(self, tool_specs: List[ToolSpec] | None = None) -> None:
        self.tool_specs = tool_specs or get_tool_specs()
        self.tool_map: Dict[str, ToolSpec] = {t.name: t for t in self.tool_specs}
        # ROS-related publishers/subscribers will be initialized later on the VM.

    def execute_plan(self, plan: List[ToolCall]) -> None:
        logger.info(f"Executing plan with {len(plan)} steps.")
        for idx, call in enumerate(plan):
            logger.info(f"Executing step {idx}: {call.tool}({call.args})")
            self.execute_step(call)

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

    # The methods below are stubs. On the VM, you'll fill them with ROS logic.

    def _move_to_position(self, x: float, y: float, z: float, speed: float | None = None) -> None:
        logger.info(f"[STUB] move_to_position to ({x}, {y}, {z}) with speed={speed}")

    def _move_to_named_pose(self, pose_name: str) -> None:
        logger.info(f"[STUB] move_to_named_pose: {pose_name}")

    def _move_above_object(self, object_name: str, dz: float) -> None:
        logger.info(f"[STUB] move_above_object: {object_name} with dz={dz}")

    def _move_relative(self, dx: float, dy: float, dz: float) -> None:
        logger.info(f"[STUB] move_relative by ({dx}, {dy}, {dz})")

    def _open_gripper(self) -> None:
        logger.info("[STUB] open_gripper")

    def _close_gripper(self) -> None:
        logger.info("[STUB] close_gripper")

    def _pick_object(self, object_name: str) -> None:
        logger.info(f"[STUB] pick_object: {object_name}")
        # Later: call _move_above_object, move down, close_gripper, lift, etc.

    def _place_object_at(
        self,
        target_type: str,
        x: float | None = None,
        y: float | None = None,
        z: float | None = None,
        pose_name: str | None = None,
    ) -> None:
        logger.info(f"[STUB] place_object_at: target_type={target_type}, pos=({x}, {y}, {z}), pose_name={pose_name}")

    def _press_button(self, button_name: str) -> None:
        logger.info(f"[STUB] press_button: {button_name}")

    def _get_world_state(self) -> None:
        logger.info("[STUB] get_world_state")

    def _wait_until_still(self, timeout_s: float) -> None:
        logger.info(f"[STUB] wait_until_still: timeout_s={timeout_s}")
