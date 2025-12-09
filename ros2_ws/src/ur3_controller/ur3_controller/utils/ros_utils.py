from __future__ import annotations

from threading import Lock
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from .logging_utils import setup_logger

logger = setup_logger(__name__)


class UR3ROSInterface(Node):
    """
    ROS2 interface for the UR3 + FalconSim environment.

    Responsibilities:
    - Subscribe to robot + world topics (EE pose, object poses, held item, moving flag).
    - Provide a clean Python API for getting world state.
    - Provide methods to send IK/FK/gripper commands.
    """

    def __init__(self, node_name: str = "ur3_ros_interface") -> None:
        super().__init__(node_name)

        qos = QoSProfile(depth=10)

        # TODO: Replace `Any` with actual message types from the provided ROS package
        # e.g. from geometry_msgs.msg import PoseStamped
        # from some_msgs.msg import ObjectPoses, HeldItem, Bool

        # --- Internal state (protected by a lock) ---
        self._lock = Lock()
        self._ee_pose: Optional[Any] = None
        self._object_poses: Dict[str, Any] = {}
        self._held_item: Optional[str] = None
        self._is_moving: bool = False

        # --- Subscribers ---
        # TODO: replace topic names + message types with actual ones from VM
        # Example:
        # self.ee_pose_sub = self.create_subscription(
        #     PoseStamped,
        #     "/ur3/ee_pose",
        #     self._ee_pose_callback,
        #     qos,
        # )

        # self.object_poses_sub = self.create_subscription(
        #     ObjectPoses,
        #     "/ur3/object_poses",
        #     self._object_poses_callback,
        #     qos,
        # )

        # self.held_item_sub = self.create_subscription(
        #     HeldItem,
        #     "/ur3/held_item",
        #     self._held_item_callback,
        #     qos,
        # )

        # self.is_moving_sub = self.create_subscription(
        #     Bool,
        #     "/ur3/is_moving",
        #     self._is_moving_callback,
        #     qos,
        # )

        # --- Publishers / Service Clients for commands ---
        # TODO: check if UR3 uses publishers or services for IK/FK/gripper
        # and initialize them accordingly.

        # Example (if using publishers):
        # self.ik_pub = self.create_publisher(IKRequest, "/ur3/set_ik", qos)
        # self.fk_pub = self.create_publisher(FKRequest, "/ur3/set_fk", qos)
        # self.gripper_pub = self.create_publisher(GripperCommand, "/ur3/set_gripper", qos)

        logger.info("UR3ROSInterface initialized.")

    # ==========================================================
    # Subscriber callbacks
    # ==========================================================

    def _ee_pose_callback(self, msg: Any) -> None:
        with self._lock:
            self._ee_pose = msg
        # logger.debug(f"EE pose updated: {msg}")

    def _object_poses_callback(self, msg: Any) -> None:
        # TODO: parse message and fill dict mapping object_name -> pose
        parsed: Dict[str, Any] = {}
        # Example:
        # for obj in msg.objects:
        #     parsed[obj.name] = obj.pose
        with self._lock:
            self._object_poses = parsed
        # logger.debug(f"Object poses updated: {parsed}")

    def _held_item_callback(self, msg: Any) -> None:
        # TODO: extract held item name (string) from msg
        held_item_name: Optional[str] = None
        with self._lock:
            self._held_item = held_item_name
        # logger.debug(f"Held item updated: {held_item_name}")

    def _is_moving_callback(self, msg: Any) -> None:
        # TODO: extract bool from msg
        is_moving: bool = False
        with self._lock:
            self._is_moving = is_moving
        # logger.debug(f"Is moving updated: {is_moving}")

    # ==========================================================
    # World state access
    # ==========================================================

    def get_world_state(self) -> Dict[str, Any]:
        """
        Snapshot of the current known world state.
        Used by the planner (for get_world_state tool).
        """
        with self._lock:
            return {
                "ee_pose": self._ee_pose,
                "object_poses": dict(self._object_poses),
                "held_item": self._held_item,
                "is_moving": self._is_moving,
            }

    def is_robot_moving(self) -> bool:
        with self._lock:
            return self._is_moving

    # ==========================================================
    # Command methods (IK/FK/Gripper)
    # ==========================================================

    def move_to_position_ik(
        self,
        x: float,
        y: float,
        z: float,
        roll: float | None = None,
        pitch: float | None = None,
        yaw: float | None = None,
        speed: float | None = None,
    ) -> None:
        """
        Send an IK motion command to move the EE to the given pose.
        """
        # TODO: construct and publish / call service request
        # Example:
        # msg = IKRequest()
        # msg.target.position.x = x
        # ...
        # self.ik_pub.publish(msg)
        self.get_logger().info(f"[IK] move_to_position_ik: ({x}, {y}, {z}), speed={speed}")

    def move_to_named_pose(self, pose_name: str) -> None:
        """
        Move to a predefined, safe named pose (e.g., 'home', 'drop_zone').
        Implementation depends on what the challenge exposes (FK / predefined poses).
        """
        # TODO: send appropriate FK / named-pose command
        self.get_logger().info(f"[FK] move_to_named_pose: {pose_name}")

    def move_relative(self, dx: float, dy: float, dz: float) -> None:
        """
        Relative cartesian move. Might be implemented via current EE pose + IK.
        """
        # TODO: read current pose, apply offsets, call IK
        self.get_logger().info(f"[IK] move_relative: dx={dx}, dy={dy}, dz={dz}")

    def set_gripper(self, open_gripper: bool) -> None:
        """
        Open or close the gripper.
        """
        # TODO: send gripper command
        state = "open" if open_gripper else "close"
        self.get_logger().info(f"[GRIPPER] set_gripper: {state}")

    # ==========================================================
    # Motion monitoring
    # ==========================================================

    def wait_until_still(self, timeout_s: float) -> None:
        """
        Block until the robot is no longer moving, or timeout.
        """
        import time

        start = time.time()
        while time.time() - start < timeout_s:
            if not self.is_robot_moving():
                self.get_logger().info("Robot stopped moving.")
                return
            time.sleep(0.05)
        self.get_logger().warning(f"wait_until_still timed out after {timeout_s} seconds.")
