import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
import time


class UR3ControllerServer(Node):

    def __init__(self):
        super().__init__('ur3_controller_server')

        self.get_logger().info("UR3 Controller Server Node has been started.")

        # WORLD STATE
        self.object_names = []
        self.object_poses = {}
        self.ee_pose = None
        self.joint_states = None
        self.held_item = None
        self.is_moving = False

        # FLAGS TO PREVENT LOG SPAM
        self._objects_logged = False
        self._last_object_count = 0
        self._last_object_names = []

        # SUBSCRIBERS
        self.create_subscription(String, 'falcon/object_names', self.cb_object_names, 10)
        self.create_subscription(PoseArray, 'falcon/object_poses', self.cb_object_poses, 10)
        self.create_subscription(Pose, 'ur3/ee_pose', self.cb_ee_pose, 10)
        self.create_subscription(JointState, 'ur3/joint_states', self.cb_joint_states, 10)
        self.create_subscription(String, 'ur3/held_item', self.cb_held_item, 10)
        self.create_subscription(Bool, 'ur3/is_moving', self.cb_is_moving, 10)

        # PUBLISHERS
        self.ik_pub = self.create_publisher(Pose, 'ur3/ik_goal', 10)
        self.gripper_pub = self.create_publisher(Bool, 'ur3/gripper_command', 10)

    # ============================================================
    # CALLBACKS - OPTIMIZED TO PREVENT LOG SPAM
    # ============================================================
    def cb_object_names(self, msg):
        new_names = msg.data.lower().split(',')
        
        # Only log if names changed
        if new_names != self._last_object_names:
            self.object_names = new_names
            self._last_object_names = new_names
            self.get_logger().info(f"Object names updated: {self.object_names}")

    def cb_object_poses(self, msg):
        for i, name in enumerate(self.object_names):
            if i < len(msg.poses):
                self.object_poses[name] = msg.poses[i]
        
        # Only log once or when object count changes
        current_count = len(self.object_poses)
        if not self._objects_logged or current_count != self._last_object_count:
            self.get_logger().info(f"Objects available: {list(self.object_poses.keys())}")
            if self.object_poses and 'cube' in self.object_poses:
                cube = self.object_poses['cube']
                self.get_logger().info(
                    f"Cube position: x={cube.position.x:.2f}, "
                    f"y={cube.position.y:.2f}, z={cube.position.z:.2f}"
                )
            self._objects_logged = True
            self._last_object_count = current_count

    def cb_ee_pose(self, msg):
        self.ee_pose = msg
        # Don't log - too frequent

    def cb_joint_states(self, msg):
        self.joint_states = msg
        # Don't log - too frequent

    def cb_held_item(self, msg):
        old_item = self.held_item
        self.held_item = msg.data
        
        # Only log if changed
        if old_item != self.held_item:
            self.get_logger().info(f"Held item: {self.held_item}")

    def cb_is_moving(self, msg):
        old_moving = self.is_moving
        self.is_moving = msg.data
        
        # Only log if changed
        if old_moving != self.is_moving:
            self.get_logger().info(f"Robot moving: {self.is_moving}")

    # ============================================================
    # REQUIRED METHODS FOR TOOLEXECUTOR
    # ============================================================
    
    def find_object_fuzzy(self, name: str):
        """
        Fuzzy match object name. Required by ToolExecutor.
        
        Args:
            name: Object name to search for
            
        Returns:
            Exact object name from object_poses, or None if not found
        """
        if not self.object_poses:
            self.get_logger().warn("No objects available in object_poses")
            return None
        
        # Clean up the input name
        name_clean = name.lower().replace("the ", "").replace("a ", "").replace("an ", "").strip()
        
        # Try exact match first
        for obj_name in self.object_poses.keys():
            if obj_name.lower() == name_clean:
                self.get_logger().info(f"Found object: '{name}' -> '{obj_name}'")
                return obj_name
        
        # Try partial match
        for obj_name in self.object_poses.keys():
            if name_clean in obj_name.lower() or obj_name.lower() in name_clean:
                self.get_logger().info(f"Fuzzy match: '{name}' -> '{obj_name}'")
                return obj_name
        
        # No match found
        self.get_logger().error(f"Object '{name}' not found! Available: {list(self.object_poses.keys())}")
        return None
    
    def collect_world_state(self):
        """
        Collect current world state. Required by ToolExecutor.
        
        Returns:
            Dictionary with objects, held_item, and robot status
        """
        world_state = {
            "objects": {
                name: {
                    "x": pose.position.x,
                    "y": pose.position.y,
                    "z": pose.position.z,
                    "orientation": {
                        "x": pose.orientation.x,
                        "y": pose.orientation.y,
                        "z": pose.orientation.z,
                        "w": pose.orientation.w
                    }
                }
                for name, pose in self.object_poses.items()
            },
            "held_item": self.held_item,
            "robot_status": {
                "is_moving": self.is_moving,
                "ee_pose": {
                    "x": self.ee_pose.position.x,
                    "y": self.ee_pose.position.y,
                    "z": self.ee_pose.position.z
                } if self.ee_pose else None
            }
        }
        
        return world_state

    # ============================================================
    # INTERNAL UTILS (kept for backward compatibility)
    # ============================================================
    def wait(self, seconds):
        time.sleep(seconds)

    def fuzzy_find(self, name: str):
        """Legacy method - redirects to find_object_fuzzy"""
        return self.find_object_fuzzy(name)

    def move_to_pose(self, pose):
        self.ik_pub.publish(pose)
        self.wait(1.5)

    def control_gripper(self, close: bool):
        self.gripper_pub.publish(Bool(data=close))
        self.wait(0.8)

    # ============================================================
    # LEGACY TOOL IMPLEMENTATIONS (kept for backward compatibility)
    # These are NOT used by the new ToolExecutor, but kept in case
    # you have other code that calls them directly
    # ============================================================

    def tool_pick_up_object(self, object_name: str):
        """Legacy pick method - not used by new ToolExecutor"""
        real = self.fuzzy_find(object_name)
        if not real:
            raise RuntimeError(f"Object not found: {object_name}")

        target = self.object_poses[real]

        # Approach
        approach = Pose()
        approach.position.x = target.position.x
        approach.position.y = target.position.y
        approach.position.z = target.position.z + 12.0
        approach.orientation = target.orientation
        self.move_to_pose(approach)

        # Descend
        grab = Pose()
        grab.position.x = target.position.x
        grab.position.y = target.position.y
        grab.position.z = target.position.z + 2.0
        grab.orientation = target.orientation
        self.move_to_pose(grab)

        # Grab
        self.control_gripper(True)

        # Lift
        self.move_to_pose(approach)

        return {"status": "success"}

    def tool_place_on_object(self, target_name: str, offset_z: float = 0.0):
        """Legacy place method - not used by new ToolExecutor"""
        real = self.fuzzy_find(target_name)
        if not real:
            raise RuntimeError(f"Target not found: {target_name}")

        target = self.object_poses[real]

        # Approach
        approach = Pose()
        approach.position.x = target.position.x
        approach.position.y = target.position.y
        approach.position.z = target.position.z + 12.0 + offset_z
        approach.orientation = target.orientation
        self.move_to_pose(approach)

        # Drop position
        drop = Pose()
        drop.position.x = target.position.x
        drop.position.y = target.position.y
        drop.position.z = target.position.z + offset_z
        drop.orientation = target.orientation
        self.move_to_pose(drop)

        # Release
        self.control_gripper(False)

        # Retreat
        self.move_to_pose(approach)

        return {"status": "success"}

    def tool_move_relative(self, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0):
        """Legacy move_relative - not used by new ToolExecutor"""
        if self.ee_pose is None:
            raise RuntimeError("EE pose unknown")

        newp = Pose()
        newp.position.x = self.ee_pose.position.x + dx
        newp.position.y = self.ee_pose.position.y + dy
        newp.position.z = self.ee_pose.position.z + dz
        newp.orientation = self.ee_pose.orientation
        self.move_to_pose(newp)
        return {"status": "success"}

    def tool_wait_seconds(self, duration: float):
        """Legacy wait method - not used by new ToolExecutor"""
        self.wait(duration)
        return {"status": "success"}

    def tool_push_button(self, button_name: str):
        """Legacy button press - not used by new ToolExecutor"""
        real = self.fuzzy_find(button_name)
        if not real:
            raise RuntimeError("Button not found")

        target = self.object_poses[real]

        hover = Pose()
        hover.position.x = target.position.x
        hover.position.y = target.position.y
        hover.position.z = target.position.z + 10.0
        hover.orientation = target.orientation

        press = Pose()
        press.position = target.position
        press.orientation = target.orientation

        self.move_to_pose(hover)
        self.move_to_pose(press)
        self.move_to_pose(hover)

        return {"status": "success"}
