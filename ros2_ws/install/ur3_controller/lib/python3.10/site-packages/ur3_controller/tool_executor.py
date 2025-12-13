import time
import json
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool


class ToolExecutor:
    def __init__(self, server):
        self.server = server

        # Calibration - will be auto-adjusted
        self.TABLE_Z = 8.68
        self.MIN_SAFE_Z = self.TABLE_Z + 3.0
        self.APPROACH_OFFSET = 15.0
        self.GRASP_HEIGHT_ABOVE_TABLE = 4.0
        self.MOTION_TIMEOUT = 4.0
        self.GRIPPER_DELAY = 0.8

        # Workspace limits (UR3 typical: ~60cm reach)
        self.MIN_REACHABLE_DISTANCE = 8.0   # Minimum distance from base (cm)
        self.MAX_REACH = 65.0               # Maximum reach (cm)
        self.ARM_BASE_X = 0.0               # Base position (center)
        self.ARM_BASE_Y = 0.0

        # For relative placement
        self.last_picked_position = None
        self.last_picked_object = None
        self.failed_positions = []

    def execute(self, step: dict):
        """Execute a tool command from a step dictionary."""
        tool = step.get("tool", "").strip()
        args = step.get("args", {})

        if not tool:
            raise ValueError("Step missing 'tool' key")

        # Handle both "tool_name" and direct name
        tool = tool.replace("tool_", "").lower()

        fn = getattr(self, tool, None)
        if not fn:
            available = [m for m in dir(self) if not m.startswith('_') and callable(getattr(self, m))]
            raise AttributeError(f"Unknown tool: {tool}. Available: {available}")

        if not callable(fn):
            raise TypeError(f"Tool {tool} is not callable")

        try:
            return fn(**args)
        except Exception as e:
            print(f"\n❌ [ERROR] {tool}: {str(e)}")
            raise

    # =========================================================
    # WORKSPACE VALIDATION
    # =========================================================
    def _is_reachable(self, x, y, z, verbose=False):
        """Check if position is within robot workspace."""
        # Calculate distance from arm base
        distance = ((x - self.ARM_BASE_X)**2 + (y - self.ARM_BASE_Y)**2) ** 0.5
        
        issues = []

        # Check distance bounds
        if distance < self.MIN_REACHABLE_DISTANCE:
            issues.append(f"too close to base (dist={distance:.1f}cm, min={self.MIN_REACHABLE_DISTANCE:.1f}cm)")
        
        if distance > self.MAX_REACH:
            issues.append(f"too far from base (dist={distance:.1f}cm, max={self.MAX_REACH:.1f}cm)")
        
        # Check height bounds
        if z < self.MIN_SAFE_Z:
            issues.append(f"Z too low ({z:.1f}cm < {self.MIN_SAFE_Z:.1f}cm)")
        
        if issues:
            if verbose:
                print(f"   ⚠️  NOT REACHABLE: {', '.join(issues)}")
            return False
        
        return True

    def _get_workspace_status(self, x, y, z):
        """Get detailed workspace status."""
        distance = ((x - self.ARM_BASE_X)**2 + (y - self.ARM_BASE_Y)**2) ** 0.5
        
        status = {
            "x": x,
            "y": y,
            "z": z,
            "distance_from_base": distance,
            "reachable": self._is_reachable(x, y, z),
            "issues": []
        }
        
        if distance < self.MIN_REACHABLE_DISTANCE:
            status["issues"].append(f"Too close to base ({distance:.1f}cm)")
        elif distance > self.MAX_REACH:
            status["issues"].append(f"Too far from base ({distance:.1f}cm)")
        
        if z < self.MIN_SAFE_Z:
            status["issues"].append(f"Z too low ({z:.1f}cm)")
        
        return status

    def _send_ik_with_validation(self, pose, description="", allow_unreachable=False):
        """Send IK command and verify it actually moves."""
        target_x = pose.position.x
        target_y = pose.position.y
        target_z = pose.position.z

        # Check reachability
        if not self._is_reachable(target_x, target_y, target_z, verbose=True):
            if not allow_unreachable:
                status = self._get_workspace_status(target_x, target_y, target_z)
                raise RuntimeError(
                    f"Position ({target_x:.1f}, {target_y:.1f}, {target_z:.1f}) unreachable: {status['issues']}"
                )
            else:
                print(f"   ⚠️  WARNING: Attempting unreachable position (may fail)")

        # Get current pose
        cur = self._require_ee_pose()
        distance = (
            (target_x - cur.position.x)**2 + 
            (target_y - cur.position.y)**2 + 
            (target_z - cur.position.z)**2
        ) ** 0.5

        print(f"   📤 {description}")
        print(f"      From: ({cur.position.x:.1f}, {cur.position.y:.1f}, {cur.position.z:.1f})")
        print(f"      To:   ({target_x:.1f}, {target_y:.1f}, {target_z:.1f})")
        print(f"      Distance: {distance:.1f}cm")

        # Publish command
        self.server.ik_pub.publish(pose)
        
        # Wait for motion to start (with timeout)
        start = time.time()
        motion_started = False
        while time.time() - start < 1.0:
            if getattr(self.server, 'is_moving', False):
                motion_started = True
                print(f"      Motion started ✓")
                break
            time.sleep(0.05)

        if not motion_started:
            print(f"      ⚠️  Motion didn't start - IK may have failed, waiting anyway...")
            time.sleep(0.8)

        # Wait for motion to complete
        self._sleep_motion()
        
        # Verify final position
        final = self._require_ee_pose()
        error = (
            (final.position.x - target_x)**2 +
            (final.position.y - target_y)**2 +
            (final.position.z - target_z)**2
        ) ** 0.5

        if error > 2.0:
            print(f"      ⚠️  Position error: {error:.2f}cm (large)")
            print(f"         Requested: ({target_x:.1f}, {target_y:.1f}, {target_z:.1f})")
            print(f"         Actual:    ({final.position.x:.1f}, {final.position.y:.1f}, {final.position.z:.1f})")
            raise RuntimeError(f"IK failed - position error {error:.2f}cm")
        else:
            print(f"      ✓ Reached target (error: {error:.2f}cm)")

    # =========================================================
    # MOTION PRIMITIVES
    # =========================================================
    def move_to_position(self, x=None, y=None, z=None):
        """Move end-effector to absolute position."""
        cur = self._require_ee_pose()

        p = Pose()
        p.position.x = x if x is not None else cur.position.x
        p.position.y = y if y is not None else cur.position.y
        p.position.z = z if z is not None else cur.position.z
        p.orientation = cur.orientation

        if p.position.z < self.MIN_SAFE_Z:
            raise ValueError(f"Z {p.position.z:.2f} < MIN_SAFE_Z {self.MIN_SAFE_Z:.2f}")

        print(f"\n🤖 MOVE_TO: ({p.position.x:.1f}, {p.position.y:.1f}, {p.position.z:.1f})")
        self._send_ik_with_validation(p, "move_to")

    def move_relative(self, dx=0.0, dy=0.0, dz=0.0):
        """Move end-effector relative to current position."""
        cur = self._require_ee_pose()

        p = Pose()
        p.position.x = cur.position.x + dx
        p.position.y = cur.position.y + dy
        p.position.z = cur.position.z + dz
        p.orientation = cur.orientation

        if p.position.z < self.MIN_SAFE_Z:
            raise ValueError(f"Relative move would go below MIN_SAFE_Z")

        print(f"\n🤖 MOVE_REL: ({dx:+.1f}, {dy:+.1f}, {dz:+.1f})")
        self._send_ik_with_validation(p, "move_relative")

    # =========================================================
    # GRIPPER CONTROL
    # =========================================================
    def open_gripper(self):
        """Open the gripper."""
        if not hasattr(self.server, 'gripper_pub'):
            raise AttributeError("Server missing 'gripper_pub'")

        print(f"🔓 GRIPPER OPEN")
        self.server.gripper_pub.publish(Bool(data=False))
        time.sleep(self.GRIPPER_DELAY)

    def close_gripper(self):
        """Close the gripper with verification."""
        if not hasattr(self.server, 'gripper_pub'):
            raise AttributeError("Server missing 'gripper_pub'")

        print(f"🔒 GRIPPER CLOSE")
        self.server.gripper_pub.publish(Bool(data=True))
        
        # Wait for gripper to close and detect object
        max_attempts = 8
        for attempt in range(max_attempts):
            time.sleep(self.GRIPPER_DELAY)
            held = getattr(self.server, 'held_item', '')
            
            if held and held != '' and held != 'unknown':
                print(f"   ✅ Grasped: '{held}'")
                return
        
        # Still nothing - warn but continue
        print(f"   ⚠️  No object detected")

    # =========================================================
    # SMART PICKING WITH WORKSPACE AWARENESS
    # =========================================================
    def pick_object(self, object_name: str):
        """Pick up an object with workspace-aware retry logic."""
        name = self._resolve_object(object_name)
        target = self.server.object_poses[name]

        target_x = target.position.x
        target_y = target.position.y
        target_z = target.position.z

        print(f"\n📦 PICK: '{name}'")
        print(f"   Position: ({target_x:.1f}, {target_y:.1f}, {target_z:.1f})")

        # Check reachability
        distance = ((target_x - self.ARM_BASE_X)**2 + (target_y - self.ARM_BASE_Y)**2) ** 0.5
        status = self._get_workspace_status(target_x, target_y, target_z)

        print(f"   Distance from base: {distance:.1f}cm")
        if status["issues"]:
            print(f"   Issues: {status['issues']}")

        # If too close, try offset approach
        if distance < self.MIN_REACHABLE_DISTANCE:
            print(f"   ⚠️  Object too close to base, trying offset approach...")
            
            # Calculate offset direction (angle away from base)
            if target_x == 0 and target_y == 0:
                offset_x = self.MIN_REACHABLE_DISTANCE + 5.0
                offset_y = 0.0
            else:
                angle = (target_y if target_y != 0 else 0.1)
                magnitude = self.MIN_REACHABLE_DISTANCE + 8.0
                offset_x = magnitude * target_x / max(distance, 0.1)
                offset_y = magnitude * target_y / max(distance, 0.1)
            
            print(f"   Offset approach: ({target_x + offset_x:.1f}, {target_y + offset_y:.1f})")
            
            # Temporarily modify target
            target.position.x = target_x + offset_x
            target.position.y = target_y + offset_y

        if distance > self.MAX_REACH:
            raise RuntimeError(f"Object too far ({distance:.1f}cm > {self.MAX_REACH:.1f}cm)")

        self.last_picked_position = Pose().position
        self.last_picked_position.x = target_x
        self.last_picked_position.y = target_y
        self.last_picked_position.z = target_z
        self.last_picked_object = name

        # Calculate positions
        approach_z = target.position.z + self.APPROACH_OFFSET
        grasp_z = target.position.z + 1.0

        # Safety clamp
        if grasp_z < self.TABLE_Z:
            grasp_z = self.TABLE_Z + 1.0
        if grasp_z < self.MIN_SAFE_Z:
            grasp_z = self.MIN_SAFE_Z

        try:
            # STEP 1: Open gripper
            print(f"   [1/6] Opening gripper...")
            self.open_gripper()
            time.sleep(0.5)

            # STEP 2: Approach
            print(f"   [2/6] Approaching...")
            approach = Pose()
            approach.position.x = target.position.x
            approach.position.y = target.position.y
            approach.position.z = approach_z
            approach.orientation = target.orientation
            self._send_ik_with_validation(approach, "approach")
            time.sleep(0.3)

            # STEP 3: Descend
            print(f"   [3/6] Descending...")
            descend = Pose()
            descend.position.x = target.position.x
            descend.position.y = target.position.y
            descend.position.z = grasp_z
            descend.orientation = target.orientation
            self._send_ik_with_validation(descend, "descend")
            time.sleep(0.3)

            # STEP 4: Close gripper
            print(f"   [4/6] Closing gripper...")
            self.close_gripper()
            time.sleep(1.2)

            # STEP 5: Verify grip
            held = getattr(self.server, 'held_item', '')
            print(f"   [5/6] Verifying grip... (held='{held}')")
            if not held or held == '':
                print(f"   Retry closing...")
                self.close_gripper()
                time.sleep(1.2)
                held = getattr(self.server, 'held_item', '')
                if not held or held == '':
                    raise RuntimeError(f"Failed to grip {name}")

            # STEP 6: Lift
            print(f"   [6/6] Lifting...")
            self._send_ik_with_validation(approach, "lift")
            time.sleep(0.5)

            print(f"✅ PICK COMPLETE: '{name}'")

        except RuntimeError as e:
            print(f"   ❌ Pick failed: {str(e)}")
            self.failed_positions.append((target_x, target_y, target_z))
            raise

    # =========================================================
    # PLACEMENT WITH VALIDATION
    # =========================================================
    def place_object_at(self, x=None, y=None, z=None):
        """Place object at absolute coordinates."""
        cur = self._require_ee_pose()

        if z is None:
            z = self.TABLE_Z + 5.0

        p = Pose()
        p.position.x = x if x is not None else cur.position.x
        p.position.y = y if y is not None else cur.position.y
        p.position.z = max(z, self.MIN_SAFE_Z)
        p.orientation = cur.orientation

        print(f"\n📍 PLACE_AT: ({p.position.x:.1f}, {p.position.y:.1f}, {p.position.z:.1f})")

        # Check reachability
        if not self._is_reachable(p.position.x, p.position.y, p.position.z, verbose=True):
            raise RuntimeError(f"Placement position unreachable")

        try:
            # STEP 1: Move to position
            print(f"   [1/3] Moving to position...")
            self._send_ik_with_validation(p, "move_to_place")
            time.sleep(0.3)

            # STEP 2: Release
            print(f"   [2/3] Releasing...")
            self.open_gripper()
            time.sleep(0.5)

            # STEP 3: Retreat
            retreat = Pose()
            retreat.position.x = p.position.x
            retreat.position.y = p.position.y
            retreat.position.z = p.position.z + self.APPROACH_OFFSET
            retreat.orientation = p.orientation
            
            print(f"   [3/3] Retreating...")
            self._send_ik_with_validation(retreat, "retreat")
            time.sleep(0.3)

            print(f"✅ PLACE COMPLETE")

        except RuntimeError as e:
            print(f"   ❌ Placement failed: {str(e)}")
            raise

    def place_object_relative(self, dx=0.0, dy=0.0, dz=0.0, use_cm=True):
        """Place object relative to pickup location."""
        if self.last_picked_position is None:
            raise RuntimeError("No object picked yet")

        factor = 1.0 if use_cm else 0.01

        new_x = self.last_picked_position.x + (dx * factor)
        new_y = self.last_picked_position.y + (dy * factor)
        new_z = self.last_picked_position.z + (dz * factor) if dz != 0 else self.TABLE_Z + 2.0

        print(f"\n📏 PLACE_RELATIVE: ({dx:.0f}, {dy:.0f}, {dz:.0f})")
        print(f"   From: ({self.last_picked_position.x:.1f}, {self.last_picked_position.y:.1f})")
        print(f"   To:   ({new_x:.1f}, {new_y:.1f}, {new_z:.1f})")

        self.place_object_at(x=new_x, y=new_y, z=new_z)

    # =========================================================
    # BUTTON PRESSING
    # =========================================================
    def press_button(self, button_name: str):
        """Press a button."""
        name = self._resolve_object(button_name)
        target = self.server.object_poses[name]

        print(f"\n🔘 PRESS: '{name}'")

        if target.position.z < self.MIN_SAFE_Z:
            raise ValueError(f"Button {name} is below MIN_SAFE_Z")

        # Hover
        hover = Pose()
        hover.position.x = target.position.x
        hover.position.y = target.position.y
        hover.position.z = target.position.z + 10.0
        hover.orientation = target.orientation

        print(f"   [1/3] Hovering...")
        self._send_ik_with_validation(hover, "hover")

        # Press
        press = Pose()
        press.position.x = target.position.x
        press.position.y = target.position.y
        press.position.z = target.position.z
        press.orientation = target.orientation

        print(f"   [2/3] Pressing...")
        self.server.ik_pub.publish(press)
        time.sleep(0.8)

        # Retreat
        print(f"   [3/3] Retreating...")
        self._send_ik_with_validation(hover, "retreat")

        print(f"✅ BUTTON PRESS COMPLETE")

    # =========================================================
    # STATE QUERIES
    # =========================================================
    def get_world_state(self):
        """Get current world state."""
        if not hasattr(self.server, 'collect_world_state'):
            raise AttributeError("Server missing 'collect_world_state'")
        return self.server.collect_world_state()

    def wait_until_still(self, timeout_s=3.0):
        """Wait for robot to stop moving."""
        start = time.time()
        while time.time() - start < timeout_s:
            if not getattr(self.server, 'is_moving', False):
                return
            time.sleep(0.05)

    # =========================================================
    # INTERNAL HELPERS
    # =========================================================
    def _require_ee_pose(self):
        """Get end-effector pose or fail."""
        pose = getattr(self.server, 'ee_pose', None)
        if pose is None:
            raise RuntimeError("End-effector pose not available")
        return pose

    def _resolve_object(self, name: str) -> str:
        """Fuzzy match object name."""
        name_clean = name.lower().replace("the ", "").replace("a ", "").strip()
        
        objects = getattr(self.server, 'object_poses', {})
        if not objects:
            raise ValueError("No objects available")

        # Exact match
        for obj_name in objects.keys():
            if obj_name.lower() == name_clean:
                return obj_name

        # Partial match
        for obj_name in objects.keys():
            if name_clean in obj_name.lower() or obj_name.lower() in name_clean:
                return obj_name

        available = list(objects.keys())
        raise ValueError(f"Object '{name}' not found. Available: {available}")

    def _sleep_motion(self, timeout=None):
        """Wait for motion to complete."""
        timeout = timeout or self.MOTION_TIMEOUT
        start = time.time()
        
        while time.time() - start < timeout:
            if not getattr(self.server, 'is_moving', False):
                return
            time.sleep(0.05)

        raise TimeoutError(f"Motion timeout after {timeout}s")

    def calibrate_table_height(self):
        """Auto-detect table height from objects."""
        objects = getattr(self.server, 'object_poses', {})
        if not objects:
            print("[CAL] No objects to calibrate from")
            return self.TABLE_Z

        min_z = min(pose.position.z for pose in objects.values())
        estimated = min_z - 2.0

        print(f"\n[CAL] Table height detection:")
        for name, pose in objects.items():
            dist = ((pose.position.x - self.ARM_BASE_X)**2 + (pose.position.y - self.ARM_BASE_Y)**2) ** 0.5
            print(f"  • {name}: z={pose.position.z:.2f}, dist_from_base={dist:.1f}cm")
        
        print(f"\n[CAL] Analysis:")
        print(f"  Min Z: {min_z:.2f}cm")
        print(f"  Estimated TABLE_Z: {estimated:.2f}cm")
        print(f"  Current TABLE_Z: {self.TABLE_Z:.2f}cm")

        return estimated