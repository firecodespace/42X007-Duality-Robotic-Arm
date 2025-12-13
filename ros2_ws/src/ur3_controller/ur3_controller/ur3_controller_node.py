import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import json
import time

from .server_node import UR3ControllerServer
from .defined_tools import define_tools
from .planner import Planner
from .tool_executor import ToolExecutor
<<<<<<< Updated upstream
from .task_runner import TaskRunner
from .utils.logging_utils import setup_logger
=======
>>>>>>> Stashed changes


class UR3ControllerNode(UR3ControllerServer):

<<<<<<< Updated upstream
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
=======
    def __init__(self):
        super().__init__()

        self.get_logger().info("🤖 UR3 CONTROLLER v3.0 READY")

        self.tools = define_tools()
        self.planner = Planner(self.tools)
        self.tool_executor = ToolExecutor(self)

        self._calibrated = False
        self._calib_lock = threading.Lock()
        self.task_count = 0
        self.success_count = 0

        self.create_subscription(String, "/falcon/llm_prompt", self.on_prompt, 10)
        self.result_pub = self.create_publisher(String, "/falcon/execution_result", 10)

    def auto_calibrate(self):
        """Auto-calibrate on first task."""
        with self._calib_lock:
            if self._calibrated or not self.object_poses:
                return

            self.get_logger().info("🔧 Calibrating table height...")
            try:
                est_z = self.tool_executor.calibrate_table_height()
                if est_z and 5.0 < est_z < 15.0:
                    self.tool_executor.TABLE_Z = est_z
                    self.tool_executor.MIN_SAFE_Z = est_z + 3.0
                    self.get_logger().info(f"✅ Table Z: {est_z:.2f}cm")
                self._calibrated = True
            except Exception as e:
                self.get_logger().error(f"Calibration failed: {e}")

    def on_prompt(self, msg: String):
        task = msg.data.strip()
        self.get_logger().info(f"📝 Task: {task}")
        threading.Thread(target=self.handle_task, args=(task,), daemon=True).start()

    def handle_task(self, task: str):
        self.task_count += 1
        start = time.time()

        try:
            # Wait for objects
            if not self.object_poses:
                for _ in range(50):
                    if self.object_poses:
                        break
                    time.sleep(0.1)

            if not self.object_poses:
                raise RuntimeError("No objects detected")

            # Calibrate
            self.auto_calibrate()

            # Build world state
            world_state = {
                "objects": {
                    name: {"x": p.position.x, "y": p.position.y, "z": p.position.z}
                    for name, p in self.object_poses.items()
                },
                "held_item": self.held_item
            }

            self.get_logger().info(f"🌍 Objects: {list(world_state['objects'].keys())}")

            # Plan
            plan = self.planner.plan(task, world_state)
            if not plan:
                raise RuntimeError("Empty plan")

            # Execute
            for i, step in enumerate(plan):
                tool = step.get("tool", "").replace("tool_", "")
                args = step.get("args", {})
                
                self.get_logger().info(f"▶️  [{i+1}/{len(plan)}] {tool}")
                
                self.tool_executor.execute({"tool": tool, "args": args})
                time.sleep(0.05)

            # Success
            duration = time.time() - start
            self.success_count += 1
            rate = (self.success_count / self.task_count) * 100

            self.result_pub.publish(String(data=json.dumps({
                "task": task,
                "status": "success",
                "duration": duration,
                "steps": len(plan)
            })))

            self.get_logger().info(
                f"✅ Task #{self.task_count} OK ({duration:.1f}s) | "
                f"Success: {rate:.0f}%"
            )

        except Exception as e:
            duration = time.time() - start
            self.get_logger().error(f"❌ Task failed: {str(e)}")

            self.result_pub.publish(String(data=json.dumps({
                "task": task,
                "status": "failed",
                "error": str(e),
                "duration": duration
            })))


def main(args=None):
    rclpy.init(args=args)
    node = UR3ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
>>>>>>> Stashed changes
