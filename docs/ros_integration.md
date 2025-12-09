# ROS Integration – UR3 Controller

## ROS Node Overview
The node `ur3_controller_node.py` manages:
- Robot state subscribers
- Command publishers
- Tool execution
- Plan execution

---

## Core Topics (Provided by Challenge)
- `/ur3/ee_pose`
- `/ur3/object_poses`
- `/ur3/held_item`
- `/ur3/is_moving`
- `/ur3/set_ik`
- `/ur3/set_fk`
- `/ur3/set_gripper`

---

## Executor Responsibilities
- Convert tool calls into IK/FK requests.
- Ensure movements complete before next tool.
- Use consistent logging.

---

## State Handling
Poll world state frequently:
- Positions
- Orientation
- Movement status

State accuracy directly impacts tool logic.
