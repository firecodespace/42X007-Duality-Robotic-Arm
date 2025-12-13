"""
Defines all tools available to the LLM.
Returned as a list of DICTS (not Python classes).
"""

def define_tools():
    return [
        {
            "name": "pick_object",
            "description": "Pick up an object by name. Approach, descend, grasp and lift the object.",
            "parameters": {
                "type": "object",
                "properties": {
                    "object_name": {
                        "type": "string",
                        "description": "Name of object to pick up (e.g., 'cube', 'button')"
                    }
                },
                "required": ["object_name"]
            }
        },
        {
            "name": "place_object_relative",
            "description": "Place held object relative to its pickup position. Use for commands like 'move 30cm left' or 'move 10cm forward'. Negative Y = left, Positive Y = right, Positive X = forward.",
            "parameters": {
                "type": "object",
                "properties": {
                    "dx": {
                        "type": "number",
                        "description": "X displacement in centimeters (forward/backward)",
                        "default": 0.0
                    },
                    "dy": {
                        "type": "number",
                        "description": "Y displacement in centimeters (left/right). Negative = left.",
                        "default": 0.0
                    },
                    "dz": {
                        "type": "number",
                        "description": "Z displacement in centimeters (up/down)",
                        "default": 0.0
                    },
                    "use_cm": {
                        "type": "boolean",
                        "description": "Whether values are in centimeters (true) or meters (false)",
                        "default": True
                    }
                },
                "required": ["dx", "dy", "dz"]
            }
        },
        {
            "name": "place_object_at",
            "description": "Place held object at absolute world coordinates.",
            "parameters": {
                "type": "object",
                "properties": {
                    "x": {
                        "type": "number",
                        "description": "Absolute X position in world frame"
                    },
                    "y": {
                        "type": "number",
                        "description": "Absolute Y position in world frame"
                    },
                    "z": {
                        "type": "number",
                        "description": "Absolute Z position (optional, will use safe height if not provided)"
                    }
                }
            }
        },
        {
            "name": "move_to_position",
            "description": "Move end-effector to absolute position without gripper action.",
            "parameters": {
                "type": "object",
                "properties": {
                    "x": {"type": "number", "description": "Absolute X position"},
                    "y": {"type": "number", "description": "Absolute Y position"},
                    "z": {"type": "number", "description": "Absolute Z position"}
                }
            }
        },
        {
            "name": "move_relative",
            "description": "Move end-effector relative to current position (in meters, not cm). For moving objects, use place_object_relative instead.",
            "parameters": {
                "type": "object",
                "properties": {
                    "dx": {
                        "type": "number",
                        "description": "X displacement in meters",
                        "default": 0.0
                    },
                    "dy": {
                        "type": "number",
                        "description": "Y displacement in meters",
                        "default": 0.0
                    },
                    "dz": {
                        "type": "number",
                        "description": "Z displacement in meters",
                        "default": 0.0
                    }
                },
                "required": ["dx", "dy", "dz"]
            }
        },
        {
            "name": "press_button",
            "description": "Press a button by moving to it, descending, and returning.",
            "parameters": {
                "type": "object",
                "properties": {
                    "button_name": {
                        "type": "string",
                        "description": "Name of the button object to press"
                    }
                },
                "required": ["button_name"]
            }
        },
        {
            "name": "open_gripper",
            "description": "Open the gripper to release an object.",
            "parameters": {
                "type": "object",
                "properties": {}
            }
        },
        {
            "name": "close_gripper",
            "description": "Close the gripper to grasp an object.",
            "parameters": {
                "type": "object",
                "properties": {}
            }
        },
        {
            "name": "get_world_state",
            "description": "Get current state of all objects and robot status.",
            "parameters": {
                "type": "object",
                "properties": {}
            }
        },
        {
            "name": "wait_until_still",
            "description": "Wait for robot to stop moving before proceeding.",
            "parameters": {
                "type": "object",
                "properties": {
                    "timeout_s": {
                        "type": "number",
                        "description": "Maximum seconds to wait",
                        "default": 5.0
                    }
                }
            }
        }
    ]
