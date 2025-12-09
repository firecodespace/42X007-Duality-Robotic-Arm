# Tools Design – 42X007 Duality Robotic Arm

## Purpose of the Toolset
The Tools API is the vocabulary the LLM uses to interact with the robot.  
A good toolset is:
- Small enough to avoid confusion.
- Expressive enough to solve multi-step manipulation tasks.
- Generalizable and stable across scenarios.

---

# Final Toolset (Production-Level)

## 1. Movement Tools

### **move_to_position**
Moves UR3 end-effector to a 3D coordinate using IK.

**Params**
- x (float)
- y (float)
- z (float)
- speed (optional float)

**Preconditions**
- Robot must be idle.

**Postconditions**
- End-effector is at target pose.

---

### **move_above_object**
Positions end-effector directly above the object's pose plus an offset.

**Params**
- object_name (string)
- dz (float)

---

### **move_to_named_pose**
Moves to predefined poses such as:
- "home"
- "pre_grasp"
- "drop_zone"

---

## 2. Gripper Tools

### **open_gripper**
Opens gripper fully.  
### **close_gripper**
Closes gripper.  
Requires proper approach pose before use.

---

## 3. Manipulation Tools

### **pick_object**
Sequence:
1. move_above_object
2. descend
3. close_gripper
4. lift

LLM may call this directly.

---

### **place_object_at**
Sequence:
1. move_to_position(target pose)
2. open_gripper
3. retract

---

## 4. State Tools

### **get_world_state**
Returns:
- Object poses
- End-effector pose
- Held object
- If robot is moving

### **wait_until_still**
Pauses until robot finishes motion.

---

# Tool Design Principles
1. Avoid redundant tools.
2. Keep tools atomic but expressive.
3. Use clear parameter naming.
4. Avoid deep low-level control (LLM shouldn't choose joint angles).
5. Keep tool count ≤ 12 for stability.

This toolset is optimized for the Duality UR3 challenge.
