import os
import json
import time
from google import genai


def call_gemini(prompt: str, retry_count=3) -> str:
    """Call Gemini with automatic fallback and retry."""
    api_key = os.getenv("GEMINI_API_KEY") or "AIzaSyDGDTZLgA-mlXYbsbXRRmSk9fNz6Dv2QMM"
    client = genai.Client(api_key=api_key)

    models = ["gemini-2.0-flash", "gemini-1.5-flash"]

    for model in models:
        for attempt in range(retry_count):
            try:
                print(f"[GEMINI] Calling {model} (attempt {attempt+1}/{retry_count})")
                
                response = client.models.generate_content(
                    model=model,
                    contents=prompt,
                    config={
                        "response_mime_type": "application/json",
                        "temperature": 0.05,  # Very low for consistency
                        "top_p": 0.95,
                    }
                )

                print(f"[GEMINI] ✅ Success")
                return response.text

            except Exception as e:
                error_msg = str(e)
                print(f"[GEMINI] ⚠️  {model} failed: {error_msg[:80]}")

                if "404" in error_msg or "NOT_FOUND" in error_msg:
                    break  # Try next model

                if attempt < retry_count - 1:
                    time.sleep(0.5 * (attempt + 1))

    raise RuntimeError("All Gemini models exhausted")


def plan_actions(task_text: str, tools: list, system_instructions: str = None, world_state: dict = None):
    """Generate plan from task using Gemini."""
    
    # Extract context
    object_names = list(world_state.get("objects", {}).keys()) if world_state else []
    held = world_state.get("held_item", "") if world_state else ""

    # Build prompt
    prompt_dict = {
        "system_instructions": system_instructions,
        "task": task_text,
        "current_state": {
            "available_objects": object_names,
            "currently_holding": held
        },
        "available_tools": [
            {
                "name": tool["name"],
                "description": tool.get("description", ""),
                "parameters": tool.get("parameters", {})
            }
            for tool in tools
        ]
    }

    prompt = json.dumps(prompt_dict, indent=2)

    print(f"[GEMINI] Task: {task_text}")
    print(f"[GEMINI] Objects: {object_names}")
    if held:
        print(f"[GEMINI] Holding: {held}")

    # Get response
    raw = call_gemini(prompt)

    # Parse and validate
    try:
        plan = json.loads(raw)

        if not isinstance(plan, list):
            raise ValueError(f"Expected list, got {type(plan)}")

        normalized = []
        for i, step in enumerate(plan):
            if not isinstance(step, dict):
                raise ValueError(f"Step {i} not dict")

            if "tool" not in step:
                raise ValueError(f"Step {i} missing 'tool'")

            # Normalize parameters -> args
            if "parameters" in step:
                step["args"] = step.pop("parameters")
            elif "args" not in step:
                step["args"] = {}

            # Validate place_object_relative
            if step["tool"] == "place_object_relative":
                for param in ["dx", "dy", "dz"]:
                    if param not in step["args"]:
                        step["args"][param] = 0.0

            normalized.append(step)

        print(f"[GEMINI] ✅ Plan: {len(normalized)} steps")
        for i, s in enumerate(normalized):
            print(f"       {i+1}. {s['tool']}")

        return normalized

    except json.JSONDecodeError:
        print(f"[GEMINI] Invalid JSON: {raw[:100]}")
        raise RuntimeError(f"Invalid JSON from LLM")

    except ValueError as e:
        print(f"[GEMINI] Validation failed: {e}")
        raise RuntimeError(f"Plan validation: {e}")
