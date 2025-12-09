from pathlib import Path

# Base paths (relative to this file)
BASE_DIR = Path(__file__).resolve().parent.parent.parent  # points to ros2_ws/src/ur3_controller
PROJECT_ROOT = BASE_DIR.parent.parent  # adjust if needed
CONFIG_DIR = PROJECT_ROOT / "config"
LOG_DIR = PROJECT_ROOT / "logs"
LLM_PLAN_LOG_DIR = LOG_DIR / "llm_plans"
RUN_LOG_DIR = LOG_DIR / "run_logs"

TOOL_CONFIG_PATH = CONFIG_DIR / "tool_config.yaml"

# LLM settings
DEFAULT_MODEL_NAME = "gemini-2.5"  # placeholder, will be used on VM
LLM_TEMPERATURE = 0.0
LLM_MAX_TOKENS = 1024

# Scenario defaults (we'll refine later)
DEFAULT_TIMEOUT_S = 10.0
