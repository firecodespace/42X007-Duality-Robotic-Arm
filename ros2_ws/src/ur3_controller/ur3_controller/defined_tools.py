from dataclasses import dataclass
from typing import Any, Dict, List
import yaml
from .config import TOOL_CONFIG_PATH
from .utils.logging_utils import setup_logger

logger = setup_logger(__name__)

@dataclass
class ToolParamSpec:
    type: str
    required: bool = True
    enum: list[str] | None = None

@dataclass
class ToolSpec:
    name: str
    description: str
    params: Dict[str, ToolParamSpec]

def load_tool_config() -> List[ToolSpec]:
    logger.info(f"Loading tool configuration from {TOOL_CONFIG_PATH}")
    with open(TOOL_CONFIG_PATH, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    tools: List[ToolSpec] = []
    for tool_data in data.get("tools", []):
        params_spec: Dict[str, ToolParamSpec] = {}
        for param_name, param_info in (tool_data.get("params") or {}).items():
            params_spec[param_name] = ToolParamSpec(
                type=str(param_info.get("type", "any")),
                required=bool(param_info.get("required", True)),
                enum=param_info.get("enum"),
            )
        tools.append(
            ToolSpec(
                name=tool_data["name"],
                description=tool_data.get("description", ""),
                params=params_spec,
            )
        )
    logger.info(f"Loaded {len(tools)} tools from configuration")
    return tools

# Convenience accessor used by planner + LLM interface
TOOL_SPECS: List[ToolSpec] = load_tool_config()

def get_tool_specs() -> List[ToolSpec]:
    return TOOL_SPECS
