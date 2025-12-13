"""
Graceful error handling and fallback strategies for tool execution.
"""

from typing import Dict, Any, List, Callable
from dataclasses import dataclass

from .utils.logging_utils import setup_logger

logger = setup_logger(__name__)


@dataclass
class FallbackStrategy:
    """Represents a fallback strategy for handling tool failures."""
    tool_name: str
    strategy_name: str
    description: str
    fallback_tools: List[str]
    parameters: Dict[str, Any]


class FallbackManager:
    """
    Manages fallback strategies for graceful degradation.
    
    When a tool fails, provides alternative strategies to continue execution.
    """
    
    def __init__(self):
        self.strategies: Dict[str, List[FallbackStrategy]] = {}
        self._initialize_strategies()
    
    def _initialize_strategies(self) -> None:
        """Initialize default fallback strategies."""
        
        # Fallback for pick_object failures
        self.strategies["pick_object"] = [
            FallbackStrategy(
                tool_name="pick_object",
                strategy_name="retry_with_offset",
                description="Retry pick from slightly different angle",
                fallback_tools=["move_above_object", "move_relative", "close_gripper"],
                parameters={"offset_xy": 0.05, "max_retries": 2}
            ),
            FallbackStrategy(
                tool_name="pick_object",
                strategy_name="use_lower_level_tools",
                description="Use lower-level movement and gripper tools",
                fallback_tools=["move_to_position", "close_gripper", "move_relative"],
                parameters={"approach_height": 0.15}
            ),
            FallbackStrategy(
                tool_name="pick_object",
                strategy_name="skip_and_continue",
                description="Skip picking and continue with next task",
                fallback_tools=[],
                parameters={"log_failure": True}
            ),
        ]
        
        # Fallback for place_object_at failures
        self.strategies["place_object_at"] = [
            FallbackStrategy(
                tool_name="place_object_at",
                strategy_name="place_at_home",
                description="If target placement fails, move to home position",
                fallback_tools=["move_to_named_pose", "open_gripper"],
                parameters={"pose_name": "home"}
            ),
            FallbackStrategy(
                tool_name="place_object_at",
                strategy_name="place_at_safe_location",
                description="Place at a safe default location",
                fallback_tools=["move_to_position", "open_gripper"],
                parameters={"x": 0.0, "y": 0.0, "z": 0.5}
            ),
        ]
        
        # Fallback for movement timeouts
        self.strategies["wait_until_still"] = [
            FallbackStrategy(
                tool_name="wait_until_still",
                strategy_name="force_proceed",
                description="Proceed despite timeout",
                fallback_tools=[],
                parameters={"timeout_s": 3.0}
            ),
            FallbackStrategy(
                tool_name="wait_until_still",
                strategy_name="reduce_timeout",
                description="Retry with reduced timeout",
                fallback_tools=["wait_until_still"],
                parameters={"timeout_s": 2.0}
            ),
        ]
        
        # Fallback for gripper operations
        self.strategies["close_gripper"] = [
            FallbackStrategy(
                tool_name="close_gripper",
                strategy_name="retry_close",
                description="Retry gripper close",
                fallback_tools=["close_gripper"],
                parameters={"retry_count": 1}
            ),
            FallbackStrategy(
                tool_name="close_gripper",
                strategy_name="assume_closed",
                description="Assume gripper is closed and continue",
                fallback_tools=[],
                parameters={}
            ),
        ]
        
        logger.info(f"Initialized {len(self.strategies)} fallback strategies")
    
    def get_fallback_strategies(self, tool_name: str) -> List[FallbackStrategy]:
        """Get available fallback strategies for a tool."""
        return self.strategies.get(tool_name, [])
    
    def has_fallback(self, tool_name: str) -> bool:
        """Check if fallback strategies exist for a tool."""
        return tool_name in self.strategies and len(self.strategies[tool_name]) > 0
    
    def get_first_fallback(self, tool_name: str) -> FallbackStrategy | None:
        """Get the first (most likely) fallback strategy."""
        strategies = self.get_fallback_strategies(tool_name)
        return strategies[0] if strategies else None
    
    def log_strategy(self, strategy: FallbackStrategy) -> None:
        """Log that a fallback strategy is being applied."""
        logger.warning(f"Applying fallback: {strategy.strategy_name}")
        logger.warning(f"  Tool: {strategy.tool_name}")
        logger.warning(f"  Description: {strategy.description}")
        logger.warning(f"  Fallback tools: {', '.join(strategy.fallback_tools) if strategy.fallback_tools else 'none'}")


class GracefulErrorHandler:
    """
    Handles errors gracefully and decides on recovery strategy.
    """
    
    def __init__(self):
        self.fallback_manager = FallbackManager()
        self.error_history: List[Dict[str, Any]] = []
    
    def handle_tool_failure(
        self,
        tool_name: str,
        error: Exception,
        attempt: int = 1,
        max_retries: int = 2
    ) -> Dict[str, Any]:
        """
        Handle a tool failure gracefully.
        
        Returns:
            Dict with handling strategy and recommended action
        """
        error_record = {
            "tool_name": tool_name,
            "error": str(error),
            "attempt": attempt,
            "max_retries": max_retries,
            "strategy": None,
            "action": "continue"
        }
        self.error_history.append(error_record)
        
        logger.warning(f"Tool '{tool_name}' failed on attempt {attempt}/{max_retries}")
        logger.warning(f"  Error: {error}")
        
        # Check if should retry
        if attempt < max_retries:
            error_record["action"] = "retry"
            logger.info(f"  → Retrying...")
            return error_record
        
        # Look for fallback strategy
        if self.fallback_manager.has_fallback(tool_name):
            strategy = self.fallback_manager.get_first_fallback(tool_name)
            error_record["strategy"] = strategy.strategy_name
            self.fallback_manager.log_strategy(strategy)
            
            if not strategy.fallback_tools:
                error_record["action"] = "skip"
                logger.info(f"  → Skipping tool and continuing")
            else:
                error_record["action"] = "fallback"
                logger.info(f"  → Using fallback strategy")
        else:
            error_record["action"] = "abort"
            logger.error(f"  → No fallback available, aborting")
        
        return error_record
    
    def get_error_summary(self) -> Dict[str, Any]:
        """Get a summary of all errors encountered."""
        if not self.error_history:
            return {"total_errors": 0, "tools_affected": []}
        
        tools_affected = set()
        for error in self.error_history:
            tools_affected.add(error["tool_name"])
        
        return {
            "total_errors": len(self.error_history),
            "tools_affected": list(tools_affected),
            "error_history": self.error_history
        }
