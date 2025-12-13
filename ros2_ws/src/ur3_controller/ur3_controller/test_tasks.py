#!/usr/bin/env python3
"""
Test harness for running all 6 predefined robotic manipulation tasks.

Usage:
    python test_tasks.py                    # Run all tasks
    python test_tasks.py --task 1          # Run specific task
    python test_tasks.py --task 1 2 3      # Run multiple tasks
"""

import sys
import argparse
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent))

from ur3_controller_node import UR3ControllerApp
from utils.logging_utils import setup_logger

logger = setup_logger(__name__)


def print_banner(text: str) -> None:
    """Print a formatted banner."""
    print(f"\n{'='*70}")
    print(f"  {text}")
    print(f"{'='*70}\n")


def run_test_suite(app: UR3ControllerApp, task_ids: list | None = None) -> None:
    """Run test suite for specified tasks."""
    
    print_banner("ROBOTIC ARM TASK EXECUTION TEST SUITE")
    
    # Display available tasks
    tasks = app.task_runner.list_tasks()
    print("Available Tasks:")
    print("-" * 70)
    for task in tasks:
        print(f"  Task {task.id}: {task.name}")
        print(f"    Description: {task.description}")
        print(f"    Instructions: {task.instructions}\n")
    
    # Determine which tasks to run
    if task_ids is None:
        task_ids = [task.id for task in tasks]
        logger.info(f"Running all {len(task_ids)} tasks")
    else:
        logger.info(f"Running selected tasks: {task_ids}")
    
    # Execute tasks
    print_banner("EXECUTION PHASE")
    
    results = {
        "total": len(task_ids),
        "completed": 0,
        "successful": 0,
        "failed": 0,
        "details": []
    }
    
    for task_id in task_ids:
        task = app.task_runner.get_task(task_id)
        if not task:
            logger.error(f"Task {task_id} not found")
            continue
        
        success, summary, result = app.task_runner.run_task(
            task_id=task_id,
            world_state=None
        )
        
        results["completed"] += 1
        results["details"].append({
            "task_id": task_id,
            "task_name": task.name,
            "success": success,
            "summary": summary
        })
        
        if success:
            results["successful"] += 1
        else:
            results["failed"] += 1
    
    # Print results
    print_banner("RESULTS SUMMARY")
    
    print(f"Total tasks to run: {results['total']}")
    print(f"Completed: {results['completed']}")
    print(f"Successful: {results['successful']}")
    print(f"Failed: {results['failed']}")
    
    if results['total'] > 0:
        success_rate = (results['successful'] / results['total'] * 100)
        print(f"Success rate: {success_rate:.1f}%\n")
    
    print("Task Details:")
    print("-" * 70)
    for detail in results["details"]:
        status = "✓ PASS" if detail["success"] else "✗ FAIL"
        print(f"{status} | Task {detail['task_id']}: {detail['task_name']}")
        print(f"       {detail['summary']}\n")
    
    # Save results
    print_banner("SAVING RESULTS")
    output_path = app.task_runner.save_results()
    print(f"Results saved to: {output_path}")
    
    print_banner("TEST SUITE COMPLETE")


def main():
    parser = argparse.ArgumentParser(
        description="Test harness for robotic arm task execution"
    )
    parser.add_argument(
        "--task",
        type=int,
        nargs="+",
        help="Specific task IDs to run (default: run all tasks)",
        default=None
    )
    parser.add_argument(
        "--list",
        action="store_true",
        help="List available tasks and exit"
    )
    
    args = parser.parse_args()
    
    # Initialize app
    logger.info("Initializing UR3 Controller App...")
    app = UR3ControllerApp()
    
    # Handle --list flag
    if args.list:
        print_banner("AVAILABLE TASKS")
        for task in app.task_runner.list_tasks():
            print(f"Task {task.id}: {task.name}")
            print(f"  {task.description}")
            print(f"  Instructions: {task.instructions}\n")
        return
    
    # Run test suite
    try:
        run_test_suite(app, args.task)
    except KeyboardInterrupt:
        logger.warning("\nTest interrupted by user")
        sys.exit(1)
    except Exception as e:
        logger.error(f"Test failed with error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
