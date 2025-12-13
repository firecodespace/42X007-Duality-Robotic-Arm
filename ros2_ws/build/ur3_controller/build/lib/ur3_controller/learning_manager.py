import json
import os
from datetime import datetime

class LearningManager:
    """Tracks successes/failures and adapts prompts."""
    
    def __init__(self, log_file="~/robot_learning_log.json"):
        self.log_file = os.path.expanduser(log_file)
        self.history = self._load_history()
        
    def _load_history(self):
        """Load previous execution history."""
        if os.path.exists(self.log_file):
            with open(self.log_file, 'r') as f:
                return json.load(f)
        return {"successes": [], "failures": [], "statistics": {}}
    
    def _save_history(self):
        """Persist learning history."""
        with open(self.log_file, 'w') as f:
            json.dump(self.history, f, indent=2)
    
    def log_success(self, task, plan, duration):
        """Record successful execution."""
        self.history["successes"].append({
            "timestamp": datetime.now().isoformat(),
            "task": task,
            "plan": plan,
            "duration_seconds": duration
        })
        self._save_history()
        
    def log_failure(self, task, plan, error, step_failed):
        """Record failure with details."""
        self.history["failures"].append({
            "timestamp": datetime.now().isoformat(),
            "task": task,
            "plan": plan,
            "error": str(error),
            "step_failed": step_failed
        })
        self._save_history()
        
    def get_similar_successes(self, task):
        """Find similar successful tasks for guidance."""
        # Simple keyword matching
        task_lower = task.lower()
        similar = []
        
        for success in self.history["successes"]:
            if any(word in success["task"].lower() for word in task_lower.split()):
                similar.append(success)
        
        return similar[-3:]  # Return last 3 similar successes
    
    def get_failure_patterns(self):
        """Identify common failure patterns."""
        failures = self.history["failures"]
        if not failures:
            return []
        
        # Group by error type
        error_types = {}
        for fail in failures:
            error_key = fail["error"][:50]  # First 50 chars
            if error_key not in error_types:
                error_types[error_key] = []
            error_types[error_key].append(fail)
        
        return error_types
    
    def suggest_improvements(self, task):
        """Suggest plan improvements based on history."""
        similar = self.get_similar_successes(task)
        if similar:
            return f"Similar successful tasks used: {[s['plan'] for s in similar]}"
        return "No similar successful examples found."
