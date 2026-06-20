"""
Per-task JSON logger. Collects everything in memory during the closed loop
and writes one JSON file at the end -- this is the paper data (replan
latency, tool calls per task), so keep the schema flat and parseable.
"""

import json
import os
import re
import time
from typing import Any, Dict, List


def _slugify(text: str, max_len: int = 40) -> str:
    slug = re.sub(r'[^a-zA-Z0-9]+', '_', text.strip().lower()).strip('_')
    return slug[:max_len] or 'task'


class TaskLogger:
    def __init__(self, log_dir: str, command: str, provider: str, model: str):
        self.log_dir = os.path.expanduser(log_dir)
        os.makedirs(self.log_dir, exist_ok=True)
        self.command = command
        self.provider = provider
        self.model = model
        self.start_time = time.time()
        self.llm_turns: List[Dict[str, Any]] = []
        self.tool_calls: List[Dict[str, Any]] = []

    def log_llm_turn(self, kind: str, turn_result, latency_sec: float) -> None:
        self.llm_turns.append({
            'timestamp': time.time(),
            'kind': kind,  # "start" | "tool_results"
            'latency_sec': round(latency_sec, 4),
            'assistant_text': turn_result.assistant_text,
            'tool_calls_requested': [
                {'name': tc.name, 'arguments': tc.arguments} for tc in turn_result.tool_calls
            ],
            'input_tokens': turn_result.input_tokens,
            'output_tokens': turn_result.output_tokens,
            'stop_reason': turn_result.stop_reason,
        })

    def log_tool_call(self, name: str, arguments: dict, success: bool, message: str,
                       duration_sec: float, is_replan: bool = False) -> None:
        self.tool_calls.append({
            'timestamp': time.time(),
            'name': name,
            'arguments': arguments,
            'success': success,
            'message': message,
            'duration_sec': round(duration_sec, 4),
            'is_replan': is_replan,
        })

    def finalize(self, success: bool, summary: str, num_llm_calls: int, num_replans: int,
                 raw_conversation: list) -> str:
        end_time = time.time()
        record = {
            'command': self.command,
            'provider': self.provider,
            'model': self.model,
            'start_time': self.start_time,
            'end_time': end_time,
            'duration_sec': round(end_time - self.start_time, 4),
            'success': success,
            'summary': summary,
            'num_llm_calls': num_llm_calls,
            'num_replans': num_replans,
            'num_tool_calls': len(self.tool_calls),
            'total_input_tokens': sum(t['input_tokens'] for t in self.llm_turns),
            'total_output_tokens': sum(t['output_tokens'] for t in self.llm_turns),
            'llm_turns': self.llm_turns,
            'tool_calls': self.tool_calls,
            'raw_conversation': raw_conversation,
        }
        fname = f"{time.strftime('%Y%m%d_%H%M%S')}_{_slugify(self.command)}.json"
        path = os.path.join(self.log_dir, fname)
        with open(path, 'w') as f:
            json.dump(record, f, indent=2, default=str)
        return path
