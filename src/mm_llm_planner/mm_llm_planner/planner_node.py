#!/usr/bin/env python3
"""
planner_node.py — Phase 5 closed-loop LLM task planner.

Exposes an /execute_task action. On each goal: fetch scene state, hand the
command + state to an LLM via native tool-calling, dispatch whatever tool
calls it emits as ROS service calls against tool_server_node /
state_manager_node, feed results (+ a fresh scene state after
navigate/pick/place) back to the LLM, and repeat until it calls
task_complete or a safety limit trips.
"""

import json
import os
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionServer
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node

from mm_interfaces.action import ExecuteTask
from mm_interfaces.srv import GetSceneState

from .llm_clients import ToolResultItem, make_client
from .task_logger import TaskLogger
from .tool_schemas import RESCAN_AFTER, TOOLS, get_tool_spec


class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner_node')

        self.declare_parameter('provider', 'gemini')
        self.declare_parameter('model', 'gemini-3.5-flash')
        self.declare_parameter('max_turns', 25)
        self.declare_parameter('tool_timeout_sec', 200.0)
        self.declare_parameter('log_dir', '~/aset_ws/llm_task_logs')
        self.declare_parameter('system_prompt_path', '')

        self.provider = self.get_parameter('provider').value
        self.model = self.get_parameter('model').value
        self.max_turns = self.get_parameter('max_turns').value
        self.tool_timeout_sec = self.get_parameter('tool_timeout_sec').value
        self.log_dir = self.get_parameter('log_dir').value
        self.system_prompt = self._load_system_prompt()

        self._tool_clients = {}
        client_group = MutuallyExclusiveCallbackGroup()
        for spec in TOOLS:
            self._tool_clients[spec['name']] = self.create_client(
                spec['ros_type'], spec['ros_service'], callback_group=client_group)

        self._action_server = ActionServer(
            self, ExecuteTask, '/execute_task', self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        self.get_logger().info(
            f'planner_node ready (provider={self.provider}, model={self.model}, '
            f'max_turns={self.max_turns}, tool_timeout_sec={self.tool_timeout_sec})')

    def _load_system_prompt(self) -> str:
        path = self.get_parameter('system_prompt_path').value
        if not path:
            path = os.path.join(
                get_package_share_directory('mm_llm_planner'), 'config', 'system_prompt.txt')
        with open(path, 'r') as f:
            return f.read()

    # ── Generic tool dispatch ───────────────────────────────────────────────

    def _call_tool_service(self, name: str, arguments: dict) -> tuple:
        """Returns (success: bool, content: str)."""
        spec = get_tool_spec(name)
        if spec is None or name not in self._tool_clients:
            return False, f"Unknown tool '{name}'"

        client = self._tool_clients[name]
        if not client.wait_for_service(timeout_sec=5.0):
            return False, f"Service for '{name}' is not available"

        request = spec['ros_type'].Request()
        for field in spec['request_fields']:
            if field not in arguments:
                return False, f"Missing required argument '{field}' for tool '{name}'"
            setattr(request, field, arguments[field])

        future = client.call_async(request)
        deadline = time.time() + self.tool_timeout_sec
        while not future.done():
            if time.time() > deadline:
                return False, f"Tool '{name}' timed out after {self.tool_timeout_sec}s"
            time.sleep(0.02)

        response = future.result()
        if response is None:
            return False, f"Tool '{name}' call failed (no response)"

        if name == 'get_state':
            return True, response.state_text
        if name == 'detect_objects':
            detections = [
                {'label': d.label, 'color': d.color, 'x': d.x, 'y': d.y, 'z': d.z,
                 'confidence': d.confidence}
                for d in response.detections
            ]
            content = response.message + '\n' + json.dumps(detections)
            return response.success, content
        return response.success, response.message

    def _get_scene_state_text(self) -> str:
        ok, content = self._call_tool_service('get_state', {})
        return content if ok else f'(failed to fetch scene state: {content})'

    # ── Main closed loop ─────────────────────────────────────────────────────

    def execute_callback(self, goal_handle):
        command = goal_handle.request.command
        t0 = time.time()
        self.get_logger().info(f"[execute_task] command='{command}'")

        logger = TaskLogger(self.log_dir, command, self.provider, self.model)
        client = make_client(self.provider, self.model, self.system_prompt)

        initial_state = self._get_scene_state_text()
        user_text = f'TASK: {command}\n\n{initial_state}'

        t_llm = time.time()
        turn = client.start(user_text)
        logger.log_llm_turn('start', turn, time.time() - t_llm)
        num_llm_calls = 1
        num_replans = 0

        last_failure_sig = None
        repeat_count = 0
        success = False
        summary = 'Exceeded max LLM turns without completing the task'

        for turn_idx in range(self.max_turns):
            goal_handle.publish_feedback(
                ExecuteTask.Feedback(status_message=f'LLM turn {turn_idx + 1}/{self.max_turns}'))

            if not turn.tool_calls:
                # Model produced text but didn't act — nudge it instead of
                # silently burning turns with no progress.
                t_llm = time.time()
                turn = client.start(
                    'You did not call a tool. Call one of the available tools to make '
                    'progress, or task_complete if the task is actually finished.')
                logger.log_llm_turn('nudge', turn, time.time() - t_llm)
                num_llm_calls += 1
                continue

            results = []
            done = False
            for tc in turn.tool_calls:
                if tc.name == 'task_complete':
                    success = bool(tc.arguments.get('success', True))
                    summary = tc.arguments.get('summary', '')
                    done = True
                    break

                t_tool = time.time()
                ok, content = self._call_tool_service(tc.name, tc.arguments)
                duration = time.time() - t_tool

                sig = (tc.name, json.dumps(tc.arguments, sort_keys=True), content)
                is_replan = False
                if not ok:
                    num_replans += 1
                    if sig == last_failure_sig:
                        repeat_count += 1
                    else:
                        repeat_count = 1
                        last_failure_sig = sig
                    is_replan = True
                    if repeat_count >= 2:
                        content += (
                            '\n[HINT: this exact tool call has now failed twice with the '
                            'same error. Do not repeat it unchanged — try a different '
                            'location, recheck object positions, or a different approach.]'
                        )
                else:
                    last_failure_sig = None
                    repeat_count = 0

                logger.log_tool_call(tc.name, tc.arguments, ok, content, duration, is_replan)

                if tc.name in RESCAN_AFTER:
                    content += '\n\n--- Updated scene state ---\n' + self._get_scene_state_text()

                results.append(ToolResultItem(id=tc.id, name=tc.name, content=content,
                                               is_error=not ok))

            if done:
                break

            t_llm = time.time()
            turn = client.send_tool_results(results)
            logger.log_llm_turn('tool_results', turn, time.time() - t_llm)
            num_llm_calls += 1
        else:
            success = False  # for-loop exhausted without break -> max turns hit

        duration = time.time() - t0
        log_path = logger.finalize(success, summary, num_llm_calls, num_replans,
                                    client.raw_messages)

        self.get_logger().info(
            f"[execute_task] done success={success} llm_calls={num_llm_calls} "
            f"replans={num_replans} duration={duration:.1f}s log={log_path}")

        goal_handle.succeed()
        result = ExecuteTask.Result()
        result.success = success
        result.summary = summary
        result.num_llm_calls = num_llm_calls
        result.num_replans = num_replans
        result.duration_sec = duration
        result.log_path = log_path
        return result


def main():
    rclpy.init()
    node = PlannerNode()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
