#!/usr/bin/env python3
"""CLI client: ros2 run mm_llm_planner send_task "<natural language command>" """

import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from mm_interfaces.action import ExecuteTask


class SendTaskClient(Node):
    def __init__(self):
        super().__init__('send_task_client')
        self._client = ActionClient(self, ExecuteTask, '/execute_task')

    def send(self, command: str):
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('execute_task action server not available')
            return None

        goal = ExecuteTask.Goal()
        goal.command = command

        def feedback_cb(feedback_msg):
            self.get_logger().info(f'  ... {feedback_msg.feedback.status_message}')

        send_future = self._client.send_goal_async(goal, feedback_callback=feedback_cb)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return None

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result().result


def main():
    if len(sys.argv) < 2:
        print('Usage: ros2 run mm_llm_planner send_task "<command>"')
        sys.exit(1)
    command = ' '.join(sys.argv[1:])

    rclpy.init()
    node = SendTaskClient()
    print(f'Sending task: {command!r}')
    result = node.send(command)
    if result is None:
        print('FAILED: no result (server unavailable or goal rejected)')
    else:
        print('─' * 60)
        print(f'success:       {result.success}')
        print(f'summary:       {result.summary}')
        print(f'num_llm_calls: {result.num_llm_calls}')
        print(f'num_replans:   {result.num_replans}')
        print(f'duration_sec:  {result.duration_sec:.1f}')
        print(f'log_path:      {result.log_path}')
        print('─' * 60)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
