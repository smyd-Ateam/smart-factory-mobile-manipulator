"""Actual ObservePose action server for Gazebo Fortress."""

import subprocess
import threading
import time

import rclpy
from project_interfaces.action import ObservePose
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


JOINTS = tuple(f'joint{i}' for i in range(1, 6))

# Verified arm poses, in radians
POSES = {
    'far': (
        0.0, 0.0, -1.146400, -0.823700, 0.0
    ),
    'near': (
        0.0, -0.008482, -1.282087, -1.299052, -0.000157
    ),
    'pre_grasp': (
        0.0, -0.636802, -0.959445, -1.190737, -0.000157
    ),
}


class ObservePoseServer(Node):

    def __init__(self):
        super().__init__('observe_pose_server')

        self.declare_parameter('goal_timeout_sec', 180.0)
        self.declare_parameter('joint_tolerance_rad', 0.02)

        self._lock = threading.Lock()
        self._positions = None
        self._state_time = 0.0
        self._busy = False

        self.create_subscription(
            JointState,
            '/joint_states',
            self._on_joint_state,
            qos_profile_sensor_data
        )

        self._action = ActionServer(
            self,
            ObservePose,
            '/arm/observe_pose',
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=ReentrantCallbackGroup()
        )

        self.get_logger().info(
            'ObservePose server ready: far, near, pre_grasp'
        )

    def _on_joint_state(self, msg):
        states = dict(zip(msg.name, msg.position))

        if all(j in states for j in JOINTS):
            with self._lock:
                self._positions = tuple(
                    float(states[j]) for j in JOINTS
                )
                self._state_time = time.monotonic()

    def _snapshot(self):
        with self._lock:
            return self._positions, self._state_time

    def _on_goal(self, goal):
        if goal.pose_name not in POSES:
            self.get_logger().warn(
                f'Unsupported pose: {goal.pose_name!r}'
            )
            return GoalResponse.REJECT

        with self._lock:
            if self._busy:
                return GoalResponse.REJECT
            self._busy = True

        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle):
        return CancelResponse.ACCEPT

    def _send_positions(self, positions, joints=JOINTS, cancel_requested=None):
        for joint, angle in zip(joints, positions):
            if cancel_requested is not None and cancel_requested():
                return False
            command = [
                'ign', 'topic',
                '-t', f'/mm_robot/{joint}/cmd_pos',
                '-m', 'ignition.msgs.Double',
                '-p', f'data: {angle:.6f}'
            ]

            proc = subprocess.run(
                command,
                capture_output=True,
                text=True,
                timeout=8
            )

            if proc.returncode != 0:
                self.get_logger().error(
                    f'{joint} command failed: {proc.stderr}'
                )
                return False

        return True

    def _hold_position(self):
        current, stamp = self._snapshot()
        if current is not None and time.monotonic() - stamp < 3.0:
            try:
                self._send_positions(current)
            except (OSError, subprocess.TimeoutExpired) as exc:
                self.get_logger().error(
                    f'Hold command failed: {exc}'
                )

    def _execute(self, goal_handle):
        result = ObservePose.Result()
        target = POSES[goal_handle.request.pose_name]

        tolerance = float(
            self.get_parameter('joint_tolerance_rad').value
        )
        timeout_sec = float(
            self.get_parameter('goal_timeout_sec').value
        )

        try:
            current, stamp = self._snapshot()

            if current is None or time.monotonic() - stamp > 3.0:
                self.get_logger().error(
                    'No recent /joint_states received'
                )
                result.success = False
                result.error_code = 'UNKNOWN'
                goal_handle.abort()
                return result

            if not self._send_positions(
                target, cancel_requested=lambda: goal_handle.is_cancel_requested
            ):
                self._hold_position()
                result.success = False
                result.error_code = 'UNKNOWN'
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                else:
                    goal_handle.abort()
                return result

            start = time.monotonic()
            last_sample_stamp = 0.0
            stable_samples = 0
            last_retry = start
            retries = 0

            while rclpy.ok():

                if goal_handle.is_cancel_requested:
                    self._hold_position()
                    result.success = False
                    result.error_code = 'UNKNOWN'
                    goal_handle.canceled()
                    return result

                current, stamp = self._snapshot()

                fresh = (
                    current is not None
                    and time.monotonic() - stamp < 3.0
                )

                if fresh and stamp > max(start, last_sample_stamp):
                    last_sample_stamp = stamp

                    reached = all(
                        abs(value - desired) <= tolerance
                        for value, desired in zip(current, target)
                    )

                    if reached:
                        stable_samples += 1

                        if stable_samples >= 3:
                            result.success = True
                            result.error_code = ''
                            goal_handle.succeed()

                            self.get_logger().info(
                                'Reached '
                                + goal_handle.request.pose_name
                            )
                            return result
                    else:
                        stable_samples = 0


                # 목표에 도달하지 못한 관절에만 재전송
                now = time.monotonic()

                if (
                    fresh
                    and stamp > start
                    and now - last_retry >= 3.0
                    and retries < 3
                ):
                    pending = [
                        (joint, desired)
                        for joint, actual, desired
                        in zip(JOINTS, current, target)
                        if abs(actual - desired) > tolerance
                    ]

                    if pending:
                        self.get_logger().warn(
                            'Retrying joints: '
                            + ', '.join(j for j, _ in pending)
                        )

                        ok = self._send_positions(
                            [v for _, v in pending],
                            [j for j, _ in pending],
                            cancel_requested=lambda: goal_handle.is_cancel_requested
                        )
                        if not ok:
                            self._hold_position()
                            result.success = False
                            result.error_code = 'UNKNOWN'
                            if goal_handle.is_cancel_requested:
                                goal_handle.canceled()
                            else:
                                goal_handle.abort()
                            return result

                        retries += 1

                    last_retry = time.monotonic()

                if time.monotonic() - start > timeout_sec:
                    self.get_logger().error(
                        'ObservePose movement timeout'
                    )
                    self._hold_position()
                    result.success = False
                    result.error_code = 'UNKNOWN'
                    goal_handle.abort()
                    return result

                time.sleep(0.1)

            result.success = False
            result.error_code = 'UNKNOWN'
            goal_handle.abort()
            return result

        except (OSError, subprocess.TimeoutExpired, ValueError) as exc:
            self.get_logger().error(str(exc))
            self._hold_position()
            result.success = False
            result.error_code = 'UNKNOWN'
            goal_handle.abort()
            return result

        finally:
            with self._lock:
                self._busy = False


def main(args=None):
    rclpy.init(args=args)

    node = ObservePoseServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
