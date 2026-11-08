
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import Constraints, JointConstraint

from mm_grasp.kinematics_solver import (
    KinematicsSolver,
    JOINT_NAMES
)


class MotionPlanner(Node):
    def __init__(self):
        super().__init__('motion_planner')

        self.current_positions = None

        # 1. 현재 관절 상태 구독
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            qos_profile_sensor_data
        )

        # 2. MoveIt2 경로 계획 서비스
        self.client = self.create_client(
            GetMotionPlan,
            '/plan_kinematic_path'
        )

        self.get_logger().info('Waiting for MoveIt2 planner...')

        while rclpy.ok() and not self.client.wait_for_service(
            timeout_sec=1.0
        ):
            self.get_logger().info('Planner not available yet')

    def joint_callback(self, msg):
        # 관절 이름과 각도를 딕셔너리로 묶기
        joint_map = dict(zip(msg.name, msg.position))

        # 필요한 5개 관절이 모두 있는지 확인
        if not all(name in joint_map for name in JOINT_NAMES):
            return

        # joint_states의 순서와 상관없이 저장
        self.current_positions = [
            float(joint_map[name]) for name in JOINT_NAMES
        ]

    def get_current_joints(self):
        self.get_logger().info('Waiting for /joint_states...')

        # 최대 약 10초 동안 관절 상태 기다리기
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.5)

            if self.current_positions is not None:
                self.get_logger().info(
                    'Current joint states received'
                )

                for name, angle in zip(
                    JOINT_NAMES, self.current_positions
                ):
                    self.get_logger().info(
                        f'{name}: {angle:.6f} rad'
                    )

                return list(self.current_positions)

        self.get_logger().error('Failed to read /joint_states')
        return None

    def plan(self, start_angles, target_angles):
        request = GetMotionPlan.Request()
        motion = request.motion_plan_request

        motion.group_name = 'dofbot_arm'

        # 3. 실제 현재 관절각을 시작 상태로 지정
        motion.start_state.is_diff = False
        motion.start_state.joint_state.name = list(JOINT_NAMES)
        motion.start_state.joint_state.position = list(start_angles)

        # 4. IK 결과를 목표 관절각으로 설정
        goal = Constraints()

        for name, angle in zip(JOINT_NAMES, target_angles):
            constraint = JointConstraint()
            constraint.joint_name = name
            constraint.position = float(angle)
            constraint.tolerance_above = 0.01
            constraint.tolerance_below = 0.01
            constraint.weight = 1.0

            goal.joint_constraints.append(constraint)

        motion.goal_constraints = [goal]

        motion.allowed_planning_time = 5.0
        motion.num_planning_attempts = 5
        motion.max_velocity_scaling_factor = 0.2
        motion.max_acceleration_scaling_factor = 0.2

        self.get_logger().info('Requesting motion plan...')

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.exception() is not None:
            self.get_logger().error(
                f'Planning service error: {future.exception()}'
            )
            return

        response = future.result()

        if response is None:
            self.get_logger().error('No planning response')
            return

        result = response.motion_plan_response

        if result.error_code.val != 1:
            self.get_logger().error(
                f'PLANNING FAILED: {result.error_code.val}'
            )
            return

        # 5. 경로 계획 결과
        trajectory = result.trajectory.joint_trajectory

        self.get_logger().info('PLANNING SUCCESS')
        self.get_logger().info(
            f'Trajectory points: {len(trajectory.points)}'
        )

        if trajectory.points:
            first = trajectory.points[0]
            last = trajectory.points[-1]

            self.get_logger().info('Planned start:')
            for name, angle in zip(
                trajectory.joint_names, first.positions
            ):
                self.get_logger().info(
                    f'  {name}: {angle:.6f} rad'
                )

            self.get_logger().info('Planned goal:')
            for name, angle in zip(
                trajectory.joint_names, last.positions
            ):
                self.get_logger().info(
                    f'  {name}: {angle:.6f} rad'
                )


def main(args=None):
    rclpy.init(args=args)

    planner = None
    solver = None

    try:
        planner = MotionPlanner()

        # STEP 1: 현재 관절각 읽기
        start_angles = planner.get_current_joints()

        if start_angles is None:
            return

        # STEP 2: IK 계산 + FK 검증
        solver = KinematicsSolver()

        target_angles = solver.run_ik_fk(
            seed_positions=start_angles
        )

        if target_angles is None:
            planner.get_logger().error(
                'IK/FK failed. Planning cancelled.'
            )
            return

        # STEP 3: IK 결과로 MoveIt2 경로 계획
        planner.plan(start_angles, target_angles)

    finally:
        if solver is not None:
            solver.destroy_node()

        if planner is not None:
            planner.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()
