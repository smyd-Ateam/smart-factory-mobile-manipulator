"""모의 입력 폐루프 정렬 반복 시험.

mock_scene_node와 align_node가 떠 있는 상태에서 실행한다.
시험마다 차체를 Nav2 도착 오차만큼 흐트러진 위치에 두고 Align 목표를 보낸 뒤,
액션 결과와 모의 차체의 실제 오차를 비교한다.

  ros2 run visual_servoing mock_trials --ros-args -p trials:=20

Gazebo 시험(sim_align.launch.py)에서는 scene:=sim, use_sim_time:=true 를 준다.
정렬 시간은 노드 시계 기준이라 Gazebo에서는 시뮬레이션 시간이다.

시작 위치: 물체에서 standoff_m + nav_margin 떨어진 곳(물체 정면)을 기준으로
위치 오차 반지름 nav_error_m 안 균등, yaw 오차 ±nav_yaw_error 균등.
"""
import csv
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from project_interfaces.action import Align
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Trigger

from visual_servoing.control import ControlParams

SCENES = ('mock', 'sim')   # mock_scene_node, sim_scene_node


class Trials(Node):

    def __init__(self):
        super().__init__('servoing_mock_trials')
        self.declare_parameter('scene', 'mock')
        self.declare_parameter('settle_sec', 0.5)       # 리셋 뒤 기다리는 시간 (벽시계)
        self.declare_parameter('wall_factor', 1.0)      # 결과 대기 = (timeout_sec + 10) × 이 값
        self.declare_parameter('trials', 20)
        self.declare_parameter('target_class', 'pin')
        self.declare_parameter('standoff_m', 0.25)
        self.declare_parameter('timeout_sec', 60.0)
        self.declare_parameter('nav_margin', 0.15)      # interfaces.md: Nav2 목표 = standoff + 여유
        self.declare_parameter('nav_error_m', 0.13)     # A 측정 Nav2 도착 오차 최대 약 0.126 m
        self.declare_parameter('nav_yaw_error', 0.15)   # rad, 가정값
        self.declare_parameter('object_xy', [0.60, 0.08])
        self.declare_parameter('seed', 0)
        self.declare_parameter('csv', '')
        # 예상 위치를 목표에 넣는 시험 (설계 문서 8장). 빈 값이면 넣지 않는다.
        # sim에서는 sim_scene_node가 world → base_link TF를 낸다.
        self.declare_parameter('expected_frame', '')
        self.declare_parameter('expected_radius_m', 0.07)
        self.declare_parameter('expected_noise_m', 0.0)  # 위치 추정 오차 흉내: 반지름 안 균등

        scene = self.p('scene')
        if scene not in SCENES:
            raise ValueError(f'scene은 {SCENES} 중 하나: {scene!r}')
        self.truth = None
        self.create_subscription(PointStamped, f'/servoing/{scene}_object_in_base',
                                 lambda m: setattr(self, 'truth', m.point), 10)
        self.client = ActionClient(self, Align, '/servoing/align')
        self.reset = self.create_client(Trigger, f'/servoing/{scene}_reset')
        self.set_params = self.create_client(
            SetParameters, f'/servoing_{scene}_scene/set_parameters')

    def p(self, name):
        return self.get_parameter(name).value

    def wait(self, future, timeout):
        end = time.time() + timeout
        while not future.done() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        return future.result() if future.done() else None

    def set_start(self, pose):
        prm = Parameter(name='robot_start', value=ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE_ARRAY, double_array_value=list(pose)))
        self.wait(self.set_params.call_async(SetParameters.Request(parameters=[prm])), 5.0)
        res = self.wait(self.reset.call_async(Trigger.Request()), 60.0)   # sim: ign CLI 재시도 포함
        return res is not None and res.success

    def run(self):
        for c, name in ((self.client, 'Align 서버'), (self.reset, 'mock_reset'),
                        (self.set_params, 'mock 파라미터')):
            ok = c.wait_for_server(timeout_sec=10.0) if c is self.client \
                else c.wait_for_service(timeout_sec=10.0)
            if not ok:
                self.get_logger().error(f'{name} 없음')
                return
        rng = np.random.default_rng(self.p('seed'))
        cp = ControlParams()
        standoff = self.p('standoff_m')
        ox, oy = self.p('object_xy')
        d = standoff + self.p('nav_margin')
        rows = []
        for i in range(self.p('trials')):
            r = self.p('nav_error_m') * math.sqrt(rng.random())
            a = rng.uniform(-math.pi, math.pi)
            start = (ox - d + r * math.cos(a), oy + r * math.sin(a),
                     rng.uniform(-self.p('nav_yaw_error'), self.p('nav_yaw_error')))
            if not self.set_start(start):
                self.get_logger().error(f'{i}: 시작 위치 리셋 실패, 시험 중단')
                break
            time.sleep(self.p('settle_sec'))   # 리셋 뒤 새 프레임이 들어오도록

            goal = Align.Goal(target_class=self.p('target_class'), standoff_m=standoff,
                              timeout_sec=self.p('timeout_sec'))
            if self.p('expected_frame'):
                en = self.p('expected_noise_m') * math.sqrt(rng.random())
                ea = rng.uniform(-math.pi, math.pi)
                goal.expected_position.header.frame_id = self.p('expected_frame')
                goal.expected_position.point.x = ox + en * math.cos(ea)
                goal.expected_position.point.y = oy + en * math.sin(ea)
                goal.expected_radius_m = self.p('expected_radius_m')
            t0 = self.get_clock().now()
            gh = self.wait(self.client.send_goal_async(goal), 5.0)
            if gh is None or not gh.accepted:
                self.get_logger().error(f'{i}: 목표 거부')
                continue
            res = self.wait(gh.get_result_async(),
                            (self.p('timeout_sec') + 10.0) * self.p('wall_factor'))
            if res is None:
                self.get_logger().error(f'{i}: 결과 대기 시간 초과, 시험 중단')
                break
            elapsed = (self.get_clock().now() - t0).nanoseconds / 1e9
            time.sleep(0.2)
            rclpy.spin_once(self, timeout_sec=0.1)
            tx, ty = self.truth.x, self.truth.y
            true_r = math.hypot(tx, ty) - standoff
            true_b = math.atan2(ty, tx)
            out = res.result
            ok = out.success and abs(true_r) <= cp.tol_range and abs(true_b) <= cp.tol_bearing
            rows.append(dict(trial=i, start_x=start[0], start_y=start[1], start_yaw=start[2],
                             success=out.success, error_code=out.error_code,
                             reported_range=out.final_range_error_m,
                             reported_bearing=out.final_bearing_error_rad,
                             true_range=true_r, true_bearing=true_b,
                             true_within_tol=ok, time_s=elapsed))
            self.get_logger().info(
                f'{i:2d}: success={out.success} code={out.error_code!r} '
                f'실제 e_r={true_r * 1000:+.1f} mm e_b={true_b:+.3f} rad '
                f'보고 e_r={out.final_range_error_m * 1000:+.1f} mm  {elapsed:.1f} s')

        if not rows:
            return
        succ = [x for x in rows if x['success']]
        print('\n=== 요약 ===')
        print(f'시험 {len(rows)}회, 액션 성공 {len(succ)}회 ({100 * len(succ) / len(rows):.0f} %)')
        if succ:
            tr = np.abs([x['true_range'] for x in succ]) * 1000
            tb = np.abs([x['true_bearing'] for x in succ])
            tt = [x['time_s'] for x in succ]
            bad = sum(not x['true_within_tol'] for x in succ)
            print(f'성공 중 실제 오차가 허용 범위 밖: {bad}회 (성공이라 보고했는데 실제로는 아닌 경우)')
            print(f'실제 거리 오차 |e_r|: 평균 {tr.mean():.1f} mm, p95 {np.percentile(tr, 95):.1f} mm, '
                  f'최대 {tr.max():.1f} mm')
            print(f'실제 방위 오차 |e_b|: 평균 {tb.mean():.3f} rad, p95 {np.percentile(tb, 95):.3f} rad, '
                  f'최대 {tb.max():.3f} rad')
            print(f'정렬 시간: 평균 {np.mean(tt):.1f} s, 최대 {np.max(tt):.1f} s')
        codes = {}
        for x in rows:
            if not x['success']:
                codes[x['error_code']] = codes.get(x['error_code'], 0) + 1
        if codes:
            print('실패 코드:', codes)
        if self.p('csv'):
            with open(self.p('csv'), 'w', newline='') as f:
                w = csv.DictWriter(f, fieldnames=list(rows[0]))
                w.writeheader()
                w.writerows(rows)
            print('CSV 저장:', self.p('csv'))


def main(args=None):
    rclpy.init(args=args)
    node = Trials()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
