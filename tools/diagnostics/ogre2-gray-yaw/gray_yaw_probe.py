#!/usr/bin/env python3
"""제자리 한 바퀴 회전하면서 /camera/image_raw 회색 프레임을 찾는다 (진단 전용).

사용: gray_yaw_probe.py <출력 폴더> [회전 속도 rad/s, 기본 0.35] [회전량 rad, 기본 2*pi]

- /odom 으로 yaw를 읽고 /cmd_vel 로 제자리 회전 (로봇은 원점, yaw 0에서 생성되므로 odom yaw = 월드 yaw)
- 이미지마다 픽셀 표준편차와 색 개수(4픽셀 간격 표본)를 계산해 거의 0이면 회색 프레임으로 판정
- 각 이미지에는 header.stamp와 가장 가까운 odom의 yaw를 붙인다
- 같은 stamp의 /camera/depth/image_raw 유효 픽셀 비율도 기록 (회색 + 깊이 유효 0 이면 렌더링 실패, 무늬 없는 벽이면 깊이는 유효)
- 결과: frames.csv (프레임별), summary.json, 콘솔 요약
"""
import bisect
import csv
import json
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import camera_health as ch   # noqa: E402  회색 판정 공용 모듈 (tools/diagnostics/camera_health.py)

OUT = sys.argv[1]
SPEED = float(sys.argv[2]) if len(sys.argv) > 2 else 0.35
TURN = float(sys.argv[3]) if len(sys.argv) > 3 else 2 * math.pi


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def stamp_ns(s):
    return s.sec * 1_000_000_000 + s.nanosec


class Probe(Node):
    def __init__(self):
        super().__init__('ogre2_gray_yaw_probe')
        # ros_gz_bridge는 RELIABLE로 발행. 구독도 RELIABLE이어야 프레임이 빠지지 않는다
        img_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Image, '/camera/image_raw', self.on_image, img_qos)
        self.create_subscription(Odometry, '/odom', self.on_odom, 50)
        self.create_subscription(Image, '/camera/depth/image_raw', self.on_depth, img_qos)
        self.depth_valid = {}                 # stamp_ns -> 유효 깊이 비율
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_t, self.odom_yaw = [], []   # stamp_ns 순서
        self.prev_yaw = None
        self.turned = 0.0                    # 누적 회전량 (rad)
        self.rows = []
        self.skipped = {}

    def on_odom(self, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        if self.prev_yaw is not None:
            self.turned += wrap(yaw - self.prev_yaw)
        self.prev_yaw = yaw
        self.odom_t.append(stamp_ns(m.header.stamp))
        self.odom_yaw.append(yaw)

    def yaw_at(self, t):
        if not self.odom_t:
            return None, None
        i = bisect.bisect_left(self.odom_t, t)
        cand = [k for k in (i - 1, i) if 0 <= k < len(self.odom_t)]
        k = min(cand, key=lambda k: abs(self.odom_t[k] - t))
        return self.odom_yaw[k], (self.odom_t[k] - t) / 1e6

    def on_depth(self, m):
        v = ch.depth_valid_ratio(m)
        if v is None:
            self.skipped['depth:' + m.encoding] = self.skipped.get('depth:' + m.encoding, 0) + 1
            return
        self.depth_valid[stamp_ns(m.header.stamp)] = v

    def on_image(self, m):
        r = ch.check_image_msg(m)
        if r is None:
            self.skipped[m.encoding] = self.skipped.get(m.encoding, 0) + 1
            return
        t = stamp_ns(m.header.stamp)
        yaw, dt_ms = self.yaw_at(t)
        self.rows.append({'stamp_ns': t, 'stamp_s': round(t / 1e9, 3), 'yaw_deg': round(math.degrees(yaw), 2) if yaw is not None else None,
                          'odom_dt_ms': round(dt_ms, 1) if dt_ms is not None else None, 'std': r['std'],
                          'unique_colors': r['unique_colors'], 'mean': r['mean'],
                          'gray': r['gray'], 'phase': self.phase})

    phase = 'wait'

    def spin_for(self, sec):
        t = time.monotonic()
        while time.monotonic() - t < sec:
            rclpy.spin_once(self, timeout_sec=0.02)

    def drive(self, wz, sec):
        t = time.monotonic()
        while time.monotonic() - t < sec:
            tw = Twist(); tw.angular.z = float(wz); self.cmd.publish(tw)
            self.spin_for(0.05)


def intervals(rows):
    out, cur = [], None
    for r in rows:
        if r['gray'] and r['yaw_deg'] is not None:
            if cur is None:
                cur = {'yaw_from_deg': r['yaw_deg'], 'yaw_to_deg': r['yaw_deg'], 'frames': 0, 't_from_s': r['stamp_s']}
            cur['yaw_to_deg'] = r['yaw_deg']; cur['frames'] += 1; cur['t_to_s'] = r['stamp_s']
            if r.get('depth_valid') is not None:
                cur.setdefault('depth_valid', []).append(r['depth_valid'])
        elif cur is not None:
            out.append(cur); cur = None
    if cur is not None:
        out.append(cur)
    for x in out:
        dv = x.pop('depth_valid', [])
        x['depth_valid_max'] = max(dv) if dv else None
        x['depth_frames'] = len(dv)
    return out


def main():
    os.makedirs(OUT, exist_ok=True)
    rclpy.init()
    n = Probe()
    t0 = time.monotonic()
    while (not n.odom_t or not n.rows) and time.monotonic() - t0 < 15:
        n.spin_for(0.1)
    if not n.odom_t or not n.rows:
        print('FAIL: /odom 또는 /camera/image_raw 수신 없음 (15 s)', flush=True)
        n.destroy_node(); rclpy.shutdown(); return 3
    n.phase = 'still'; n.spin_for(1.0)                 # 회전 전 1 s 정지 상태 기록
    n.phase = 'rotate'; start = n.turned
    limit = abs(TURN / SPEED) * 1.5 + 5
    t1 = time.monotonic()
    while abs(n.turned - start) < TURN and time.monotonic() - t1 < limit:
        n.drive(SPEED, 0.1)
    turned = abs(n.turned - start)
    n.phase = 'stop'
    for _ in range(10):
        n.cmd.publish(Twist()); n.spin_for(0.05)
    n.spin_for(0.5)

    rows = n.rows
    for r in rows:   # 같은 stamp 깊이 (늦게 온 깊이까지 반영하려고 끝나고 붙임)
        v = n.depth_valid.get(r.pop('stamp_ns'))
        r['depth_valid'] = round(v, 3) if v is not None else None
    gray = [r for r in rows if r['gray']]
    iv = intervals(rows)
    rot = [r for r in rows if r['phase'] == 'rotate']
    summary = {'frames': len(rows), 'gray_frames': len(gray), 'rotate_frames': len(rot),
               'turned_deg': round(math.degrees(turned), 1), 'speed_rad_s': SPEED,
               'deg_per_frame': round(math.degrees(turned) / len(rot), 2) if rot else None,
               'gray_rule': ch.RULE,
               'gray_intervals': iv, 'skipped_encodings': n.skipped,
               'std_range_nongray': [min((r['std'] for r in rows if not r['gray']), default=None),
                                     max((r['std'] for r in rows if not r['gray']), default=None)]}
    with open(os.path.join(OUT, 'frames.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
    json.dump(summary, open(os.path.join(OUT, 'summary.json'), 'w'), indent=1, ensure_ascii=False)

    print(f"프레임 {len(rows)}개 (회전 중 {len(rot)}개), 회전량 {summary['turned_deg']}°, "
          f"프레임당 약 {summary['deg_per_frame']}°, 회색 프레임 {len(gray)}개")
    if rot and all(r['gray'] for r in rot):
        print('회전하는 동안 모든 프레임이 회색 (yaw와 무관하게 카메라 전체가 렌더링되지 않음). '
              'Gazebo 시작 때 가끔 생기는 현상일 수 있으니 한 번 더 실행해 주세요.')
    elif iv:
        print('회색 구간 (odom yaw, 월드 기준):')
        for k, x in enumerate(iv, 1):
            dv = x['depth_valid_max']
            kind = ch.gray_kind(dv)
            print(f"  {k}) yaw {x['yaw_from_deg']:+.1f}° ~ {x['yaw_to_deg']:+.1f}°  ({x['frames']}프레임, sim {x['t_from_s']}~{x['t_to_s']} s, "
                  f"깊이 유효 최대 {dv} [{x['depth_frames']}프레임] → {kind})")
    else:
        print('회색 현상 없음')
    if n.skipped:
        print('처리하지 못한 이미지 인코딩:', n.skipped)
    if turned < TURN * 0.95:
        print(f'주의: 목표 회전량에 못 미침 ({summary["turned_deg"]}°)')
    n.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
