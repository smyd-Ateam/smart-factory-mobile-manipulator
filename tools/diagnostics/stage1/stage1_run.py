"""통합 1단계 실제 시험 (scratchpad 전용): Nav2(object 모드) → Near → Align 1차 → Pre-grasp → Align 2차.
실제 YOLO(/perception/detections) + 실제 깊이. /servoing/cmd_vel → /cmd_vel 중계는 이 스크립트가 함
(상태머신 2단계 호출은 아직 미구현이라 순서를 여기서 진행). 사용: stage1_run.py <out.json> <target_class>"""
import json, math, os, subprocess, sys, time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
import bisect
# 회색 프레임 판정 공용 모듈: 같은 폴더에 복사본이 있으면 그것을, 없으면 레포의 tools/diagnostics/camera_health.py를 쓴다
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import camera_health as ch  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy
from vision_msgs.msg import Detection2DArray
from nav2_msgs.action import NavigateToPose
from project_interfaces.action import Align
from tf2_ros import Buffer, TransformListener
from mm_task_manager.geometry import approach_goal

S = os.environ.get('STAGE1_DIR', '/tmp/sfmm-brass-v7')   # 시험 폴더 (env_s1.sh, src/, yolo_weights/, tools/ 가 있는 곳)
OUT, TARGET = sys.argv[1], sys.argv[2]
GOAL_OVERRIDE = [float(v) for v in sys.argv[3].split(',')] if len(sys.argv) > 3 and sys.argv[3] else None   # x,y,yaw (yaw 대조 시험용)
OBJ = {'block': (0.8, 0.0, 0.04), 'bolt': (0.8, 0.24, 0.06), 'hex_nut': (0.8, 0.12, 0.015), 'pin': (0.8, -0.12, 0.05)}[TARGET]
STANDOFF, MARGIN, ALIGN_TIMEOUT = 0.35, 0.15, 30.0
NEAR_STANDOFF = float(os.environ.get('NEAR_STANDOFF', STANDOFF))   # Align 1차(Near) 목표 거리. Pre-grasp(2차)는 STANDOFF
SCRIPTS = f'{S}/src/mm_description/scripts'
POSES = {'far': ('set_observe_pose_far.sh', -1.1464, -0.8237), 'near': ('set_observe_pose_near.sh', -1.2821, -1.2991),
         'pregrasp': ('set_pre_grasp_pose_01.sh', -0.959445, -1.190737)}
log = {'camera_gray_detected': False, 'camera_gray_windows': [], 'target': TARGET, 'object_xyz': OBJ, 'standoff_m': STANDOFF, 'near_standoff_m': NEAR_STANDOFF, 'steps': {}}
T0 = time.monotonic()
def stamp(): return round(time.monotonic() - T0, 2)
def save(): json.dump(log, open(OUT, 'w'), indent=1, ensure_ascii=False)
log['phases'] = []
def note(k, v): log['steps'][k] = v; log['phases'].append([k, stamp()]); save(); print(f'[{stamp():7.1f}s] {k}: {json.dumps(v, ensure_ascii=False)[:400]}', flush=True)

def gz(model='mm_robot'):
    out = subprocess.run(['timeout', '10', 'ign', 'model', '-m', model, '-p'], capture_output=True, text=True).stdout
    l = out.splitlines(); i = [k for k, x in enumerate(l) if 'Pose [' in x][0]
    p = [float(v) for v in l[i + 1].strip().strip('[]').split()]; r = [float(v) for v in l[i + 2].strip().strip('[]').split()]
    return p[0], p[1], r[2]
def wrap(a): return (a + math.pi) % (2 * math.pi) - math.pi
def truth(standoff=STANDOFF):
    x, y, yaw = gz(); dx, dy = OBJ[0] - x, OBJ[1] - y
    rng = math.hypot(dx, dy); brg = wrap(math.atan2(dy, dx) - yaw)
    return {'gz_pose': [round(x, 4), round(y, 4), round(yaw, 4)], 'range_m': round(rng, 4), 'range_err_m': round(rng - standoff, 4), 'bearing_rad': round(brg, 4)}

rclpy.init(); n = Node('stage1_run')
buf = Buffer(); TransformListener(buf, n)
cmd_pub = n.create_publisher(Twist, '/cmd_vel', 10)
relay = {'on': False, 'count': 0, 'last': None}
lat = {'servo': [], 'cmd': []}
def on_servo(m):
    if relay['on']:
        lat['servo'].append((time.monotonic(), round(m.linear.x, 5), round(m.angular.z, 5)))
        cmd_pub.publish(m); relay['count'] += 1; relay['last'] = (round(m.linear.x, 3), round(m.angular.z, 3))
def on_cmd(m):
    if relay['on']: lat['cmd'].append((time.monotonic(), round(m.linear.x, 5), round(m.angular.z, 5)))
n.create_subscription(Twist, '/cmd_vel', on_cmd, 50)
def latency_stats():
    d = []; j = 0; c = lat['cmd']
    for t, vx, wz in lat['servo']:
        while j < len(c) and c[j][0] < t: j += 1
        k = j
        while k < len(c) and c[k][0] - t < 0.5 and (c[k][1], c[k][2]) != (vx, wz): k += 1
        if k < len(c) and c[k][0] - t < 0.5: d.append(c[k][0] - t); j = k + 1
    d.sort()
    return {'pairs': len(d), 'servo_msgs': len(lat['servo']), 'median_ms': round(1000 * d[len(d) // 2], 2) if d else None,
            'p95_ms': round(1000 * d[int(.95 * (len(d) - 1))], 2) if d else None, 'max_ms': round(1000 * d[-1], 2) if d else None}
n.create_subscription(Twist, '/servoing/cmd_vel', on_servo, 10)
dets = []   # (header.stamp sim 초, 목표 클래스 최고 점수, 클래스 목록, 수신 monotonic)
def on_det(m):
    best = max((d.results[0].hypothesis.score for d in m.detections if d.results and d.results[0].hypothesis.class_id == TARGET), default=None)
    st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    dets.append((st, best, sorted({d.results[0].hypothesis.class_id for d in m.detections if d.results}), time.monotonic()))
n.create_subscription(Detection2DArray, '/perception/detections', on_det, 50)
# 구간 경계용 sim 시간. KEEP_LAST 1이라 블로킹 뒤에도 최신 값 하나만 남음
simclock = {'t': None, 'n': 0}
def on_clock(m): simclock['t'] = m.clock.sec + m.clock.nanosec * 1e-9; simclock['n'] += 1
n.create_subscription(Clock, '/clock', on_clock, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
def spin(sec):
    t = time.monotonic()
    while time.monotonic() - t < sec: rclpy.spin_once(n, timeout_sec=0.02)
def stop_robot():
    for _ in range(10): cmd_pub.publish(Twist()); spin(0.05)
# ---- 카메라 회색 가드: 회색 프레임(ogre2 렌더링 버그)을 감지하면 경고와 결과 플래그만 남긴다 (시험은 계속) ----
odom_t, odom_yaw = [], []
def on_odom(m):
    q = m.pose.pose.orientation
    odom_t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
    odom_yaw.append(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
    if len(odom_t) > 4000: del odom_t[:2000], odom_yaw[:2000]
n.create_subscription(Odometry, '/odom', on_odom, 50)
def yaw_deg_at(t):   # odom yaw (로봇이 원점·yaw 0에서 생성되므로 월드 yaw와 같음)
    if not odom_t: return None
    i = bisect.bisect_left(odom_t, t); k = min((j for j in (i - 1, i) if 0 <= j < len(odom_t)), key=lambda j: abs(odom_t[j] - t))
    return round(math.degrees(odom_yaw[k]), 2)
camg = {'sub': None, 'tag': None, 'frames': [], 'in_gray': False}
CAM_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)   # 감시용 표본. 시험 부하를 늘리지 않으려고 BEST_EFFORT
def on_cam(m):
    r = ch.check_image_msg(m)
    if r is None: return
    st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    r.update(stamp=round(st, 3), yaw_deg=yaw_deg_at(st), tag=camg['tag']); camg['frames'].append(r)
    if r['gray'] and not camg['in_gray']:
        print(f"[{stamp():7.1f}s] [CAMERA_GRAY_WARNING] tag={camg['tag']} yaw={r['yaw_deg']} deg stamp={r['stamp']} std={r['std']} colors={r['unique_colors']}", flush=True)
    elif camg['in_gray'] and not r['gray']:
        print(f"[{stamp():7.1f}s] [CAMERA_GRAY_RECOVERED] tag={camg['tag']} yaw={r['yaw_deg']} deg stamp={r['stamp']}", flush=True)
    camg['in_gray'] = r['gray']
def cam_watch(tag):
    camg['tag'] = tag; camg['in_gray'] = False
    if camg['sub'] is None: camg['sub'] = n.create_subscription(Image, '/camera/image_raw', on_cam, CAM_QOS)
def cam_unwatch():
    if camg['sub'] is not None: n.destroy_subscription(camg['sub']); camg['sub'] = None
def cam_stats(t_from, t_to, name):
    """stamp(sim)가 [t_from, t_to] 안인 감시 프레임 집계. 회색이 있으면 경고 + 결과 플래그."""
    if t_from is None or t_to is None: return {'error': 'no /clock'}
    w = [f for f in camg['frames'] if t_from <= f['stamp'] <= t_to]; g = [f for f in w if f['gray']]
    out = {'rule': ch.RULE, 'qos': 'BEST_EFFORT depth 1 (표본)', 'window_sim': [round(t_from, 3), round(t_to, 3)],
           'frames': len(w), 'gray_frames': len(g), 'gray_detected': bool(g)}
    if not w: out['warning'] = '구간 안 카메라 프레임 없음 (판정 불가)'
    if g:
        ys = [f['yaw_deg'] for f in g if f['yaw_deg'] is not None]
        out.update(first_gray={k: g[0][k] for k in ('stamp', 'yaw_deg', 'std', 'unique_colors')},
                   gray_yaw_range_deg=[min(ys), max(ys)] if ys else None)
        log['camera_gray_detected'] = True; log['camera_gray_windows'].append(name); save()
        print(f"[{stamp():7.1f}s] [CAMERA_GRAY_WARNING] window={name} gray {len(g)}/{len(w)} frames, yaw {out['gray_yaw_range_deg']} deg -> 이 구간 결과는 카메라 회색 상태에서 나온 것", flush=True)
    return out

def sim_now():
    """현재 sim 시간 (s). 대기 중이던 콜백을 먼저 처리하고, 호출 뒤 새로 받은 /clock 값을 쓴다."""
    c0, t = simclock['n'], time.monotonic()
    while simclock['n'] < c0 + 2 and time.monotonic() - t < 2.0: rclpy.spin_once(n, timeout_sec=0.02)
    return simclock['t']
def det_stats(t_from, t_to):
    """detection header.stamp(sim 초)가 [t_from, t_to] 안인 메시지 집계. rate_hz는 sim 시간 기준."""
    base = {'time_base': 'header.stamp (sim)', 'window_sim': [round(t_from, 3), round(t_to, 3)] if t_from is not None and t_to is not None else None}
    if t_from is None or t_to is None: return dict(base, msgs=0, error='no /clock')
    w = sorted((d for d in dets if t_from <= d[0] <= t_to), key=lambda d: d[0])
    if not w: return dict(base, msgs=0)
    sc = [d[1] for d in w if d[1] is not None]; dur = w[-1][0] - w[0][0]
    return {**base, 'msgs': len(w), 'rate_hz': round((len(w) - 1) / dur, 2) if dur > 0 else None, 'target_frames': len(sc),
            'target_ratio': round(len(sc) / len(w), 3), 'score_mean': round(sum(sc) / len(sc), 3) if sc else None,
            'score_min': round(min(sc), 3) if sc else None, 'score_max': round(max(sc), 3) if sc else None,
            'classes_seen': sorted({c for d in w for c in d[2]})}
def set_pose(name):
    script, j3, j4 = POSES[name]
    for a in range(1, 4):
        subprocess.run(['timeout', '60', 'bash', f'{SCRIPTS}/{script}'], capture_output=True)
        spin(6.0)
        out = subprocess.run(['timeout', '20', 'python3', f'{S}/tools/joints.py', '10'], capture_output=True, text=True).stdout.strip().splitlines()
        j = eval(out[-1]) if out and out[-1].startswith('{') else {}
        if j and abs(j['joint3'] - j3) < 0.01 and abs(j['joint4'] - j4) < 0.01:
            return {'pose': name, 'attempt': a, 'joint3': j['joint3'], 'joint4': j['joint4']}
    return {'pose': name, 'reached': False, 'joints': j}
def checkpoint(tag):
    c = snapshot(tag); c['truth'] = truth(); c['t'] = stamp()
    c['gray'] = not ((c.get('rgb_unique_colors') or 0) >= 50 and (c.get('depth_valid_ratio') or 0) >= 0.9)
    log.setdefault('checkpoints', {})[tag] = c; save()
    print(f"[{stamp():7.1f}s] checkpoint {tag}: colors={c.get('rgb_unique_colors')} depth_valid={c.get('depth_valid_ratio')} gray={c['gray']}", flush=True)
    return c
def snapshot(tag):
    x, y, yaw = gz()
    bp = subprocess.run(['python3', f'{S}/tools/world_to_base.py', str(x), str(y), '0', '0', '0', str(yaw), f'{TARGET}_top={OBJ[0]},{OBJ[1]},{OBJ[2]}'],
                        capture_output=True, text=True).stdout.split()
    subprocess.run(['timeout', '40', 'python3', f'{S}/tools/cam_check.py', f'{S}/s1_cam_{tag}.json', '3', *bp], capture_output=True)
    try:
        c = json.load(open(f'{S}/s1_cam_{tag}.json')); d = c.get('depth') if isinstance(c.get('depth'), dict) else {}
        r = c.get('rgb') if isinstance(c.get('rgb'), dict) else {}
        return {'depth_valid_ratio': d.get('valid_ratio'), 'depth_inf_ratio': d.get('inf_ratio'), 'rgb_unique_colors': r.get('unique_colors'),
                'rgb_std': r.get('std'), 'target_proj': (d.get('parts') or {}).get(f'{TARGET}_top'), 'rates_3s': c.get('rates_3s')}
    except Exception as e:
        return {'error': str(e)}

def run_align(tag, standoff=STANDOFF):
    ac = ActionClient(n, Align, '/servoing/align')
    if not ac.wait_for_server(timeout_sec=20): return {'error': 'no /servoing/align server'}
    g = Align.Goal(); g.target_class = TARGET; g.standoff_m = standoff; g.timeout_sec = ALIGN_TIMEOUT
    cam_watch(tag)
    s0 = sim_now()   # 집계 구간 시작 (sim). 블로킹 중 쌓인 메시지는 stamp가 이보다 앞이라 빠짐
    fb = []; t0 = time.monotonic()
    relay['on'] = True; relay['count'] = 0; lat['servo'].clear(); lat['cmd'].clear()
    f = ac.send_goal_async(g, feedback_callback=lambda m: fb.append((round(time.monotonic() - t0, 2), m.feedback.target_visible,
                                                                     round(m.feedback.range_error_m, 4), round(m.feedback.bearing_error_rad, 4))))
    while not f.done() and time.monotonic() - t0 < 10: spin(0.05)
    h = f.result()
    if not h or not h.accepted: relay['on'] = False; cam_unwatch(); return {'error': 'goal rejected'}
    r = h.get_result_async()
    while not r.done() and time.monotonic() - t0 < ALIGN_TIMEOUT + 15: spin(0.05)
    dt = time.monotonic() - t0
    s1 = sim_now()   # 집계 구간 끝 (sim)
    spin(0.5); relay['on'] = False; stop_robot(); spin(1.0)
    res = r.result().result if r.done() else None
    out = {'elapsed_s': round(dt, 2), 'status': r.result().status if r.done() else 'NO RESULT',
           'success': res.success if res else None, 'error_code': res.error_code if res else None,
           'final_range_err_m': round(res.final_range_error_m, 4) if res else None,
           'final_bearing_err_rad': round(res.final_bearing_error_rad, 4) if res else None,
           'relayed_cmds': relay['count'], 'feedback_n': len(fb), 'feedback_first': fb[:3], 'feedback_last': fb[-5:],
           'feedback_visible_ratio': round(sum(1 for x in fb if x[1]) / len(fb), 3) if fb else None,
           'detections': det_stats(s0, s1)}
    cam_unwatch()
    out['camera'] = cam_stats(s0, s1, tag)
    if s1 is not None:   # 정렬 마지막 1 s ≈ CHECKING(정지 후 확인) 구간
        out['camera']['last_1s'] = {k: v for k, v in cam_stats(s1 - 1.0, s1, tag + '_last_1s').items() if k in ('frames', 'gray_frames', 'gray_detected', 'warning')}
    out['relay_latency'] = latency_stats()
    out['standoff_m'] = standoff
    out['truth_after'] = truth(standoff)
    if res is not None:
        out['reported_minus_truth_range_mm'] = round(1000 * (res.final_range_error_m - out['truth_after']['range_err_m']), 1)
    return out

procs = []
try:
    # 1) 준비 확인
    t = time.monotonic()
    while not buf.can_transform('map', 'odom', Time()) and time.monotonic() - t < 120: spin(0.5)
    nav = ActionClient(n, NavigateToPose, 'navigate_to_pose')
    ok = nav.wait_for_server(timeout_sec=60); spin(8.0)   # map→odom 오래된 상태 회피
    note('ready', {'map_odom': buf.can_transform('map', 'odom', Time()), 'nav_server': ok})
    note('pose_far', set_pose('far'))
    checkpoint('1_origin_far')
    # 2) object 모드 목표 (물체에서 standoff + margin)
    x, y, yaw = gz()
    gx, gy, gyaw = GOAL_OVERRIDE if GOAL_OVERRIDE else approach_goal(OBJ[:2], (0.0, 0.0, 0.0), STANDOFF + MARGIN)
    goal = NavigateToPose.Goal(); goal.pose.header.frame_id = 'map'
    goal.pose.pose.position.x, goal.pose.pose.position.y = gx, gy
    goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(gyaw / 2), math.cos(gyaw / 2)
    phantom = 0
    for nav_try in range(2):
        t0 = time.monotonic(); recs = {}
        f = nav.send_goal_async(goal, feedback_callback=lambda m: recs.update(rec=m.feedback.number_of_recoveries))
        while not f.done() and time.monotonic() - t0 < 10: spin(0.05)
        r = f.result().get_result_async()
        while not r.done() and time.monotonic() - t0 < 120: spin(0.05)
        px, py, _ = gz()
        # 가짜 성공: 1 s 안에 SUCCEEDED인데 목표에서 15 cm 넘게 떨어짐 (map→odom 오래된 상태) → 5 s 뒤 한 번 다시
        if r.done() and r.result().status == 4 and time.monotonic() - t0 < 1.0 and math.hypot(px - gx, py - gy) > 0.15:
            phantom += 1; print(f'[{stamp():7.1f}s] phantom nav success, retry after 5 s', flush=True); spin(5.0); continue
        break
    nav_out = {'goal': [round(gx, 4), round(gy, 4), round(gyaw, 4)], 'status': r.result().status if r.done() else 'TIMEOUT',
               'elapsed_s': round(time.monotonic() - t0, 2), 'recoveries': recs.get('rec', 0), 'phantom_retries': phantom}
    spin(1.0); nav_out['truth'] = truth()
    gzp = nav_out['truth']['gz_pose']; nav_out['goal_err_xy_cm'] = round(100 * math.hypot(gzp[0] - gx, gzp[1] - gy), 1)
    nav_out['goal_err_yaw_deg'] = round(math.degrees(abs(wrap(gzp[2] - gyaw))), 1)
    note('nav', nav_out)
    checkpoint('2_after_nav_far')
    cm = subprocess.run(['timeout', '35', 'python3', f'{S}/tools/costmap_self.py', '/local_costmap/costmap', '25'], capture_output=True, text=True).stdout.strip().splitlines()
    try:
        cmj = json.loads(cm[-1]); note('costmap_after_nav', {k: cmj.get(k) for k in ('robot', 'footprint_inside', 'ring_0.1m', 'rear_-0.20_to_-0.60', 'lethal_within_1m_base_xy(first15)', 'published_footprint_pts')})
    except Exception as e:
        note('costmap_after_nav', {'error': str(e), 'raw': cm[-2:] if cm else None})
    note('parts_gz', {p: list(gz('test_' + p))[:2] for p in ('block', 'pin', 'hex_nut', 'bolt')})
    spin(3.0); checkpoint('3_before_near_far')
    if nav_out['status'] != 4: raise SystemExit('nav failed')
    # 3) Near 자세, Nav2 종료(/cmd_vel 충돌 방지), YOLO · align_node 시작
    note('pose_near', set_pose('near'))
    checkpoint('4_after_near')
    subprocess.run(['pkill', '-f', '[o]pt/ros/humble/lib/nav2_']); spin(3.0)
    note('nav2_killed', {'cmd_vel_publishers_left': n.count_publishers('/cmd_vel') - 1})
    yenv = dict(os.environ); yenv.pop('PYTHONNOUSERSITE', None); yenv['YOLO_AUTOINSTALL'] = 'False'
    procs.append(subprocess.Popen(['timeout', '600', 'ros2', 'run', 'mm_perception', 'yolo_detector', '--ros-args',
        '-p', f'model:={S}/yolo_weights/synth_v7_640_openvino_model', '-p', 'ov_precision:=f32',
        '-p', 'image_reliability:=reliable', '-p', 'image_depth:=2',
        '-p', f'save_dir:={S}/frames_{os.environ.get("TAG", TARGET)}', '-p', 'save_every:=1',
        '-p', 'publish_image:=false', '-p', 'use_sim_time:=true'], env=yenv, stdout=open(f'{S}/s1_yolo_{os.environ.get("TAG", TARGET)}.log', 'w'), stderr=subprocess.STDOUT))
    procs.append(subprocess.Popen(['timeout', '600', 'ros2', 'run', 'visual_servoing', 'align_node', '--ros-args', '-p', 'use_sim_time:=true'],
        stdout=open(f'{S}/s1_align_{os.environ.get("TAG", TARGET)}.log', 'w'), stderr=subprocess.STDOUT))
    t = time.monotonic(); n_det0 = len(dets)
    while time.monotonic() - t < 60 and len(dets) == n_det0: spin(0.2)
    cam_watch('near_before_align')
    spin(3.0); tn = sim_now(); spin(3.0); tn_end = sim_now()
    spin(0.5)   # 구간 끝 무렵 stamp의 메시지(YOLO 지연 약 50 ms)가 도착한 뒤 집계
    cam_unwatch()
    note('near_before_align', {'detections_3s': det_stats(tn, tn_end), 'camera_3s': cam_stats(tn, tn_end, 'near_before_align'), 'cam': snapshot('near'), 'truth': truth(NEAR_STANDOFF)})
    # 4) Align 1차 (Near)
    a1 = run_align('align1', NEAR_STANDOFF); note('align1_near', a1)
    if not a1.get('success'): raise SystemExit('align1 failed')
    if os.environ.get('STOP_AFTER_ALIGN1') == '1': raise SystemExit('stop after align1 (STOP_AFTER_ALIGN1=1)')
    # 5) Pre-grasp 전환 후 Align 2차
    note('pose_pregrasp', set_pose('pregrasp'))
    cam_watch('pregrasp_before_align')
    spin(2.0); tp = sim_now(); spin(3.0); tp_end = sim_now()
    spin(0.5)   # 위와 같음
    cam_unwatch()
    note('pregrasp_before_align', {'detections_3s': det_stats(tp, tp_end), 'camera_3s': cam_stats(tp, tp_end, 'pregrasp_before_align'), 'cam': snapshot('pregrasp'), 'truth': truth()})
    a2 = run_align('align2'); note('align2_pregrasp', a2)
except SystemExit as e:
    note('stopped', {'reason': str(e)})
finally:
    relay['on'] = False; stop_robot()
    for p in procs: p.terminate()
    for p in procs:
        try: p.wait(timeout=10)
        except Exception: p.kill()
    log['total_s'] = stamp(); save(); print('STAGE1_DONE', flush=True)
