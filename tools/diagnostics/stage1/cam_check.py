"""3개 카메라 토픽 점검 + 깊이 통계 + 부품 깊이 정답 비교 + 주기 (0-3 재시험용).
사용: cam_check.py <out.json> <hz_window_s> [name=x,y,z ...(base_footprint 기준 부품 윗면 점)]"""
import json, sys, time
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import Buffer, TransformListener
OUT, HZW = sys.argv[1], float(sys.argv[2]); pts = {}
for a in sys.argv[3:]:
    n, xyz = a.split('='); pts[n] = [float(v) for v in xyz.split(',')]
T = {'rgb': '/camera/image_raw', 'depth': '/camera/depth/image_raw', 'info': '/camera/camera_info'}
rclpy.init(); node = Node('cam_check'); buf = Buffer(); TransformListener(buf, node)
got = {k: [] for k in T}
for k, t in T.items():
    node.create_subscription(CameraInfo if k == 'info' else Image, t, lambda m, k=k: got[k].append((time.monotonic(), m)), qos_profile_sensor_data)
t0 = time.monotonic()
while time.monotonic() - t0 < 2.0: rclpy.spin_once(node, timeout_sec=0.05)
out = {'publishers': {k: node.count_publishers(t) for k, t in T.items()}}
for k in got: got[k].clear()
t0 = time.monotonic()
while time.monotonic() - t0 < HZW: rclpy.spin_once(node, timeout_sec=0.01)
def rate(lst):
    if len(lst) < 2: return None
    w = lst[-1][0] - lst[0][0]
    st = [m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for _, m in lst]
    return {'n': len(lst), 'wall_hz': round((len(lst) - 1) / w, 2) if w > 0 else None,
            'sim_hz': round((len(lst) - 1) / (st[-1] - st[0]), 2) if st[-1] > st[0] else None}
out['rates_%ds' % HZW] = {k: rate(v) for k, v in got.items()}
for k in ('rgb', 'depth', 'info'):
    if not got[k]: out[k] = 'NO MESSAGE'; continue
    m = got[k][-1][1]
    if k == 'info':
        out[k] = {'frame_id': m.header.frame_id, 'size': [m.width, m.height], 'K': [round(v, 2) for v in (m.k[0], m.k[4], m.k[2], m.k[5])]}
        continue
    r = {'encoding': m.encoding, 'size': [m.width, m.height], 'frame_id': m.header.frame_id}
    if k == 'rgb' and m.encoding in ('rgb8', 'bgr8'):
        a = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.step)[:, :m.width * 3].reshape(m.height, m.width, 3)
        r.update(mean=round(float(a.mean()), 1), std=round(float(a.std()), 1), unique_colors=int(np.unique(a.reshape(-1, 3), axis=0).shape[0]))
    if k == 'depth' and m.encoding == '32FC1':
        d = np.frombuffer(bytes(m.data), np.float32).reshape(m.height, m.width)
        f = d[np.isfinite(d)]
        r.update(inf_ratio=round(float(np.isinf(d).mean()), 4), nan_ratio=round(float(np.isnan(d).mean()), 4),
                 valid_ratio=round(float(np.isfinite(d).mean()), 4),
                 valid_min=round(float(f.min()), 4) if f.size else None, valid_median=round(float(np.median(f)), 4) if f.size else None,
                 valid_max=round(float(f.max()), 4) if f.size else None)
        if pts and got['info']:
            ci = got['info'][-1][1]; fx, fy, cx, cy = ci.k[0], ci.k[4], ci.k[2], ci.k[5]
            fr = m.header.frame_id or 'camera_optical_frame'
            tw = time.monotonic()
            while not buf.can_transform(fr, 'base_footprint', Time()) and time.monotonic() - tw < 10: rclpy.spin_once(node, timeout_sec=0.1)
            tf = buf.lookup_transform(fr, 'base_footprint', Time()); tq, q = tf.transform.translation, tf.transform.rotation
            x, y, z, w = q.x, q.y, q.z, q.w
            R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)], [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
            parts = {}
            for n_, p in pts.items():
                pc = R @ np.array(p) + np.array([tq.x, tq.y, tq.z]); e = {'expected_m': round(float(pc[2]), 4)}
                if pc[2] > 0:
                    u, v = fx * pc[0] / pc[2] + cx, fy * pc[1] / pc[2] + cy; e['pixel'] = [round(float(u), 1), round(float(v), 1)]
                    if 0 <= u < m.width and 0 <= v < m.height:
                        ui, vi = int(round(u)), int(round(v)); patch = d[max(vi-2, 0):vi+3, max(ui-2, 0):ui+3]; pf = patch[np.isfinite(patch)]
                        if pf.size:
                            e['measured_m'] = round(float(np.median(pf)), 4); e['error_mm'] = round((e['measured_m'] - e['expected_m']) * 1000, 1)
                        else:
                            e['measured_m'] = 'no valid pixel'
                    else:
                        e['in_image'] = False
                parts[n_] = e
            r['parts'] = parts
    out[k] = r
json.dump(out, open(OUT, 'w'), indent=1); print(json.dumps(out))
