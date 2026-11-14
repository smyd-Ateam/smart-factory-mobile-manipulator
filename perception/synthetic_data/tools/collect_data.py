#!/usr/bin/env python3
"""YOLO 합성 데이터 자동 수집기 (Gazebo Fortress + ROS2 Humble).

매 장면마다
  1) 부품 종류·개수·위치·자세, 방해물, 카메라 위치·각도, 조명을 무작위로 바꾸고
  2) boundingbox 카메라의 이미지(/boxes_image)와 정답 박스(/boxes)를 받아서
  3) YOLO 형식(images/*.jpg + labels/*.txt)으로 저장한다.

보통은 tools/run_collect.sh 로 실행한다 (Gazebo, 브리지, 이 스크립트를 한 번에 실행).
직접 실행하려면 Gazebo 서버와 ros_gz_bridge가 먼저 떠 있어야 한다.

    python3 tools/collect_data.py --num 200 --out ~/yolo_dataset
"""
import argparse
import collections
import math
import os
import random
import re
import subprocess
import sys
import threading
import time

# ---------------------------------------------------------------------------
# 설정
# ---------------------------------------------------------------------------
CLASSES = ["block", "pin", "hex_nut", "bolt"]          # YOLO class id 0~3
INSTANCES = {c: ["%s_0" % c, "%s_1" % c] for c in CLASSES}
DISTRACTORS = ["distractor_0", "distractor_1"]
CAMERA_MODEL = "camera_rig"
WORLD = "collect"

IMG_W, IMG_H = 640, 480

# 바닥에 놓일 수 있는 자세: (roll, pitch, 바닥에서 띄울 z, 확률 가중치)
REST_POSES = {
    "block":   [(0.0, 0.0, 0.0, 0.6), (math.pi / 2, 0.0, 0.0125, 0.4)],
    "pin":     [(0.0, 0.0, 0.0, 0.4), (math.pi / 2, 0.0, 0.0100, 0.6)],
    "hex_nut": [(0.0, 0.0, 0.0, 0.85), (math.pi / 2, 0.0, 0.0175, 0.15)],
    "bolt":    [(0.0, 0.0, 0.0, 0.4), (0.0, math.pi / 2, 0.0145, 0.6)],
}
MATS = ["mat_%d" % i for i in range(5)]   # 색이 다른 바닥 매트 (collect.sdf)
MAT_THICK = 0.002
DISTRACTOR_Z = {"distractor_0": 0.015, "distractor_1": 0.0025}
DROP_MARGIN = 0.002      # 바닥에 박히지 않게 살짝 띄워서 놓는다
MIN_SEPARATION = 0.09    # 물체끼리 최소 거리 (m)

# 카메라 무작위 범위 (far 자세 ~ near 자세를 모두 덮도록)
PITCH_DEG = (20.0, 70.0)   # 아래로 숙인 각도
DIST_M = (0.25, 1.2)       # 카메라 → 바라보는 바닥 점까지 거리
ROLL_RAD = 0.08            # 약간의 기울어짐


# ---------------------------------------------------------------------------
# 수학
# ---------------------------------------------------------------------------
def rpy_to_quat(roll, pitch, yaw):
    """R = Rz(yaw) * Ry(pitch) * Rx(roll) 에 해당하는 쿼터니언 (x, y, z, w)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


# ---------------------------------------------------------------------------
# 장면 무작위 생성
# ---------------------------------------------------------------------------
def sample_scene(rng):
    """카메라 1개 + 모든 물체의 자세를 정한다. 쓰지 않는 물체는 멀리 대기시킨다."""
    pitch = math.radians(rng.uniform(*PITCH_DEG))
    dist = DIST_M[0] + (DIST_M[1] - DIST_M[0]) * rng.random() ** 1.5   # 가까운 쪽에 더 많이
    yaw = rng.uniform(-math.pi, math.pi)
    roll = rng.uniform(-ROLL_RAD, ROLL_RAD)
    tx, ty = rng.uniform(-1.0, 1.0), rng.uniform(-1.0, 1.0)   # 바라보는 바닥 점

    # 카메라는 +x 방향을 본다. 양수 pitch = 아래로 숙임
    fx = math.cos(pitch) * math.cos(yaw)
    fy = math.cos(pitch) * math.sin(yaw)
    fz = -math.sin(pitch)
    cam = (tx - dist * fx, ty - dist * fy, max(0.06, -dist * fz))
    poses = {CAMERA_MODEL: (cam[0], cam[1], cam[2], roll, pitch, yaw)}

    # 물체 수: 가까이 볼수록 적게
    max_obj = max(1, min(6, 1 + int(dist * 5)))
    n_obj = rng.randint(1, max_obj)
    spread = max(0.08, 0.4 * dist)

    # 바닥: 매트 5종 중 하나 또는 매트 없음 → 부품과 바닥의 색 대비가 매번 달라짐
    mat = rng.choice(MATS + [None])
    base_z = MAT_THICK if mat else 0.0
    for k, m in enumerate(MATS):
        poses[m] = ((tx, ty, MAT_THICK / 2, 0.0, 0.0, rng.uniform(-math.pi, math.pi))
                    if m == mat else (70.0 + 5 * k, 70.0, MAT_THICK / 2, 0.0, 0.0, 0.0))

    pool = [name for c in CLASSES for name in INSTANCES[c]]
    rng.shuffle(pool)
    chosen = pool[:n_obj]
    n_dis = rng.choice([0, 0, 1, 2])
    chosen_dis = rng.sample(DISTRACTORS, n_dis)

    placed = []
    for name in chosen + chosen_dis:
        for _ in range(40):
            x = tx + rng.uniform(-spread, spread)
            y = ty + rng.uniform(-spread, spread)
            if all((x - px) ** 2 + (y - py) ** 2 >= MIN_SEPARATION ** 2 for px, py in placed):
                break
        else:
            continue  # 자리가 없으면 이 물체는 빼고 진행
        placed.append((x, y))
        oyaw = rng.uniform(-math.pi, math.pi)
        if name in DISTRACTORS:
            poses[name] = (x, y, base_z + DISTRACTOR_Z[name] + DROP_MARGIN, 0.0, 0.0, oyaw)
        else:
            cls = name.rsplit("_", 1)[0]
            opts = REST_POSES[cls]
            r, p, z, _ = rng.choices(opts, weights=[o[3] for o in opts])[0]
            poses[name] = (x, y, base_z + z + DROP_MARGIN, r, p, oyaw)

    # 나머지는 대기 위치로
    k = 0
    for name in [n for c in CLASSES for n in INSTANCES[c]] + DISTRACTORS:
        if name not in poses:
            poses[name] = (50.0 + k, 50.0, 0.02, 0.0, 0.0, 0.0)
        k += 1

    # 조명
    az = rng.uniform(-math.pi, math.pi)
    el = math.radians(rng.uniform(30.0, 85.0))
    g = rng.uniform(0.55, 1.0)
    light = {
        "dir": (-math.cos(el) * math.cos(az), -math.cos(el) * math.sin(az), -math.sin(el)),
        "rgb": (g * rng.uniform(0.92, 1.0), g * rng.uniform(0.92, 1.0), g * rng.uniform(0.9, 1.0)),
        "shadows": rng.random() < 0.8,
    }
    info = {"pitch_deg": math.degrees(pitch), "dist_m": dist,
            "n_obj": len([n for n in chosen if n in poses and poses[n][0] < 40]),
            "n_dis": n_dis, "mat": mat or "none"}
    return poses, light, info


# ---------------------------------------------------------------------------
# Gazebo 명령 (ign CLI 사용)
# ---------------------------------------------------------------------------
def ign_service(service, reqtype, req, timeout_ms=3000):
    cmd = ["ign", "service", "-s", service, "--reqtype", reqtype,
           "--reptype", "ignition.msgs.Boolean", "--timeout", str(timeout_ms), "--req", req]
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout_ms / 1000 + 5)
    except subprocess.TimeoutExpired:
        return False
    return "data: true" in r.stdout


def set_poses(poses):
    parts = []
    for name, (x, y, z, r, p, yw) in poses.items():
        qx, qy, qz, qw = rpy_to_quat(r, p, yw)
        parts.append('pose { name: "%s" position { x: %.5f y: %.5f z: %.5f } '
                     'orientation { x: %.6f y: %.6f z: %.6f w: %.6f } }'
                     % (name, x, y, z, qx, qy, qz, qw))
    return ign_service("/world/%s/set_pose_vector" % WORLD, "ignition.msgs.Pose_V", " ".join(parts))


def set_light(light):
    (dx, dy, dz), (r, g, b) = light["dir"], light["rgb"]
    # intensity를 꼭 넣어야 한다 (빠지면 0으로 들어가 햇빛이 꺼짐)
    req = ('name: "sun" type: DIRECTIONAL cast_shadows: %s intensity: 1 range: 1000 '
           'attenuation_constant: 1 '
           'diffuse { r: %.3f g: %.3f b: %.3f a: 1 } specular { r: 0.3 g: 0.3 b: 0.3 a: 1 } '
           'direction { x: %.4f y: %.4f z: %.4f }'
           % ("true" if light["shadows"] else "false", r, g, b, dx, dy, dz))
    return ign_service("/world/%s/light_config" % WORLD, "ignition.msgs.Light", req)


# ---------------------------------------------------------------------------
# /boxes 메시지 (ign topic 출력 텍스트) 파싱
# ---------------------------------------------------------------------------
def _num(text, key):
    m = re.search(r"\b" + key + r":\s*(-?[0-9.eE+-]+)", text)
    return float(m.group(1)) if m else 0.0   # protobuf 텍스트는 0인 값을 생략한다


def _sub(text, key):
    m = re.search(r"\b" + key + r"\s*\{([^{}]*)\}", text)
    return m.group(1) if m else ""


def parse_boxes(text):
    """반환: (stamp_sec, [(label, x0, y0, x1, y1), ...]) 또는 None."""
    if not text or "header" not in text:
        return None
    chunks = text.split("annotated_box")
    st = _sub(chunks[0], "stamp")
    stamp = _num(st, "sec") + _num(st, "nsec") * 1e-9
    boxes = []
    for ch in chunks[1:]:
        mn, mx = _sub(ch, "min_corner"), _sub(ch, "max_corner")
        boxes.append((int(_num(ch, "label")),
                      _num(mn, "x"), _num(mn, "y"), _num(mx, "x"), _num(mx, "y")))
    return stamp, boxes


def read_boxes(timeout=6.0):
    try:
        r = subprocess.run(["ign", "topic", "-e", "-t", "/boxes", "-n", "1"],
                           capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return None
    return parse_boxes(r.stdout)


def boxes_to_yolo(boxes, w=IMG_W, h=IMG_H, min_px=3):
    lines = []
    for label, x0, y0, x1, y1 in boxes:
        if not 1 <= label <= len(CLASSES):
            continue                      # 0 = 배경, 그 외는 무시
        x0, x1 = max(0.0, min(x0, w)), max(0.0, min(x1, w))
        y0, y1 = max(0.0, min(y0, h)), max(0.0, min(y1, h))
        bw, bh = x1 - x0, y1 - y0
        if bw < min_px or bh < min_px:
            continue
        lines.append("%d %.6f %.6f %.6f %.6f" % (label - 1, (x0 + x1) / 2 / w,
                                                 (y0 + y1) / 2 / h, bw / w, bh / h))
    return lines


# ---------------------------------------------------------------------------
# ROS2 이미지 수신
# ---------------------------------------------------------------------------
def make_image_node():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile
    from sensor_msgs.msg import Image

    class ImageGrabber(Node):
        def __init__(self):
            super().__init__("synth_data_collector")
            self.lock = threading.Lock()
            self.frames = collections.OrderedDict()   # stamp -> msg
            self.create_subscription(Image, "/boxes_image", self.cb, QoSProfile(depth=20))

        def cb(self, msg):
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            with self.lock:
                self.frames[round(t, 6)] = msg
                while len(self.frames) > 60:
                    self.frames.popitem(last=False)

        def latest_stamp(self):
            with self.lock:
                return next(reversed(self.frames)) if self.frames else None

        def find(self, stamp, tol=0.02, wait=2.0):
            end = time.time() + wait
            while time.time() < end:
                with self.lock:
                    if self.frames:
                        k = min(self.frames, key=lambda s: abs(s - stamp))
                        if abs(k - stamp) <= tol:
                            return self.frames[k]
                time.sleep(0.02)
            return None

    rclpy.init()
    node = ImageGrabber()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    return node


def msg_to_bgr(msg):
    import cv2
    import numpy as np
    ch = msg.step // msg.width
    arr = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.step)
    arr = arr[:, :msg.width * ch].reshape(msg.height, msg.width, ch)
    enc = msg.encoding.lower()
    if enc == "rgb8":
        return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
    if enc == "rgba8":
        return cv2.cvtColor(arr, cv2.COLOR_RGBA2BGR)
    if enc == "bgra8":
        return cv2.cvtColor(arr, cv2.COLOR_BGRA2BGR)
    return arr.copy()


# ---------------------------------------------------------------------------
# 메인
# ---------------------------------------------------------------------------
def write_data_yaml(out):
    names = "\n".join("  %d: %s" % (i, c) for i, c in enumerate(CLASSES))
    with open(os.path.join(out, "data.yaml"), "w") as f:
        f.write("path: %s\ntrain: images/train\nval: images/val\nnames:\n%s\n" % (out, names))


def main():
    ap = argparse.ArgumentParser(description="YOLO 합성 데이터 수집")
    ap.add_argument("--num", type=int, default=200, help="이번에 모을 이미지 수")
    ap.add_argument("--out", default=os.path.expanduser("~/yolo_dataset"))
    ap.add_argument("--val-ratio", type=float, default=0.1)
    ap.add_argument("--settle", type=float, default=0.4, help="물체를 옮긴 뒤 기다리는 시간 (s)")
    ap.add_argument("--seed", type=int, default=None)
    args = ap.parse_args()

    import cv2

    out = os.path.abspath(os.path.expanduser(args.out))
    for sub in ("images/train", "images/val", "labels/train", "labels/val"):
        os.makedirs(os.path.join(out, sub), exist_ok=True)
    write_data_yaml(out)

    # 이어서 모으기: 이미 있는 이미지 수부터 번호 시작
    start = sum(len(os.listdir(os.path.join(out, "images", s))) for s in ("train", "val"))
    rng = random.Random(args.seed if args.seed is not None else time.time_ns())

    node = make_image_node()
    print("[수집] /boxes_image 기다리는 중...")
    t0 = time.time()
    while node.latest_stamp() is None:
        if time.time() - t0 > 30:
            sys.exit("[오류] 30초 동안 /boxes_image가 안 들어옵니다. Gazebo와 브리지가 실행 중인지 확인하세요.")
        time.sleep(0.2)
    print("[수집] 이미지 수신 확인. %d장부터 %d장 수집 시작 → %s" % (start, args.num, out))

    meta_path = os.path.join(out, "meta.csv")
    new_meta = not os.path.exists(meta_path)
    meta = open(meta_path, "a")
    if new_meta:
        meta.write("index,split,pitch_deg,dist_m,mat,n_obj,n_dis,n_boxes\n")

    counts = collections.Counter()
    saved, fails, t_start = 0, 0, time.time()
    light_warned = False
    idx = start
    while saved < args.num:
        poses, light, info = sample_scene(rng)
        t_ref = node.latest_stamp() or 0.0
        if not set_poses(poses):
            fails += 1
            print("[경고] set_pose 실패 (%d회)" % fails)
            if fails > 20:
                sys.exit("[오류] set_pose가 계속 실패합니다. 월드 이름(collect)과 Gazebo 실행 상태를 확인하세요.")
            continue
        if not set_light(light) and not light_warned:   # 실패해도 계속 진행 (조명만 안 바뀜)
            print("[경고] 조명 변경이 안 됩니다. 기본 조명으로 계속 수집합니다.")
            light_warned = True
        time.sleep(args.settle)

        res = None
        for _ in range(6):        # 옮긴 뒤에 찍힌 프레임인지 시간으로 확인
            res = read_boxes()
            if res and res[0] >= t_ref + 0.25:
                break
            res = None
        if res is None:
            fails += 1
            print("[경고] /boxes를 못 받았습니다 (%d회)" % fails)
            continue
        stamp, boxes = res
        msg = node.find(stamp)
        if msg is None:
            fails += 1
            print("[경고] 박스와 같은 시각의 이미지를 못 찾았습니다 (%d회)" % fails)
            continue

        split = "val" if random.Random(idx).random() < args.val_ratio else "train"
        name = "%06d" % idx
        cv2.imwrite(os.path.join(out, "images", split, name + ".jpg"), msg_to_bgr(msg),
                    [cv2.IMWRITE_JPEG_QUALITY, 95])
        lines = boxes_to_yolo(boxes, msg.width, msg.height)
        with open(os.path.join(out, "labels", split, name + ".txt"), "w") as f:
            f.write("\n".join(lines) + ("\n" if lines else ""))
        for ln in lines:
            counts[CLASSES[int(ln.split()[0])]] += 1
        meta.write("%d,%s,%.1f,%.3f,%s,%d,%d,%d\n" % (idx, split, info["pitch_deg"], info["dist_m"],
                                                      info["mat"], info["n_obj"], info["n_dis"], len(lines)))
        meta.flush()
        idx += 1
        saved += 1
        if saved % 25 == 0 or saved == args.num:
            rate = saved / (time.time() - t_start)
            print("[수집] %d/%d장 (%.2f장/s, 남은 시간 약 %.0f분) 박스: %s"
                  % (saved, args.num, rate, (args.num - saved) / rate / 60,
                     ", ".join("%s %d" % (c, counts[c]) for c in CLASSES)))

    meta.close()
    print("[완료] %s 에 %d장 저장 (실패 %d회). 다음: python3 tools/draw_labels.py --data %s"
          % (out, saved, fails, out))
    os._exit(0)   # ROS 스레드까지 깔끔하게 종료


if __name__ == "__main__":
    main()
