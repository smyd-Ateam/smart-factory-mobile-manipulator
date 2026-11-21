#!/usr/bin/env python3
"""YOLO 검출 노드 (C · 인식).

구독  /camera/image_raw            sensor_msgs/Image
발행  /perception/detections       vision_msgs/Detection2DArray   (px, frame_id = 이미지와 동일)
      /perception/detections/image sensor_msgs/Image (박스를 그린 확인용 영상)

- header.stamp는 처리 시각이 아니라 **원본 이미지의 stamp를 그대로** 쓴다 (D가 지연 계산에 사용).
- 검출이 없어도 빈 배열을 발행한다 (받는 쪽이 "미검출"과 "노드 멈춤"을 구분할 수 있게).
- 처리가 카메라보다 느리면 밀린 프레임은 버리고 가장 최근 프레임만 처리한다.

패키지로 빌드하지 않고 바로 실행할 수도 있다:
    python3 yolo_detector.py --ros-args -p model:=$HOME/yolo_models/synth_v1_640.pt -p use_sim_time:=true

OpenVINO 모델에서 정밀도/추가 설정이 필요할 때 (예: ARM Mac VM):
    ... -p model:=$HOME/yolo_models/synth_v2_640_openvino_model -p ov_precision:=f32
    ... -p ov_config:="NUM_STREAMS=1,INFERENCE_NUM_THREADS=4"
"""
import collections
import os
import threading
import time

import numpy as np

CLASSES = ["block", "pin", "hex_nut", "bolt"]                 # 인터페이스 문서 5.4와 같은 순서
COLORS_BGR = [(235, 140, 40), (60, 200, 60), (40, 60, 230), (200, 60, 200)]


# ---------------------------------------------------------------------------
# ROS와 무관한 순수 함수 (테스트하기 쉽게 분리)
# ---------------------------------------------------------------------------
def image_msg_to_bgr(msg):
    """sensor_msgs/Image → BGR numpy 배열 (cv_bridge 없이)."""
    import cv2
    enc = msg.encoding.lower()
    ch = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}.get(enc)
    if ch is None:
        raise ValueError("지원하지 않는 인코딩: %s" % msg.encoding)
    arr = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.step)
    arr = arr[:, :msg.width * ch].reshape(msg.height, msg.width, ch) if ch > 1 \
        else arr[:, :msg.width]
    if enc == "rgb8":
        return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
    if enc == "rgba8":
        return cv2.cvtColor(arr, cv2.COLOR_RGBA2BGR)
    if enc == "bgra8":
        return cv2.cvtColor(arr, cv2.COLOR_BGRA2BGR)
    if enc == "mono8":
        return cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
    return arr.copy()


def result_to_dets(result, names, conf_min):
    """ultralytics 결과 1장 → [(class_name, score, x0, y0, x1, y1), ...] (점수 높은 순)."""
    boxes = result.boxes
    if boxes is None or len(boxes) == 0:
        return []
    xyxy = boxes.xyxy.cpu().numpy()
    conf = boxes.conf.cpu().numpy()
    cls = boxes.cls.cpu().numpy().astype(int)
    dets = []
    for (x0, y0, x1, y1), s, c in zip(xyxy, conf, cls):
        if s < conf_min:
            continue
        name = names[c] if isinstance(names, (list, tuple)) else names.get(c, str(c))
        dets.append((name, float(s), float(x0), float(y0), float(x1), float(y1)))
    dets.sort(key=lambda d: -d[1])
    return dets


def parse_ov_config(precision="", extra=""):
    """노드 파라미터 → OpenVINO compile_model config dict.

    precision: "" (기본값 그대로), "f32", "f16", "bf16" → INFERENCE_PRECISION_HINT
    extra    : "KEY=VAL,KEY2=VAL2" 형식의 추가 설정 (예: "NUM_STREAMS=1,INFERENCE_NUM_THREADS=4")
    """
    cfg = {}
    if precision:
        cfg["INFERENCE_PRECISION_HINT"] = precision.strip().lower()
    for item in (extra or "").split(","):
        if "=" in item:
            k, v = item.split("=", 1)
            if k.strip():
                cfg[k.strip()] = v.strip()
    return cfg


def apply_openvino_config(core_cls, cfg):
    """OpenVINO Core.compile_model에 cfg를 합쳐 넣는다.

    ultralytics가 모델을 불러올 때 compile_model을 직접 호출해서 밖에서 설정을 넘길 수 없으므로,
    호출 직전에 config를 덧붙이도록 감싼다. cfg가 비어 있으면 아무것도 바꾸지 않는다.
    """
    if not cfg or getattr(core_cls.compile_model, "_mm_patched", False):
        return False
    orig = core_cls.compile_model

    def compile_model(self, model, *args, **kwargs):
        args = list(args)
        if "config" in kwargs:
            kwargs["config"] = {**(kwargs["config"] or {}), **cfg}
        elif len(args) >= 2:                       # (device_name, config, ...)
            args[1] = {**(args[1] or {}), **cfg}
        else:
            kwargs["config"] = dict(cfg)
        return orig(self, model, *args, **kwargs)

    compile_model._mm_patched = True
    core_cls.compile_model = compile_model
    return True


def draw_dets(bgr, dets, info=""):
    import cv2
    img = bgr.copy()
    for name, s, x0, y0, x1, y1 in dets:
        col = COLORS_BGR[CLASSES.index(name) % len(COLORS_BGR)] if name in CLASSES else (255, 255, 255)
        cv2.rectangle(img, (int(x0), int(y0)), (int(x1), int(y1)), col, 2)
        cv2.putText(img, "%s %.2f" % (name, s), (int(x0), max(14, int(y0) - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1, cv2.LINE_AA)
    if info:
        cv2.putText(img, info, (8, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 255), 1, cv2.LINE_AA)
    return img


def fill_detection_array(msg_array, header, dets, Detection2D, ObjectHypothesisWithPose):
    """Detection2DArray 메시지를 채운다 (vision_msgs Humble 4.x 기준)."""
    msg_array.header = header
    msg_array.detections = []
    for name, s, x0, y0, x1, y1 in dets:
        d = Detection2D()
        d.header = header
        d.id = name
        d.bbox.center.position.x = (x0 + x1) / 2.0
        d.bbox.center.position.y = (y0 + y1) / 2.0
        d.bbox.center.theta = 0.0
        d.bbox.size_x = x1 - x0
        d.bbox.size_y = y1 - y0
        h = ObjectHypothesisWithPose()
        h.hypothesis.class_id = name
        h.hypothesis.score = s
        d.results.append(h)
        msg_array.detections.append(d)
    return msg_array


# ---------------------------------------------------------------------------
# ROS2 노드
# ---------------------------------------------------------------------------
def main(args=None):
    import cv2
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
    from ultralytics import YOLO

    class YoloDetector(Node):
        def __init__(self):
            super().__init__("yolo_detector")
            p = self.declare_parameter
            self.model_path = os.path.expanduser(
                p("model", os.path.expanduser("~/yolo_models/synth_v1_640.pt")).value)
            self.imgsz = int(p("imgsz", 640).value)
            self.conf = float(p("conf", 0.5).value)
            self.device = str(p("device", "cpu").value)
            image_topic = p("image_topic", "/camera/image_raw").value
            self.pub_image = bool(p("publish_image", True).value)
            self.save_dir = os.path.expanduser(str(p("save_dir", "").value))
            self.save_every = max(1, int(p("save_every", 10).value))
            ov_precision = str(p("ov_precision", "").value)   # OpenVINO 모델일 때만: "", f32, f16, bf16
            ov_config = str(p("ov_config", "").value)         # 추가 설정 "KEY=VAL,KEY2=VAL2"

            if not os.path.exists(self.model_path):
                raise SystemExit("[오류] 모델 파일이 없습니다: %s" % self.model_path)
            if "openvino" in os.path.basename(self.model_path.rstrip("/")):
                cfg = parse_ov_config(ov_precision, ov_config)
                if cfg:
                    import openvino as ov
                    apply_openvino_config(ov.Core, cfg)
                    self.get_logger().info("OpenVINO 설정: %s" % cfg)
            elif ov_precision or ov_config:
                self.get_logger().warn("ov_precision/ov_config는 OpenVINO 모델(_openvino_model 폴더)일 때만 적용됩니다")
            self.model = YOLO(self.model_path)
            self.names = self.model.names
            unknown = [n for n in self.names.values() if n not in CLASSES]
            if unknown:
                self.get_logger().warn("모델 클래스 이름이 인터페이스 문서와 다릅니다: %s" % unknown)
            # 첫 추론은 느리므로 미리 한 번 돌려 둔다
            self.model.predict(np.zeros((480, 640, 3), np.uint8), imgsz=self.imgsz,
                               device=self.device, verbose=False)
            if self.save_dir:
                os.makedirs(self.save_dir, exist_ok=True)

            self.pub_det = self.create_publisher(Detection2DArray, "/perception/detections", 10)
            self.pub_img = self.create_publisher(Image, "/perception/detections/image", 2)
            self.create_subscription(Image, image_topic, self.on_image, qos_profile_sensor_data)

            # 최신 프레임 1장만 들고 있다가 별도 스레드에서 처리 (밀린 프레임은 버림)
            self.lock = threading.Lock()
            self.latest = None
            self.stats = collections.Counter()
            self.infer_ms = collections.deque(maxlen=50)
            self.class_count = collections.Counter()
            self.t_last_log = time.time()
            self.n_proc = 0
            self.n_at_log = 0
            self.stop = threading.Event()
            self.thread = threading.Thread(target=self.worker, daemon=True)
            self.thread.start()

            self.get_logger().info(
                "시작: model=%s imgsz=%d conf=%.2f device=%s | 구독 %s → 발행 /perception/detections"
                % (os.path.basename(self.model_path), self.imgsz, self.conf, self.device, image_topic))

        def on_image(self, msg):
            with self.lock:
                if self.latest is not None:
                    self.stats["dropped"] += 1
                self.latest = msg
            self.stats["received"] += 1

        def worker(self):
            while rclpy.ok() and not self.stop.is_set():
                with self.lock:
                    msg, self.latest = self.latest, None
                if msg is None:
                    time.sleep(0.005)
                    continue
                try:
                    self.process(msg)
                except Exception as e:   # 한 프레임 오류로 노드가 죽지 않게
                    if not self.stop.is_set():
                        self.get_logger().error("처리 실패: %s" % e)

        def process(self, msg):
            bgr = image_msg_to_bgr(msg)
            t0 = time.perf_counter()
            res = self.model.predict(bgr, imgsz=self.imgsz, conf=self.conf,
                                     device=self.device, verbose=False)[0]
            ms = (time.perf_counter() - t0) * 1000
            self.infer_ms.append(ms)
            dets = result_to_dets(res, self.names, self.conf)

            self.pub_det.publish(fill_detection_array(
                Detection2DArray(), msg.header, dets, Detection2D, ObjectHypothesisWithPose))

            self.n_proc += 1
            for d in dets:
                self.class_count[d[0]] += 1

            want_save = self.save_dir and self.n_proc % self.save_every == 0
            if self.pub_image or want_save:
                vis = draw_dets(bgr, dets, "%.0f ms | %d det" % (ms, len(dets)))
                if self.pub_image:
                    out = Image()
                    out.header = msg.header
                    out.height, out.width = vis.shape[:2]
                    out.encoding = "bgr8"
                    out.step = vis.shape[1] * 3
                    out.data = vis.tobytes()
                    self.pub_img.publish(out)
                if want_save:
                    cv2.imwrite(os.path.join(self.save_dir, "det_%05d.jpg" % self.n_proc), vis)

            now = time.time()
            if now - self.t_last_log >= 5.0:
                dt = now - self.t_last_log
                n_new = self.n_proc - self.n_at_log
                self.get_logger().info(
                    "처리 %.1f Hz | 추론 평균 %.0f ms | 수신 %d, 버린 프레임 %d | 누적 검출: %s"
                    % (n_new / dt, float(np.mean(self.infer_ms)), self.stats["received"],
                       self.stats["dropped"],
                       ", ".join("%s %d" % (c, self.class_count[c]) for c in CLASSES)))
                self.n_at_log = self.n_proc
                self.t_last_log = now

    rclpy.init(args=args)
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop.set()
        node.thread.join(timeout=2.0)   # 처리 중인 프레임을 마치고 끝내기
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
