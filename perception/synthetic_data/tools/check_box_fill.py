#!/usr/bin/env python3
"""박스 '채움 비율' 검사: 박스 안에서 배경과 다른 픽셀이 몇 %인가.

배경 밝기 = 박스 바로 바깥 테두리(두께 4 px)의 중앙값.
채움 비율 = 박스 안 픽셀 중 |밝기 - 배경| > diff 인 비율.
진짜 부품은 박스의 상당 부분을 채우고, 무늬 없는 면 위의 오검출은 거의 비어 있다.

1) 데이터셋 라벨 박스의 채움 비율 분포 (기준값을 정하는 근거)
    python3 check_box_fill.py --dataset ~/yolo_dataset_team_rgbd

2) 이미지 1장에 모델을 돌려서 검출 박스의 채움 비율 보기
    python3 check_box_fill.py --image raw_init.png --model ~/models/synth_v7_640_openvino_model
"""
import argparse
import glob
import os
import sys

import cv2
import numpy as np

CLASSES = ["block", "pin", "hex_nut", "bolt"]


def box_fill_ratio(gray, x0, y0, x1, y1, ring=4, diff=12.0):
    """gray: 2D 배열. 반환: 0~1 채움 비율, 바깥 테두리를 못 구하면 None."""
    h, w = gray.shape
    x0, y0 = max(0, int(x0)), max(0, int(y0))
    x1, y1 = min(w, int(round(x1))), min(h, int(round(y1)))
    if x1 - x0 < 2 or y1 - y0 < 2:
        return None
    X0, Y0, X1, Y1 = max(0, x0 - ring), max(0, y0 - ring), min(w, x1 + ring), min(h, y1 + ring)
    outer = gray[Y0:Y1, X0:X1].astype(np.float32)
    mask = np.ones(outer.shape, bool)
    mask[y0 - Y0:y1 - Y0, x0 - X0:x1 - X0] = False
    if mask.sum() < 8:
        return None
    bg = float(np.median(outer[mask]))
    inner = gray[y0:y1, x0:x1].astype(np.float32)
    return float((np.abs(inner - bg) > diff).mean())


def run_dataset(root, show):
    root = os.path.expanduser(root)
    imgs = sorted(glob.glob(os.path.join(root, "images", "*", "*.jpg")))
    if not imgs:
        sys.exit("이미지를 못 찾았습니다: %s" % root)
    vals = {i: [] for i in range(len(CLASSES))}
    worst = []
    for ip in imgs:
        split = os.path.basename(os.path.dirname(ip))
        lp = os.path.join(root, "labels", split, os.path.splitext(os.path.basename(ip))[0] + ".txt")
        if not os.path.exists(lp):
            continue
        lines = sorted(set(ln.strip() for ln in open(lp) if ln.strip()))
        if not lines:
            continue
        img = cv2.imread(ip)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape
        for ln in lines:
            c, cx, cy, bw, bh = ln.split()
            c, cx, cy, bw, bh = int(c), float(cx) * w, float(cy) * h, float(bw) * w, float(bh) * h
            r = box_fill_ratio(gray, cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2)
            if r is None:
                continue
            vals[c].append(r)
            worst.append((r, ip, c, cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2))
    print("데이터셋: %s (이미지 %d장)" % (root, len(imgs)))
    print("클래스별 채움 비율 (최소 / 하위 0.1% / 하위 1% / 하위 5% / 중앙값):")
    for i, n in enumerate(CLASSES):
        v = np.array(vals[i])
        if len(v) == 0:
            continue
        print("  %-8s n=%5d  %.3f / %.3f / %.3f / %.3f / %.3f" % (
            n, len(v), v.min(), np.percentile(v, 0.1), np.percentile(v, 1), np.percentile(v, 5), np.median(v)))
    for th in (0.05, 0.10, 0.15, 0.20):
        n_cut = sum(int((np.array(vals[i]) < th).sum()) for i in vals)
        n_all = sum(len(vals[i]) for i in vals)
        print("  기준 %.2f 미만 박스: %d / %d (%.2f%%)" % (th, n_cut, n_all, 100.0 * n_cut / max(1, n_all)))
    worst.sort(key=lambda t: t[0])
    tiles = []
    for r, ip, c, x0, y0, x1, y1 in worst[:show]:
        img = cv2.imread(ip)
        cv2.rectangle(img, (int(x0), int(y0)), (int(x1), int(y1)), (0, 0, 255), 2)
        cv2.putText(img, "%s fill %.3f" % (CLASSES[c], r), (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        tiles.append(cv2.resize(img, (320, 240)))
    if tiles:
        while len(tiles) % 4:
            tiles.append(np.zeros_like(tiles[0]))
        out = os.path.join(root, "low_fill_boxes.jpg")
        cv2.imwrite(out, np.vstack([np.hstack(tiles[i:i + 4]) for i in range(0, len(tiles), 4)]))
        print("채움 비율이 가장 낮은 박스 %d개: %s" % (min(show, len(worst)), out))


def run_image(path, model_path):
    from ultralytics import YOLO
    img = cv2.imread(os.path.expanduser(path))
    if img is None:
        sys.exit("이미지를 못 읽었습니다: %s" % path)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    model = YOLO(os.path.expanduser(model_path))
    res = model.predict(img, imgsz=640, conf=0.5, verbose=False)[0]
    if res.boxes is None or len(res.boxes) == 0:
        print("검출 없음")
        return
    for (x0, y0, x1, y1), cf, c in zip(res.boxes.xyxy.cpu().numpy(), res.boxes.conf.cpu().numpy(),
                                       res.boxes.cls.cpu().numpy()):
        r = box_fill_ratio(gray, x0, y0, x1, y1)
        print("%-8s %.2f  박스 (%d,%d)-(%d,%d)  채움 비율 %s" % (
            model.names[int(c)], cf, x0, y0, x1, y1, "없음" if r is None else "%.3f" % r))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dataset")
    ap.add_argument("--image")
    ap.add_argument("--model")
    ap.add_argument("--show", type=int, default=16)
    a = ap.parse_args()
    if a.dataset:
        run_dataset(a.dataset, a.show)
    if a.image:
        if not a.model:
            sys.exit("--image 와 함께 --model 을 주세요")
        run_image(a.image, a.model)
    if not a.dataset and not a.image:
        ap.print_help()


if __name__ == "__main__":
    main()
