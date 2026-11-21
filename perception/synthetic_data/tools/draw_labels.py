#!/usr/bin/env python3
"""수집한 데이터 검수: 이미지 위에 정답 박스를 그려서 한 장짜리 모음 이미지로 저장한다.

    python3 tools/draw_labels.py --data ~/yolo_dataset
    → ~/yolo_dataset/check_labels.jpg 를 열어서 박스가 부품에 딱 맞는지 눈으로 확인

ROS 없이 실행된다 (OpenCV만 사용).
"""
import argparse
import glob
import os
import random

import cv2
import numpy as np

CLASSES = ["block", "pin", "hex_nut", "bolt"]
COLORS = [(235, 140, 40), (60, 200, 60), (40, 60, 230), (200, 60, 200)]   # BGR


def draw(img_path, label_path):
    img = cv2.imread(img_path)
    h, w = img.shape[:2]
    n = 0
    if os.path.exists(label_path):
        for line in open(label_path):
            p = line.split()
            if len(p) != 5:
                continue
            c = int(p[0])
            cx, cy, bw, bh = (float(v) for v in p[1:])
            x0, y0 = int((cx - bw / 2) * w), int((cy - bh / 2) * h)
            x1, y1 = int((cx + bw / 2) * w), int((cy + bh / 2) * h)
            col = COLORS[c % len(COLORS)]
            cv2.rectangle(img, (x0, y0), (x1, y1), col, 2)
            cv2.putText(img, CLASSES[c] if c < len(CLASSES) else str(c), (x0, max(12, y0 - 4)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1, cv2.LINE_AA)
            n += 1
    cv2.putText(img, os.path.basename(img_path), (6, h - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    return img, n


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default=os.path.expanduser("~/yolo_dataset"))
    ap.add_argument("--n", type=int, default=16, help="모음에 넣을 이미지 수")
    ap.add_argument("--split", default="train", choices=["train", "val"])
    ap.add_argument("--cols", type=int, default=4)
    args = ap.parse_args()

    data = os.path.expanduser(args.data)
    imgs = sorted(glob.glob(os.path.join(data, "images", args.split, "*.jpg")))
    if not imgs:
        raise SystemExit("이미지가 없습니다: %s" % os.path.join(data, "images", args.split))

    # 전체 통계
    counts, empty = [0] * len(CLASSES), 0
    for lp in glob.glob(os.path.join(data, "labels", "*", "*.txt")):
        lines = [l for l in open(lp) if l.strip()]
        empty += not lines
        for l in lines:
            c = int(l.split()[0])
            if c < len(CLASSES):
                counts[c] += 1
    print("박스 수: " + ", ".join("%s %d" % (c, n) for c, n in zip(CLASSES, counts)))
    print("박스 없는 이미지: %d장" % empty)

    pick = random.sample(imgs, min(args.n, len(imgs)))
    tiles = []
    for ip in pick:
        lp = os.path.join(data, "labels", args.split, os.path.splitext(os.path.basename(ip))[0] + ".txt")
        img, _ = draw(ip, lp)
        tiles.append(cv2.resize(img, (320, 240)))
    while len(tiles) % args.cols:
        tiles.append(np.zeros_like(tiles[0]))
    rows = [np.hstack(tiles[i:i + args.cols]) for i in range(0, len(tiles), args.cols)]
    out = os.path.join(data, "check_labels.jpg")
    cv2.imwrite(out, np.vstack(rows))
    print("저장: %s" % out)


if __name__ == "__main__":
    main()
