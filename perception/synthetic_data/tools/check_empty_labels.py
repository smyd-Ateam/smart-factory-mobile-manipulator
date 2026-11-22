#!/usr/bin/env python3
"""라벨은 있는데 그 자리에 물체가 안 보이는 박스(빈 라벨)와 중복 라벨을 찾는다."""
import argparse, collections, glob, os, sys
import cv2
import numpy as np

CLASSES = ["block", "pin", "hex_nut", "bolt"]

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dataset")
    ap.add_argument("--std", type=float, default=4.0)
    ap.add_argument("--max-show", type=int, default=24)
    args = ap.parse_args()
    root = os.path.expanduser(args.dataset)
    imgs = sorted(glob.glob(os.path.join(root, "images", "*", "*.jpg")))
    if not imgs:
        sys.exit("이미지를 못 찾았습니다: %s/images/<train|val>/*.jpg" % root)
    total, empty = collections.Counter(), collections.Counter()
    dup_files = dup_lines = 0
    rows, tiles = [], []
    for ip in imgs:
        split = os.path.basename(os.path.dirname(ip))
        lp = os.path.join(root, "labels", split, os.path.splitext(os.path.basename(ip))[0] + ".txt")
        if not os.path.exists(lp):
            continue
        lines = [ln.strip() for ln in open(lp) if ln.strip()]
        if len(lines) != len(set(lines)):
            dup_files += 1
            dup_lines += len(lines) - len(set(lines))
        if not lines:
            continue
        img = cv2.imread(ip)
        if img is None:
            continue
        h, w = img.shape[:2]
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        for ln in set(lines):
            c, cx, cy, bw, bh = ln.split()
            c = int(c)
            x0 = int(max(0, (float(cx) - float(bw) / 2) * w)); x1 = int(min(w, (float(cx) + float(bw) / 2) * w))
            y0 = int(max(0, (float(cy) - float(bh) / 2) * h)); y1 = int(min(h, (float(cy) + float(bh) / 2) * h))
            if x1 - x0 < 2 or y1 - y0 < 2:
                continue
            total[c] += 1
            s = float(gray[y0:y1, x0:x1].std())
            if s < args.std:
                empty[c] += 1
                rows.append("%s\t%s\t%.1f" % (os.path.relpath(ip, root), CLASSES[c], s))
                if len(tiles) < args.max_show:
                    vis = img.copy()
                    cv2.rectangle(vis, (x0, y0), (x1, y1), (0, 0, 255), 2)
                    cv2.putText(vis, "%s std %.1f" % (CLASSES[c], s), (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    cv2.putText(vis, os.path.basename(ip), (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                    tiles.append(cv2.resize(vis, (320, 240)))
    print("데이터셋: %s (이미지 %d장)" % (root, len(imgs)))
    print("클래스별 빈 라벨 (박스 안 표준편차 < %.1f):" % args.std)
    for i, n in enumerate(CLASSES):
        t, e = total[i], empty[i]
        print("  %-8s %5d / %5d  (%.1f%%)" % (n, e, t, 100.0 * e / t if t else 0.0))
    print("중복 라벨: 파일 %d개, 줄 %d개" % (dup_files, dup_lines))
    with open(os.path.join(root, "empty_labels.txt"), "w") as f:
        f.write("image\tclass\tstd\n" + "\n".join(rows) + ("\n" if rows else ""))
    if tiles:
        while len(tiles) % 4:
            tiles.append(np.zeros_like(tiles[0]))
        grid = np.vstack([np.hstack(tiles[i:i + 4]) for i in range(0, len(tiles), 4)])
        cv2.imwrite(os.path.join(root, "empty_labels.jpg"), grid)
        print("예시 저장: %s" % os.path.join(root, "empty_labels.jpg"))
    else:
        print("빈 라벨 없음")

if __name__ == "__main__":
    main()
