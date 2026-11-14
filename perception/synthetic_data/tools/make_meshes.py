#!/usr/bin/env python3
"""hex_nut, bolt STL 메쉬 생성기 (단위: m).

CAD 없이 numpy만으로 육각 너트와 육각 볼트 메쉬를 만든다.
치수를 바꾸고 싶으면 아래 상수만 고친 뒤 다시 실행하면 된다.

    python3 tools/make_meshes.py

결과:
    models/hex_nut/meshes/hex_nut.stl
    models/bolt/meshes/bolt.stl
"""
import os
import struct

import numpy as np

# ---------------- 치수 (m) ----------------
NUT_ACROSS_FLATS = 0.030   # 너트 맞변 거리 (폭)
NUT_HEIGHT = 0.015         # 너트 두께
NUT_HOLE_D = 0.016         # 가운데 구멍 지름

BOLT_HEAD_ACROSS_FLATS = 0.025  # 볼트 머리 맞변 거리
BOLT_HEAD_HEIGHT = 0.008        # 볼트 머리 높이
BOLT_SHANK_D = 0.012            # 몸통 지름
BOLT_SHANK_LEN = 0.052          # 몸통 길이 (전체 길이 = 머리 + 몸통 = 6 cm)

SEGMENTS = 48  # 원 둘레 분할 수 (6의 배수여야 육각형 꼭짓점이 정확히 들어감)


def hex_radius(theta, across_flats):
    """각도 theta 방향에서 육각형 외곽까지의 거리 (꼭짓점이 0°, 60°, ...)."""
    apothem = across_flats / 2.0
    local = np.mod(theta, np.pi / 3) - np.pi / 6
    return apothem / np.cos(local)


def ring_prism(r_outer, r_inner, z0, z1):
    """외곽 반지름 배열과 내부 반지름 배열로 닫힌 (구멍 있는) 기둥을 만든다.

    r_inner가 None이면 구멍 없는 기둥 (윗면/아랫면은 중심점과 부채꼴로 연결).
    반환: (N, 3, 3) 삼각형 배열, 법선은 바깥 방향.
    """
    n = len(r_outer)
    th = np.linspace(0, 2 * np.pi, n, endpoint=False)
    c, s = np.cos(th), np.sin(th)
    ob = np.stack([r_outer * c, r_outer * s, np.full(n, z0)], 1)
    ot = np.stack([r_outer * c, r_outer * s, np.full(n, z1)], 1)
    tris = []
    for i in range(n):
        j = (i + 1) % n
        # 바깥 옆면
        tris += [[ob[i], ob[j], ot[j]], [ob[i], ot[j], ot[i]]]
        if r_inner is None:
            cb = np.array([0, 0, z0])
            ct = np.array([0, 0, z1])
            tris += [[ct, ot[i], ot[j]]]        # 윗면
            tris += [[cb, ob[j], ob[i]]]        # 아랫면
        else:
            ib = np.array([r_inner[i] * c[i], r_inner[i] * s[i], z0])
            ibj = np.array([r_inner[j] * c[j], r_inner[j] * s[j], z0])
            it = np.array([r_inner[i] * c[i], r_inner[i] * s[i], z1])
            itj = np.array([r_inner[j] * c[j], r_inner[j] * s[j], z1])
            # 안쪽 옆면 (법선은 구멍 중심 방향)
            tris += [[ib, itj, ibj], [ib, it, itj]]
            # 윗면, 아랫면 (링)
            tris += [[it, ot[i], ot[j]], [it, ot[j], itj]]
            tris += [[ib, ob[j], ob[i]], [ib, ibj, ob[j]]]
    return np.array(tris, dtype=np.float64)


def write_stl(path, tris, name):
    tris = np.asarray(tris, dtype=np.float32)
    with open(path, "wb") as f:
        f.write(name.encode().ljust(80, b" ")[:80])
        f.write(struct.pack("<I", len(tris)))
        for t in tris:
            nrm = np.cross(t[1] - t[0], t[2] - t[0])
            ln = np.linalg.norm(nrm)
            nrm = nrm / ln if ln > 0 else nrm
            f.write(struct.pack("<3f", *nrm))
            for v in t:
                f.write(struct.pack("<3f", *v))
            f.write(b"\x00\x00")


def main():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    th = np.linspace(0, 2 * np.pi, SEGMENTS, endpoint=False)

    # 육각 너트: 바닥이 z=0
    nut = ring_prism(hex_radius(th, NUT_ACROSS_FLATS),
                     np.full(SEGMENTS, NUT_HOLE_D / 2), 0.0, NUT_HEIGHT)
    write_stl(os.path.join(root, "models/hex_nut/meshes/hex_nut.stl"), nut, "hex_nut")

    # 육각 볼트: 머리가 아래(z=0~머리 높이), 몸통이 위로 서 있는 자세
    head = ring_prism(hex_radius(th, BOLT_HEAD_ACROSS_FLATS), None,
                      0.0, BOLT_HEAD_HEIGHT)
    shank = ring_prism(np.full(SEGMENTS, BOLT_SHANK_D / 2), None,
                       BOLT_HEAD_HEIGHT, BOLT_HEAD_HEIGHT + BOLT_SHANK_LEN)
    write_stl(os.path.join(root, "models/bolt/meshes/bolt.stl"),
              np.concatenate([head, shank]), "bolt")

    print("hex_nut.stl: %d triangles" % len(nut))
    print("bolt.stl:    %d triangles" % (len(head) + len(shank)))


if __name__ == "__main__":
    main()
