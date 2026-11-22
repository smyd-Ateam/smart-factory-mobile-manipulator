"""approach_yaw=0 반복 시험 집계 (시험 도구). 사용: yaw0rep_summary.py [결과 폴더, 기본 /tmp/sfmm-brass-v7]

회차별: Nav2 도착 yaw(목표 0 대비), Align 1차 결과, 카메라 회색 가드, -x(뒤쪽) costmap, 부품까지 기하 여유.
안전 구간: 카메라 회색이 나지 않는 yaw |yaw| <= 11° (Mac VM ogre2 진단 결과, 경계 10.6~12.6°)
"""
import glob, json, math, os, re, sys

D = sys.argv[1] if len(sys.argv) > 1 else os.environ.get('STAGE1_DIR', '/tmp/sfmm-brass-v7')
SAFE_DEG = 11.0
FP = (-0.20, 0.24, -0.24, 0.24)   # nav2 footprint (base 기준 x_min, x_max, y_min, y_max)
PART_R = {'block': 0.0177, 'pin': 0.010, 'hex_nut': 0.015, 'bolt': 0.006}   # 평면 반경 근사 (block은 2.5 cm 정사각 대각선 절반)


def clearance(pose, part_xy, r):
    """로봇 footprint 사각형에서 부품 원까지 거리 (m, 음수면 겹침)."""
    x, y, yaw = pose
    dx, dy = part_xy[0] - x, part_xy[1] - y
    bx, by = math.cos(yaw) * dx + math.sin(yaw) * dy, -math.sin(yaw) * dx + math.cos(yaw) * dy
    ex = max(FP[0] - bx, 0, bx - FP[1]); ey = max(FP[2] - by, 0, by - FP[3])
    return math.hypot(ex, ey) - r


rows = []
for f in sorted(glob.glob(f'{D}/s1_result_yaw0rep_*.json')):
    tag = os.path.basename(f)[len('s1_result_'):-5]
    m = re.match(r'yaw0rep_(.+)_(\d+)$', tag); target, k = m.group(1), int(m.group(2))
    d = json.load(open(f)); st = d['steps']
    nav = st.get('nav', {}); a1 = st.get('align1_near', {}); cm = st.get('costmap_after_nav', {})
    parts = st.get('parts_gz', {})
    arr = (nav.get('truth') or {}).get('gz_pose'); after = (a1.get('truth_after') or {}).get('gz_pose')
    r = {'tag': tag, 'target': target, 'k': k, 'nav_status': nav.get('status'),
         'arrive_yaw_deg': round(math.degrees(arr[2]), 2) if arr else None,
         'arrive_xy_err_cm': nav.get('goal_err_xy_cm'),
         'align1': 'SUCCESS' if a1.get('success') else (a1.get('error_code') or a1.get('error') or ('NOT RUN' if not a1 else 'FAIL')),
         'align1_s': a1.get('elapsed_s'), 'after_yaw_deg': round(math.degrees(after[2]), 2) if after else None,
         'range_err_mm_truth': round(1000 * a1['truth_after']['range_err_m'], 1) if a1.get('truth_after') else None,
         'pin_or_target_frames': f"{(a1.get('detections') or {}).get('target_frames')}/{(a1.get('detections') or {}).get('msgs')}",
         'cam_gray': d.get('camera_gray_detected'), 'cam_gray_windows': d.get('camera_gray_windows'),
         'cam_frames': {w: (st.get(s, {}).get(c) or {}).get('frames') for w, s, c in (('near3s', 'near_before_align', 'camera_3s'), ('align1', 'align1_near', 'camera'))},
         'gray_detail': [{'window': w, **{x: (st.get(s, {}).get(c) or {}).get(x) for x in ('gray_frames', 'frames', 'gray_yaw_range_deg')}}
                         for w, s, c in (('near3s', 'near_before_align', 'camera_3s'), ('align1', 'align1_near', 'camera'))
                         if (st.get(s, {}).get(c) or {}).get('gray_detected')],
         'costmap_inside_lethal': (cm.get('footprint_inside') or {}).get('lethal100'),
         'costmap_ring_lethal': (cm.get('ring_0.1m') or {}).get('lethal100'),
         'rear_cells': (cm.get('rear_-0.20_to_-0.60') or {}).get('cells'),
         'rear_lethal_inscribed': None if not cm.get('rear_-0.20_to_-0.60') else cm['rear_-0.20_to_-0.60']['lethal100'] + cm['rear_-0.20_to_-0.60']['inscribed99'],
         'min_part_clear_cm_arrive': round(100 * min(clearance(arr, parts[p], PART_R[p]) for p in parts), 1) if arr and parts else None,
         'min_part_clear_cm_after': round(100 * min(clearance(after, parts[p], PART_R[p]) for p in parts), 1) if after and parts else None,
         'stopped': (st.get('stopped') or {}).get('reason')}
    y = r['arrive_yaw_deg']
    if r['nav_status'] != 4: cause = 'Nav2 실패'
    elif y is not None and abs(y) > SAFE_DEG and r['cam_gray']: cause = '도착 yaw 안전 구간 밖 + 카메라 회색'
    elif r['cam_gray']: cause = '카메라 회색 (yaw는 안전 구간 안)'
    elif y is not None and abs(y) > SAFE_DEG: cause = '도착 yaw 안전 구간 밖 (회색은 미감지)'
    elif r['align1'] != 'SUCCESS': cause = f"기타 ({r['align1']})"
    else: cause = ''
    r['fail_cause'] = cause if r['align1'] != 'SUCCESS' else ('' if not cause else f'성공했지만 주의: {cause}')
    rows.append(r)

json.dump(rows, open(f'{D}/yaw0rep/summary_rows.json', 'w'), indent=1, ensure_ascii=False)
for t in sorted({r['target'] for r in rows}):
    rs = sorted((r for r in rows if r['target'] == t), key=lambda r: r['k'])
    print(f'===== {t}: {sum(r["align1"] == "SUCCESS" for r in rs)}/{len(rs)} Align 1차 성공')
    print(' k  nav  도착yaw°  xy오차cm  align1          s     정렬후yaw°  거리오차mm  검출     회색  costmap(내부/띠/뒤쪽 lethal+inscribed)  부품여유cm(도착/정렬후)  원인')
    for r in rs:
        print(f" {r['k']:2d}  {r['nav_status']}  {r['arrive_yaw_deg']:+7.2f}   {r['arrive_xy_err_cm']!s:6}  {r['align1']:15s} {r['align1_s']!s:5}  {r['after_yaw_deg']!s:8}    {r['range_err_mm_truth']!s:6}     {r['pin_or_target_frames']:7s}  {'Y' if r['cam_gray'] else 'n'}     "
              f"{r['costmap_inside_lethal']}/{r['costmap_ring_lethal']}/{r['rear_lethal_inscribed']} (뒤쪽 {r['rear_cells']}칸)          {r['min_part_clear_cm_arrive']}/{r['min_part_clear_cm_after']}   {r['fail_cause']}"
              + (f"  {r['gray_detail']}" if r['gray_detail'] else ''))
    ys = [r['arrive_yaw_deg'] for r in rs if r['arrive_yaw_deg'] is not None]
    if ys:
        print(f"  도착 yaw: 평균 {sum(ys)/len(ys):+.2f}°, 최소 {min(ys):+.2f}°, 최대 {max(ys):+.2f}°, |yaw| 최대 {max(abs(v) for v in ys):.2f}° (안전 구간 ±{SAFE_DEG}°)")
