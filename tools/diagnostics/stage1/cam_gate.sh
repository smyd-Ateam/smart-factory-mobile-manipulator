#!/usr/bin/env bash
# 카메라 정상 확인: Far 자세 → 3 s 측정 → RGB 색 50가지 이상, 깊이 유효 90% 이상이면 exit 0
S=${STAGE1_DIR:-/tmp/sfmm-brass-v7}   # 시험 폴더
OUT=$1; cd $S
for a in 1 2 3; do timeout 60 bash src/mm_description/scripts/set_observe_pose_far.sh > /dev/null 2>&1; sleep 5
  J=$(timeout 20 python3 tools/joints.py 10 | tail -1)
  python3 -c "import ast,sys; j=ast.literal_eval(sys.argv[1]); sys.exit(0 if abs(j['joint3']+1.1464)<0.01 and abs(j['joint4']+0.8237)<0.01 else 1)" "$J" && break; done
timeout 40 python3 tools/cam_check.py $OUT 3 > /dev/null 2>&1
python3 - $OUT "$a" <<'PY'
import json, sys
try:
    c = json.load(open(sys.argv[1]))
except Exception as e:
    print(json.dumps({'ok': False, 'reason': f'cam_check failed: {e}'})); sys.exit(1)
r = c.get('rgb') if isinstance(c.get('rgb'), dict) else {}; d = c.get('depth') if isinstance(c.get('depth'), dict) else {}
ok = (r.get('unique_colors') or 0) >= 50 and (d.get('valid_ratio') or 0) >= 0.9
print(json.dumps({'ok': ok, 'far_pose_attempts': int(sys.argv[2]), 'rgb_unique_colors': r.get('unique_colors'), 'rgb_std': r.get('std'),
                  'depth_valid_ratio': d.get('valid_ratio'), 'rates_3s': c.get('rates_3s')}))
sys.exit(0 if ok else 1)
PY
