"""world 점들을 Gazebo 로봇 실제 자세(x,y,z,roll,pitch,yaw)로 base_footprint 기준으로 바꿔 name=x,y,z 인자로 출력."""
import math, sys
import numpy as np
x, y, z, r, p, yw = [float(v) for v in sys.argv[1:7]]
def R(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr], [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])
Rm = R(r, p, yw); t = np.array([x, y, z])
out = []
for a in sys.argv[7:]:
    name, xyz = a.split('=')
    pb = Rm.T @ (np.array([float(v) for v in xyz.split(',')]) - t)
    out.append(f'{name}={pb[0]:.5f},{pb[1]:.5f},{pb[2]:.5f}')
print(' '.join(out))
