"""local/global costmap 한 장: footprint 안 · 바깥 0.1 m 띠 · 뒤쪽(x -0.20 ~ -0.60, |y|<0.24) 칸 통계.
OccupancyGrid 값: 100 = lethal, 99 = inscribed, 1~98 = 팽창, 0 = free, -1 = unknown."""
import math, sys, time, json
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PolygonStamped
from tf2_ros import Buffer, TransformListener
topic, limit = sys.argv[1], float(sys.argv[2])
rclpy.init(); n = Node('costmap_self'); buf = Buffer(); TransformListener(buf, n)
grids, fps = [], []
qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
n.create_subscription(OccupancyGrid, topic, lambda m: grids.append(m), qos)
n.create_subscription(PolygonStamped, '/local_costmap/published_footprint', lambda m: fps.append(m), 10)
t0 = time.monotonic()
while (not grids or not fps) and time.monotonic() - t0 < limit: rclpy.spin_once(n, timeout_sec=0.2)
if not grids: print('NO COSTMAP'); sys.exit(1)
g = grids[-1]; fr = g.header.frame_id
while not buf.can_transform(fr, 'base_footprint', Time()) and time.monotonic() - t0 < limit: rclpy.spin_once(n, timeout_sec=0.2)
t = buf.lookup_transform(fr, 'base_footprint', Time()); tr, q = t.transform.translation, t.transform.rotation
yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
W, H, res = g.info.width, g.info.height, g.info.resolution
ox, oy = g.info.origin.position.x, g.info.origin.position.y
d = np.array(g.data, dtype=np.int16).reshape(H, W)
ys, xs = np.mgrid[0:H, 0:W]
wx, wy = ox + (xs + 0.5) * res, oy + (ys + 0.5) * res
dx, dy = wx - tr.x, wy - tr.y
bx, by = math.cos(yaw) * dx + math.sin(yaw) * dy, -math.sin(yaw) * dx + math.cos(yaw) * dy   # base 기준
def stat(mask):
    v = d[mask]
    return {'cells': int(v.size), 'lethal100': int((v == 100).sum()), 'inscribed99': int((v == 99).sum()),
            'inflated1_98': int(((v >= 1) & (v <= 98)).sum()), 'free0': int((v == 0).sum()), 'unknown': int((v == -1).sum())}
inside = (bx >= -0.20) & (bx <= 0.24) & (np.abs(by) <= 0.24)
ring = (bx >= -0.30) & (bx <= 0.34) & (np.abs(by) <= 0.34) & ~inside
rear = (bx < -0.20) & (bx >= -0.60) & (np.abs(by) <= 0.24)
lethal_xy = [(round(float(a), 2), round(float(b), 2)) for a, b in zip(bx[(d == 100) & (np.hypot(bx, by) < 1.0)], by[(d == 100) & (np.hypot(bx, by) < 1.0)])][:15]
out = {'topic': topic, 'frame': fr, 'res': res, 'robot': [round(tr.x, 3), round(tr.y, 3), round(yaw, 3)],
       'footprint_inside': stat(inside), 'ring_0.1m': stat(ring), 'rear_-0.20_to_-0.60': stat(rear),
       'lethal_within_1m_base_xy(first15)': lethal_xy}
if fps:
    out['published_footprint_pts'] = [(round(p.x, 3), round(p.y, 3)) for p in fps[-1].polygon.points]
print(json.dumps(out))
