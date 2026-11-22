import sys, time, rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
rclpy.init(); n = Node('joints_once'); got = []
n.create_subscription(JointState, '/joint_states', lambda m: got.append(m), 10)
t0 = time.monotonic()
while not got and time.monotonic() - t0 < float(sys.argv[1]): rclpy.spin_once(n, timeout_sec=0.2)
m = got[-1] if got else None
print({k: round(v, 4) for k, v in zip(m.name, m.position)} if m else 'NO joint_states')
