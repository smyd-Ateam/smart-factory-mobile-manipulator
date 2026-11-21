import json
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

if len(sys.argv) != 4:
    raise SystemExit("usage: ros_stamp_collect.py OUT_JSONL READY_FILE DURATION_SECONDS")

out_path, ready_path, duration = sys.argv[1], sys.argv[2], float(sys.argv[3])
out = open(out_path, "w", buffering=1)
counts = {"ros_rgb": 0, "ros_depth": 0}
ready_written = False

class Collector(Node):
    def __init__(self):
        super().__init__("stamp_origin_ros_collector")
        self.create_subscription(Image, "/camera/image_raw", lambda m: self.record(m, "ros_rgb"), qos_profile_sensor_data)
        self.create_subscription(Image, "/camera/depth/image_raw", lambda m: self.record(m, "ros_depth"), qos_profile_sensor_data)

    def record(self, msg, stream):
        global ready_written
        counts[stream] += 1
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        out.write(json.dumps({"stream": stream, "count": counts[stream], "stamp_sec": msg.header.stamp.sec,
            "stamp_nsec": msg.header.stamp.nanosec, "stamp_ns": stamp_ns, "seq": None,
            "seq_reason": "ROS2 Header has no seq", "frame_id": msg.header.frame_id,
            "recv_wall_ns": time.time_ns()}) + "\n")
        if not ready_written and counts["ros_rgb"] >= 1 and counts["ros_depth"] >= 1:
            with open(ready_path, "w") as ready:
                ready.write("READY after first RGB and depth JSONL rows\n")
            ready_written = True

rclpy.init()
node = Collector()
deadline = time.monotonic() + duration
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.05)
out.close()
node.destroy_node()
rclpy.shutdown()
