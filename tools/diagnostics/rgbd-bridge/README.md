# Minimal RGB-D bridge comparison bundle

This directory is for diagnostics only; it is not production code. It contains only the test launch, collectors, world, and four synthetic object models. It does not contain a ROS workspace or VM build products.

## Requirements

- Ubuntu with ROS 2 Humble
- Gazebo Fortress / Ignition Gazebo 6
- `ros_gz_bridge` and `ros_gz_sim`
- A sourced and built ROS workspace containing `mm_description`, `dofbot_description`, their xacro/config dependencies, and the `ros_gz` packages. Build and source that workspace before running this bundle. The robot description is resolved from the sourced workspace; no workspace files are copied here.
- C++ compiler, `pkg-config`, Python 3, `rclpy`, `sensor_msgs`, and the Gazebo Transport development packages.

Check packages and libraries:

```bash
source /opt/ros/humble/setup.bash
ros2 pkg prefix mm_description
ros2 pkg prefix dofbot_description
ros2 pkg prefix ros_gz_bridge
ros2 pkg prefix ros_gz_sim
pkg-config --modversion ignition-transport11 ignition-msgs8 ignition-math6
```

If package prefix checks fail, install the required ROS packages and build the team's ROS workspace first. For a colcon workspace, from its root run `rosdep install --from-paths src --ignore-src -r -y`, `colcon build`, then `source install/setup.bash`. The workspace must provide `mm_description`, `dofbot_description`, `ros_gz_bridge`, `ros_gz_sim`, and the xacro dependencies. Gazebo development libraries must expose the three `pkg-config` modules above.

## Resource path and topics

`env_s1.sh` computes its directory from its own file location and prepends its local `models/` folder to `IGN_GAZEBO_RESOURCE_PATH`. It has no machine-specific workspace path.

Gazebo source topics:

- `/rgbd_camera/image` (`ignition.msgs.Image`)
- `/rgbd_camera/depth_image` (`ignition.msgs.Image`)

ROS topics:

- `/camera/image_raw` (`sensor_msgs/msg/Image`)
- `/camera/depth/image_raw` (`sensor_msgs/msg/Image`)

## Why the integrated bridge is omitted

The standard `mm_description/sim.launch.py` includes an unconditional combined `parameter_bridge`; it has no launch argument that disables it. Running it alongside test bridges creates duplicate ROS publishers and invalidates the comparison. `min_split_bridge.launch.py` retains Gazebo, robot spawn, and `robot_state_publisher`, omits that integrated bridge entirely, then starts one bridge process for each image stream.

The bridge mappings preserve the existing Gazebo-to-ROS type and direction:

```text
/rgbd_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image
  remap /rgbd_camera/image:=/camera/image_raw
/rgbd_camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image
  remap /rgbd_camera/depth_image:=/camera/depth/image_raw
```

## Run one 30-second capture

From the extracted bundle directory, first source the team's ROS workspace, then run the bundle-relative script. It finds all bundle files relative to its own location, so moving the extracted directory does not require editing paths:

```bash
source /path/to/team_ros_ws/install/setup.bash
./bundle-relative-run.sh
```

Optional output directory:

```bash
./bundle-relative-run.sh ./results/run1
```

The runner checks Gazebo publishers/types, ROS publisher count and bridge node names, starts metadata-only collectors, waits for both READY files, live PIDs, and at least one JSONL row per collector, then records a 30-second window. READY is written by each collector only after it has written its first RGB and depth JSONL rows. The runner verifies both collector PIDs with `kill -0` and verifies each JSONL has at least one row before starting the 30-second timer. It saves outputs under `results/` by default. The collectors record stamps and receive times only; image payloads are not written.

READY means both RGB and depth have each produced their first JSONL row. Before capture, the runner also checks collector PID liveness and JSONL row counts. Review `ros_rgb_topic.txt` and `ros_depth_topic.txt` to confirm exactly one publisher each, named `rgb_parameter_bridge` and `depth_parameter_bridge`.

## Interpret JSONL

Each line is one image message and includes stream, header stamp in sec/nsec/ns, receive wall time, frame ID, sequence when available, and collector count. GZ sequence is read from the message header if present; ROS `std_msgs/Header` has no sequence field. Compare GZ RGB against GZ depth for source pairing, ROS RGB against ROS depth for downstream pairing, then match each ROS stamp against its GZ stream to estimate delivery and preservation. A ROS stamp found in GZ but absent from ROS indicates a message not observed by the collector; it does not alone identify which queue dropped it.

## Optional load measurements

CPU, memory, and Gazebo RTF sampling are optional. Stamp pairing and GZ-to-ROS delivery comparisons are required for the test objective. This runner does not collect resource metrics by default. It runs only Gazebo, robot spawn/state publishing, and the two image bridges; it does not start Nav2, YOLO, or Align.

## Verify bundle integrity and results

The initial GitHub source archive can be checked with:

```bash
sha256sum rgbd-bridge-bundle.tar.gz
```

The expected SHA-256 for the originally prepared scratchpad tarball is `786861ba09656b80e3e2f3f34f938e044dd594e4273271cd09b4ece90545ac08`. Git clones contain the unpacked files rather than this tarball; for a checkout, compare the commit ID provided by the team and inspect `manifest.txt` to confirm the listed relative paths exist.

Each JSONL row is one image message. Exact stamp equality indicates a matched timestamp; a ROS stamp missing from its corresponding GZ stream indicates a delivery/observation gap. Pair counts and nearest stamp deltas distinguish same-stamp matches from frames that arrived at different timestamps. The stamp fields are in simulation/header time, while `recv_wall_ns` is local wall-clock receive time.

This bundle does not include `build/`, `install/`, `log/`, prior test JSONL/logs, or edited copies of repository files. The ROS workspace and its packages remain external dependencies.
