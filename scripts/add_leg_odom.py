#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Step 2: add /leg_odom (onboard body speed) to a conditioned bag.

Speed is computed from /tf odom->base_link in the raw bag and written as
/leg_odom.twist.linear.x at every LiDAR scan stamp. FAST-LIO uses it when FASTLIO_LEG_K > 0.

Usage: python3 add_leg_odom.py <raw_bag_dir> <conditioned_bag_dir> <out_bag_dir>
  e.g. python3 add_leg_odom.py bags/raw_walk bags/raw_walk_ref bags/raw_walk_ref_leg
"""
import os, sys, glob, sqlite3, struct
import numpy as np
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py

NS = 1_000_000_000


def db3(p):
    return p if p.endswith('.db3') else sorted(glob.glob(p.rstrip('/') + '/*.db3'))[0]


SRC = db3(sys.argv[1]); REFDIR = sys.argv[2]; OUT = sys.argv[3]
if os.path.exists(OUT):
    print("ERROR exists", OUT); raise SystemExit(1)

# 1) body speed from raw /tf odom->base_link (receive clock)
con = sqlite3.connect(f"file:{SRC}?mode=ro&immutable=1", uri=True)
tp = {r[1]: r[0] for r in con.execute("SELECT id,name,type FROM topics")}
types = {r[1]: r[2] for r in con.execute("SELECT id,name,type FROM topics")}
TF = get_message(types['/tf'])
ot = []; op = []
for recv, data in con.execute(f"SELECT timestamp,data FROM messages WHERE topic_id={tp['/tf']} ORDER BY timestamp"):
    m = deserialize_message(bytes(data), TF)
    for t in m.transforms:
        if t.header.frame_id == 'odom' and t.child_frame_id == 'base_link':
            ot.append(recv / NS); op.append([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
ot = np.array(ot); op = np.array(op)
spd = np.r_[0, np.linalg.norm(np.diff(op, axis=0), axis=1) / np.clip(np.diff(ot), 1e-3, None)]
spd_t = ot

# 2) raw LiDAR (header_ns, recv_s): maps the LiDAR clock to the receive clock
lid_pairs = []
for recv, blob in con.execute(f"SELECT timestamp,data FROM messages WHERE topic_id={tp['/lidar_points']} ORDER BY timestamp"):
    b0 = bytes(blob); e = '<' if b0[1] == 1 else '>'
    sec = struct.unpack_from(e + 'i', b0, 4)[0]; nsec = struct.unpack_from(e + 'I', b0, 8)[0]
    lid_pairs.append((sec * NS + nsec, recv / NS))
con.close()
lid_pairs = np.array(lid_pairs, dtype=np.float64)

# 3) copy the conditioned bag and append /leg_odom
rdb = db3(REFDIR)
con = sqlite3.connect(f"file:{rdb}?mode=ro&immutable=1", uri=True)
rtopics = {n: (i, t) for i, n, t in con.execute("SELECT id,name,type FROM topics")}
OD = get_message("nav_msgs/msg/Odometry")
wr = rosbag2_py.SequentialWriter()
wr.open(rosbag2_py.StorageOptions(uri=OUT, storage_id="sqlite3"), rosbag2_py.ConverterOptions("", ""))
for n, (i, t) in rtopics.items():
    wr.create_topic(rosbag2_py.TopicMetadata(name=n, type=t, serialization_format="cdr"))
wr.create_topic(rosbag2_py.TopicMetadata(name="/leg_odom", type="nav_msgs/msg/Odometry", serialization_format="cdr"))
ncopy = 0
for tid, name in [(v[0], k) for k, v in rtopics.items()]:
    for ts, data in con.execute(f"SELECT timestamp,data FROM messages WHERE topic_id={tid} ORDER BY timestamp"):
        wr.write(name, bytes(data), ts); ncopy += 1
con.close()
nleg = 0
for header_ns, recv_s in lid_pairs:
    s = float(np.interp(recv_s, spd_t, spd))
    o = OD()
    o.header.stamp.sec = int(header_ns // NS); o.header.stamp.nanosec = int(header_ns % NS)
    o.header.frame_id = "odom"; o.twist.twist.linear.x = s
    wr.write("/leg_odom", serialize_message(o), int(header_ns)); nleg += 1
del wr
print(f"OK: {OUT}  copied {ncopy} + {nleg} /leg_odom (speed {spd.min():.2f}~{spd.max():.2f} m/s)")
