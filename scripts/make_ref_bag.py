#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Step 1: IMU conditioning for a raw Go2 + Hesai bag.

- Aligns IMU header stamps (robot clock) to the LiDAR clock with a constant shift,
  estimated from the bag receive times of both topics.
- Resamples the IMU to a uniform rate (default 200 Hz) to fill burst gaps.
- Removes the gyro bias measured over the first bias_s seconds (the robot must be standing still).
- No low-pass or clipping. LiDAR messages are copied unchanged.
Output bag: /lidar_points + /utlidar/imu, both on the LiDAR clock.

Usage: python3 make_ref_bag.py <raw_bag_dir|file.db3> <out_bag_dir> [dur_s=0 (whole bag)] [fs=200] [bias_s=1.5]
  e.g. python3 make_ref_bag.py bags/raw_walk bags/raw_walk_ref 0 200 1.5
"""
import os, sys, glob, sqlite3, struct
import numpy as np
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py

NS = 1_000_000_000
LID = "/lidar_points"; IMU_T = "/utlidar/imu"


def db3(p):
    return p if p.endswith('.db3') else sorted(glob.glob(p.rstrip('/') + '/*.db3'))[0]


def main():
    SRC = db3(sys.argv[1]); OUT = sys.argv[2]
    dur_s = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0   # 0 = whole bag
    fs = float(sys.argv[4]) if len(sys.argv) > 4 else 200.0
    bias_s = float(sys.argv[5]) if len(sys.argv) > 5 else 1.5
    if os.path.exists(OUT):
        print(f"ERROR: exists {OUT}"); sys.exit(1)

    con = sqlite3.connect(f"file:{SRC}?mode=ro&immutable=1", uri=True); cur = con.cursor()
    topics = {n: (i, t) for i, n, t in cur.execute("SELECT id,name,type FROM topics")}
    assert LID in topics and IMU_T in topics, f"missing topics {list(topics)}"
    IMU = get_message(topics[IMU_T][1])

    # --- clock shift: median(lidar_header - imu_header), bridged by receive time (constant offset between the two clocks) ---
    lidr = []  # (recv_s, header_s)
    for recv, blob in cur.execute(f"SELECT timestamp,data FROM messages WHERE topic_id={topics[LID][0]} ORDER BY timestamp"):
        b0 = bytes(blob); e = '<' if b0[1] == 1 else '>'
        sec = struct.unpack_from(e + 'i', b0, 4)[0]; nsec = struct.unpack_from(e + 'I', b0, 8)[0]
        lidr.append((recv / NS, sec + nsec / NS))
    lidr = np.array(lidr)
    imur = []  # (recv_s, header_s)
    for recv, data in cur.execute(f"SELECT timestamp,data FROM messages WHERE topic_id={topics[IMU_T][0]} ORDER BY timestamp"):
        m = deserialize_message(bytes(data), IMU)
        imur.append((recv / NS, m.header.stamp.sec + m.header.stamp.nanosec / NS))
    imur = np.array(imur)
    imu_h_at_lidrecv = np.interp(lidr[:, 0], imur[:, 0], imur[:, 1])
    shift = float(np.median(lidr[:, 1] - imu_h_at_lidrecv))   # imu_header + shift -> lidar clock
    print(f"shift(imu_header->lidar clock) = {shift:.6f}s (skew std {np.std(lidr[:,1]-imu_h_at_lidrecv)*1000:.1f}ms)")

    # --- load IMU (header + shift = LiDAR clock) and crop to the segment ---
    t0 = lidr[:, 1].min() * NS
    hi = (t0 + dur_s * NS) if dur_s > 0 else (lidr[:, 1].max() * NS + NS)
    t = []; A = [[], [], []]; G = [[], [], []]
    for recv, data in cur.execute(f"SELECT timestamp,data FROM messages WHERE topic_id={topics[IMU_T][0]} ORDER BY timestamp"):
        m = deserialize_message(bytes(data), IMU)
        hh = (m.header.stamp.sec + m.header.stamp.nanosec / NS + shift) * NS
        if hh < t0 or hh >= hi:
            continue
        t.append(hh); a = m.linear_acceleration; g = m.angular_velocity
        A[0].append(a.x); A[1].append(a.y); A[2].append(a.z)
        G[0].append(g.x); G[1].append(g.y); G[2].append(g.z)
    t = np.array(t, np.float64); A = np.array(A); G = np.array(G)

    tg = np.arange(t[0], t[-1], NS / fs)
    Ai = np.array([np.interp(tg, t, A[k]) for k in range(3)])
    Gi = np.array([np.interp(tg, t, G[k]) for k in range(3)])
    nb = int(bias_s * fs); gbias = Gi[:, :nb].mean(axis=1); Gi = Gi - gbias[:, None]
    print(f"IMU: {len(t)} raw -> {len(tg)} uniform @ {fs:.0f}Hz (NO LP/clip)")
    print(f"  gyro bias(first {bias_s}s)={gbias} rad/s -> subtracted")

    # --- writer ---
    wr = rosbag2_py.SequentialWriter()
    wr.open(rosbag2_py.StorageOptions(uri=OUT, storage_id="sqlite3"), rosbag2_py.ConverterOptions("", ""))
    for name in (LID, IMU_T):
        wr.create_topic(rosbag2_py.TopicMetadata(name=name, type=topics[name][1], serialization_format="cdr"))
    # LiDAR copied verbatim (header already on the LiDAR clock), within the segment
    nl = 0
    for _, blob in cur.execute(f"SELECT timestamp,data FROM messages WHERE topic_id={topics[LID][0]} ORDER BY timestamp"):
        b0 = bytes(blob); e = '<' if b0[1] == 1 else '>'
        sec = struct.unpack_from(e + 'i', b0, 4)[0]; nsec = struct.unpack_from(e + 'I', b0, 8)[0]
        ts = sec * NS + nsec
        if ts < t0 or ts >= hi:
            continue
        wr.write(LID, b0, ts); nl += 1
    proto = deserialize_message(bytes(cur.execute(f"SELECT data FROM messages WHERE topic_id={topics[IMU_T][0]} LIMIT 1").fetchone()[0]), IMU)
    for k in range(len(tg)):
        msg = IMU(); msg.header.frame_id = proto.header.frame_id
        tt = int(tg[k]); msg.header.stamp.sec = tt // NS; msg.header.stamp.nanosec = tt % NS
        msg.linear_acceleration.x = float(Ai[0, k]); msg.linear_acceleration.y = float(Ai[1, k]); msg.linear_acceleration.z = float(Ai[2, k])
        msg.angular_velocity.x = float(Gi[0, k]); msg.angular_velocity.y = float(Gi[1, k]); msg.angular_velocity.z = float(Gi[2, k])
        msg.orientation = proto.orientation
        msg.orientation_covariance = proto.orientation_covariance
        msg.angular_velocity_covariance = proto.angular_velocity_covariance
        msg.linear_acceleration_covariance = proto.linear_acceleration_covariance
        wr.write(IMU_T, serialize_message(msg), tt)
    del wr; con.close()
    print(f"OK: {OUT}  (lidar {nl} verbatim, imu {len(tg)} conditioned)")


if __name__ == "__main__":
    main()
