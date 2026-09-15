#!/usr/bin/env python3
"""Export FAST-LIO /Odometry from an output bag to CSV and print a short summary.

Usage: python3 export_odometry_csv.py <output_bag_dir> <out.csv> [--keep-mirror]

Columns: t_sec (header stamp, LiDAR clock), x, y, z (m), yaw_rad

With config/fastlio_go2_hesai_xt16.yaml the FAST-LIO output is left-right mirrored
compared with the real building (see README). By default this script undoes it:
x := -x, yaw := -yaw. Use --keep-mirror to write the raw values.
"""
import csv, glob, math, os, sqlite3, sys
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    if len(args) != 2:
        print(__doc__); sys.exit(2)
    bag, out = args
    keep_mirror = '--keep-mirror' in sys.argv
    if os.path.exists(out):
        sys.exit(f'ERROR: {out} already exists')
    dbs = sorted(glob.glob(os.path.join(bag, '*.db3'))) if os.path.isdir(bag) else [bag]
    if not dbs:
        sys.exit(f'ERROR: no .db3 in {bag}')

    rows = []
    for db in dbs:
        con = sqlite3.connect(f'file:{db}?mode=ro&immutable=1', uri=True)
        topics = {n: (i, t) for i, n, t in con.execute('SELECT id, name, type FROM topics')}
        if '/Odometry' not in topics:
            continue
        msg_type = get_message(topics['/Odometry'][1])
        for (data,) in con.execute(f"SELECT data FROM messages WHERE topic_id={topics['/Odometry'][0]} ORDER BY timestamp"):
            m = deserialize_message(bytes(data), msg_type)
            p, q = m.pose.pose.position, m.pose.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            s = 1.0 if keep_mirror else -1.0
            rows.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, s * p.x, p.y, p.z, s * yaw))
        con.close()
    if not rows:
        sys.exit('ERROR: no /Odometry messages found')
    rows.sort()

    with open(out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t_sec', 'x', 'y', 'z', 'yaw_rad'])
        w.writerows((f'{t:.6f}', f'{x:.4f}', f'{y:.4f}', f'{z:.4f}', f'{yaw:.6f}') for t, x, y, z, yaw in rows)

    a = np.array(rows)
    dt = np.diff(a[:, 0])
    length = np.hypot(np.diff(a[:, 1]), np.diff(a[:, 2])).sum()
    print(f'{out}: {len(a)} poses over {np.ptp(a[:, 0]):.1f} s, median step {1e3 * np.median(dt):.0f} ms '
          f'(100 ms expected; ~50 ms means a duplicate fastlio_mapping was running)')
    print(f'  path {length:.1f} m | end-start {np.hypot(*(a[-1, 1:3] - a[0, 1:3])):.2f} m | '
          f'z change {a[-1, 3] - a[0, 3]:+.2f} m | mirror {"kept" if keep_mirror else "undone (x:=-x, yaw:=-yaw)"}')


if __name__ == '__main__':
    main()
