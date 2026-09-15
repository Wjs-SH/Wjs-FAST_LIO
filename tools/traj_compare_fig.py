#!/usr/bin/env python3
"""FAST-LIO vs onboard odometry figure for the repository README (English labels).
Usage: traj_compare_fig.py <raw_bag_dir> <fastlio_bag_dir> <out.png> <title> [--cloud <bag_with_/cloud_registered>] [--every 3]
- Onboard path: /tf odom->base_link from the raw bag (header time, robot clock).
- FAST-LIO path: /Odometry from the output bag, mirror undone (x := -x, yaw := -yaw).
- Clock link: offset = median(bag receive time - header time) of raw /lidar_points (FAST-LIO stamps use LiDAR time).
- Onboard is rigidly aligned (rotation + translation, no scale) onto FAST-LIO using time-matched samples.
"""
import sys, glob, math, sqlite3, numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
from matplotlib.colors import LogNorm

RAW, FL, OUT, TITLE = sys.argv[1:5]
CLOUD = sys.argv[sys.argv.index('--cloud') + 1] if '--cloud' in sys.argv else FL
EVERY = int(sys.argv[sys.argv.index('--every') + 1]) if '--every' in sys.argv else 3
db = lambda p: p if p.endswith('.db3') else sorted(glob.glob(p.rstrip('/') + '/*.db3'))[0]
stamp = lambda h: h.stamp.sec + h.stamp.nanosec * 1e-9

c = sqlite3.connect(f'file:{db(RAW)}?mode=ro&immutable=1', uri=True)
tp = {n: (i, t) for i, n, t in c.execute('select id,name,type from topics')}
TF = get_message(tp['/tf'][1]); ON = []
for (d,) in c.execute(f"select data from messages where topic_id={tp['/tf'][0]} order by timestamp"):
    for tr in deserialize_message(bytes(d), TF).transforms:
        if tr.header.frame_id.strip('/') == 'odom' and tr.child_frame_id.strip('/') == 'base_link':
            ON.append((stamp(tr.header), tr.transform.translation.x, tr.transform.translation.y))
ON = np.array(sorted(ON))
PCm = get_message(tp['/lidar_points'][1]); offs = []
ids = c.execute(f"select rowid,timestamp from messages where topic_id={tp['/lidar_points'][0]} order by timestamp").fetchall()
for rid, ts in ids[::max(1, len(ids) // 40)]:          # read only ~40 scans, never the whole LiDAR stream
    (d,) = c.execute('select data from messages where rowid=?', (rid,)).fetchone()
    offs.append(ts * 1e-9 - stamp(deserialize_message(bytes(d), PCm).header))
OFF = float(np.median(offs))

f = sqlite3.connect(f'file:{db(FL)}?mode=ro&immutable=1', uri=True)
tf_ = {n: (i, t) for i, n, t in f.execute('select id,name,type from topics')}
OD = get_message(tf_['/Odometry'][1]); FO = []
for (d,) in f.execute(f"select data from messages where topic_id={tf_['/Odometry'][0]} order by timestamp"):
    m = deserialize_message(bytes(d), OD); p = m.pose.pose.position
    FO.append((stamp(m.header), -p.x, p.y, p.z))
FO = np.array(sorted(FO))

# time match onboard to FAST-LIO (skip onboard gaps > 1 s)
tq = FO[:, 0] + OFF
k = np.searchsorted(ON[:, 0], tq); ok = (k > 0) & (k < len(ON))
gap = np.full(len(tq), np.inf); gap[ok] = ON[k[ok], 0] - ON[k[ok] - 1, 0]; ok &= gap < 1.0
ox = np.interp(tq, ON[:, 0], ON[:, 1]); oy = np.interp(tq, ON[:, 0], ON[:, 2])
A = np.c_[ox[ok], oy[ok]]; Bm = FO[ok, 1:3]
ma, mb = A.mean(0), Bm.mean(0); U, _, Vt = np.linalg.svd((A - ma).T @ (Bm - mb)); R = Vt.T @ U.T
if np.linalg.det(R) < 0: Vt[1] *= -1; R = Vt.T @ U.T
ONa = (R @ (ON[:, 1:3] - ma).T).T + mb
seg = np.hypot(np.diff(FO[:, 1]), np.diff(FO[:, 2])); Lf = seg.sum()
inr = (ON[:, 0] >= tq[0]) & (ON[:, 0] <= tq[-1]); Lon = np.sum(np.hypot(np.diff(ON[inr, 1]), np.diff(ON[inr, 2])))
cl_f = np.hypot(*(FO[-1, 1:3] - FO[0, 1:3])); oi = np.where(inr)[0]; cl_o = np.hypot(*(ON[oi[-1], 1:3] - ON[oi[0], 1:3]))
dur = np.ptp(FO[:, 0]); zd = FO[-1, 3] - FO[0, 3]
print(f'{TITLE}: clock offset {OFF:.3f} s | matched {ok.sum()}/{len(FO)} | FAST-LIO {Lf:.1f} m closure {cl_f:.2f} m | onboard {Lon:.1f} m closure {cl_o:.2f} m | ratio {Lf / max(Lon, 1e-6):.3f} | z {zd:+.1f} m / {dur:.0f} s')

W = np.zeros((0, 2))
g = sqlite3.connect(f'file:{db(CLOUD)}?mode=ro&immutable=1', uri=True)
tg = {n: (i, t) for i, n, t in g.execute('select id,name,type from topics')}
if '/cloud_registered' in tg:
    PC = get_message(tg['/cloud_registered'][1]); parts = []
    for (d,) in g.execute(f"select data from messages where topic_id={tg['/cloud_registered'][0]} order by timestamp").fetchall()[::EVERY]:
        m = deserialize_message(bytes(d), PC); fo = {fl.name: fl.offset for fl in m.fields}
        b = np.frombuffer(bytes(m.data), np.uint8).reshape(-1, m.point_step)
        x, y, z = (b[:, fo[q]:fo[q] + 4].copy().view(np.float32).ravel().astype(float) for q in 'xyz')
        zr = np.interp(stamp(m.header), FO[:, 0], FO[:, 3])
        s = np.isfinite(x) & (z > zr - 0.2) & (z < zr + 1.6)          # wall-height band around the robot
        parts.append(np.c_[-x[s], y[s]][::2])
    W = np.vstack(parts)

plt.rcParams.update({'font.size': 13})
fig, ax = plt.subplots(1, 2, figsize=(22, 10), facecolor='white')
if len(W):
    H, xe, ye = np.histogram2d(W[:, 0], W[:, 1], bins=[int(np.ptp(W[:, 0]) / .05) + 1, int(np.ptp(W[:, 1]) / .05) + 1])
    ax[0].pcolormesh(xe, ye, np.where(H > 0, H, np.nan).T, norm=LogNorm(vmin=1, vmax=max(H.max(), 10)), cmap='magma')
    ax[0].set_facecolor('black')
ax[0].plot(FO[:, 1], FO[:, 2], '-', c='#00d0ff', lw=1.0, label='FAST-LIO trajectory')
ax[0].set_title('FAST-LIO map (wall-height band of /cloud_registered, top view)'); ax[0].legend(loc='upper right')
ax[1].plot(ONa[inr, 0], ONa[inr, 1], '--', c='#888888', lw=1.4, label=f'Onboard odometry  {Lon:.0f} m, end-start {cl_o:.2f} m')
ax[1].plot(FO[:, 1], FO[:, 2], '-', c='#d62728', lw=1.6, label=f'FAST-LIO (this repo)  {Lf:.0f} m, end-start {cl_f:.2f} m')
ax[1].plot(*FO[0, 1:3], 'o', c='green', ms=10, label='start'); ax[1].plot(*FO[-1, 1:3], 's', c='black', ms=8, label='end')
ax[1].set_title('Trajectory comparison (onboard rigidly aligned, no scaling)'); ax[1].legend(loc='best')
for a in ax: a.set_aspect('equal'); a.grid(alpha=.25); a.set_xlabel('x (m)'); a.set_ylabel('y (m)')
# view limits: trajectory extent + 6 m, so stray far points do not shrink the map
lo = FO[:, 1:3].min(0) - 6; hi = FO[:, 1:3].max(0) + 6
for a in ax: a.set_xlim(lo[0], hi[0]); a.set_ylim(lo[1], hi[1])
fig.suptitle(f'{TITLE}  —  {dur:.0f} s, FAST-LIO / onboard path length = {Lf / max(Lon, 1e-6):.2f}', fontsize=16)
plt.tight_layout(rect=[0, 0, 1, .95]); plt.savefig(OUT, dpi=70); print('saved', OUT)
