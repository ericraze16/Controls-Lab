"""Plot a Lab 3 serial capture (lab_3.ino or lab_3d.ino) against the simulation.

Capture format (CoolTerm text file):  t_ms, y_ref, y, theta_ref_sat, theta, V [, dropped]

Run with:
  ~/anaconda3/bin/python plot_lab3.py captures/<file>.txt --part b
  ~/anaconda3/bin/python plot_lab3.py captures/<file>.txt --part d
Options: --level <rad>   gear angle at which the beam is level, for the simulation
                         overlay (default -0.1005 from the Lab 2 K3 fit; 0 = ideal beam)
Prints overshoot, 2% settling time and steady-state error for every complete step
and saves <file>.png next to the capture.
"""
import argparse
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
import partc_design as P

ap = argparse.ArgumentParser()
ap.add_argument("capture")
ap.add_argument("--part", choices=["b", "d"], required=True)
ap.add_argument("--level", type=float, default=-0.1005)
args = ap.parse_args()

# ---------------- load ----------------
rows = []
with open(args.capture) as f:
    for line in f:
        parts = line.strip().split(",")
        if len(parts) not in (6, 7):
            continue
        try:
            rows.append([float(p) for p in parts[:6]])
        except ValueError:
            continue
d = np.array(rows)
gap = np.abs(np.diff(d[:, 0]))                      # drop lines cut off mid-capture
near = np.minimum(np.r_[np.inf, gap], np.r_[gap, np.inf]) <= 50
d = d[near]
t, r, y, th_ref, th, V = (d[:, 0] / 1000.0, d[:, 1], d[:, 2], d[:, 3], d[:, 4], d[:, 5])

# ---------------- metrics per complete step ----------------
edges = np.flatnonzero(np.diff(r) != 0) + 1
print(f"\n{'step at':>8} {'step':>12} {'OS':>7} {'ts (2%)':>9} {'final y':>9} {'e_ss':>9}")
rows_out = []
for j in range(len(edges) - 1):
    i0, i1 = edges[j], edges[j + 1]
    y0, y1 = r[i0 - 1], r[i0]
    seg, tt = y[i0:i1], t[i0:i1] - t[i0]
    os_pct = max(0.0, ((seg - y1) / (y1 - y0)).max() * 100)
    out = np.flatnonzero(np.abs(seg - y1) > 0.02 * abs(y1 - y0))
    ts = 0.0 if len(out) == 0 else (tt[out[-1] + 1] if out[-1] + 1 < len(tt) else np.inf)
    y_end = seg[tt >= tt[-1] - 1.0].mean()            # mean over the last second
    rows_out.append((y0, y1, os_pct, ts, y_end))
    ts_s = "never" if np.isinf(ts) else f"{ts:.2f} s"
    print(f"{t[i0]:7.1f}s {y0:.2f}->{y1:.2f} {os_pct:6.1f}% {ts_s:>9} {y_end:8.4f}m {(y_end - y1) * 1000:+7.1f}mm")
for name, sel in (("up", lambda a: a[1] > a[0]), ("down", lambda a: a[1] < a[0])):
    g = [a for a in rows_out if sel(a)]
    if g:
        print(f"  {name:4s} mean: OS {np.mean([a[2] for a in g]):.1f}%, "
              f"ts {np.mean([a[3] for a in g]):.2f} s, e_ss {np.mean([a[4] - a[1] for a in g]) * 1000:+.1f} mm")

# ---------------- simulation overlay, aligned to the first upward step ----------------
up = [e for e in edges if r[e] > r[e - 1]]
sim = None
if up:
    half = np.median(np.diff(t[edges])) if len(edges) > 1 else 10.0
    P.PERIOD = 2 * round(half)
    if args.part == "b":
        num, den, _ = signal.cont2discrete(([-7, -7 * 0.35], [1, 2.5]), P.T2, method="bilinear")
        num = num.ravel()
    else:
        num, den = P.tustin(P.design()["c2"], P.T2)
    n_per = int(np.ceil((t[-1] - t[up[0]]) / P.PERIOD)) + 1
    sim = P.simulate(num, den, level=args.level, n_periods=n_per, stops=True)
    sim["t"] = sim["t"] + t[up[0]]

# ---------------- plot ----------------
INK, MUTED, GRID, BLUE, ORANGE, GREEN = "#222222", "#6b6b6b", "#e4e4e0", "#2a78d6", "#eb6834", "#1baf7a"
fig, ax = plt.subplots(2, 1, figsize=(12, 7.5), sharex=True, gridspec_kw=dict(height_ratios=[2, 1]))
for lvl in (P.Y_LO, P.Y_HI):
    ax[0].axhspan(lvl - 0.003, lvl + 0.003, color=GREEN, alpha=0.15, lw=0)
ax[0].axhline(0.035, color="#c0392b", ls="-.", lw=0.8)  # left end stop, 2026-11-26 calibration
ax[0].plot(t, r, color=INK, lw=1.2, drawstyle="steps-post", label="reference")
if sim is not None:
    m = sim["t"] <= t[-1]
    ax[0].plot(sim["t"][m], sim["y"][m], color=ORANGE, lw=1.3, ls="--",
               label=f"simulation (beam level at θ = {args.level:+.3f} rad, with end stops)")
ax[0].plot(t, y, color=BLUE, lw=1.6, label="measured ball position")
ax[0].set_ylabel("ball position (m)", color=INK)
ax[0].legend(fontsize=8, frameon=False, loc="upper right")
ax[1].plot(t, th_ref, color=BLUE, lw=1.2, label="θ_ref,sat")
ax[1].plot(t, th, color=MUTED, lw=0.8, label="θ measured")
for v in (P.REF_SAT, -P.REF_SAT):
    ax[1].axhline(v, color="#c0392b", ls="--", lw=0.8)
ax[1].set_ylabel("gear angle (rad)", color=INK)
ax[1].set_xlabel("time (s)", color=INK)
ax[1].legend(fontsize=8, frameon=False, loc="upper right", ncol=2)
for a in ax:
    a.grid(color=GRID, lw=0.8)
    a.set_axisbelow(True)
    for sp in ("top", "right"):
        a.spines[sp].set_visible(False)
    a.tick_params(colors=MUTED, labelsize=8)
title = "part (b): lead controller" if args.part == "b" else "part (d): pole-placement D2[z]"
fig.suptitle(f"Lab 3 {title} — {args.capture.rsplit('/', 1)[-1]}", color=INK, fontsize=11, x=0.01, ha="left")
fig.tight_layout()
out = args.capture.rsplit(".", 1)[0] + ".png"
fig.savefig(out, dpi=130)
print(f"\nSaved {out}")
