"""Cycle-by-cycle step analysis of a Lab 3 capture, with error bars and spec lines.

Run with: ~/anaconda3/bin/python plot_steps.py captures/<file>.txt [--title "part (b)"]

Top: full capture with the reference, the +-2% settling bands and the 45% overshoot limits.
Middle: every complete up step and down step overlaid from the moment of the step, with the
mean +- 1 standard deviation (shaded, and error bars every second) and the specification
lines (target, +-2% band, 45% overshoot, 7 s settling time).
Bottom: overshoot, 2% settling time and steady-state error per direction, mean +- 1 std
over the cycles, against the specification.
Saves <file>_steps.png next to the capture and prints the per-step table.
"""
import argparse
import numpy as np
import matplotlib.pyplot as plt

OS_SPEC, TS_SPEC, BAND = 45.0, 7.0, 0.02   # %, s, fraction of the step
END_STOP = 0.035                           # m, left end stop (2026-11-26 calibration)

ap = argparse.ArgumentParser()
ap.add_argument("capture")
ap.add_argument("--title", default="")
args = ap.parse_args()

rows = []
for line in open(args.capture):
    p = line.strip().split(",")
    if len(p) in (6, 7):
        try:
            rows.append([float(x) for x in p[:6]])
        except ValueError:
            pass
d = np.array(rows)
d = d[np.r_[True, np.diff(d[:, 0]) > 0]]   # drop repeated/out-of-order lines
t, r, y = d[:, 0] / 1000.0, d[:, 1], d[:, 2]

# ---------------- complete steps ----------------
edges = np.flatnonzero(np.diff(r) != 0) + 1
steps = []
for i0, i1 in zip(edges[:-1], edges[1:]):
    y0, y1 = r[i0 - 1], r[i0]
    tt, seg = t[i0:i1] - t[i0], y[i0:i1]
    os_pct = max(0.0, ((seg - y1) / (y1 - y0)).max() * 100)
    out = np.flatnonzero(np.abs(seg - y1) > BAND * abs(y1 - y0))
    settled = len(out) == 0 or out[-1] + 1 < len(tt)
    ts = (0.0 if len(out) == 0 else tt[out[-1] + 1]) if settled else np.nan
    e_ss = seg[tt >= tt[-1] - 1.0].mean() - y1       # mean error over the last second
    steps.append(dict(up=y1 > y0, t0=t[i0], y0=y0, y1=y1, tt=tt, seg=seg, os=os_pct, ts=ts, e_ss=e_ss))

print(f"{'t (s)':>7} {'step':>11} {'OS':>7} {'ts (2%)':>8} {'e_ss':>9}")
for s in steps:
    ts_s = "never" if np.isnan(s["ts"]) else f"{s['ts']:.2f}"
    print(f"{s['t0']:7.1f} {s['y0']:.2f}->{s['y1']:.2f} {s['os']:6.1f}% {ts_s:>8} {s['e_ss']*1000:+8.1f}mm")
summary = {}
for name, up in (("up", True), ("down", False)):
    g = [s for s in steps if s["up"] == up]
    os_ = np.array([s["os"] for s in g]); ts_ = np.array([s["ts"] for s in g]); es = np.array([s["e_ss"] for s in g])
    summary[name] = (os_, ts_, es)
    n_set = np.sum(~np.isnan(ts_))
    ts_txt = f"{np.nanmean(ts_):.2f} ± {np.nanstd(ts_):.2f} s ({n_set}/{len(g)} settled)" if n_set else f"never settled (0/{len(g)})"
    print(f"{name:>4} ({len(g)} steps): OS {os_.mean():.1f} ± {os_.std():.1f}%, ts {ts_txt}, "
          f"e_ss {es.mean()*1000:+.1f} ± {es.std()*1000:.1f} mm")

# ---------------- figure ----------------
INK, MUTED, GRID = "#222222", "#6b6b6b", "#e4e4e0"
BLUE, ORANGE, GREEN, RED = "#2a78d6", "#eb6834", "#1baf7a", "#c0392b"
fig = plt.figure(figsize=(13, 11))
gs = fig.add_gridspec(3, 3, height_ratios=[1.1, 1.3, 0.9], hspace=0.38, wspace=0.28, top=0.93)


def style(a):
    a.grid(color=GRID, lw=0.8)
    a.set_axisbelow(True)
    for sp in ("top", "right"):
        a.spines[sp].set_visible(False)
    a.tick_params(colors=MUTED, labelsize=8)


# full capture
a = fig.add_subplot(gs[0, :])
for lvl in (0.10, 0.25):
    a.axhspan(lvl - BAND * 0.15, lvl + BAND * 0.15, color=GREEN, alpha=0.25, lw=0)
a.axhline(0.25 + OS_SPEC / 100 * 0.15, color=RED, ls=":", lw=1)
a.axhline(0.10 - OS_SPEC / 100 * 0.15, color=RED, ls=":", lw=1)
a.axhline(END_STOP, color=MUTED, ls="-.", lw=0.8)
a.plot(t, r, color=INK, lw=1.2, drawstyle="steps-post", label="reference")
a.plot(t, y, color=BLUE, lw=1.4, label="ball position")
a.text(t[0], 0.25 + OS_SPEC / 100 * 0.15 + 0.004, "45% overshoot limit", fontsize=7.5, color=RED)
a.text(t[0], END_STOP + 0.003, "left end stop", fontsize=7.5, color=MUTED)
a.set_ylabel("ball position (m)", color=INK)
a.set_xlabel("time (s)", color=INK)
a.legend(fontsize=8, frameon=False, loc="upper right", ncol=2)
a.set_title("Full capture (green: ±2% settling bands)", fontsize=10, color=INK, loc="left")
style(a)

# overlaid steps per direction
for col, (name, up) in enumerate((("Up steps 0.10 → 0.25 m", True), ("Down steps 0.25 → 0.10 m", False))):
    a = fig.add_subplot(gs[1, :2] if col == 0 else gs[1, 2])
    g = [s for s in steps if s["up"] == up]
    n = min(len(s["seg"]) for s in g)
    tt = g[0]["tt"][:n]
    Y = np.array([s["seg"][:n] for s in g])
    mu, sd = Y.mean(0), Y.std(0)
    y0, y1 = g[0]["y0"], g[0]["y1"]
    step = y1 - y0
    a.axhspan(y1 - BAND * abs(step), y1 + BAND * abs(step), color=GREEN, alpha=0.25, lw=0, label="±2% band")
    a.axhline(y1, color=INK, lw=1, label="target")
    a.axhline(y1 + OS_SPEC / 100 * step, color=RED, ls=":", lw=1.2, label="45% overshoot limit")
    a.axvline(TS_SPEC, color=RED, ls="--", lw=1.2, label="7 s settling limit")
    if not up:
        a.axhline(END_STOP, color=MUTED, ls="-.", lw=0.8, label="left end stop")
    for s in g:
        a.plot(tt, s["seg"][:n], color=BLUE, lw=0.6, alpha=0.35)
    a.fill_between(tt, mu - sd, mu + sd, color=ORANGE, alpha=0.25, lw=0)
    a.plot(tt, mu, color=ORANGE, lw=2, label=f"mean of {len(g)} steps ± 1σ")
    k = np.searchsorted(tt, np.arange(0, tt[-1], 1.0))
    a.errorbar(tt[k], mu[k], yerr=sd[k], fmt="none", ecolor=ORANGE, elinewidth=1.2, capsize=3)
    a.set_title(name, fontsize=10, color=INK, loc="left")
    a.set_xlabel("time since step (s)", color=INK)
    a.set_ylabel("ball position (m)", color=INK)
    a.legend(fontsize=7.5, frameon=False, loc="lower right" if up else "upper right")
    style(a)

# metric summary
labels = ["up", "down"]
x = np.arange(2)
metrics = [("Overshoot (%)", 0, OS_SPEC, "< 45%"), ("2% settling time (s)", 1, TS_SPEC, "≤ 7 s"),
           ("Steady-state error (mm)", 2, None, "0 (band ±3 mm)")]
for j, (title, idx, spec, spec_txt) in enumerate(metrics):
    a = fig.add_subplot(gs[2, j])
    vals = [summary[l][idx] * (1000 if idx == 2 else 1) for l in labels]
    for xi, v in zip(x, vals):
        ok = v[~np.isnan(v)]
        if len(ok) == 0:
            a.text(xi, (spec or 1) * 0.5, "never\nsettled", ha="center", va="center", fontsize=9, color=RED)
            continue
        a.bar(xi, ok.mean(), width=0.55, color=[BLUE, ORANGE][xi], alpha=0.75)
        a.errorbar(xi, ok.mean(), yerr=ok.std(), fmt="none", ecolor=INK, capsize=6, elinewidth=1.4)
        a.scatter(np.full(len(ok), xi) + np.linspace(-0.12, 0.12, len(ok)), ok, s=10, color=INK, zorder=3)
        a.text(xi, ok.mean(), f" {ok.mean():.1f}±{ok.std():.1f}", ha="left", va="bottom", fontsize=8, color=INK)
    if spec is not None:
        a.axhline(spec, color=RED, ls="--", lw=1.2)
        a.set_ylim(0, max(spec * 1.25, np.nanmax(np.concatenate(vals)) * 1.15 if np.any(~np.isnan(np.concatenate(vals))) else spec * 1.25))
    else:
        a.axhspan(-3, 3, color=GREEN, alpha=0.25, lw=0)
        a.axhline(0, color=RED, ls="--", lw=1.2)
    a.set_xticks(x, labels)
    a.set_xlim(-0.6, 1.6)
    a.set_title(f"{title}, spec {spec_txt}", fontsize=9.5, color=INK, loc="left")
    style(a)

fig.suptitle(f"Lab 3 {args.title} — {args.capture.rsplit('/', 1)[-1]}: mean ± 1σ over "
             f"{len(steps)} steps", color=INK, fontsize=12, x=0.07, y=0.97, ha="left")
out = args.capture.rsplit(".", 1)[0] + "_steps.png"
fig.savefig(out, dpi=130, bbox_inches="tight")
print(f"\nSaved {out}")
