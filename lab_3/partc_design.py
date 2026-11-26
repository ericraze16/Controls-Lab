"""Lab 3 (c): outer-loop controller design and verification.

Design (emulation):
  outer plant   P(s) = T1(s) K2 K3 / s^2  ~  K / s^2,  K = K2 K3   (inner loop T1 ~ 1)
  controller    C2(s) = (b2 s^2 + b1 s + b0) / (s (s + a))      integrator + lead
  closed loop   s^3 (s + a) + K (b2 s^2 + b1 s + b0) = Delta(s)
  Delta(s) = (s^2 + 2 zeta wn s + wn^2)(s + p)^2,  sigma = zeta wn
C2 is discretized with Tustin at T2 = 10 ms. (simulate() also accepts an
optional reference prefilter, kept for part (e).)

Verification: full sampled-data model, nonlinear where the hardware is:
  r -> (+) -> D2[z] -> theta_ref saturator (+-0.7 rad) -> inner loop
  inner loop: D1[z] (Lab 2 Tustin, 1 ms) -> +-6 V -> K1 / (s (tau s + 1))
  beam/ball:  y'' = K2 K3 (theta - theta_level),  y sampled every T2 (ZOH)

Run with: ~/anaconda3/bin/python partc_design.py
"""
import numpy as np
from scipy import signal
from scipy.linalg import expm
import matplotlib.pyplot as plt

np.set_printoptions(precision=15)

# ---------------- Plant (Labs 1-2) ----------------
K1, TAU = -1.81, 0.0231          # motor, rad/(V s) and s
K2 = 0.0609                      # beam angle / gear angle (manual)
K3 = -4.454                      # m/s^2 per rad of beam angle (k3_auto_run1 fit)
K = K2 * K3                      # outer plant gain, m/s^2 per rad of gear angle
THETA_LEVEL = -0.1005            # gear angle at which the beam is level (same fit)

# Inner loop, Lab 2 revised design, Tustin at 1 ms (as implemented)
T1 = 1e-3
NUM1 = np.array([-0.07356126214266934, -0.0031170026331643896, 0.07044425950950561])
DEN1 = np.array([1.0, -1.9385879020494574, 0.9398836903182997])
C1_NUM, C1_DEN = [-148.5178047, -6429.342196], [1, 62, 1336.390625]

REF_SAT, V_SAT = 0.7, 6.0
Y_LO, Y_HI, PERIOD = 0.10, 0.25, 20.0

# ---------------- Outer-loop design ----------------
T2 = 0.010                       # outer-loop sample time (10 x inner)
DIV = int(round(T2 / T1))
ZETA, SIGMA, P_MULT = 0.8, 0.8, 3.0


def design(zeta=ZETA, sigma=SIGMA, p_mult=P_MULT, k=K):
    wn = sigma / zeta
    p = p_mult * sigma
    delta = np.polymul([1, 2 * sigma, wn ** 2], np.polymul([1, p], [1, p]))
    # s^3 (s + a) + K (b2 s^2 + b1 s + b0) = delta
    a = delta[1]
    b2, b1, b0 = delta[2] / k, delta[3] / k, delta[4] / k
    return dict(wn=wn, p=p, delta=delta, a=a, b=(b2, b1, b0),
                c2=([b2, b1, b0], [1, a, 0]), f=([b0], [b2, b1, b0]))


def tustin(tf, T):
    n, d, _ = signal.cont2discrete(tf, T, method="bilinear")
    return n.ravel(), d


# ---------------- Full sampled-data simulation ----------------
def _plant(k):
    # x = [theta, omega, y, ydot]
    A = np.array([[0, 1, 0, 0], [0, -1 / TAU, 0, 0], [0, 0, 0, 1], [k, 0, 0, 0]])
    B = np.array([0, K1 / TAU, 0, 0])
    E = np.array([0, 0, 0, 1.0])     # constant ball acceleration input (beam offset)
    M = expm(np.block([[A, np.c_[B, E]], [np.zeros((2, 6))]]) * T1)
    return M[:4, :4], M[:4, 4], M[:4, 5]


def simulate(num2, den2, numf=None, denf=None, k=K, level=0.0, n_periods=3, sat=True, stops=False):
    """Ball starts at rest at Y_LO; reference steps to Y_HI at t = 0.
    sat=False removes both the theta_ref saturator and the +-6 V motor limit.
    stops=True adds the beam end stops (ball held at 0.035 m and 0.412 m).
    level: gear angle at which the beam is level (0 = ideal beam)."""
    Ad, Bd, Ed = _plant(k)
    n = int(n_periods * PERIOD / T1)
    t = np.arange(n) * T1
    r = np.where((t % PERIOD) < PERIOD / 2, Y_HI, Y_LO)
    x = np.array([level, 0.0, Y_LO, 0.0])           # gear holding the beam level
    e_in, u_in = np.zeros(2), np.zeros(2)
    f_in, f_out = np.zeros(len(numf) if numf is not None else 1), np.zeros(2)
    e_out, r_out = np.zeros(len(num2)), np.full(len(den2) - 1, level)
    rf = th_ref = th_sat = 0.0
    out = {k_: np.zeros(n) for k_ in ("r", "rf", "y", "th_ref", "th_sat", "th", "V")}
    for i in range(n):
        if i % DIV == 0:                              # outer loop
            if numf is None:
                rf = r[i]
            else:                                     # prefilter on r - Y_LO (DC gain 1)
                f_in = np.r_[r[i] - Y_LO, f_in[:-1]]
                v = numf @ f_in - denf[1:] @ f_out
                f_out = np.r_[v, f_out[:-1]]
                rf = Y_LO + v
            e_out = np.r_[rf - x[2], e_out[:-1]]
            th_ref = num2 @ e_out - den2[1:] @ r_out
            r_out = np.r_[th_ref, r_out[:-1]]       # unsaturated value fed back
            th_sat = np.clip(th_ref, -REF_SAT, REF_SAT) if sat else th_ref
        e = th_sat - x[0]                             # inner loop
        u = NUM1[0] * e + NUM1[1] * e_in[0] + NUM1[2] * e_in[1] - DEN1[1] * u_in[0] - DEN1[2] * u_in[1]
        e_in, u_in = np.r_[e, e_in[0]], np.r_[u, u_in[0]]
        V = np.clip(u, -V_SAT, V_SAT) if sat else u
        x = Ad @ x + Bd * V + Ed * (-k * level)       # y'' = k (theta - level)
        if stops and not 0.035 <= x[2] <= 0.412:
            x[2], x[3] = min(max(x[2], 0.035), 0.412), 0.0
        for key, val in (("r", r[i]), ("rf", rf), ("y", x[2]), ("th_ref", th_ref),
                         ("th_sat", th_sat), ("th", x[0]), ("V", V)):
            out[key][i] = val
    out["t"] = t
    return out


def step_metrics(s):
    """OS, 2% settling time and final error for every reference change."""
    t, r, y = s["t"], s["r"], s["y"]
    edges = np.flatnonzero(np.diff(r) != 0) + 1
    edges = np.r_[0, edges]
    res = []
    for j, i0 in enumerate(edges):
        i1 = edges[j + 1] if j + 1 < len(edges) else len(t)
        y0 = Y_LO if j == 0 else r[i0 - 1]
        y1 = r[i0]
        seg, tt = y[i0:i1], t[i0:i1] - t[i0]
        os_pct = max(0.0, ((seg - y1) / (y1 - y0)).max() * 100)
        out = np.flatnonzero(np.abs(seg - y1) > 0.02 * abs(y1 - y0))
        ts = 0.0 if len(out) == 0 else (tt[out[-1] + 1] if out[-1] + 1 < len(tt) else np.inf)
        res.append(dict(step=f"{y0:.2f}->{y1:.2f}", os=os_pct, ts=ts, ess=seg[-1] - y1))
    return res


def worst(res):
    return max(m["os"] for m in res), max(m["ts"] for m in res), max(abs(m["ess"]) for m in res)


if __name__ == "__main__":
    d = design()
    b2, b1, b0 = d["b"]
    print("=== Design (continuous) ===")
    print(f"K = K2 K3 = {K:.6f} m/s^2/rad")
    print(f"zeta = {ZETA}, sigma = {SIGMA} 1/s, wn = {d['wn']:.4f} rad/s, p = {d['p']} (double)")
    print("Delta(s) =", d["delta"])
    print("closed-loop poles:", np.roots(d["delta"]))
    print(f"C2(s) = ({b2:.12f} s^2 + {b1:.12f} s + {b0:.12f}) / (s (s + {d['a']:.1f}))")
    print("C2 zeros:", np.roots(d["c2"][0]), " |zeros|:", abs(np.roots(d["c2"][0])))
    print(f"PID form: Kp = {b1 / d['a']:.4f}, Ki = {b0 / d['a']:.4f}, Kd = {b2 / d['a']:.4f}, "
          f"filter {d['a']:.0f}/(s+{d['a']:.0f})")

    # inner loop approximation check
    inner = signal.TransferFunction(np.polymul(C1_NUM, [K1]),
                                    np.polyadd(np.polymul(C1_DEN, [TAU, 1, 0]), np.polymul(C1_NUM, [K1])))
    print("\ninner closed-loop poles:", np.roots(inner.den))
    L_num = np.polymul(np.polymul(d["c2"][0], [K]), inner.num)
    L_den = np.polymul(np.polymul(d["c2"][1], [1, 0, 0]), inner.den)
    w = np.logspace(-2, 2, 200000)
    _, H = signal.freqs(L_num, L_den, worN=w)
    i_c = np.argmin(abs(abs(H) - 1))
    pm = 180 + np.degrees(np.angle(H[i_c]))
    _, Ti = signal.freqs(inner.num, inner.den, worN=[w[i_c]])
    print(f"loop crossover wc = {w[i_c]:.3f} rad/s, phase margin = {pm:.1f} deg (inner loop included)")
    print(f"inner loop at wc: |T1| = {abs(Ti[0]):.4f}, angle = {np.degrees(np.angle(Ti[0])):.2f} deg")
    print(f"ZOH delay T2/2 at wc: {np.degrees(w[i_c] * T2 / 2):.2f} deg")
    phase = np.unwrap(np.angle(H))
    i180 = np.flatnonzero(np.diff(np.sign(phase + np.pi)))
    for i in []:  # margins reported by the block below
        print(f"phase = -180 deg at {w[i]:.3f} rad/s, |L| = {abs(H[i]):.3f} -> gain margin {20*np.log10(1/abs(H[i])):+.1f} dB")
    # gain margins (loop is conditionally stable: type 3) and delay margin, with T2/2 ZOH delay
    Hd = H * np.exp(-1j * w * T2 / 2)
    ph = np.unwrap(np.angle(Hd))
    ph -= 2 * np.pi * np.round(ph[0] / (2 * np.pi) + 0.75)          # start near -270 deg
    ic = np.flatnonzero(np.diff(np.sign(abs(Hd) - 1)))[-1]
    pm_d = 180 + np.degrees(ph[ic])
    print(f"with ZOH delay: wc = {w[ic]:.3f} rad/s, PM = {pm_d:.1f} deg, "
          f"delay margin = {np.radians(pm_d) / w[ic] * 1000:.0f} ms")
    for i in np.flatnonzero(np.diff(np.sign(ph + np.pi))):
        print(f"  phase -180 deg at {w[i]:.3f} rad/s: gain margin {20 * np.log10(1 / abs(Hd[i])):+.1f} dB")
    T_bw_num, T_bw_den = [K * b0], d["delta"]
    _, Tbw = signal.freqs(T_bw_num, T_bw_den, worN=w)
    print(f"Y/R bandwidth (-3 dB) = {w[np.flatnonzero(abs(Tbw) < 1/np.sqrt(2))[0]]:.3f} rad/s")

    # discretization
    n2, d2 = tustin(d["c2"], T2)
    print(f"\n=== Tustin, T2 = {T2*1000:.0f} ms ===")
    print("D2 num =", n2, "\nD2 den =", d2)
    print("D2 poles z:", np.roots(d2), " zeros z:", np.roots(n2))
    print(f"fastest controller root |s| T2 = {max(abs(np.r_[np.roots(d['c2'][0]), d['a']])) * T2:.3f}")
    print(f"theta_ref kick at a 0.15 m step: b2 x 0.15 = {b2 * 0.15:.2f} rad")

    # verification
    full = simulate(n2, d2)                 # with the theta_ref saturator
    lin = simulate(n2, d2, sat=False)       # same sampled-data loop, theta_ref and voltage limits removed
    print("\n=== Sampled-data model ===")
    for name, s_ in (("full", full), ("no sat", lin)):
        for m in step_metrics(s_):
            print(f"  {name:6s} {m['step']}: OS {m['os']:5.1f}%  ts {m['ts']:5.2f} s  "
                  f"e_end {m['ess']*1000:+.3f} mm")
        print(f"  {name:6s} max |theta_ref| = {abs(s_['th_ref']).max():.3f} rad, "
              f"min y = {s_['y'].min():.4f} m")

    print("\n=== Robustness, full model (worst over all steps) ===")
    for label, k, lev in [("nominal", K, 0.0), ("K3 = -3.3", K2 * -3.3, 0.0),
                          ("K3 = -6.0", K2 * -6.0, 0.0), ("beam offset", K, THETA_LEVEL)]:
        w_ = worst(step_metrics(simulate(n2, d2, k=k, level=lev)))
        print(f"  {label:12s} OS {w_[0]:5.1f}%  ts {w_[1]:5.2f} s  |e_end| {w_[2]*1000:.3f} mm")

    # continuous-time closed loop for the emulation check (no saturator)
    cl = signal.TransferFunction(np.polymul([K], d["c2"][0]), d["delta"])
    tc = np.linspace(0, 10, 10001)
    _, yc = signal.step(cl, T=tc)
    yc = Y_LO + (Y_HI - Y_LO) * yc

    # ---------------- Figure ----------------
    INK, MUTED, GRID = "#222222", "#6b6b6b", "#e4e4e0"
    BLUE, ORANGE, GREEN = "#2a78d6", "#eb6834", "#1baf7a"
    fig, ax = plt.subplots(2, 1, figsize=(11, 8), sharex=True,
                           gridspec_kw=dict(height_ratios=[2.2, 1]))
    a0 = ax[0]
    for lvl in (Y_LO, Y_HI):
        a0.axhspan(lvl - 0.003, lvl + 0.003, color=GREEN, alpha=0.15, lw=0)
    a0.axhline(Y_HI + 0.45 * 0.15, color=MUTED, ls=":", lw=1)
    a0.axhline(Y_LO - 0.45 * 0.15, color=MUTED, ls=":", lw=1)
    a0.axhline(0.05, color="#c0392b", ls="-.", lw=0.8)
    a0.text(0.2, Y_HI + 0.45 * 0.15 + 0.003, "45% overshoot", fontsize=7.5, color=MUTED)
    a0.text(0.2, 0.052, "left end stop (≈0.05 m, not in the model)", fontsize=7.5, color="#c0392b")
    a0.plot(full["t"], full["r"], color=INK, lw=1.2, drawstyle="steps-post", label="reference r")
    a0.plot(tc, yc, color=INK, lw=1, ls=":", label="continuous design, inner loop = 1")
    a0.plot(full["t"], full["y"], color=BLUE, lw=2, label="ball y, full model (design)")
    a0.set_ylim(0.0, 0.38)
    a0.set_ylabel("ball position (m)", color=INK)
    a0.legend(fontsize=8, frameon=False, loc="upper right", ncol=2)
    for j, m in enumerate(step_metrics(full)):
        a0.annotate(f"OS {m['os']:.1f}%\nts {m['ts']:.2f} s", xy=(j * PERIOD / 2 + 6.0, 0.205 if j % 2 == 0 else 0.15),
                    fontsize=8, color=BLUE)
    ax[1].plot(full["t"], np.clip(full["th_ref"], -1.5, 1.5), color=MUTED, lw=1, ls="--",
               label="θ_ref before saturator (clipped to ±1.5 for display)")
    ax[1].plot(full["t"], full["th_sat"], color=BLUE, lw=1.8, label="θ_ref,sat")
    for v in (REF_SAT, -REF_SAT):
        ax[1].axhline(v, color="#c0392b", ls="--", lw=0.8)
    ax[1].set_ylim(-1.6, 1.6)
    ax[1].set_ylabel("gear angle ref (rad)", color=INK)
    ax[1].set_xlabel("time (s)", color=INK)
    ax[1].legend(fontsize=8, frameon=False, loc="upper right", ncol=2)
    for a_ in ax:
        a_.grid(color=GRID, lw=0.8)
        a_.set_axisbelow(True)
        for sp in ("top", "right"):
            a_.spines[sp].set_visible(False)
        a_.tick_params(colors=MUTED, labelsize=8)
    fig.suptitle(f"Lab 3 (c): D2[z], Tustin T2 = {T2*1000:.0f} ms, full sampled-data model "
                 f"(ζ = {ZETA}, σ = {SIGMA}, p = {P_MULT:.0f}σ)", color=INK, fontsize=11, x=0.01, ha="left")
    fig.tight_layout()
    fig.savefig("partc_sim.png", dpi=140)
    print("\nSaved partc_sim.png")
