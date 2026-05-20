"""
Simulate wall-correction heading error and steer_p at various SSAM values.

Paste one or more StraightLine log lines, then Ctrl+Z+Enter (Windows) or Ctrl+D.
For each parsed line the script prints what heading_error and steer_p would be
under the SSAM values listed in SSAM_CANDIDATES.

Useful for choosing SSAM without flashing: pick the smallest value that keeps
steer_p well below MAX_STEERING (0.30) for normal running while providing
enough correction for the worst-case initial misalignment.

Physical constants must be kept in sync with src/trajectory/straight_line.rs
and src/dimensions.rs.
"""

import re
import sys
import math
import matplotlib.pyplot as plt

# ── Constants (keep in sync with firmware) ───────────────────────────────────
ROBOT_WIDTH       = 0.10   # m
LATERAL_CLEARANCE = 0.04   # m   = (LAB_CELL - ROBOT_WIDTH) / 2
PREFERRED_CLEARANCE = LATERAL_CLEARANCE * 2.5  # 0.10 m
KP_STEERING       = 0.6
KD_WALL           = 0.2
MAX_STEERING      = 0.30

SSAM_CANDIDATES   = [0.45, 0.55, 0.70, 0.90]

# ── Log parsing ───────────────────────────────────────────────────────────────
LINE_RE = re.compile(
    r"StraightLine \((?P<dist>[\d.]+)/(?P<total>[\d.]+)m\).*?"
    r"L: (?P<L>-?[\d.]+|NaN).*?"
    r"M: (?P<M>-?[\d.]+|NaN).*?"
    r"R: (?P<R>-?[\d.]+|NaN).*?"
    r"L_rate: (?P<Lr>-?[\d.]+).*?"
    r"R_rate: (?P<Rr>-?[\d.]+).*?"
    r"theta: (?P<theta>-?[\d.]+)deg"
)

def parse_float(s):
    try:
        return float(s)
    except ValueError:
        return float("nan")

print("Paste StraightLine log lines, then Ctrl+Z+Enter (Windows) / Ctrl+D:")
raw = sys.stdin.read()

rows = []
initial_theta = None
for line in raw.splitlines():
    m = LINE_RE.search(line)
    if not m:
        continue
    theta = parse_float(m.group("theta"))
    if initial_theta is None:
        initial_theta = theta
    rows.append({
        "dist":   parse_float(m.group("dist")),
        "total":  parse_float(m.group("total")),
        "L":      parse_float(m.group("L")),
        "M":      parse_float(m.group("M")),
        "R":      parse_float(m.group("R")),
        "L_rate": parse_float(m.group("Lr")),
        "R_rate": parse_float(m.group("Rr")),
        "theta":  theta,
    })

if not rows:
    print("No StraightLine entries found.")
    sys.exit(1)

print(f"Parsed {len(rows)} entries.  initial_theta={initial_theta:.1f}°\n")

# ── Simulation ────────────────────────────────────────────────────────────────
def wall_heading_error(d, ssam):
    """Returns heading error contribution (radians) from one wall at distance d."""
    if math.isnan(d) or d <= 0:
        return 0.0
    remaining = max(d - ROBOT_WIDTH / 2.0, 0.0)
    if remaining < PREFERRED_CLEARANCE:
        return ssam * (PREFERRED_CLEARANCE - remaining) / PREFERRED_CLEARANCE
    return 0.0

# ── Text table ────────────────────────────────────────────────────────────────
header = f"{'dist':>6}  {'L':>6}  {'M':>6}  {'R':>6}  {'theta':>7}  " + \
         "  ".join(f"SSAM={s:.2f} (hdg  steer_p)" for s in SSAM_CANDIDATES)
print(header)
print("-" * len(header))

ssam_data = {s: [] for s in SSAM_CANDIDATES}  # dist → list of (hdg_deg, steer_p) for plotting

for r in rows:
    theta_comp = math.radians(r["theta"] - initial_theta)
    kd_contrib = KD_WALL * (r["L_rate"] - r["R_rate"])
    parts = []
    for ssam in SSAM_CANDIDATES:
        h_L = wall_heading_error(r["L"], ssam)
        h_R = wall_heading_error(r["R"], ssam)
        wall_corr = h_L - h_R
        hdg = theta_comp + wall_corr + kd_contrib
        steer_p = -KP_STEERING * hdg
        capped = " CAP" if abs(steer_p) >= MAX_STEERING else "    "
        parts.append(f"{math.degrees(hdg):+6.1f}° {steer_p:+.3f}{capped}")
        ssam_data[ssam].append(steer_p)
    L_str = f"{r['L']:.3f}" if not math.isnan(r["L"]) else "  NaN"
    M_str = f"{r['M']:.3f}" if not math.isnan(r["M"]) else "  NaN"
    R_str = f"{r['R']:.3f}" if not math.isnan(r["R"]) else "  NaN"
    print(f"{r['dist']:6.2f}  {L_str:>6}  {M_str:>6}  {R_str:>6}  {r['theta']:+6.1f}°  " +
          "  ".join(parts))

# ── Plots ─────────────────────────────────────────────────────────────────────
dists = [r["dist"] for r in rows]
Ls    = [r["L"] for r in rows]
Ms    = [r["M"] for r in rows]
Rs    = [r["R"] for r in rows]

fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(12, 8))
fig.suptitle("StraightLine wall correction simulation")

# ── Subplot 1: distances ──────────────────────────────────────────────────────
ax1.plot(dists, Ls, label="L (left wall)", color="steelblue")
ax1.plot(dists, Rs, label="R (right wall)", color="darkorange")
ax1.plot(dists, Ms, label="M (front wall)", color="green", linestyle="--")
ax1.axhline(PREFERRED_CLEARANCE + ROBOT_WIDTH / 2.0,
            color="steelblue", linestyle=":", alpha=0.5,
            label=f"L/R preferred ({PREFERRED_CLEARANCE + ROBOT_WIDTH / 2.0:.3f} m)")
ax1.axhline(LATERAL_CLEARANCE + ROBOT_WIDTH / 2.0,
            color="steelblue", linestyle="-.", alpha=0.5,
            label=f"L/R single-wall ({LATERAL_CLEARANCE + ROBOT_WIDTH / 2.0:.3f} m)")
ax1.set_ylabel("Distance (m)")
ax1.legend(fontsize=8)
ax1.grid(True, alpha=0.3)

# ── Subplot 2: steer_p per SSAM candidate ────────────────────────────────────
colors = ["tab:blue", "tab:orange", "tab:green", "tab:red"]
for ssam, color in zip(SSAM_CANDIDATES, colors):
    ax2.plot(dists, ssam_data[ssam], label=f"SSAM={ssam:.2f}", color=color)
ax2.axhline( MAX_STEERING, color="red", linestyle="--", linewidth=0.8,
             label=f"±MAX_STEERING ({MAX_STEERING})")
ax2.axhline(-MAX_STEERING, color="red", linestyle="--", linewidth=0.8)
ax2.axhline(0, color="black", linewidth=0.5)
ax2.set_xlabel("Odometry distance (m)")
ax2.set_ylabel("steer_p")
ax2.legend(fontsize=8)
ax2.grid(True, alpha=0.3)

plt.tight_layout()
plt.show()
