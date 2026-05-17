"""
Paste StraightLine log output, press Ctrl+Z (Windows) or Ctrl+D (Linux/Mac) then Enter.
Non-StraightLine lines (battery, INFO, etc.) are silently skipped.
"""

import re
import sys
import statistics
import matplotlib.pyplot as plt

PATTERN = re.compile(
    r"(?:\[\d+ms\] )?"
    r"StraightLine \((?P<dist>[\d.]+)/(?P<total>[\d.]+)m\): "
    r"target: (?P<target>-?[\d.]+)(?P<decel> \(decel\))?, "
    r"current: (?P<current>-?[\d.]+), "
    r"error: (?P<error>-?[\d.]+), "
    r"commanded: (?P<commanded>-?[\d.]+), "
    r"p: (?P<p_term>-?[\d.]+), "
    r"i: (?P<i_term>-?[\d.]+), "
    r"steer: (?P<steer>-?[\d.]+), "
    r"steer_p: (?P<steer_p>-?[\d.]+), "
    r"steer_i: (?P<steer_i>-?[\d.]+), "
    r"wall_steer: (?P<wall_steer>-?[\d.]+), "
    r"L: (?P<l_mm>-?\d+) M: (?P<m_mm>-?\d+) R: (?P<r_mm>-?\d+) "
    r"\(drift_mm: (?P<drift_mm>-?[\d.]+)\), "
    r"hdg: (?P<hdg>-?[\d.]+)deg"
)

print("Paste log data, then press Ctrl+Z + Enter (Windows) or Ctrl+D (Linux/Mac):")
raw = sys.stdin.read()

rows = []
for line in raw.splitlines():
    m = PATTERN.search(line)
    if m:
        d = {}
        for k, v in m.groupdict().items():
            if k == "decel":
                d[k] = v  # keep as string or None
            elif v is not None:
                d[k] = float(v)
        rows.append(d)

if not rows:
    print("No StraightLine entries found.")
    sys.exit(1)

print(f"Parsed {len(rows)} entries.")

dist      = [r["dist"]      for r in rows]
target    = [r["target"]    for r in rows]
current   = [r["current"]   for r in rows]
commanded = [r["commanded"] for r in rows]
error     = [r["error"]     for r in rows]
p_term    = [r["p_term"]    for r in rows]
i_term    = [r["i_term"]    for r in rows]
steer     = [r["steer"]     for r in rows]
steer_p   = [r.get("steer_p", 0.0) for r in rows]
steer_i   = [r.get("steer_i", 0.0) for r in rows]
wall_steer = [r.get("wall_steer", 0.0) for r in rows]
hdg       = [r["hdg"]       for r in rows]
total     = rows[0]["total"]
decel_start = next((r["dist"] for r in rows if r.get("decel")), None)

# -1 mm is the firmware's "no/invalid reading" sentinel — drop those points so the trace
# doesn't dip to -1 between valid samples.
def nan_if_neg(v):
    return v if v >= 0 else float("nan")

l_mm     = [nan_if_neg(r["l_mm"])    for r in rows]
m_mm     = [nan_if_neg(r["m_mm"])    for r in rows]
r_mm     = [nan_if_neg(r["r_mm"])    for r in rows]
drift_mm = [r["drift_mm"]            for r in rows]

def σ(series):
    finite = [x for x in series if x == x]  # filter NaN
    return statistics.stdev(finite) if len(finite) > 1 else 0.0

def mean(series):
    finite = [x for x in series if x == x]
    return statistics.mean(finite) if finite else 0.0

fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 11), sharex=True)
fig.suptitle(f"StraightLine — {total} m run")

if decel_start is not None:
    for ax in (ax1, ax2, ax3):
        ax.axvspan(decel_start, float(total), alpha=0.07, color="orange", label="_decel zone")

# ── Speed panel ─────────────────────────────────────────────────────────────
ax1.plot(dist, target,    label="target speed",    linestyle="--", color="gray")
ax1.plot(dist, current,   label=f"current speed  (σ={σ(current):.3f}, μ={mean(current):.3f})",  color="tab:blue")
ax1.plot(dist, commanded, label=f"commanded speed (σ={σ(commanded):.3f}, μ={mean(commanded):.3f})", color="tab:orange")
if decel_start is not None:
    ax1.axvline(decel_start, color="darkorange", linewidth=0.8, linestyle=":", label=f"decel start ({decel_start:.2f} m)")
ax1.set_ylabel("Speed (m/s)")
ax1.legend()
ax1.grid(True, alpha=0.4)

# ── Steering / PI panel ─────────────────────────────────────────────────────
ax2.plot(dist, error,                       label=f"error      (σ={σ(error):.3f}, μ={mean(error):.3f})", color="tab:red")
ax2.plot(dist, p_term,                      label=f"P term (m/s)", color="tab:purple")
ax2.plot(dist, i_term,                      label=f"I term (m/s)", color="tab:green")
ax2.plot(dist, [v * 10 for v in steer],     label="steering ×10",  color="tab:brown")
ax2.plot(dist, [v * 10 for v in steer_p],   label="steer_p ×10",   color="tab:cyan",   linestyle="--")
ax2.plot(dist, [v * 10 for v in steer_i],   label="steer_i ×10",   color="tab:pink",   linestyle="--")
ax2.plot(dist, [v * 10 for v in wall_steer], label=f"wall_steer ×10 (μ={mean(wall_steer):.3f})", color="tab:gray", linestyle="--")
ax2.plot(dist, [h / 10 for h in hdg],       label=f"heading error (deg / 10)", color="tab:olive")
ax2.axhline(0, color="black", linewidth=0.7)
ax2.set_ylabel("m/s")
ax2.legend(loc="upper right", fontsize=8)
ax2.grid(True, alpha=0.4)

# ── Wall-follow ToF panel ───────────────────────────────────────────────────
ax3.plot(dist, l_mm,     label=f"L (looking-left)  (μ={mean(l_mm):.0f} mm)",  color="tab:blue",   marker=".", linestyle="-", linewidth=0.8)
ax3.plot(dist, m_mm,     label=f"M (front)         (μ={mean(m_mm):.0f} mm)",  color="tab:green",  marker=".", linestyle="-", linewidth=0.8)
ax3.plot(dist, r_mm,     label=f"R (looking-right) (μ={mean(r_mm):.0f} mm)",  color="tab:orange", marker=".", linestyle="-", linewidth=0.8)
ax3.plot(dist, drift_mm, label=f"drift (mm, +=left) (σ={σ(drift_mm):.1f}, μ={mean(drift_mm):.1f})", color="tab:red", linewidth=1.2)
ax3.axhline(0, color="black", linewidth=0.7)
ax3.axhline(127, color="tab:blue", linewidth=0.5, linestyle=":", alpha=0.6, label="diag centered ≈127 mm")
ax3.set_ylabel("Distance (mm) / drift (mm)")
ax3.set_xlabel("Distance (m)")
ax3.legend(loc="upper right", fontsize=8)
ax3.grid(True, alpha=0.4)

plt.tight_layout()
plt.show()
