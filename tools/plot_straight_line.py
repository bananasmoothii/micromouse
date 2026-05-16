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
hdg       = [r["hdg"]       for r in rows]
total     = rows[0]["total"]
decel_start = next((r["dist"] for r in rows if r.get("decel")), None)

def σ(series):
    return statistics.stdev(series) if len(series) > 1 else 0.0

def mean(series):
    return statistics.mean(series)

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
fig.suptitle(f"StraightLine — {total} m run")

if decel_start is not None:
    for ax in (ax1, ax2):
        ax.axvspan(decel_start, float(total), alpha=0.07, color="orange", label="_decel zone")
ax1.plot(dist, target,    label="target speed",    linestyle="--", color="gray")
ax1.plot(dist, current,   label=f"current speed  (σ={σ(current):.3f}, μ={mean(current):.3f})",  color="tab:blue")
ax1.plot(dist, commanded, label=f"commanded speed (σ={σ(commanded):.3f}, μ={mean(commanded):.3f})", color="tab:orange")
if decel_start is not None:
    ax1.axvline(decel_start, color="darkorange", linewidth=0.8, linestyle=":", label=f"decel start ({decel_start:.2f} m)")
ax1.set_ylabel("Speed (m/s)")
ax1.legend()
ax1.grid(True, alpha=0.4)

ax2.plot(dist, error,                      label=f"error      (σ={σ(error):.3f}, μ={mean(error):.3f})", color="tab:red")
ax2.plot(dist, p_term,                     label=f"P term (m/s)", color="tab:purple")
ax2.plot(dist, i_term,                     label=f"I term (m/s)", color="tab:green")
ax2.plot(dist, [v * 10 for v in steer],   label="steering ×10",  color="tab:brown")
ax2.plot(dist, [v * 10 for v in steer_p], label="steer_p ×10",   color="tab:cyan",   linestyle="--")
ax2.plot(dist, [v * 10 for v in steer_i], label="steer_i ×10",   color="tab:pink",   linestyle="--")
ax2.plot(dist, [h / 10 for h in hdg],     label=f"heading error (deg / 10)", color="tab:olive")
ax2.axhline(0, color="black", linewidth=0.7)
ax2.set_ylabel("m/s")
ax2.set_xlabel("Distance (m)")
ax2.legend()
ax2.grid(True, alpha=0.4)

plt.tight_layout()
plt.show()
