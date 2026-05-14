"""
Paste StraightLine log output, press Ctrl+Z (Windows) or Ctrl+D (Linux/Mac) then Enter.
Non-StraightLine lines (battery, INFO, etc.) are silently skipped.
"""

import re
import sys
import matplotlib.pyplot as plt

PATTERN = re.compile(
    r"(?:\[\d+ms\] )?"
    r"StraightLine \((?P<dist>[\d.]+)/(?P<total>[\d.]+)m\): "
    r"target_speed: (?P<target>-?[\d.]+) m/s, "
    r"current_speed: (?P<current>-?[\d.]+) m/s, "
    r"error: (?P<error>-?[\d.]+) m/s, "
    r"commanded_speed: (?P<commanded>-?[\d.]+) m/s, "
    r"P: (?P<P>-?[\d.]+) m/s, "
    r"I: (?P<I>-?[\d.]+) m/s"
)

print("Paste log data, then press Ctrl+Z + Enter (Windows) or Ctrl+D (Linux/Mac):")
raw = sys.stdin.read()

rows = []
for line in raw.splitlines():
    m = PATTERN.search(line)
    if m:
        rows.append({k: float(v) for k, v in m.groupdict().items()})

if not rows:
    print("No StraightLine entries found.")
    sys.exit(1)

print(f"Parsed {len(rows)} entries.")

dist      = [r["dist"]      for r in rows]
target    = [r["target"]    for r in rows]
current   = [r["current"]   for r in rows]
commanded = [r["commanded"] for r in rows]
error     = [r["error"]     for r in rows]
P         = [r["P"]         for r in rows]
I         = [r["I"]         for r in rows]
total     = rows[0]["total"]

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
fig.suptitle(f"StraightLine — {total} m run")

ax1.plot(dist, target,    label="target speed",    linestyle="--", color="gray")
ax1.plot(dist, current,   label="current speed",   color="tab:blue")
ax1.plot(dist, commanded, label="commanded speed",  color="tab:orange")
ax1.set_ylabel("Speed (m/s)")
ax1.legend()
ax1.grid(True, alpha=0.4)

ax2.plot(dist, error,              label="error",           color="tab:red")
ax2.plot(dist, [v * 10 for v in P], label="P correction ×10", color="tab:purple")
ax2.plot(dist, [v * 10 for v in I], label="I correction ×10", color="tab:green")
ax2.axhline(0, color="black", linewidth=0.7)
ax2.set_ylabel("m/s")
ax2.set_xlabel("Distance (m)")
ax2.legend()
ax2.grid(True, alpha=0.4)

plt.tight_layout()
plt.show()
