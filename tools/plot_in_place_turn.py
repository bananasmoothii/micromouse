"""
Paste InPlaceTurn log output, press Ctrl+Z (Windows) or Ctrl+D (Linux/Mac) then Enter.
Non-InPlaceTurn lines are silently skipped.
"""

import re
import sys
import statistics
import matplotlib.pyplot as plt

STEP_PATTERN = re.compile(
    r"(?:\[\d+ms\] )?"
    r"InPlaceTurn: remaining (?P<remaining>-?[\d.]+)deg, "
    r"coast_est (?P<coast_est>-?[\d.]+)deg, "
    r"pi_out (?P<pi_out>-?[\d.]+), "
    r"turn_speed (?P<turn_speed>[\d.]+), "
    r"integral (?P<integral>-?[\d.]+), "
    r"cur_omega (?P<cur_omega>-?[\d.]+)d/s, "
    r"cmd_omega (?P<cmd_omega>-?[\d.]+)d/s"
)

DONE_PATTERN = re.compile(
    r"(?:\[\d+ms\] )?"
    r"InPlaceTurn done: target (?P<target>-?[\d.]+)deg, final error (?P<final_error>-?[\d.]+)deg"
)

print("Paste log data, then press Ctrl+Z + Enter (Windows) or Ctrl+D (Linux/Mac):")
raw = sys.stdin.read()

rows = []
target_deg = None
final_error_deg = None

for line in raw.splitlines():
    m = STEP_PATTERN.search(line)
    if m:
        rows.append({k: float(v) for k, v in m.groupdict().items()})
        continue
    m = DONE_PATTERN.search(line)
    if m:
        target_deg = float(m.group("target"))
        final_error_deg = float(m.group("final_error"))

if not rows:
    print("No InPlaceTurn entries found.")
    sys.exit(1)

print(f"Parsed {len(rows)} step entries.")
if target_deg is not None:
    print(f"Target: {target_deg:.1f}°  Final error: {final_error_deg:.2f}°")

steps      = list(range(len(rows)))
remaining  = [r["remaining"]  for r in rows]
coast_est  = [r["coast_est"]  for r in rows]
pi_out     = [r["pi_out"]     for r in rows]
turn_speed = [r["turn_speed"] for r in rows]
integral   = [r["integral"]   for r in rows]
cur_omega  = [r["cur_omega"]  for r in rows]
cmd_omega  = [r["cmd_omega"]  for r in rows]

def σ(s): return statistics.stdev(s) if len(s) > 1 else 0.0
def μ(s): return statistics.mean(s)

title = f"InPlaceTurn — {target_deg:.1f}°" if target_deg is not None else "InPlaceTurn"
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
fig.suptitle(title)

# --- Panel 1: angle remaining + coast estimate ---
ax1.plot(steps, remaining,  label=f"angle remaining (deg)  final={final_error_deg:.2f}°" if final_error_deg is not None else "angle remaining (deg)", color="tab:blue")
ax1.plot(steps, coast_est,  label="coast estimate (deg)", color="tab:orange", linestyle="--")
ax1.axhline(0, color="black", linewidth=0.7)
ax1.set_ylabel("Degrees")
ax1.legend()
ax1.grid(True, alpha=0.4)

# --- Panel 2: angular speeds ---
ax2.plot(steps, cmd_omega, label=f"commanded ω (deg/s)  μ={μ(cmd_omega):.1f}", color="tab:orange")
ax2.plot(steps, cur_omega, label=f"current ω  (deg/s)  μ={μ(cur_omega):.1f}", color="tab:blue")
ax2.axhline(0, color="black", linewidth=0.7)
ax2.set_ylabel("deg/s")
ax2.legend()
ax2.grid(True, alpha=0.4)

# --- Panel 3: PI internals ---
ax3.plot(steps, [v * 10 for v in pi_out],    label="pi_out ×10 (m/s)", color="tab:purple")
ax3.plot(steps, [v * 10 for v in turn_speed], label="turn_speed ×10 (m/s)", color="tab:brown")
ax3.plot(steps, [v * 100 for v in integral],  label="integral ×100 (rad·s)", color="tab:green")
ax3.axhline(0, color="black", linewidth=0.7)
ax3.set_ylabel("mixed (scaled)")
ax3.set_xlabel("Step")
ax3.legend()
ax3.grid(True, alpha=0.4)

plt.tight_layout()
plt.show()
