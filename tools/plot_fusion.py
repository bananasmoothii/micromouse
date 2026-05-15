"""
Paste combined log output (Fusion + InPlaceTurn lines), press Ctrl+Z (Windows) or Ctrl+D (Linux/Mac) then Enter.
Other lines are silently skipped.
"""

import re
import sys
import statistics
import matplotlib.pyplot as plt

FUSION_PATTERN = re.compile(
    r"(?:\[\d+ms\] )?"
    r"Fusion: gyro_d (?P<gyro_d>-?[\d.]+)deg, "
    r"odom_d (?P<odom_d>-?[\d.]+)deg, "
    r"mag (?P<mag>-?[\d.]+)deg, "
    r"theta (?P<theta>-?[\d.]+)deg"
)

TURN_PATTERN = re.compile(
    r"(?:\[\d+ms\] )?"
    r"InPlaceTurn: remaining (?P<remaining>-?[\d.]+)deg"
)

DONE_PATTERN = re.compile(
    r"(?:\[\d+ms\] )?"
    r"InPlaceTurn done: target (?P<target>-?[\d.]+)deg, final error (?P<final_error>-?[\d.]+)deg"
)

print("Paste log data, then press Ctrl+Z + Enter (Windows) or Ctrl+D (Linux/Mac):")
raw = sys.stdin.read()

fusion_rows = []
turn_remaining = {}  # step index → remaining (deg)
target_deg = None
final_error_deg = None

fusion_step = 0
for line in raw.splitlines():
    m = FUSION_PATTERN.search(line)
    if m:
        fusion_rows.append({k: float(v) for k, v in m.groupdict().items()})
        # Check if the same line also has an InPlaceTurn entry (unlikely, but safe)
        fusion_step += 1
        continue
    m = TURN_PATTERN.search(line)
    if m:
        turn_remaining[fusion_step] = float(m.group("remaining"))
        continue
    m = DONE_PATTERN.search(line)
    if m:
        target_deg = float(m.group("target"))
        final_error_deg = float(m.group("final_error"))

if not fusion_rows:
    print("No Fusion entries found.")
    sys.exit(1)

print(f"Parsed {len(fusion_rows)} fusion steps.")
if target_deg is not None:
    print(f"InPlaceTurn target: {target_deg:.1f}°  final error: {final_error_deg:.2f}°")

steps = list(range(len(fusion_rows)))
theta   = [r["theta"]   for r in fusion_rows]
gyro_d  = [r["gyro_d"]  for r in fusion_rows]
odom_d  = [r["odom_d"]  for r in fusion_rows]
mag     = [r["mag"]     for r in fusion_rows]

# Cumulative gyro and odometry (reconstructed from deltas)
gyro_cum = []
odom_cum = []
acc_g = 0.0
acc_o = 0.0
for g, o in zip(gyro_d, odom_d):
    acc_g += g
    acc_o += o
    gyro_cum.append(acc_g)
    odom_cum.append(acc_o)

title = f"Fusion debug — InPlaceTurn {target_deg:.1f}°" if target_deg is not None else "Fusion debug"
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
fig.suptitle(title)

# --- Panel 1: fused theta vs magnetometer vs InPlaceTurn rotated-so-far ---
ax1.plot(steps, theta, label="fused theta (deg)", color="tab:blue", linewidth=1.5)
ax1.plot(steps, mag, label="mag heading (deg)", color="tab:orange", linestyle="--", linewidth=1)
if turn_remaining:
    tr_steps = sorted(turn_remaining.keys())
    tr_vals  = [turn_remaining[s] for s in tr_steps]
    if target_deg is not None:
        rotated = [target_deg - v for v in tr_vals]
        ax1.plot(tr_steps, rotated, label="rotated so far (deg, from InPlaceTurn)", color="tab:green", linestyle=":", linewidth=1.5)
ax1.axhline(0, color="black", linewidth=0.7)
if target_deg is not None:
    ax1.axhline(target_deg, color="gray", linewidth=0.7, linestyle="--", label=f"target {target_deg:.1f}°")
ax1.set_ylabel("Degrees")
ax1.legend(fontsize=8)
ax1.grid(True, alpha=0.4)

# --- Panel 2: cumulative gyro vs cumulative odometry vs fused theta ---
ax2.plot(steps, theta,    label="fused theta (deg)",          color="tab:blue",   linewidth=1.5)
ax2.plot(steps, gyro_cum, label="cumulative gyro (deg)",      color="tab:red",    linestyle="--", linewidth=1)
ax2.plot(steps, odom_cum, label="cumulative odometry (deg)",  color="tab:purple", linestyle=":",  linewidth=1)
ax2.axhline(0, color="black", linewidth=0.7)
ax2.set_ylabel("Degrees")
ax2.legend(fontsize=8)
ax2.grid(True, alpha=0.4)

def σ(s): return statistics.stdev(s) if len(s) > 1 else 0.0

# --- Panel 3: per-step deltas ---
ax3.plot(steps, gyro_d, label=f"gyro d_theta/step (deg)  σ={σ(gyro_d):.2f}", color="tab:red",    linewidth=1)
ax3.plot(steps, odom_d, label=f"odom d_theta/step (deg)  σ={σ(odom_d):.2f}", color="tab:purple", linewidth=1)
ax3.axhline(0, color="black", linewidth=0.7)
ax3.set_ylabel("Degrees / step")
ax3.set_xlabel("Fusion step")
ax3.legend(fontsize=8)
ax3.grid(True, alpha=0.4)

plt.tight_layout()
plt.show()
