"""
Histogram of Hall-sensor pulse widths (in cycles) from flash_log output.

Lines matched:
  [1234ms] L ok 12345
  [1234ms] L skip 42
  [1234ms] R ok 67890
  [1234ms] R skip 7
Other lines are silently ignored.

Usage:
  python tools/plot_pulse_histogram.py
  <paste the log dump>
  <Ctrl+Z + Enter on Windows, Ctrl+D on Linux/Mac>

Assumes SYSCLK = 84 MHz (CYCLES_PER_US = 84). Edit CYCLES_PER_US below if you change it.
"""

import re
import sys
import statistics
import matplotlib.pyplot as plt
import numpy as np

CYCLES_PER_US = 84  # STM32F446 @ 84 MHz

LINE_RE = re.compile(
    r"(?:\[\d+ms\]\s+)?(?P<side>[LR])\s+(?P<verdict>ok|skip)\s+(?P<cycles>\d+)"
)

print("Paste flash_log output, then press Ctrl+Z + Enter (Windows) or Ctrl+D (Linux/Mac):")
raw = sys.stdin.read()

samples = {"L": {"ok": [], "skip": []}, "R": {"ok": [], "skip": []}}
for line in raw.splitlines():
    m = LINE_RE.search(line)
    if not m:
        continue
    samples[m["side"]][m["verdict"]].append(int(m["cycles"]))

total = sum(len(v) for s in samples.values() for v in s.values())
if total == 0:
    print("No pulse-width samples found. Make sure flash_log lines like 'L ok 12345' are in the input.")
    sys.exit(1)


def summarize(label, data):
    if not data:
        print(f"  {label}: (none)")
        return
    us = [c / CYCLES_PER_US for c in data]
    print(
        f"  {label}: n={len(data):5d} "
        f"min={min(us):8.2f}µs  "
        f"median={statistics.median(us):8.2f}µs  "
        f"mean={statistics.mean(us):8.2f}µs  "
        f"max={max(us):10.2f}µs"
    )


print("\n=== Summary (µs) ===")
for side in ("L", "R"):
    print(f"{side} side:")
    summarize("ok  ", samples[side]["ok"])
    summarize("skip", samples[side]["skip"])

# --- Plot ---
fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
fig.suptitle(
    f"Hall pulse-width distribution  ({total} samples, CYCLES_PER_US={CYCLES_PER_US})"
)

for ax, side in zip(axes, ("L", "R")):
    ok = np.array(samples[side]["ok"], dtype=float)
    skip = np.array(samples[side]["skip"], dtype=float)
    combined = np.concatenate([ok, skip]) if (len(ok) + len(skip)) else np.array([1.0])
    combined = combined[combined > 0]
    if len(combined) == 0:
        ax.set_title(f"{side} side — no samples")
        continue

    lo = max(1.0, combined.min())
    hi = combined.max()
    bins = np.logspace(np.log10(lo), np.log10(hi * 1.05), 80)

    if len(skip):
        ax.hist(skip, bins=bins, color="tab:red",   alpha=0.6, label=f"skip (n={len(skip)})")
    if len(ok):
        ax.hist(ok,   bins=bins, color="tab:green", alpha=0.6, label=f"ok   (n={len(ok)})")

    # Vertical reference markers for "expected real pulse width" at a few speeds.
    # Assumes 12 ticks/rev, wheel radius 0.02 m → 1 tick of travel ≈ 10.5 mm.
    # Hall LOW duration ≈ (magnet arc / wheel circumference) × tick period.
    # Without knowing the magnet/sensor geometry exactly, just mark tick periods at common speeds.
    wheel_r = 0.02
    tick_dist = 2 * np.pi * wheel_r / 12  # ~10.5 mm
    for v_mps, label in [(0.3, "0.3 m/s"), (0.6, "0.6 m/s"), (1.0, "1.0 m/s")]:
        period_s = tick_dist / v_mps
        cycles = period_s * CYCLES_PER_US * 1e6
        ax.axvline(cycles, linestyle=":", color="gray", alpha=0.7)
        ax.text(cycles, ax.get_ylim()[1] * 0.9, f"  tick @ {label}",
                rotation=90, color="gray", fontsize=8, va="top")

    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_ylabel("count")
    ax.set_title(f"{side} side")
    ax.legend(loc="upper left")
    ax.grid(True, which="both", alpha=0.3)

# Secondary x-axis showing µs alongside cycles
ax_top = axes[0].twiny()
ax_top.set_xscale("log")
ax_top.set_xlim(axes[0].get_xlim())
# Build ticks at decade points
ticks = [10, 100, 1_000, 10_000, 100_000, 1_000_000, 10_000_000]
ax_top.set_xticks(ticks)
ax_top.set_xticklabels([f"{t / CYCLES_PER_US:.2f} µs" for t in ticks], fontsize=8)

axes[-1].set_xlabel("pulse width (cycles)  —  log scale")

plt.tight_layout()
plt.show()
