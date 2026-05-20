"""
Paste StraightLine log output, press Ctrl+Z (Windows) or Ctrl+D (Linux/Mac) then Enter.
Non-StraightLine lines (battery, INFO, etc.) are silently skipped.

Resilient parser: each field is matched by its own regex against the line. Missing fields
are simply omitted from the row dict; the corresponding plot series is skipped at render time
instead of bailing out. New flash_log fields can be added without breaking the script.
"""

import re
import sys
import statistics
import matplotlib.pyplot as plt

# Anchors a line as "this is a StraightLine log entry" AND captures dist/total.
PREFIX_RE = re.compile(
    r"(?:\[\d+ms\] )?"
    r"StraightLine \((?P<dist>[\d.]+)/(?P<total>[\d.]+)m\):"
)

# Each entry is (column_name, regex pattern with one capturing group, type-cast).
# Patterns are matched independently on the line; any miss is just absent from the row.
FIELDS: list[tuple[str, str, type]] = [
    ("target",     r"target: (-?[\d.]+)",      float),
    ("current",    r"current: (-?[\d.]+)",     float),
    ("error",      r"error: (-?[\d.]+)",       float),
    ("commanded",  r"commanded: (-?[\d.]+)",   float),
    ("p_term",     r"\bp: (-?[\d.]+)",         float),
    ("i_term",     r"\bi: (-?[\d.]+)",         float),
    ("steer",      r"\bsteer: (-?[\d.]+)",     float),
    ("steer_p",    r"steer_p: (-?[\d.]+)",     float),
    ("steer_i",    r"steer_i: (-?[\d.]+)",     float),
    ("wall_steer", r"wall_steer: (-?[\d.]+)",  float),
    # L/M/R are now logged in meters (float, may be NaN). Convert to mm at use sites.
    ("l_m",        r"\bL: (-?[\d.]+|NaN)",     float),
    ("m_m",        r"\bM: (-?[\d.]+|NaN)",     float),
    ("r_m",        r"\bR: (-?[\d.]+|NaN)",     float),
    ("drift_mm",   r"drift_mm: (-?[\d.]+)",    float),
    ("hdg",        r"hdg: (-?[\d.]+)deg",      float),
    ("theta",      r"theta: (-?[\d.]+)deg",    float),
]
FIELDS_COMPILED = [(name, re.compile(pat), cast) for name, pat, cast in FIELDS]

DECEL_RE = re.compile(r"target: -?[\d.]+ \(decel\)")

print("Paste log data, then press Ctrl+Z + Enter (Windows) or Ctrl+D (Linux/Mac):")
raw_input = sys.stdin.read()

rows: list[dict] = []
for line in raw_input.splitlines():
    pm = PREFIX_RE.search(line)
    if not pm:
        continue
    d: dict = {
        "dist":  float(pm.group("dist")),
        "total": float(pm.group("total")),
        "decel": bool(DECEL_RE.search(line)),
    }
    for name, pat, cast in FIELDS_COMPILED:
        fm = pat.search(line)
        if fm:
            try:
                d[name] = cast(fm.group(1))
            except ValueError:
                pass
    rows.append(d)

if not rows:
    print("No StraightLine entries found.")
    sys.exit(1)

print(f"Parsed {len(rows)} entries.")


def nan_if_neg(v):
    return float(v) if v is not None and v >= 0 else float("nan")


def m_to_mm(v):
    """Convert meters → mm; pass NaN/None through as NaN, and treat negatives as invalid."""
    if v is None:
        return float("nan")
    f = float(v)
    if f != f or f < 0:  # NaN or negative
        return float("nan")
    return f * 1000.0


def col(name, default=None, transform=None):
    """Extract a column from rows. Returns None if NO row has the field."""
    if not any(name in r for r in rows):
        return None
    out = []
    for r in rows:
        v = r.get(name, default)
        if transform is not None and v is not None:
            v = transform(v)
        out.append(v if v is not None else float("nan"))
    return out


def σ(series):
    if series is None:
        return 0.0
    finite = [x for x in series if x == x]
    return statistics.stdev(finite) if len(finite) > 1 else 0.0


def mean(series):
    if series is None:
        return 0.0
    finite = [x for x in series if x == x]
    return statistics.mean(finite) if finite else 0.0


dist       = col("dist")
target     = col("target")
current    = col("current")
commanded  = col("commanded")
error      = col("error")
p_term     = col("p_term")
i_term     = col("i_term")
steer      = col("steer")
steer_p    = col("steer_p")
steer_i    = col("steer_i")
wall_steer = col("wall_steer")
hdg        = col("hdg")
l_mm       = col("l_m", transform=m_to_mm)
m_mm       = col("m_m", transform=m_to_mm)
r_mm       = col("r_m", transform=m_to_mm)
drift_mm   = col("drift_mm")
theta      = col("theta")

total = rows[0]["total"]
decel_start = next((r["dist"] for r in rows if r.get("decel")), None)
# Right edge of the decel shading. For `Distance` mode `total` IS the journey length, but for
# `DistanceToFrontWall` `total` is the target wall distance (small), so we use the last sample's
# distance instead — that's always the end of the run regardless of mode.
dist_end = max(dist) if dist else float(total)

fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 11), sharex=True)
fig.suptitle(f"StraightLine — {total} m goal, {dist_end:.2f} m travelled ({len(rows)} samples)")

if decel_start is not None:
    for ax in (ax1, ax2, ax3):
        ax.axvspan(decel_start, dist_end, alpha=0.07, color="orange", label="_decel zone")


def plot_if(ax, x, y, **kwargs):
    """Plot a series only if both x and y are present (non-None)."""
    if x is not None and y is not None:
        ax.plot(x, y, **kwargs)


# ── Speed panel ─────────────────────────────────────────────────────────────
plot_if(ax1, dist, target,    label="target speed",    linestyle="--", color="gray")
plot_if(ax1, dist, current,   label=f"current speed  (σ={σ(current):.3f}, μ={mean(current):.3f})",  color="tab:blue")
plot_if(ax1, dist, commanded, label=f"commanded speed (σ={σ(commanded):.3f}, μ={mean(commanded):.3f})", color="tab:orange")
plot_if(ax1, dist, error,     label=f"error      (σ={σ(error):.3f}, μ={mean(error):.3f})", color="tab:red")
plot_if(ax1, dist, p_term,    label="P term (m/s)", color="tab:purple")
plot_if(ax1, dist, i_term,    label="I term (m/s)", color="tab:green")
if decel_start is not None:
    ax1.axvline(decel_start, color="darkorange", linewidth=0.8, linestyle=":", label=f"decel start ({decel_start:.2f} m)")
ax1.set_ylabel("Speed (m/s)")
ax1.legend()
ax1.grid(True, alpha=0.4)

# ── Steering / PI panel ─────────────────────────────────────────────────────
plot_if(ax2, dist, [v * 10 for v in steer]      if steer      else None, label="steering ×10",  color="tab:brown")
plot_if(ax2, dist, [v * 10 for v in steer_p]    if steer_p    else None, label="steer_p ×10",   color="tab:cyan",   linestyle="--")
plot_if(ax2, dist, [v * 10 for v in steer_i]    if steer_i    else None, label="steer_i ×10",   color="tab:pink",   linestyle="--")
plot_if(ax2, dist, [v * 10 for v in wall_steer] if wall_steer else None, label=f"wall_steer ×10 (μ={mean(wall_steer):.3f})", color="tab:gray", linestyle="--")
plot_if(ax2, dist, [h / 10 for h in hdg]        if hdg        else None, label="heading error (deg / 10)", color="tab:olive")
plot_if(ax2, dist, [t / 10 for t in theta]      if theta      else None, label="theta (deg / 10)",          color="tab:red",   linestyle="-.")
ax2.axhline(0, color="black", linewidth=0.7)
ax2.set_ylabel("m/s")
ax2.legend(loc="upper right", fontsize=8)
ax2.grid(True, alpha=0.4)

# ── Wall-follow ToF panel ───────────────────────────────────────────────────
plot_if(ax3, dist, l_mm,  label=f"L  (μ={mean(l_mm):.0f} mm)",  color="tab:blue",   marker=".", linestyle="-", linewidth=0.8)
plot_if(ax3, dist, m_mm,  label=f"M  (μ={mean(m_mm):.0f} mm)",  color="tab:green",  marker=".", linestyle="-", linewidth=0.8)
plot_if(ax3, dist, r_mm,  label=f"R  (μ={mean(r_mm):.0f} mm)",  color="tab:orange", marker=".", linestyle="-", linewidth=0.8)
plot_if(ax3, dist, drift_mm, label=f"drift (mm, +=left) (σ={σ(drift_mm):.1f}, μ={mean(drift_mm):.1f})", color="tab:red", linewidth=1.2)
ax3.axhline(0, color="black", linewidth=0.7)
ax3.axhline(90, color="tab:blue", linewidth=2.0, linestyle=":", alpha=0.6, label="half cell size = 90 mm")
ax3.set_ylim(0, 300)
ax3.set_ylabel("Distance (mm) / drift (mm)")
ax3.set_xlabel("Distance (m)")
ax3.legend(loc="upper right", fontsize=8)
ax3.grid(True, alpha=0.4)

plt.tight_layout()
plt.show()
