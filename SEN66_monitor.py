#!/usr/bin/env python3
"""
sen66_monitor.py — Live serial visualiser for SEN66 Phase 1 firmware
Requires: pip install pyserial matplotlib

Usage:
    python sen66_monitor.py              # auto-detects port
    python sen66_monitor.py COM3         # Windows
    python sen66_monitor.py /dev/ttyACM0 # Linux
"""

import sys
import argparse
import threading
import collections
import time
from datetime import datetime

import serial
import serial.tools.list_ports
import matplotlib
#matplotlib.use("TkAgg")          # explicit backend — avoids macOS animation stall
matplotlib.use("MacOSX")
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.gridspec import GridSpec

# ── Config ────────────────────────────────────────────────────────────────────
BAUD        = 115200
WINDOW      = 120        # seconds of history to show
POLL_MS     = 500        # plot refresh (ms) — 500 ms is plenty, reduces GUI load
HEADER      = "ts_ms,pm1.0,pm2.5,pm4.0,pm10,rh_%,temp_C,voc_idx,nox_idx,co2_ppm"
COLS        = HEADER.split(",")

# SEN66 PM saturation value (not the same as UINT16 sentinel 0xFFFF)
# 0xEB00 / 10 = 6016.0  — sensor is overwhelmed / obstructed
PM_SATURATED = 6016.0

COLORS = {
    "pm1.0":   "#4fc3f7",
    "pm2.5":   "#0288d1",
    "pm4.0":   "#01579b",
    "pm10":    "#b71c1c",
    "rh_%":    "#26a69a",
    "temp_C":  "#ef6c00",
    "voc_idx": "#7b1fa2",
    "nox_idx": "#c62828",
    "co2_ppm": "#2e7d32",
}

RANGES = {
    "pm1.0":   (0,    1999,  "µg/m³"),
    "pm2.5":   (0,    1999,  "µg/m³"),
    "pm4.0":   (0,    1999,  "µg/m³"),
    "pm10":    (0,    1999,  "µg/m³"),
    "rh_%":    (0,    100,   "%RH"),
    "temp_C":  (-20,  60,    "°C"),
    "voc_idx": (0,    500,   "idx"),
    "nox_idx": (0,    500,   "idx"),
    "co2_ppm": (300,  5000,  "ppm"),
}

# ── Data store ────────────────────────────────────────────────────────────────
maxlen   = WINDOW * 2
data     = {col: collections.deque(maxlen=maxlen) for col in COLS}
lock     = threading.Lock()
last_raw = {}
status_lines = collections.deque(maxlen=6)   # shown in status panel

# Console throttle — only print one [data] line per N seconds
_last_console_print = 0.0
CONSOLE_INTERVAL    = 2.0    # seconds between [data] summary prints


# ── Port detection ────────────────────────────────────────────────────────────
def find_pico_port():
    ports = serial.tools.list_ports.comports()
    for p in ports:
        if p.vid == 0x2E8A:
            print(f"[auto] Found Pico on {p.device}")
            return p.device
    if ports:
        print("Available ports:")
        for i, p in enumerate(ports):
            print(f"  [{i}] {p.device}  {p.description}")
        idx = input("Select port number: ").strip()
        return ports[int(idx)].device
    raise RuntimeError("No serial ports found. Is the Pico connected?")


# ── Serial reader thread ───────────────────────────────────────────────────────
def serial_reader(port):
    global _last_console_print
    try:
        ser = serial.Serial(port, BAUD, timeout=2)
        print(f"[serial] Opened {port} @ {BAUD}")
    except serial.SerialException as e:
        print(f"[ERR] Cannot open {port}: {e}")
        sys.exit(1)

    line_count   = 0
    parse_ok     = 0
    last_health  = time.time()

    while True:
        try:
            raw = ser.readline().decode("utf-8", errors="replace").strip()
        except Exception as e:
            print(f"[serial] readline exception: {e}")
            time.sleep(0.1)
            continue

        if not raw:
            continue

        line_count += 1
        ts_wall = datetime.now().strftime("%H:%M:%S")

        # Firmware debug/error lines
        if raw.startswith("[ERR]") or raw.startswith("==="):
            msg = f"{ts_wall}  {raw}"
            print(f"[fw] {raw}")
            with lock:
                status_lines.append(("err", msg))
            continue

        # Field count gate
        parts = raw.split(",")
        if len(parts) != len(COLS):
            continue
        if parts[0] == "ts_ms":
            continue

        # Parse
        parsed = {}
        ok = True
        for col, val in zip(COLS, parts):
            if val.strip() == "null":
                parsed[col] = None
            else:
                try:
                    parsed[col] = float(val)
                except ValueError:
                    parsed[col] = None
                    ok = False

        if not ok:
            continue

        # Detect PM saturation / obstruction
        pm_sat = any(
            parsed.get(c) is not None and abs(parsed[c] - PM_SATURATED) < 1.0
            for c in ["pm1.0", "pm2.5", "pm4.0", "pm10"]
        )

        # Store — treat saturated PM as None (gap in plot) so it doesn't
        # compress the y-axis and hide real data
        with lock:
            for col in COLS:
                v = parsed.get(col)
                if pm_sat and col in ("pm1.0", "pm2.5", "pm4.0", "pm10"):
                    data[col].append(None)
                else:
                    data[col].append(v)
                    if v is not None:
                        last_raw[col] = v

        parse_ok += 1

        # Build status message
        now = time.time()
        pm_note = "  [PM SATURATED — sensor obstructed?]" if pm_sat else ""
        co2_v   = parsed.get("co2_ppm")
        co2_note = "  [CO2 warming up]" if (co2_v is not None and co2_v < 300) else ""

        # Throttled console summary
        if now - _last_console_print >= CONSOLE_INTERVAL:
            _last_console_print = now
            with lock:
                lr = dict(last_raw)
            print(
                f"[{ts_wall}] "
                f"T={lr.get('temp_C','?'):>5}°C  "
                f"RH={lr.get('rh_%','?'):>5}%  "
                f"CO2={lr.get('co2_ppm','?'):>5}ppm{co2_note}  "
                f"VOC={lr.get('voc_idx','?'):>5}  "
                f"NOx={lr.get('nox_idx','?'):>4}  "
                f"PM2.5={lr.get('pm2.5','?'):>7}{pm_note}"
            )

        # Status panel update (always)
        with lock:
            level = "warn" if (pm_sat or co2_note) else "ok"
            msg = (
                f"{ts_wall}  "
                f"T={parsed.get('temp_C','?')}°C  "
                f"RH={parsed.get('rh_%','?')}%  "
                f"CO2={parsed.get('co2_ppm','?')}ppm  "
                f"VOC={parsed.get('voc_idx','?')}"
                + pm_note + co2_note
            )
            status_lines.append((level, msg))

        # Health every 30 s
        if now - last_health >= 30:
            print(f"[health] lines={line_count}  parsed_ok={parse_ok}  deque={len(data['ts_ms'])}")
            last_health = now


# ── Helpers ───────────────────────────────────────────────────────────────────
def get_xy(col):
    with lock:
        ts = list(data["ts_ms"])
        ys = list(data[col])
    if not ts:
        return [], []
    t0 = next((t for t in ts if t is not None), 0)
    xs = [(t - t0) / 1000.0 if t is not None else None for t in ts]
    return xs, ys


def split_none(xs, ys):
    segs_x, segs_y = [], []
    sx, sy = [], []
    for x, y in zip(xs, ys):
        if x is None or y is None:
            if sx:
                segs_x.append(sx); segs_y.append(sy)
            sx, sy = [], []
        else:
            sx.append(x); sy.append(y)
    if sx:
        segs_x.append(sx); segs_y.append(sy)
    return segs_x, segs_y


def fmt(col, decimals=1):
    v = last_raw.get(col)
    if v is None:
        return "—"
    if col in ("pm1.0","pm2.5","pm4.0","pm10") and abs(v - PM_SATURATED) < 1.0:
        return "SAT"
    return f"{v:.{decimals}f}"


# ── Plot ──────────────────────────────────────────────────────────────────────
def build_figure():
    fig = plt.figure(figsize=(14, 9), facecolor="#1e1e2e")
    try:
        fig.canvas.manager.set_window_title("SEN66 Live Monitor")
    except Exception:
        pass

    gs = GridSpec(3, 3, figure=fig, hspace=0.55, wspace=0.38,
                  left=0.07, right=0.97, top=0.88, bottom=0.08)

    axes = {
        "pm":   fig.add_subplot(gs[0, :2]),
        "co2":  fig.add_subplot(gs[0, 2]),
        "temp": fig.add_subplot(gs[1, 0]),
        "rh":   fig.add_subplot(gs[1, 1]),
        "voc":  fig.add_subplot(gs[1, 2]),
        "nox":  fig.add_subplot(gs[2, 2]),
        "stat": fig.add_subplot(gs[2, :2]),
    }
    for ax in axes.values():
        ax.set_facecolor("#13131f")
        ax.tick_params(colors="white", labelsize=8)
        for sp in ax.spines.values():
            sp.set_edgecolor("#444466")
        ax.yaxis.label.set_color("white")
        ax.xaxis.label.set_color("white")
        ax.title.set_color("white")
    axes["stat"].axis("off")
    return fig, axes


def style(ax):
    ax.set_facecolor("#13131f")
    ax.tick_params(colors="white", labelsize=8)
    for sp in ax.spines.values():
        sp.set_edgecolor("#444466")


def update(frame, axes, fig):
    # PM
    ax = axes["pm"]
    ax.cla(); style(ax)
    ax.set_title(
        f"Particulate Matter (µg/m³) — saturated readings hidden\n"
        f"PM1={fmt('pm1.0')}  PM2.5={fmt('pm2.5')}  PM4={fmt('pm4.0')}  PM10={fmt('pm10')}",
        fontsize=8, color="white")
    ax.set_xlabel("seconds", fontsize=7, color="#aaaacc")
    for col in ["pm1.0", "pm2.5", "pm4.0", "pm10"]:
        xs, ys = get_xy(col)
        for sx, sy in zip(*split_none(xs, ys)):
            ax.plot(sx, sy, color=COLORS[col], linewidth=1.4, label=col)
    handles, labels = ax.get_legend_handles_labels()
    seen = {}
    uh = [h for h, l in zip(handles, labels) if l not in seen and not seen.update({l: 1})]
    ax.legend(uh, list(seen), fontsize=7, facecolor="#2a2a3e",
              labelcolor="white", loc="upper left", framealpha=0.7)

    # CO2
    ax = axes["co2"]
    ax.cla(); style(ax)
    co2_note = " ⚠ warming" if (last_raw.get("co2_ppm", 999) < 300) else ""
    ax.set_title(f"CO₂  {fmt('co2_ppm', 0)} ppm{co2_note}", fontsize=9, color="white")
    ax.set_xlabel("seconds", fontsize=7, color="#aaaacc")
    xs, ys = get_xy("co2_ppm")
    for sx, sy in zip(*split_none(xs, ys)):
        ax.fill_between(sx, sy, alpha=0.25, color=COLORS["co2_ppm"])
        ax.plot(sx, sy, color=COLORS["co2_ppm"], linewidth=1.4)
    ax.axhline(1000, color="#ffcc02", linewidth=0.8, linestyle="--", alpha=0.6)
    ax.axhline(2000, color="#ff5252", linewidth=0.8, linestyle="--", alpha=0.6)

    # Temp
    ax = axes["temp"]
    ax.cla(); style(ax)
    ax.set_title(f"Temperature  {fmt('temp_C', 2)} °C", fontsize=9, color="white")
    ax.set_xlabel("seconds", fontsize=7, color="#aaaacc")
    xs, ys = get_xy("temp_C")
    for sx, sy in zip(*split_none(xs, ys)):
        ax.plot(sx, sy, color=COLORS["temp_C"], linewidth=1.4)

    # RH
    ax = axes["rh"]
    ax.cla(); style(ax)
    ax.set_title(f"Humidity  {fmt('rh_%')} %RH", fontsize=9, color="white")
    ax.set_xlabel("seconds", fontsize=7, color="#aaaacc")
    xs, ys = get_xy("rh_%")
    for sx, sy in zip(*split_none(xs, ys)):
        ax.fill_between(sx, sy, alpha=0.2, color=COLORS["rh_%"])
        ax.plot(sx, sy, color=COLORS["rh_%"], linewidth=1.4)
    ax.set_ylim(0, 100)

    # VOC
    ax = axes["voc"]
    ax.cla(); style(ax)
    ax.set_title(f"VOC Index  {fmt('voc_idx', 0)}", fontsize=9, color="white")
    ax.set_xlabel("seconds", fontsize=7, color="#aaaacc")
    xs, ys = get_xy("voc_idx")
    for sx, sy in zip(*split_none(xs, ys)):
        ax.plot(sx, sy, color=COLORS["voc_idx"], linewidth=1.4)
    ax.set_ylim(0, 500)

    # NOx
    ax = axes["nox"]
    ax.cla(); style(ax)
    ax.set_title(f"NOx Index  {fmt('nox_idx', 0)}", fontsize=9, color="white")
    ax.set_xlabel("seconds", fontsize=7, color="#aaaacc")
    xs, ys = get_xy("nox_idx")
    for sx, sy in zip(*split_none(xs, ys)):
        ax.plot(sx, sy, color=COLORS["nox_idx"], linewidth=1.4)
    ax.set_ylim(0, 500)

    # Status panel — last 5 lines, colour-coded
    ax = axes["stat"]
    ax.cla(); ax.axis("off")
    with lock:
        lines = list(status_lines)[-5:]
    ax.text(0.0, 1.0, "Recent readings", transform=ax.transAxes,
            color="#888899", fontsize=7, va="top")
    for i, (level, msg) in enumerate(reversed(lines)):
        color = {"ok": "#aaffaa", "warn": "#ffdd55", "err": "#ff6666"}.get(level, "white")
        ax.text(0.0, 0.85 - i * 0.17, msg, transform=ax.transAxes,
                color=color, fontsize=7, fontfamily="monospace", va="top")

    fig.suptitle(
        f"SEN66 Live Monitor  ·  {datetime.now():%H:%M:%S}  "
        f"·  T={fmt('temp_C',2)}°C  RH={fmt('rh_%')}%  CO₂={fmt('co2_ppm',0)}ppm  "
        f"VOC={fmt('voc_idx',0)}  NOx={fmt('nox_idx',0)}",
        color="white", fontsize=9, y=0.97
    )


# ── Entry point ───────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="SEN66 live serial visualiser")
    parser.add_argument("port", nargs="?", default=None)
    parser.add_argument("--baud", type=int, default=BAUD)
    args = parser.parse_args()

    port = args.port or find_pico_port()

    t = threading.Thread(target=serial_reader, args=(port,), daemon=True)
    t.start()
    time.sleep(0.5)   # let thread open port before building figure

    fig, axes = build_figure()
    ani = animation.FuncAnimation(
        fig, update, fargs=(axes, fig),
        interval=POLL_MS, cache_frame_data=False, blit=False
    )

    plt.show()


if __name__ == "__main__":
    main()