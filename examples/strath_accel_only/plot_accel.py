#!/usr/bin/env python3
"""
Reads "x,y,z" acceleration lines from the strath_accel_only sketch over
serial, live-plots them, and logs every sample with a timestamp to a CSV.

Usage:
    pip install pyserial matplotlib
    python plot_accel.py [--port COM5] [--baud 115200] [--window 20] [--outfile accel_log.csv]

If --port is omitted, the script lists available serial ports and, if
exactly one is found, uses it automatically.
"""

import argparse
import csv
import math
import random
import sys
import time
from collections import deque
from datetime import datetime

import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Standing still still reads ~9.81 m/s^2 (gravity), so tiers are keyed off how far
# the magnitude strays from that baseline, not the raw magnitude. Thoroughly English gag text.
GRAVITY = 9.81

# Minimum seconds between phrase changes, so it doesn't flicker every sample.
PHRASE_UPDATE_INTERVAL = 2.5

VIBE_TIERS = [
    (0.0, ["just having a lie down", "calm as a vicar's tea party", "still as a garden gnome", "not a bother in sight"]),
    (1.5, ["bit of a wobble, nothing to write to the Queen about", "having a right old shuffle", "stirring like a proper cuppa", "getting a bit lively, innit"]),
    (4.0, ["blimey, someone's had too much fizzy pop", "proper kerfuffle happening", "shaking like a wet whippet", "gone completely bonkers, this one"]),
    (7.0, ["RIGHT THAT'S IT, RING THE QUEEN", "BLOODY HELL, FULL ENGLISH BREAKFAST OF CHAOS", "CALL 999, IT'S GONE COMPLETELY SPARE", "MAYDAY MAYDAY THE KETTLE'S BOILED OVER"]),
]


def vibe_check(deviation):
    caption = VIBE_TIERS[0][1][0]
    for threshold, phrases in VIBE_TIERS:
        if deviation >= threshold:
            caption = random.choice(phrases)
    return caption


def pick_port(explicit_port):
    if explicit_port:
        return explicit_port

    ports = list(serial.tools.list_ports.comports())
    if not ports:
        sys.exit("No serial ports found. Plug in the board or pass --port explicitly.")

    if len(ports) == 1:
        print(f"Auto-selected port: {ports[0].device} ({ports[0].description})")
        return ports[0].device

    print("Multiple serial ports found:")
    for p in ports:
        print(f"  {p.device} - {p.description}")
    sys.exit("Pass one explicitly with --port COMx")


def open_serial_with_retry(port, baud, attempts=5, delay_s=2.0):
    last_error = None
    for attempt in range(1, attempts + 1):
        try:
            return serial.Serial(port, baud, timeout=1)
        except serial.SerialException as e:
            last_error = e
            print(f"  Attempt {attempt}/{attempts}: could not open {port} ({e}).")
            if attempt < attempts:
                print("  Retrying in 2s... (make sure the Arduino IDE / Serial Monitor is fully closed)")
                time.sleep(delay_s)
    sys.exit(
        f"Giving up: could not open {port} after {attempts} attempts.\n"
        f"Last error: {last_error}\n"
        "Close the Arduino IDE (and any other program using this port), or unplug/replug the board, then retry."
    )


def parse_args():
    parser = argparse.ArgumentParser(description="Live-plot and log BNO08x acceleration data")
    parser.add_argument("--port", default=None, help="Serial port (e.g. COM5). Auto-detected if omitted.")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default 115200)")
    parser.add_argument("--window", type=float, default=20.0, help="Seconds of history shown on the live plot")
    parser.add_argument(
        "--outfile",
        default=None,
        help="CSV output path. Defaults to accel_log_<timestamp>.csv in the current directory.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    port = pick_port(args.port)

    outfile = args.outfile or f"accel_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

    print(f"Opening {port} @ {args.baud} baud")
    ser = open_serial_with_retry(port, args.baud)
    time.sleep(2)  # let the board finish resetting after the port opens
    ser.reset_input_buffer()

    csv_file = open(outfile, "w", newline="")
    writer = csv.writer(csv_file)
    writer.writerow(["timestamp", "elapsed_s", "x", "y", "z", "magnitude", "deviation"])
    print(f"Logging to {outfile}")

    start_time = None
    times = deque()
    xs, ys, zs = deque(), deque(), deque()
    latest_vibe = ""
    latest_deviation = 0.0
    latest_xyz = (0.0, 0.0, 0.0)
    last_phrase_time = -PHRASE_UPDATE_INTERVAL

    fig, ax = plt.subplots()
    line_x, = ax.plot([], [], label="X")
    line_y, = ax.plot([], [], label="Y")
    line_z, = ax.plot([], [], label="Z")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Acceleration (m/s^2)")
    ax.set_title("BNO08x Acceleration (live)")
    ax.legend(loc="upper right")
    ax.grid(True)

    vibe_text = ax.text(
        0.5, 1.08, "", transform=ax.transAxes,
        ha="center", va="bottom", fontsize=14, fontweight="bold", color="black",
    )
    xyz_text = ax.text(
        0.02, 0.97, "", transform=ax.transAxes,
        ha="left", va="top", fontsize=11, fontfamily="monospace",
        bbox=dict(boxstyle="round", facecolor="white", alpha=0.7, edgecolor="gray"),
    )

    def trim_window(now):
        while times and now - times[0] > args.window:
            times.popleft()
            xs.popleft()
            ys.popleft()
            zs.popleft()

    def read_available_samples():
        while ser.in_waiting:
            raw = ser.readline().decode("utf-8", errors="ignore").strip()
            if not raw:
                continue
            parts = raw.split(",")
            if len(parts) != 3:
                continue  # ignore setup/debug prints from the sketch
            try:
                x, y, z = (float(p) for p in parts)
            except ValueError:
                continue

            nonlocal start_time, latest_vibe, latest_deviation, latest_xyz, last_phrase_time
            now = time.monotonic()
            if start_time is None:
                start_time = now
            elapsed = now - start_time

            magnitude = math.sqrt(x * x + y * y + z * z)
            deviation = abs(magnitude - GRAVITY)
            latest_deviation = deviation
            latest_xyz = (x, y, z)
            if elapsed - last_phrase_time >= PHRASE_UPDATE_INTERVAL:
                latest_vibe = vibe_check(deviation)
                last_phrase_time = elapsed

            ts = datetime.now().isoformat(timespec="milliseconds")
            writer.writerow([ts, f"{elapsed:.3f}", x, y, z, f"{magnitude:.4f}", f"{deviation:.4f}"])
            csv_file.flush()

            times.append(elapsed)
            xs.append(x)
            ys.append(y)
            zs.append(z)
            trim_window(elapsed)

    def update(_frame):
        read_available_samples()
        line_x.set_data(times, xs)
        line_y.set_data(times, ys)
        line_z.set_data(times, zs)
        if times:
            ax.set_xlim(max(0, times[-1] - args.window), max(args.window, times[-1]))
            all_vals = list(xs) + list(ys) + list(zs)
            margin = 0.5
            ax.set_ylim(min(all_vals) - margin, max(all_vals) + margin)

            vibe_text.set_text(latest_vibe)
            if latest_deviation >= VIBE_TIERS[-1][0]:
                vibe_text.set_color("red")
                fig.patch.set_facecolor("#ffe5e5")
            else:
                vibe_text.set_color("black")
                fig.patch.set_facecolor("white")

            lx, ly, lz = latest_xyz
            xyz_text.set_text(f"X: {lx:+.2f}\nY: {ly:+.2f}\nZ: {lz:+.2f}")
        return line_x, line_y, line_z, vibe_text, xyz_text

    ani = FuncAnimation(fig, update, interval=50, cache_frame_data=False)

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        csv_file.close()
        print(f"Closed serial port and saved log to {outfile}")


if __name__ == "__main__":
    main()
