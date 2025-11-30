
#!/usr/bin/env python3
"""
live_tail_plot.py
iwconfig wlo1

Live monitor of capture.iq file (lines: "x,y").
- Reads only appended lines.
- Updates plot live.
- Stop monitoring by typing "e" + Enter.
"""

import time
import os
import sys
import select
from collections import deque

import matplotlib.pyplot as plt
import pandas as pd


FNAME = "capture.iq"       # <--- fixed filename
BUFFER_SIZE = 1000
POLL_INTERVAL = 0.2        # seconds


def parse_line(line):
    """Parse 'x,y' -> (float, float)."""
    if not line:
        return None
    line = line.strip()
    if not line:
        return None
    parts = line.split(",")
    if len(parts) < 2:
        return None
    try:
        x = float(parts[0])
        y = float(parts[1])
        return x, y
    except ValueError:
        return None


def looks_like_epoch(xs):
    if not xs:
        return False
    count = sum(1 for v in xs if v >= 1e9)
    return count > 0.6 * len(xs)


def tail_file_new_lines(f):
    """Yield new appended lines."""
    while True:
        line = f.readline()
        if not line:
            break
        yield line


def run_live_plot():
    if not os.path.exists(FNAME):
        print(f"File not found: {FNAME}")
        sys.exit(1)

    xs = deque(maxlen=BUFFER_SIZE)
    ys = deque(maxlen=BUFFER_SIZE)

    with open(FNAME, "r") as f:
        f.seek(0, os.SEEK_END)

        plt.ion()
        fig, ax = plt.subplots(figsize=(10, 4))
        line, = ax.plot([], [], marker="o", linestyle="-")
        ax.set_xlabel("x")
        ax.set_ylabel("value")
        ax.set_title(f"Live monitor: {FNAME}")
        ax.grid(True)

        epoch_mode = False

        try:
            while True:

                any_new = False
                for raw in tail_file_new_lines(f):
                    parsed = parse_line(raw)
                    if parsed:
                        x, y = parsed
                        xs.append(x)
                        ys.append(y)
                        any_new = True

                if any_new and xs:
                    x_arr = list(xs)
                    y_arr = list(ys)

                    try:
                        epoch_mode = looks_like_epoch(x_arr)

                        if epoch_mode:
                            x_dt = pd.to_datetime(x_arr, unit="s")
                            line.set_data(x_dt, y_arr)
                            ax.set_xlim(x_dt[0], x_dt[-1])
                            fig.autofmt_xdate()
                        else:
                            line.set_data(x_arr, y_arr)
                            ax.set_xlim(min(x_arr), max(x_arr))

                        if len(y_arr) > 1:
                            ymin = min(y_arr)
                            ymax = max(y_arr)
                            yr = ymax - ymin
                            if yr == 0:
                                yr = 1
                            ax.set_ylim(ymin - 0.1*yr, ymax + 0.1*yr)
                        else:
                            ax.set_ylim(y_arr[0]-1, y_arr[0]+1)

                        fig.canvas.draw()
                        fig.canvas.flush_events()

                    except Exception as e:
                        print(f"[plot error] {e}", file=sys.stderr)

                # check for exit command
                if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                    cmd = sys.stdin.readline().strip()
                    if cmd.lower() == "e":
                        print("Exiting monitor.")
                        break

                time.sleep(POLL_INTERVAL)

        except KeyboardInterrupt:
            print("Exiting (Ctrl+C).")
        finally:
            plt.ioff()
            try:
                fig.savefig("capture_live_last_snapshot.png")
            except:
                pass
            plt.close(fig)


if __name__ == "__main__":
    run_live_plot()
