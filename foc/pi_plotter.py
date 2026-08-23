import re
import argparse
import sys
import time

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque

try:
    import serial
except ImportError:
    serial = None

# =========================================================
# Configuration
# =========================================================

DEFAULT_BAUD = 115200

# Matches:
# elapsed:1147.731079
# iq_ref:0.100000A
# [d:0.063280 q:0.130589]
PATTERN = re.compile(
    r"elapsed:([-\d.]+).*?"
    r"q_ref:([-\d.]+)A.*?"
    r"\[d:([-\d.]+)\s+q:([-\d.]+)\].*?"
    r"mod:([-\d.]+)"
)

# =========================================================
# Parse one log line
# =========================================================
def parse_line(line):
    match = PATTERN.search(line)

    if not match:
        return None

    elapsed = float(match.group(1))
    iq_ref = float(match.group(2))
    q = float(match.group(4))
    modulation = float(match.group(5))

    return elapsed, iq_ref, q, modulation
# =========================================================
# FILE MODE
# =========================================================
def read_file(filename):
    time_data = []
    setpoint = []
    process_variable = []

    print(f"Reading log file: {filename}")

    with open(filename, "r") as f:
        for line in f:
            result = parse_line(line)

            if result is None:
                continue

            t, sp, pv, mod = result

            time_data.append(t)
            setpoint.append(sp)
            process_variable.append(pv)

    print(f"Loaded {len(time_data)} valid samples")

    if not time_data:
        print("ERROR: No valid PI data found.")
        sys.exit(1)

    plot_static(time_data, setpoint, process_variable)
# =========================================================
# STATIC PLOT
# =========================================================
def plot_static(time_data, setpoint, process_variable):

    # Make time relative to first sample
    t0 = time_data[0]
    time_data = [t - t0 for t in time_data]

    plt.figure(figsize=(12, 6))

    plt.plot(
        time_data,
        setpoint,
        color="red",
        linewidth=2,
        label="Setpoint (iq_ref)"
    )

    plt.plot(
        time_data,
        process_variable,
        color="blue",
        linewidth=1.2,
        label="Process Variable (iq)"
    )

    plt.xlabel("Time [s]")
    plt.ylabel("q-axis Current [A]")

    plt.title("FOC q-axis Current PI Control")

    plt.grid(
        True,
        which="both",
        linestyle="--",
        alpha=0.4
    )

    plt.legend()

    plt.tight_layout()
    plt.show()
# =========================================================
# LIVE SERIAL MODE
# =========================================================
def live_serial(port, baud):

    if serial is None:
        print(
            "ERROR: pyserial is not installed.\n"
            "Install it with:\n\n"
            "    pip install pyserial"
        )
        sys.exit(1)

    print(f"Opening serial port: {port}")
    print(f"Baud rate: {baud}")

    try:
        ser = serial.Serial(
            port=port,
            baudrate=baud,
            timeout=0.05
        )
    except serial.SerialException as e:
        print(f"ERROR opening serial port: {e}")
        sys.exit(1)

    time_data = []
    setpoint = []
    process_variable = []

    # -----------------------------------------------------
    # Create live plot
    # -----------------------------------------------------

    fig, ax = plt.subplots(figsize=(12, 6))

    latest_modulation = [0.0]

    mod_text = ax.text(
        0.02,
        0.95,
        "Modulation: 0.000",
        transform=ax.transAxes,
        fontsize=14,
        fontweight="bold",
        color="green",
        verticalalignment="top",
        bbox=dict(
            boxstyle="round",
            facecolor="white",
            alpha=0.8
        )
    )

    line_sp, = ax.plot(
        [],
        [],
        color="red",
        linewidth=2,
        label="Setpoint (iq_ref)"
    )

    line_pv, = ax.plot(
        [],
        [],
        color="blue",
        linewidth=1.2,
        label="Process Variable (iq)"
    )

    ax.set_xlabel("Time [s]")
    ax.set_ylabel("q-axis Current [A]")
    ax.set_ylim(0, 0.5)

    ax.set_title(
        f"FOC q-axis Current PI Control - {port}"
    )

    ax.grid(
        True,
        which="both",
        linestyle="--",
        alpha=0.4
    )

    ax.legend()

    # -----------------------------------------------------
    # Serial reader
    # -----------------------------------------------------
    PV_AVG_SAMPLES = 10
    pv_window = deque(maxlen=PV_AVG_SAMPLES)
    pv_average = []

    def read_serial():
        """
        Read all currently available serial data.
        """

        while ser.in_waiting:

            try:
                raw = ser.readline()
                line = raw.decode(
                    "utf-8",
                    errors="ignore"
                ).strip()

            except Exception:
                continue

            result = parse_line(line)

            if result is None:
                continue

            t, sp, pv, mod = result
            latest_modulation[0] = mod
            time_data.append(t)
            setpoint.append(sp)

            # Running average of PV
            pv_window.append(pv)
            pv_avg = sum(pv_window) / len(pv_window)

            process_variable.append(pv_avg)

    # -----------------------------------------------------
    # Update plot
    # -----------------------------------------------------

    def update(frame):

        read_serial()

        mod_text.set_text(
            f"Modulation: {latest_modulation[0]:.3f}"
        )

        if not time_data:
            return line_sp, line_pv

        # Relative time
        t0 = time_data[0]

        x = [
            t - t0
            for t in time_data
        ]

        line_sp.set_data(
            x,
            setpoint
        )

        line_pv.set_data(
            x,
            process_variable
        )

        # -------------------------------------------------
        # Automatic axis scaling
        # -------------------------------------------------

        ax.relim()
        ax.autoscale_view()

        return line_sp, line_pv

    # -----------------------------------------------------
    # Run animation
    # -----------------------------------------------------

    ani = FuncAnimation(
        fig,
        update,
        interval=50,
        blit=False,
        cache_frame_data=False
    )

    try:
        plt.tight_layout()
        plt.show()

    except KeyboardInterrupt:
        pass

    finally:
        ser.close()
        print("\nSerial port closed.")


# =========================================================
# MAIN
# =========================================================

def main():

    parser = argparse.ArgumentParser(
        description=(
            "FOC PI q-axis current plotter. "
            "Read from a saved log file or live serial port."
        )
    )

    mode = parser.add_mutually_exclusive_group(
        required=True
    )

    mode.add_argument(
        "--file",
        "-f",
        help="Read saved log file"
    )

    mode.add_argument(
        "--port",
        "-p",
        help="Read live serial port"
    )

    parser.add_argument(
        "--baud",
        "-b",
        type=int,
        default=DEFAULT_BAUD,
        help=f"Serial baud rate (default: {DEFAULT_BAUD})"
    )

    args = parser.parse_args()

    if args.file:
        read_file(args.file)

    elif args.port:
        live_serial(
            args.port,
            args.baud
        )

if __name__ == "__main__":
    main()