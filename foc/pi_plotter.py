import re
import argparse
import sys

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button

from collections import deque

try:
    import serial
except ImportError:
    serial = None


# =========================================================
# Configuration
# =========================================================

DEFAULT_BAUD = 115200


# Supports old logs and new logs:
#
# Old:
# elapsed:x q_ref:xA [d:x q:x] mod:x
#
# New:
# elapsed:x q_ref:xA [d:x q:x] mod:x s_ref:x s_mes:x
#

PATTERN = re.compile(
    r"elapsed:([-\d.]+).*?"
    r"s_ref:([-\d.]+).*?"
    r"s_mes:([-\d.]+).*?"
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

    s_ref = float(match.group(2))
    s_mes = float(match.group(3))

    iq_ref = float(match.group(4))

    d = float(match.group(5))
    q = float(match.group(6))

    modulation = float(match.group(7))


    return (
        elapsed,
        iq_ref,
        d,
        q,
        modulation,
        s_ref,
        s_mes
    )


# =========================================================
# FILE MODE
# =========================================================

def read_file(filename):

    time_data = []

    iq_setpoint = []
    iq_current = []

    d_setpoint = []
    d_current = []

    s_setpoint = []
    s_current = []


    print(f"Reading log file: {filename}")


    with open(filename, "r") as f:

        for line in f:

            result = parse_line(line)

            if result is None:
                continue


            (
                t,
                iq_ref,
                d,
                q,
                mod,
                s_ref,
                s_mes
            ) = result


            time_data.append(t)

            iq_setpoint.append(iq_ref)
            iq_current.append(q)

            d_setpoint.append(0.0)
            d_current.append(d)


            if s_ref is not None:

                s_setpoint.append(s_ref)
                s_current.append(s_mes)


            else:

                s_setpoint.append(0.0)
                s_current.append(0.0)



    print(f"Loaded {len(time_data)} valid samples")


    if not time_data:

        print("ERROR: No valid PI data found.")
        sys.exit(1)


    plot_static(
        time_data,
        iq_setpoint,
        iq_current,
        d_setpoint,
        d_current,
        s_setpoint,
        s_current
    )



# =========================================================
# STATIC PLOT
# =========================================================

def plot_static(
        time_data,
        iq_setpoint,
        iq_current,
        d_setpoint,
        d_current,
        s_setpoint,
        s_current):


    t0 = time_data[0]


    time_data = [
        t - t0
        for t in time_data
    ]



    fig, axs = plt.subplots(
        3,
        1,
        figsize=(12, 10),
        sharex=True
    )



    # -----------------------------------------------------
    # q axis
    # -----------------------------------------------------

    axs[0].plot(
        time_data,
        iq_setpoint,
        color="red",
        linewidth=2,
        label="iq_ref"
    )


    axs[0].plot(
        time_data,
        iq_current,
        color="blue",
        linewidth=1.2,
        label="iq"
    )


    axs[0].set_ylabel(
        "q-axis Current [A]"
    )


    axs[0].set_title(
        "FOC q-axis Current Control"
    )


    axs[0].grid(
        True,
        linestyle="--",
        alpha=0.4
    )


    axs[0].legend()



    # -----------------------------------------------------
    # d axis
    # -----------------------------------------------------

    axs[1].plot(
        time_data,
        d_setpoint,
        color="red",
        linewidth=2,
        label="id_ref = 0"
    )


    axs[1].plot(
        time_data,
        d_current,
        color="green",
        linewidth=1.2,
        label="id"
    )


    axs[1].set_ylabel(
        "d-axis Current [A]"
    )


    axs[1].set_title(
        "FOC d-axis Current Regulation"
    )


    axs[1].grid(
        True,
        linestyle="--",
        alpha=0.4
    )


    axs[1].legend()



    # -----------------------------------------------------
    # S axis
    # -----------------------------------------------------

    axs[2].plot(
        time_data,
        s_setpoint,
        color="red",
        linewidth=2,
        label="s_ref"
    )


    axs[2].plot(
        time_data,
        s_current,
        color="purple",
        linewidth=1.2,
        label="s_mes"
    )


    axs[2].set_xlabel(
        "Time [s]"
    )


    axs[2].set_ylabel(
        "S"
    )


    axs[2].set_title(
        "S-axis PI Control"
    )


    axs[2].grid(
        True,
        linestyle="--",
        alpha=0.4
    )


    axs[2].legend()

    axs[2].set_ylim(
        min(min(s_setpoint), min(s_current)) - 0.2,
        max(max(s_setpoint), max(s_current)) + 0.2
    )


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

        print(
            f"ERROR opening serial port: {e}"
        )

        sys.exit(1)



    time_data = []

    setpoint = []
    process_variable = []

    d_setpoint = []
    d_current = []

    s_setpoint = []
    s_current = []



    # -----------------------------------------------------
    # Create live plot
    # -----------------------------------------------------

    fig, (ax_q, ax_d, ax_s) = plt.subplots(
        3,
        1,
        figsize=(12, 10),
        sharex=True
    )



    latest_modulation = [0.0]



    mod_text = ax_q.text(
        0.02,
        0.95,
        "Modulation: 0.000",
        transform=ax_q.transAxes,
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



    # -----------------------------------------------------
    # q-axis plot
    # -----------------------------------------------------

    line_sp, = ax_q.plot(
        [],
        [],
        color="red",
        linewidth=2,
        label="Setpoint (iq_ref)"
    )


    line_pv, = ax_q.plot(
        [],
        [],
        color="blue",
        linewidth=1.2,
        label="Process Variable (iq)"
    )


    ax_q.set_ylabel(
        "q-axis Current [A]"
    )


    ax_q.set_title(
        f"FOC Current PI Control - {port}"
    )


    ax_q.grid(
        True,
        linestyle="--",
        alpha=0.4
    )


    ax_q.legend()



    # -----------------------------------------------------
    # d-axis plot
    # -----------------------------------------------------

    line_dref, = ax_d.plot(
        [],
        [],
        color="red",
        linewidth=2,
        label="Setpoint (id_ref)"
    )


    line_d, = ax_d.plot(
        [],
        [],
        color="green",
        linewidth=1.2,
        label="Process Variable (id)"
    )


    ax_d.set_ylabel(
        "d-axis Current [A]"
    )


    ax_d.set_title(
        "FOC d-axis Current Regulation"
    )


    ax_d.grid(
        True,
        linestyle="--",
        alpha=0.4
    )


    ax_d.legend()



    # -----------------------------------------------------
    # S-axis plot
    # -----------------------------------------------------

    line_sref, = ax_s.plot(
        [],
        [],
        color="red",
        linewidth=2,
        label="Setpoint (s_ref)"
    )


    line_smes, = ax_s.plot(
        [],
        [],
        color="purple",
        linewidth=1.2,
        label="Measured (s_mes)"
    )


    ax_s.set_xlabel(
        "Time [s]"
    )


    ax_s.set_ylabel(
        "S"
    )


    ax_s.set_title(
        "S-axis PI Control"
    )


    ax_s.grid(
        True,
        linestyle="--",
        alpha=0.4
    )


    ax_s.legend()

    ax_s.set_ylim(
        -2,
        2
    )

    # -----------------------------------------------------
    # Serial reader
    # -----------------------------------------------------

    PV_AVG_SAMPLES = 10

    q_window = deque(
        maxlen=PV_AVG_SAMPLES
    )

    d_window = deque(
        maxlen=PV_AVG_SAMPLES
    )

    s_window = deque(
        maxlen=PV_AVG_SAMPLES
    )

    # -----------------------------------------------------
    # RESET BUTTON
    # -----------------------------------------------------

    reset_ax = fig.add_axes(
        [0.85, 0.93, 0.10, 0.04]
    )

    reset_button = Button(
        reset_ax,
        "RESET",
        color="lightgray",
        hovercolor="red"
    )


    def reset_data(event):

        # Clear all recorded data
        time_data.clear()

        setpoint.clear()
        process_variable.clear()

        d_setpoint.clear()
        d_current.clear()

        s_setpoint.clear()
        s_current.clear()


        # Clear filter history
        q_window.clear()
        d_window.clear()
        s_window.clear()


        # Reset modulation
        latest_modulation[0] = 0.0


        # Clear plotted lines
        line_sp.set_data([], [])
        line_pv.set_data([], [])

        line_dref.set_data([], [])
        line_d.set_data([], [])

        line_sref.set_data([], [])
        line_smes.set_data([], [])


        # Reset modulation display
        mod_text.set_text(
            "Modulation: 0.000"
        )


        # Reset axes
        ax_q.relim()
        ax_q.autoscale_view()

        ax_d.relim()
        ax_d.autoscale_view()

        ax_s.set_ylim(
            -2,
            2
        )


        # Redraw
        fig.canvas.draw_idle()


    reset_button.on_clicked(
        reset_data
    )

    def read_serial():

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



            (
                t,
                iq_ref,
                d,
                q,
                mod,
                s_ref,
                s_mes
            ) = result



            latest_modulation[0] = mod



            time_data.append(t)



            setpoint.append(
                iq_ref
            )



            # q-axis filtering

            q_window.append(q)

            q_avg = (
                sum(q_window)
                /
                len(q_window)
            )

            process_variable.append(
                q_avg
            )



            # d-axis filtering

            d_window.append(d)

            d_avg = (
                sum(d_window)
                /
                len(d_window)
            )

            d_current.append(
                d_avg
            )

            d_setpoint.append(
                0.0
            )



            # S-axis

            if s_ref is not None:

                s_setpoint.append(
                    s_ref
                )


                # -----------------------------
                # S-axis speed filtering
                # -----------------------------

                s_window.append(
                    s_mes
                )


                s_avg = (
                    sum(s_window)
                    /
                    len(s_window)
                )


                s_current.append(
                    s_avg
                )


            else:

                s_setpoint.append(
                    0.0
                )

                s_current.append(
                    0.0
                )



    # -----------------------------------------------------
    # Update plot
    # -----------------------------------------------------

    def update(frame):

        read_serial()



        mod_text.set_text(
            f"Modulation: {latest_modulation[0]:.3f}"
        )



        if not time_data:

            return (
                line_sp,
                line_pv,
                line_dref,
                line_d,
                line_sref,
                line_smes
            )



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


        line_dref.set_data(
            x,
            d_setpoint
        )


        line_d.set_data(
            x,
            d_current
        )


        line_sref.set_data(
            x,
            s_setpoint
        )


        line_smes.set_data(
            x,
            s_current
        )



        ax_q.relim()
        ax_q.autoscale_view()


        ax_d.relim()
        ax_d.autoscale_view()


        if s_setpoint and s_current:

            ymin = min(
                min(s_setpoint),
                min(s_current)
            )

            ymax = max(
                max(s_setpoint),
                max(s_current)
            )

            margin = 0.2

            ax_s.set_ylim(
                ymin - margin,
                ymax + margin
            )



        return (
            line_sp,
            line_pv,
            line_dref,
            line_d,
            line_sref,
            line_smes
        )



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
            "FOC PI current plotter. "
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

        read_file(
            args.file
        )



    elif args.port:

        live_serial(
            args.port,
            args.baud
        )




if __name__ == "__main__":

    main()