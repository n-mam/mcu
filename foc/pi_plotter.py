import re
import threading
import queue
import tkinter as tk
from collections import deque

import serial
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


# ============================================================
# SETTINGS
# ============================================================

COM_PORT = "COM9"
BAUD_RATE = 115200

MAX_POINTS = 2000
UPDATE_MS = 100

DEFAULT_D_MIN = -1.0
DEFAULT_D_MAX = 1.0

DEFAULT_Q_MIN = -1.0
DEFAULT_Q_MAX = 1.0

SPEED_MIN = -10.0
SPEED_MAX = 10.0

ENABLE_DQ_FILTER = True

# 0.0 = very smooth / slow response
# 1.0 = no filtering
DQ_FILTER_ALPHA = 0.2

# ============================================================
# GLOBALS
# ============================================================

data_queue = queue.Queue()

stop_event = threading.Event()

serial_port = None
after_id = None

elapsed_data = deque(maxlen=MAX_POINTS)

d_data = deque(maxlen=MAX_POINTS)
d_ref_data = deque(maxlen=MAX_POINTS)

q_data = deque(maxlen=MAX_POINTS)
q_ref_data = deque(maxlen=MAX_POINTS)

s_mes_data = deque(maxlen=MAX_POINTS)
s_ref_data = deque(maxlen=MAX_POINTS)

latest_d = 0.0
latest_q = 0.0
latest_s = 0.0
latest_mod = 0.0

filtered_d = 0.0
filtered_q = 0.0

# ============================================================
# CURRENT AXIS RANGES
# ============================================================

d_min = DEFAULT_D_MIN
d_max = DEFAULT_D_MAX

q_min = DEFAULT_Q_MIN
q_max = DEFAULT_Q_MAX


# ============================================================
# SERIAL LOG PARSER
# ============================================================

pattern = re.compile(
    r"theta:(?P<theta>[-+0-9.eE]+)\s+"
    r"elapsed:(?P<elapsed>[-+0-9.eE]+)\s+"
    r"s_ref:(?P<s_ref>[-+0-9.eE]+)\s+"
    r"s_mes:(?P<s_mes>[-+0-9.eE]+)\s+"
    r"\[ia:(?P<ia>[-+0-9.eE]+)\s+"
    r"ib:(?P<ib>[-+0-9.eE]+)\s+"
    r"ic:(?P<ic>[-+0-9.eE]+)\]\s+"
    r"q_ref:(?P<q_ref>[-+0-9.eE]+)A\s+"
    r"\[d:(?P<d>[-+0-9.eE]+)\s+"
    r"q:(?P<q>[-+0-9.eE]+)\]\s+"
    r"vd:(?P<vd>[-+0-9.eE]+)\s+"
    r"vq:(?P<vq>[-+0-9.eE]+)\s+"
    r"mod:(?P<mod>[-+0-9.eE]+)\s+"
    r"d_i:(?P<d_i>[-+0-9.eE]+)\s+"
    r"q_i:(?P<q_i>[-+0-9.eE]+)\s+"
    r"s_i:(?P<s_i>[-+0-9.eE]+)"
)

# ============================================================
# EMA FILTER
# ============================================================

def ema_filter(previous, new):

    return (
        DQ_FILTER_ALPHA * new +
        (1.0 - DQ_FILTER_ALPHA) * previous
    )

# ============================================================
# SERIAL READER
# ============================================================

def serial_reader():

    global serial_port

    try:

        serial_port = serial.Serial(
            COM_PORT,
            BAUD_RATE,
            timeout=0.2
        )

        print(f"Connected to {COM_PORT} @ {BAUD_RATE}")

    except Exception as e:

        print("Could not open serial port:")
        print(e)

        return

    try:

        while not stop_event.is_set():

            try:

                line = serial_port.readline().decode(
                    "utf-8",
                    errors="ignore"
                ).strip()

            except Exception:

                break

            if not line:
                continue

            match = pattern.search(line)

            if match:

                values = {
                    key: float(value)
                    for key, value in match.groupdict().items()
                }

                data_queue.put(values)

    finally:

        try:

            if serial_port is not None:
                serial_port.close()

        except Exception:
            pass

        serial_port = None

        print("Serial port closed.")


# ============================================================
# CHOOSE NICE TICK SPACING
# ============================================================

def choose_tick_spacing(axis_min, axis_max):

    """
    Automatically choose a reasonable tick spacing based
    on the selected axis range.
    """

    span = abs(axis_max - axis_min)

    if span <= 0:
        return 0.1

    # Target approximately 10 major divisions
    raw = span / 10.0

    # Nice values
    nice_values = [
        0.0001,
        0.0002,
        0.0005,
        0.001,
        0.002,
        0.005,
        0.01,
        0.02,
        0.05,
        0.1,
        0.2,
        0.5,
        1.0,
        2.0,
        5.0,
        10.0,
        20.0,
        50.0,
        100.0
    ]

    for value in nice_values:

        if raw <= value:
            return value

    return nice_values[-1]


# ============================================================
# UPDATE AXIS TICKS
# ============================================================

def update_axis_ticks(ax, axis_min, axis_max):

    spacing = choose_tick_spacing(
        axis_min,
        axis_max
    )

    start = axis_min

    # Generate ticks
    ticks = []

    value = start

    max_ticks = 100

    count = 0

    while value <= axis_max + spacing * 0.001:

        ticks.append(value)

        value += spacing

        count += 1

        if count >= max_ticks:
            break

    ax.set_yticks(ticks)


# ============================================================
# APPLY D/Q RANGE
# ============================================================

def apply_axis_ranges():

    global d_min
    global d_max
    global q_min
    global q_max

    try:

        new_d_min = float(
            d_min_entry.get()
        )

        new_d_max = float(
            d_max_entry.get()
        )

        new_q_min = float(
            q_min_entry.get()
        )

        new_q_max = float(
            q_max_entry.get()
        )

    except ValueError:

        status_label.config(
            text="Invalid range",
            fg="red"
        )

        return

    # Validate D

    if new_d_min >= new_d_max:

        status_label.config(
            text="D min must be smaller than D max",
            fg="red"
        )

        return

    # Validate Q

    if new_q_min >= new_q_max:

        status_label.config(
            text="Q min must be smaller than Q max",
            fg="red"
        )

        return

    # Save

    d_min = new_d_min
    d_max = new_d_max

    q_min = new_q_min
    q_max = new_q_max

    # Apply D

    ax_d.set_ylim(
        d_min,
        d_max
    )

    update_axis_ticks(
        ax_d,
        d_min,
        d_max
    )

    # Apply Q

    ax_q.set_ylim(
        q_min,
        q_max
    )

    update_axis_ticks(
        ax_q,
        q_min,
        q_max
    )

    canvas.draw_idle()

    status_label.config(
        text="Range applied",
        fg="green"
    )


# ============================================================
# DEFAULT AXIS RANGES
# ============================================================

def default_axis_ranges():

    global d_min
    global d_max
    global q_min
    global q_max

    d_min = DEFAULT_D_MIN
    d_max = DEFAULT_D_MAX

    q_min = DEFAULT_Q_MIN
    q_max = DEFAULT_Q_MAX

    d_min_entry.delete(0, tk.END)
    d_min_entry.insert(0, "-1")

    d_max_entry.delete(0, tk.END)
    d_max_entry.insert(0, "1")

    q_min_entry.delete(0, tk.END)
    q_min_entry.insert(0, "-1")

    q_max_entry.delete(0, tk.END)
    q_max_entry.insert(0, "1")

    ax_d.set_ylim(
        d_min,
        d_max
    )

    ax_q.set_ylim(
        q_min,
        q_max
    )

    update_axis_ticks(
        ax_d,
        d_min,
        d_max
    )

    update_axis_ticks(
        ax_q,
        q_min,
        q_max
    )

    canvas.draw_idle()

    status_label.config(
        text="Default range restored",
        fg="green"
    )


# ============================================================
# RESET PLOT DATA
# ============================================================

def reset_plots():

    global latest_d
    global latest_q
    global latest_s
    global latest_mod
    global filtered_d
    global filtered_q

    filtered_d = 0.0
    filtered_q = 0.0

    elapsed_data.clear()

    d_data.clear()
    d_ref_data.clear()

    q_data.clear()
    q_ref_data.clear()

    s_mes_data.clear()
    s_ref_data.clear()

    latest_d = 0.0
    latest_q = 0.0
    latest_s = 0.0
    latest_mod = 0.0

    d_value_label.config(
        text="d: 0.0000 A"
    )

    q_value_label.config(
        text="q: 0.0000 A"
    )

    s_value_label.config(
        text="Speed: 0.0000"
    )

    modulation_label.config(
        text="Modulation: 0.0000"
    )

    d_line.set_data([], [])
    d_ref_line.set_data([], [])

    q_line.set_data([], [])
    q_ref_line.set_data([], [])

    s_line.set_data([], [])
    s_ref_line.set_data([], [])

    canvas.draw_idle()

    status_label.config(
        text="Data reset",
        fg="green"
    )


# ============================================================
# PROCESS DATA
# ============================================================

def process_data():

    global latest_d
    global latest_q
    global latest_s
    global latest_mod

    while True:

        try:

            values = data_queue.get_nowait()

        except queue.Empty:

            break

        # ----------------------------------------------------
        # TIME
        # ----------------------------------------------------

        elapsed_data.append(
            values["elapsed"]
        )

        # ----------------------------------------------------
        # D
        # ----------------------------------------------------

        global filtered_d

        if ENABLE_DQ_FILTER:

            filtered_d = ema_filter(
                filtered_d,
                values["d"]
            )

            latest_d = filtered_d

        else:

            latest_d = values["d"]


        d_data.append(
            latest_d
        )

        # ----------------------------------------------------
        # D REF
        #
        # Current COM log has no d_ref.
        # Therefore d_ref is assumed to be zero.
        # ----------------------------------------------------

        d_ref_data.append(
            0.0
        )

        # ----------------------------------------------------
        # Q
        #
        # q_ref comes directly from the COM log.
        # Nothing is hard-coded.
        # ----------------------------------------------------

        global filtered_q

        if ENABLE_DQ_FILTER:

            filtered_q = ema_filter(
                filtered_q,
                values["q"]
            )

            latest_q = filtered_q

        else:

            latest_q = values["q"]


        q_data.append(
            latest_q
        )

        q_ref_data.append(
            values["q_ref"]
        )

        # ----------------------------------------------------
        # SPEED
        # ----------------------------------------------------

        latest_s = values["s_mes"]

        s_mes_data.append(
            values["s_mes"]
        )

        s_ref_data.append(
            values["s_ref"]
        )

        # ----------------------------------------------------
        # MODULATION
        # ----------------------------------------------------

        latest_mod = values["mod"]

    # --------------------------------------------------------
    # UPDATE LIVE VALUES
    # --------------------------------------------------------

    d_value_label.config(
        text=f"d: {latest_d:+.4f} A"
    )

    q_value_label.config(
        text=f"q: {latest_q:+.4f} A"
    )

    s_value_label.config(
        text=f"Speed: {latest_s:+.4f}"
    )

    modulation_label.config(
        text=f"Modulation: {latest_mod:.4f}"
    )


# ============================================================
# UPDATE PLOTS
# ============================================================

def update_plots():

    global after_id

    if stop_event.is_set():
        return

    process_data()

    x = list(elapsed_data)

    # --------------------------------------------------------
    # D
    # --------------------------------------------------------

    d_line.set_data(
        x,
        list(d_data)
    )

    d_ref_line.set_data(
        x,
        list(d_ref_data)
    )

    # --------------------------------------------------------
    # Q
    # --------------------------------------------------------

    q_line.set_data(
        x,
        list(q_data)
    )

    q_ref_line.set_data(
        x,
        list(q_ref_data)
    )

    # --------------------------------------------------------
    # SPEED
    # --------------------------------------------------------

    s_line.set_data(
        x,
        list(s_mes_data)
    )

    s_ref_line.set_data(
        x,
        list(s_ref_data)
    )

    # --------------------------------------------------------
    # X AXIS ONLY
    # --------------------------------------------------------

    for ax in (
        ax_d,
        ax_q,
        ax_s
    ):

        ax.relim()

        ax.autoscale_view(
            scalex=True,
            scaley=False
        )

    canvas.draw_idle()

    # --------------------------------------------------------
    # Schedule next update
    # --------------------------------------------------------

    if not stop_event.is_set():

        after_id = root.after(
            UPDATE_MS,
            update_plots
        )


# ============================================================
# CLOSE APPLICATION
# ============================================================

def close_application():

    global after_id
    global serial_port

    print("Closing application...")

    # Stop serial thread

    stop_event.set()

    # Cancel Tkinter callback

    if after_id is not None:

        try:

            root.after_cancel(
                after_id
            )

        except Exception:
            pass

        after_id = None

    # Close serial port

    if serial_port is not None:

        try:

            serial_port.close()

        except Exception:
            pass

        serial_port = None

    # Destroy GUI

    root.destroy()

    print("Application terminated.")


# ============================================================
# MAIN WINDOW
# ============================================================

root = tk.Tk()

root.title(
    "Motor Current Control Monitor"
)

root.geometry(
    "1250x950"
)

root.minsize(
    950,
    700
)

root.protocol(
    "WM_DELETE_WINDOW",
    close_application
)


# ============================================================
# TOP CONTROL BAR
# ============================================================

control_frame = tk.Frame(
    root,
    bg="#eeeeee"
)

control_frame.pack(
    side=tk.TOP,
    fill=tk.X
)


# Title

title_label = tk.Label(
    control_frame,
    text="Motor Current Control Monitor",
    font=("Arial", 18, "bold"),
    bg="#eeeeee"
)

title_label.pack(
    side=tk.LEFT,
    padx=15,
    pady=10
)


# Modulation

modulation_label = tk.Label(
    control_frame,
    text="Modulation: 0.0000",
    font=("Arial", 16, "bold"),
    fg="blue",
    bg="#eeeeee"
)

modulation_label.pack(
    side=tk.LEFT,
    padx=35
)


# Reset data

reset_button = tk.Button(
    control_frame,
    text="RESET DATA",
    font=("Arial", 10, "bold"),
    command=reset_plots,
    bg="#ffdddd",
    width=12
)

reset_button.pack(
    side=tk.RIGHT,
    padx=10,
    pady=8
)


# ============================================================
# RANGE CONTROL BAR
# ============================================================

range_frame = tk.Frame(
    root,
    bg="#dddddd",
    bd=1,
    relief=tk.RIDGE
)

range_frame.pack(
    side=tk.TOP,
    fill=tk.X,
    padx=5,
    pady=(0, 4)
)


# ------------------------------------------------------------
# D MIN
# ------------------------------------------------------------

tk.Label(
    range_frame,
    text="D Min:",
    font=("Arial", 10, "bold"),
    bg="#dddddd"
).pack(
    side=tk.LEFT,
    padx=(10, 3)
)


d_min_entry = tk.Entry(
    range_frame,
    width=8,
    font=("Consolas", 10)
)

d_min_entry.insert(
    0,
    "-1"
)

d_min_entry.pack(
    side=tk.LEFT,
    padx=3
)


# ------------------------------------------------------------
# D MAX
# ------------------------------------------------------------

tk.Label(
    range_frame,
    text="D Max:",
    font=("Arial", 10, "bold"),
    bg="#dddddd"
).pack(
    side=tk.LEFT,
    padx=(10, 3)
)


d_max_entry = tk.Entry(
    range_frame,
    width=8,
    font=("Consolas", 10)
)

d_max_entry.insert(
    0,
    "1"
)

d_max_entry.pack(
    side=tk.LEFT,
    padx=3
)


# ------------------------------------------------------------
# Q MIN
# ------------------------------------------------------------

tk.Label(
    range_frame,
    text="Q Min:",
    font=("Arial", 10, "bold"),
    bg="#dddddd"
).pack(
    side=tk.LEFT,
    padx=(25, 3)
)


q_min_entry = tk.Entry(
    range_frame,
    width=8,
    font=("Consolas", 10)
)

q_min_entry.insert(
    0,
    "-1"
)

q_min_entry.pack(
    side=tk.LEFT,
    padx=3
)


# ------------------------------------------------------------
# Q MAX
# ------------------------------------------------------------

tk.Label(
    range_frame,
    text="Q Max:",
    font=("Arial", 10, "bold"),
    bg="#dddddd"
).pack(
    side=tk.LEFT,
    padx=(10, 3)
)


q_max_entry = tk.Entry(
    range_frame,
    width=8,
    font=("Consolas", 10)
)

q_max_entry.insert(
    0,
    "1"
)

q_max_entry.pack(
    side=tk.LEFT,
    padx=3
)


# ------------------------------------------------------------
# APPLY
# ------------------------------------------------------------

apply_button = tk.Button(
    range_frame,
    text="APPLY RANGE",
    font=("Arial", 10, "bold"),
    command=apply_axis_ranges,
    bg="#cce5ff",
    width=13
)

apply_button.pack(
    side=tk.LEFT,
    padx=15,
    pady=5
)


# ------------------------------------------------------------
# DEFAULT
# ------------------------------------------------------------

default_button = tk.Button(
    range_frame,
    text="DEFAULT",
    font=("Arial", 10),
    command=default_axis_ranges,
    width=9
)

default_button.pack(
    side=tk.LEFT,
    padx=3
)


# ------------------------------------------------------------
# STATUS
# ------------------------------------------------------------

status_label = tk.Label(
    range_frame,
    text="Ready",
    font=("Arial", 9),
    fg="green",
    bg="#dddddd"
)

status_label.pack(
    side=tk.LEFT,
    padx=15
)


# ============================================================
# LIVE VALUES BAR
# ============================================================

value_frame = tk.Frame(
    root,
    bg="#222222"
)

value_frame.pack(
    side=tk.TOP,
    fill=tk.X
)


# D

d_value_label = tk.Label(
    value_frame,
    text="d: 0.0000 A",
    font=("Consolas", 13, "bold"),
    fg="#00ffff",
    bg="#222222"
)

d_value_label.pack(
    side=tk.LEFT,
    padx=30,
    pady=7
)


# Q

q_value_label = tk.Label(
    value_frame,
    text="q: 0.0000 A",
    font=("Consolas", 13, "bold"),
    fg="#00ff66",
    bg="#222222"
)

q_value_label.pack(
    side=tk.LEFT,
    padx=30
)


# Speed

s_value_label = tk.Label(
    value_frame,
    text="Speed: 0.0000",
    font=("Consolas", 13, "bold"),
    fg="#ffaa00",
    bg="#222222"
)

s_value_label.pack(
    side=tk.LEFT,
    padx=30
)


# ============================================================
# MATPLOTLIB FIGURE
# ============================================================

fig = Figure(
    figsize=(12, 8),
    dpi=100
)


# ============================================================
# GRID
#
# D and Q get considerably more vertical space.
#
#             D              Q
#       ┌────────────┬────────────┐
#       │            │            │
#       │            │            │
#       │            │            │
#       │            │            │
#       └────────────┴────────────┘
#
#       ┌──────────────────────────┐
#       │          SPEED           │
#       └──────────────────────────┘
#
# ============================================================

gs = fig.add_gridspec(
    2,
    2,
    height_ratios=[1.7, 1.0],
    hspace=0.35,
    wspace=0.25
)


ax_d = fig.add_subplot(
    gs[0, 0]
)

ax_q = fig.add_subplot(
    gs[0, 1]
)

ax_s = fig.add_subplot(
    gs[1, :]
)


fig.subplots_adjust(
    left=0.07,
    right=0.97,
    top=0.96,
    bottom=0.08
)


# ============================================================
# D PLOT
# ============================================================

d_line, = ax_d.plot(
    [],
    [],
    color="blue",
    linewidth=1.5,
    label="d"
)


d_ref_line, = ax_d.plot(
    [],
    [],
    color="red",
    linestyle="--",
    linewidth=1.5,
    label="d_ref = 0"
)


ax_d.set_title(
    "D-axis Current",
    fontsize=12,
    fontweight="bold"
)

ax_d.set_ylabel(
    "d [A]"
)

ax_d.set_ylim(
    d_min,
    d_max
)

update_axis_ticks(
    ax_d,
    d_min,
    d_max
)

ax_d.grid(
    True,
    which="major",
    alpha=0.35
)

ax_d.legend(
    loc="upper right"
)


# ============================================================
# Q PLOT
# ============================================================

q_line, = ax_q.plot(
    [],
    [],
    color="blue",
    linewidth=1.5,
    label="q"
)


q_ref_line, = ax_q.plot(
    [],
    [],
    color="red",
    linestyle="--",
    linewidth=1.5,
    label="q_ref"
)


ax_q.set_title(
    "Q-axis Current",
    fontsize=12,
    fontweight="bold"
)

ax_q.set_ylabel(
    "q [A]"
)

ax_q.set_ylim(
    q_min,
    q_max
)

update_axis_ticks(
    ax_q,
    q_min,
    q_max
)

ax_q.grid(
    True,
    which="major",
    alpha=0.35
)

ax_q.legend(
    loc="upper right"
)


# ============================================================
# SPEED PLOT
# ============================================================

s_line, = ax_s.plot(
    [],
    [],
    color="blue",
    linewidth=1.5,
    label="s_mes"
)


s_ref_line, = ax_s.plot(
    [],
    [],
    color="red",
    linestyle="--",
    linewidth=1.5,
    label="s_ref"
)


ax_s.set_title(
    "Speed",
    fontsize=12,
    fontweight="bold"
)

ax_s.set_ylabel(
    "Speed"
)

ax_s.set_xlabel(
    "Elapsed Time [s]"
)

ax_s.set_ylim(
    SPEED_MIN,
    SPEED_MAX
)

ax_s.set_yticks(
    list(range(-10, 11))
)

ax_s.grid(
    True,
    which="major",
    alpha=0.35
)

ax_s.legend(
    loc="upper right"
)


# ============================================================
# EMBED MATPLOTLIB
# ============================================================

canvas = FigureCanvasTkAgg(
    fig,
    master=root
)

canvas_widget = canvas.get_tk_widget()

canvas_widget.pack(
    fill=tk.BOTH,
    expand=True,
    padx=5,
    pady=5
)


# ============================================================
# START SERIAL THREAD
# ============================================================

serial_thread = threading.Thread(
    target=serial_reader,
    daemon=True
)

serial_thread.start()


# ============================================================
# START UPDATE LOOP
# ============================================================

after_id = root.after(
    UPDATE_MS,
    update_plots
)


# ============================================================
# RUN
# ============================================================

try:

    root.mainloop()

finally:

    if not stop_event.is_set():

        close_application()