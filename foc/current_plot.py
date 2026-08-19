import serial
import re
import threading
import queue
import time
import matplotlib.pyplot as plt
from matplotlib.widgets import CheckButtons
from matplotlib.animation import FuncAnimation
from collections import deque

COM_PORT = "COM9"
BAUDRATE = 115200
DISPLAY_SECONDS = 5
MAX_HISTORY = 20000

channel_names = ["Ia", "Ib", "Ic", "Id", "Iq"]
colors = {"Ia":"red","Ib":"blue","Ic":"green","Id":"orange","Iq":"purple"}

running = True
data_queue = queue.Queue()
samples = deque(maxlen=MAX_HISTORY)
start_time = time.monotonic()

pattern = re.compile(
    r"Ia:\s*(-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?).*?"
    r"Ib:\s*(-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?).*?"
    r"Ic:\s*(-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?).*?"
    r"Id:\s*(-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?).*?"
    r"Iq:\s*(-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)"
)

def parse_line(line):
    m = pattern.search(line)
    if not m:
        return None
    return tuple(float(x) for x in m.groups())

def serial_worker():
    global running
    while running:
        ser = None
        try:
            ser = serial.Serial(COM_PORT, BAUDRATE, timeout=1)
            ser.reset_input_buffer()
            print("Connected:", COM_PORT)

            while running:
                line = ser.readline().decode(errors="ignore").strip()
                if not line:
                    continue

                values = parse_line(line)
                if values:
                    t = time.monotonic() - start_time
                    data_queue.put((t, *values))

        except serial.SerialException as e:
            print("Serial error:", e)
            time.sleep(2)

        finally:
            if ser:
                ser.close()

threading.Thread(target=serial_worker, daemon=True).start()

fig, ax = plt.subplots(figsize=(11,6))
plt.subplots_adjust(left=0.25,right=0.75)

lines = {}

for name in channel_names:
    line, = ax.plot([], [], color=colors[name], linewidth=2, label=name)
    lines[name] = line

ax.axhline(0,color="black",linestyle="--",linewidth=1)
ax.set_title("Live Motor Current Channels")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Current (A)")
ax.grid(True)

legend = ax.legend(loc="upper left",bbox_to_anchor=(1.02,1))

checkbox_ax = plt.axes([0.03,0.35,0.15,0.25])

checkbox = CheckButtons(
    checkbox_ax,
    channel_names,
    [True]*len(channel_names)
)

def update_legend():
    for text,name in zip(legend.get_texts(),channel_names):
        text.set_alpha(1 if lines[name].get_visible() else 0.25)

def toggle_channel(label):
    lines[label].set_visible(not lines[label].get_visible())
    update_legend()
    fig.canvas.draw_idle()

checkbox.on_clicked(toggle_channel)

paused = False

def key_press(event):
    global paused
    if event.key == " ":
        paused = not paused
        print("Paused" if paused else "Running")

fig.canvas.mpl_connect("key_press_event",key_press)

def update(frame):
    if paused:
        return list(lines.values())

    while not data_queue.empty():
        sample = data_queue.get()

        if len(samples) == 0 or sample[0] >= samples[-1][0]:
            samples.append(sample)

    if len(samples) < 2:
        return list(lines.values())

    data = list(samples)

    latest = data[-1][0]
    cutoff = latest - DISPLAY_SECONDS

    data = [s for s in data if s[0] >= cutoff]

    x = [s[0] for s in data]

    for index,name in enumerate(channel_names,1):
        y = [s[index] for s in data]
        lines[name].set_data(x,y)

    ax.set_xlim(
        max(0,latest-DISPLAY_SECONDS),
        latest
    )

    visible = []

    for index,name in enumerate(channel_names,1):
        if lines[name].get_visible():
            visible.extend([s[index] for s in data])

    if visible:
        ymin = min(visible)
        ymax = max(visible)
        margin = max((ymax-ymin)*0.2,0.1)
        ax.set_ylim(ymin-margin,ymax+margin)

    return list(lines.values())

ani = FuncAnimation(
    fig,
    update,
    interval=20,
    blit=False
)

try:
    plt.show()
finally:
    running = False
    print("Serial closed")