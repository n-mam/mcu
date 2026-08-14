import sys
import re
import time
import json
import threading
import webbrowser
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import serial


# ============================================================
# Configuration
# ============================================================

DEFAULT_BAUD = 115200
WEB_PORT = 8765

# Number of seconds retained in memory
HISTORY_SECONDS = 30.0


# ============================================================
# Shared data
# ============================================================

data_lock = threading.Lock()

samples = deque()

latest = {
    "Ia": 0.0,
    "Ib": 0.0,
    "Ic": None,
}

running = True


# ============================================================
# Parse MCU log
# ============================================================

# Handles:
#
# Ia: 0.123, Ib: -0.456
#
# Also handles:
#
# Ia: 0.123, raw(a): 1234, Ib: -0.456, raw(b): 2345
#
# And later:
#
# Ia: 0.123, Ib: -0.456, Ic: 0.789
#

NUMBER = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"

IA_RE = re.compile(r"\bIa\s*:\s*(" + NUMBER + r")")
IB_RE = re.compile(r"\bIb\s*:\s*(" + NUMBER + r")")
IC_RE = re.compile(r"\bIc\s*:\s*(" + NUMBER + r")")


def parse_line(line):
    ma = IA_RE.search(line)
    mb = IB_RE.search(line)
    mc = IC_RE.search(line)

    if not ma or not mb:
        return None

    ia = float(ma.group(1))
    ib = float(mb.group(1))

    ic = float(mc.group(1)) if mc else None

    return ia, ib, ic


# ============================================================
# Serial reader
# ============================================================

def serial_reader(port, baud):
    global running

    print(f"Opening {port} @ {baud} baud...")

    try:
        ser = serial.Serial(
            port=port,
            baudrate=baud,
            timeout=1,
        )
    except Exception as e:
        print(f"\nERROR: Cannot open serial port {port}")
        print(e)
        running = False
        return

    print(f"Connected to {port}")
    print("Waiting for current samples...\n")

    start_time = time.monotonic()

    try:
        while running:
            raw = ser.readline()

            if not raw:
                continue

            try:
                line = raw.decode("utf-8", errors="ignore").strip()
            except Exception:
                continue

            if not line:
                continue

            parsed = parse_line(line)

            if parsed is None:
                # Still display other MCU messages
                print(line)
                continue

            ia, ib, ic = parsed

            t = time.monotonic() - start_time

            with data_lock:

                samples.append({
                    "t": t,
                    "Ia": ia,
                    "Ib": ib,
                    "Ic": ic,
                })

                latest["Ia"] = ia
                latest["Ib"] = ib
                latest["Ic"] = ic

                # Remove old samples
                cutoff = t - HISTORY_SECONDS

                while samples and samples[0]["t"] < cutoff:
                    samples.popleft()

            # Optional terminal output
            print(
                f"Ia: {ia: .6f} A   "
                f"Ib: {ib: .6f} A"
                + (f"   Ic: {ic: .6f} A" if ic is not None else "")
            )

    except KeyboardInterrupt:
        pass

    except Exception as e:
        print(f"\nSerial reader error: {e}")

    finally:
        ser.close()
        running = False
        print("\nSerial port closed.")


# ============================================================
# HTML
# ============================================================

HTML = r"""
<!DOCTYPE html>

<html>

<head>

<meta charset="utf-8">

<title>Phase Current Monitor</title>

<style>

* {
    box-sizing: border-box;
}

body {
    margin: 0;
    background: #0d1117;
    color: #e6edf3;
    font-family: Arial, sans-serif;
}

.header {
    padding: 14px 20px;
    background: #161b22;
    border-bottom: 1px solid #30363d;
}

.header h1 {
    margin: 0;
    font-size: 20px;
}

.status {
    margin-top: 5px;
    color: #8b949e;
    font-size: 13px;
}

.controls {
    display: flex;
    gap: 10px;
    padding: 12px 20px;
    background: #161b22;
    border-bottom: 1px solid #30363d;
}

button {
    background: #21262d;
    color: #e6edf3;
    border: 1px solid #30363d;
    border-radius: 6px;
    padding: 7px 14px;
    cursor: pointer;
}

button:hover {
    background: #30363d;
}

.values {
    display: flex;
    gap: 15px;
    padding: 15px 20px;
}

.value {
    min-width: 150px;
    padding: 12px;
    background: #161b22;
    border: 1px solid #30363d;
    border-radius: 8px;
}

.label {
    color: #8b949e;
    font-size: 13px;
}

.number {
    font-size: 25px;
    font-weight: bold;
    margin-top: 4px;
}

.ia {
    color: #ff5c5c;
}

.ib {
    color: #4da6ff;
}

.ic {
    color: #45d483;
}

.chart-container {
    margin: 0 20px 20px 20px;
    background: #090d12;
    border: 1px solid #30363d;
    border-radius: 8px;
    overflow: hidden;
}

canvas {
    display: block;
    width: 100%;
    height: 500px;
}

.info {
    padding: 0 20px 20px 20px;
    color: #8b949e;
    font-size: 12px;
}

</style>

</head>

<body>

<div class="header">

    <h1>Phase Current Monitor</h1>

    <div class="status" id="status">
        Connecting...
    </div>

</div>


<div class="controls">

    <button onclick="togglePause()" id="pauseButton">
        Pause
    </button>

    <button onclick="clearGraph()">
        Clear
    </button>

</div>


<div class="values">

    <div class="value">
        <div class="label">Ia</div>
        <div class="number ia" id="ia">---</div>
    </div>

    <div class="value">
        <div class="label">Ib</div>
        <div class="number ib" id="ib">---</div>
    </div>

    <div class="value">
        <div class="label">Ic</div>
        <div class="number ic" id="ic">---</div>
    </div>

</div>


<div class="chart-container">

    <canvas id="chart"></canvas>

</div>


<div class="info">
    Red = Ia &nbsp;&nbsp;
    Blue = Ib &nbsp;&nbsp;
    Green = Ic
</div>


<script>

const canvas = document.getElementById("chart");
const ctx = canvas.getContext("2d");

let paused = false;

let data = [];

const DISPLAY_SECONDS = 10;


// ------------------------------------------------------------
// Resize canvas
// ------------------------------------------------------------

function resizeCanvas()
{
    const rect = canvas.getBoundingClientRect();

    const dpr = window.devicePixelRatio || 1;

    canvas.width = rect.width * dpr;
    canvas.height = rect.height * dpr;

    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);

    draw();
}


window.addEventListener("resize", resizeCanvas);


// ------------------------------------------------------------
// Get data from Python
// ------------------------------------------------------------

async function update()
{
    if (!paused)
    {
        try
        {
            const response = await fetch("/data");

            if (!response.ok)
                throw new Error("HTTP " + response.status);

            const obj = await response.json();

            data = obj.samples;

            if (obj.latest)
            {
                if (obj.latest.Ia !== null)
                    document.getElementById("ia").textContent =
                        obj.latest.Ia.toFixed(4) + " A";

                if (obj.latest.Ib !== null)
                    document.getElementById("ib").textContent =
                        obj.latest.Ib.toFixed(4) + " A";

                if (obj.latest.Ic !== null)
                    document.getElementById("ic").textContent =
                        obj.latest.Ic.toFixed(4) + " A";
                else
                    document.getElementById("ic").textContent = "---";
            }

            document.getElementById("status").textContent =
                "Live • " + data.length + " samples";

        }
        catch (e)
        {
            document.getElementById("status").textContent =
                "Connection error: " + e;
        }

        draw();
    }

    setTimeout(update, 50);
}


// ------------------------------------------------------------
// Draw graph
// ------------------------------------------------------------

function draw()
{
    const rect = canvas.getBoundingClientRect();

    const W = rect.width;
    const H = rect.height;

    ctx.clearRect(0, 0, W, H);

    if (data.length < 2)
    {
        ctx.fillStyle = "#8b949e";
        ctx.font = "16px Arial";
        ctx.fillText(
            "Waiting for current samples...",
            25,
            40
        );

        return;
    }


    // --------------------------------------------------------
    // Find time range
    // --------------------------------------------------------

    const newest = data[data.length - 1].t;

    const oldest = Math.max(
        0,
        newest - DISPLAY_SECONDS
    );


    // --------------------------------------------------------
    // Find current range
    // --------------------------------------------------------

    let minY = Infinity;
    let maxY = -Infinity;

    for (const p of data)
    {
        if (p.t < oldest)
            continue;

        if (p.Ia !== null)
        {
            minY = Math.min(minY, p.Ia);
            maxY = Math.max(maxY, p.Ia);
        }

        if (p.Ib !== null)
        {
            minY = Math.min(minY, p.Ib);
            maxY = Math.max(maxY, p.Ib);
        }

        if (p.Ic !== null)
        {
            minY = Math.min(minY, p.Ic);
            maxY = Math.max(maxY, p.Ic);
        }
    }


    if (!isFinite(minY))
        return;


    // Add margin

    const range = Math.max(
        0.1,
        maxY - minY
    );

    minY -= range * 0.12;
    maxY += range * 0.12;


    // --------------------------------------------------------
    // Graph geometry
    // --------------------------------------------------------

    const left = 65;
    const right = 20;
    const top = 20;
    const bottom = 35;

    const graphW = W - left - right;
    const graphH = H - top - bottom;


    // --------------------------------------------------------
    // Coordinate functions
    // --------------------------------------------------------

    function x(t)
    {
        return left +
            ((t - oldest) / DISPLAY_SECONDS) * graphW;
    }

    function y(v)
    {
        return top +
            (1 - (v - minY) / (maxY - minY)) * graphH;
    }


    // --------------------------------------------------------
    // Grid
    // --------------------------------------------------------

    ctx.lineWidth = 1;
    ctx.strokeStyle = "#21262d";

    for (let i = 0; i <= 10; i++)
    {
        const xx = left + graphW * i / 10;

        ctx.beginPath();
        ctx.moveTo(xx, top);
        ctx.lineTo(xx, top + graphH);
        ctx.stroke();
    }


    for (let i = 0; i <= 8; i++)
    {
        const yy = top + graphH * i / 8;

        ctx.beginPath();
        ctx.moveTo(left, yy);
        ctx.lineTo(left + graphW, yy);
        ctx.stroke();
    }


    // --------------------------------------------------------
    // Zero line
    // --------------------------------------------------------

    if (minY < 0 && maxY > 0)
    {
        const zeroY = y(0);

        ctx.strokeStyle = "#555";
        ctx.lineWidth = 1;

        ctx.beginPath();
        ctx.moveTo(left, zeroY);
        ctx.lineTo(left + graphW, zeroY);
        ctx.stroke();
    }


    // --------------------------------------------------------
    // Y labels
    // --------------------------------------------------------

    ctx.fillStyle = "#8b949e";
    ctx.font = "12px Arial";
    ctx.textAlign = "right";

    for (let i = 0; i <= 8; i++)
    {
        const value =
            maxY -
            (maxY - minY) * i / 8;

        const yy =
            top + graphH * i / 8;

        ctx.fillText(
            value.toFixed(2) + " A",
            left - 8,
            yy + 4
        );
    }


    // --------------------------------------------------------
    // X labels
    // --------------------------------------------------------

    ctx.textAlign = "center";

    for (let i = 0; i <= 5; i++)
    {
        const t =
            oldest +
            DISPLAY_SECONDS * i / 5;

        const xx =
            left +
            graphW * i / 5;

        ctx.fillText(
            t.toFixed(1) + " s",
            xx,
            H - 10
        );
    }


    // --------------------------------------------------------
    // Draw phase
    // --------------------------------------------------------

    function drawPhase(key, color)
    {
        ctx.strokeStyle = color;
        ctx.lineWidth = 2;

        ctx.beginPath();

        let started = false;

        for (const p of data)
        {
            if (p.t < oldest)
                continue;

            const value = p[key];

            if (value === null || value === undefined)
                continue;

            const xx = x(p.t);
            const yy = y(value);

            if (!started)
            {
                ctx.moveTo(xx, yy);
                started = true;
            }
            else
            {
                ctx.lineTo(xx, yy);
            }
        }

        if (started)
            ctx.stroke();
    }


    drawPhase("Ia", "#ff5c5c");
    drawPhase("Ib", "#4da6ff");
    drawPhase("Ic", "#45d483");
}


// ------------------------------------------------------------
// Pause
// ------------------------------------------------------------

function togglePause()
{
    paused = !paused;

    document.getElementById("pauseButton").textContent =
        paused ? "Resume" : "Pause";
}


// ------------------------------------------------------------
// Clear
// ------------------------------------------------------------

async function clearGraph()
{
    try
    {
        await fetch("/clear", {
            method: "POST"
        });

        data = [];

        draw();
    }
    catch(e)
    {
        console.log(e);
    }
}


// Start

resizeCanvas();
update();

</script>

</body>

</html>
"""


# ============================================================
# HTTP server
# ============================================================

class Handler(BaseHTTPRequestHandler):

    def log_message(self, format, *args):
        # Keep terminal clean
        pass

    def do_GET(self):

        if self.path == "/":

            content = HTML.encode("utf-8")

            self.send_response(200)
            self.send_header(
                "Content-Type",
                "text/html; charset=utf-8"
            )
            self.send_header(
                "Content-Length",
                str(len(content))
            )
            self.end_headers()

            self.wfile.write(content)

            return


        if self.path == "/data":

            with data_lock:

                payload = {
                    "samples": list(samples),
                    "latest": dict(latest),
                }

            content = json.dumps(payload).encode("utf-8")

            self.send_response(200)
            self.send_header(
                "Content-Type",
                "application/json"
            )
            self.send_header(
                "Cache-Control",
                "no-cache"
            )
            self.send_header(
                "Content-Length",
                str(len(content))
            )
            self.end_headers()

            self.wfile.write(content)

            return


        self.send_error(404)


    def do_POST(self):

        if self.path == "/clear":

            with data_lock:
                samples.clear()

            content = b'{"ok":true}'

            self.send_response(200)
            self.send_header(
                "Content-Type",
                "application/json"
            )
            self.send_header(
                "Content-Length",
                str(len(content))
            )
            self.end_headers()

            self.wfile.write(content)

            return

        self.send_error(404)


# ============================================================
# Main
# ============================================================

def main():

    global running

    if len(sys.argv) < 2:

        print()
        print("Usage:")
        print()
        print("  python visualizer.py COM5")
        print()
        print("or:")
        print()
        print("  python visualizer.py COM5 115200")
        print()

        sys.exit(1)


    port = sys.argv[1]

    baud = (
        int(sys.argv[2])
        if len(sys.argv) >= 3
        else DEFAULT_BAUD
    )


    # --------------------------------------------------------
    # Start serial reader
    # --------------------------------------------------------

    serial_thread = threading.Thread(
        target=serial_reader,
        args=(port, baud),
        daemon=True
    )

    serial_thread.start()


    # --------------------------------------------------------
    # Start web server
    # --------------------------------------------------------

    server = ThreadingHTTPServer(
        ("127.0.0.1", WEB_PORT),
        Handler
    )

    url = f"http://127.0.0.1:{WEB_PORT}/"

    print()
    print("================================================")
    print("   Phase Current Live Visualizer")
    print("================================================")
    print()
    print(f"COM port : {port}")
    print(f"Baud     : {baud}")
    print(f"Web      : {url}")
    print()
    print("Press Ctrl+C to stop.")
    print()


    # Open browser

    threading.Timer(
        0.8,
        lambda: webbrowser.open(url)
    ).start()


    try:

        server.serve_forever()

    except KeyboardInterrupt:

        print("\nStopping...")

    finally:

        running = False
        server.shutdown()
        server.server_close()


if __name__ == "__main__":
    main()