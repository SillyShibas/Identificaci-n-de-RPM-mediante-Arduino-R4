"""
RPM_RealTime.py
===============
Grafica en tiempo real las RPM de un Dodge Durango leídas por un
Arduino R4 WiFi vía CAN/OBD2.

Protocolo Arduino (Serial @ 115200 baud):
  Formato de línea:
    ID: <HEX>  DLC: <N>  DATA: <B0> <B1> ... <BN-1>  ,<timestamp_ms>
  Ejemplo:
    ID: 7E8  DLC: 8  DATA: 04 41 0C A0 00 00 00 00  ,1234

Ecuación RPM (OBD2 PID 0x0C):
  RPM = (256 * A + B) / 4
  donde A = DATA[3], B = DATA[4]  (byte 0-index, tras los bytes 04 41 0C)

Uso:
  python RPM_RealTime.py                     # auto-detecta puerto
  python RPM_RealTime.py --port COM3         # Windows
  python RPM_RealTime.py --port /dev/ttyUSB0 # Linux/Mac
  python RPM_RealTime.py --port COM3 --baud 115200 --window 60
"""

import argparse
import re
import threading
import time
from collections import deque

import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.ticker import MaxNLocator

# ─── OBD2 Constants ────────────────────────────────────────────────────────
# Standard ECU responses
OBD2_RESPONSE_IDS = {0x7E8, 0x7E9, 0x7EA, 0x7EB}   

PID_RPM           = 0x0C
MODE_RESPONSE     = 0x41  # 0x40 + modo 0x01

# ─── Buffers compartidos (Serial thread ↔ matplotlib thread) ────────────────────
_lock        = threading.Lock()
_rpm_values  = deque()   # (tiempo_s, rpm)
_status_msg  = ["Esperando datos..."]
_connected   = [False]

# ─── Auto-detect Arduino port ────────────────────────────────
def _auto_detect_port():
    """Returns the first port that looks like an Arduino."""
    for p in serial.tools.list_ports.comports():
        desc = (p.description or "").lower()
        hwid = (p.hwid or "").lower()
        if any(k in desc or k in hwid
               for k in ("arduino", "ch340", "cp210", "ftdi", "2341", "2a03")):
            return p.device
    # If it didn't find anything obvious, return the first available
    ports = serial.tools.list_ports.comports()
    return ports[0].device if ports else None


# ─── Parse Serial line ─────────────────────────────────────────────
# The Arduino R4 prints with Serial.print(CanMsg) which generates something like:
#   ID: 7E8  DLC: 8  DATA: 04 41 0C A0 00 00 00 00
# followed by a comma and the timestamp:  ,1234
#
# Also supports the original analysis code format:
#   [7E8] (8) : 04 41 0C A0 00 00 00 00

_RE_ARDUINO = re.compile(
    r'ID:\s*([0-9A-Fa-f]+)\s+DLC:\s*(\d+)\s+DATA:\s*([0-9A-Fa-f\s]+?)(?:,(\d+))?$',
    re.IGNORECASE
)
_RE_LEGACY = re.compile(
    r'\[([0-9A-Fa-f]+)\]\s*\((\d+)\)\s*:\s*([0-9A-Fa-f\s]+)',
    re.IGNORECASE
)

def _parse_line(line: str):
    """
    Retorna (can_id:int, data:bytes, timestamp_ms:int|None)
    o None si la línea no es un mensaje CAN válido.
    """
    line = line.strip()

    m = _RE_ARDUINO.search(line)
    if m:
        can_id   = int(m.group(1), 16)
        dlc      = int(m.group(2))
        hex_str  = m.group(3).strip()
        ts_raw   = m.group(4)
        ts       = int(ts_raw) if ts_raw else None
        data_hex = hex_str.replace(" ", "")
        if len(data_hex) == dlc * 2:
            try:
                return can_id, bytes.fromhex(data_hex), ts
            except ValueError:
                pass

    m = _RE_LEGACY.search(line)
    if m:
        can_id   = int(m.group(1), 16)
        dlc      = int(m.group(2))
        hex_str  = m.group(3).strip()
        data_hex = hex_str.replace(" ", "")
        if len(data_hex) == dlc * 2:
            try:
                return can_id, bytes.fromhex(data_hex), None
            except ValueError:
                pass

    return None


def _extract_rpm(can_id: int, data: bytes):
    """
    Extrae RPM si el mensaje es una respuesta OBD2 al PID 0x0C.
    Formato respuesta: [longitud] [0x41] [0x0C] [A] [B] ...
    RPM = (256*A + B) / 4
    """
    if can_id not in OBD2_RESPONSE_IDS:
        return None
    if len(data) < 5:
        return None
    # data[0] = number of additional data bytes (normally 0x04)
    # data[1] = 0x41 (mode 01 response)
    # data[2] = PID
    # data[3] = A,  data[4] = B
    if data[1] != MODE_RESPONSE or data[2] != PID_RPM:
        return None
    A = data[3]
    B = data[4]
    return (256 * A + B) / 4.0


# ─── Serial reader thread ─────────────────────────────────────────────────────
def _serial_reader(port: str, baud: int, window_s: float):
    """Reads the serial port and enters RPM into the buffer."""
    global _connected, _status_msg

    t0 = None  # reference time (first valid message)

    try:
        ser = serial.Serial(port, baud, timeout=2)
        with _lock:
            _connected[0] = True
            _status_msg[0] = f"Connected to {port} @ {baud} baud"
        print(f"✅  Port opened: {port} @ {baud}")
    except serial.SerialException as e:
        with _lock:
            _status_msg[0] = f"❌ Error opening {port}: {e}"
        print(f"❌  {e}")
        return

    try:
        while True:
            try:
                raw = ser.readline()
            except serial.SerialException:
                with _lock:
                    _status_msg[0] = "❌ Serial Connection Lost"
                break

            if not raw:
                continue

            line = raw.decode("ascii", errors="replace")
            parsed = _parse_line(line)
            if parsed is None:
                continue

            can_id, data, ts_ms = parsed
            rpm = _extract_rpm(can_id, data)
            if rpm is None:
                continue

            now = time.time()
            if t0 is None:
                t0 = now

            elapsed = now - t0

            with _lock:
                _rpm_values.append((elapsed, rpm))
                # Maintain only the desired time window
                cutoff = elapsed - window_s
                while _rpm_values and _rpm_values[0][0] < cutoff:
                    _rpm_values.popleft()

    except Exception as e:
        with _lock:
            _status_msg[0] = f"Error inesperado: {e}"
        print(f"❌ Unexpected error in serial thread: {e}")
    finally:
        try:
            ser.close()
        except Exception:
            pass


# ─── Matplotlib animation ────────────────────────────────────────────────────
def _build_figure(window_s: int, rpm_max: int):
    fig, ax = plt.subplots(figsize=(13, 5))
    fig.patch.set_facecolor("#1a1a2e")
    ax.set_facecolor("#16213e")

    line_rpm, = ax.plot([], [], color="#00d4ff", linewidth=1.8,
                        label="RPM (OBD2 PID 0x0C)")
    dot_last,  = ax.plot([], [], "o", color="#ff6b6b", markersize=7, zorder=5)

    ax.set_ylim(-50, rpm_max + 200)
    ax.set_xlim(0, window_s)
    ax.set_xlabel("Tiempo (s)", color="#c0c0d0", fontsize=11)
    ax.set_ylabel("RPM", color="#c0c0d0", fontsize=11)
    ax.tick_params(colors="#c0c0d0")
    for spine in ax.spines.values():
        spine.set_edgecolor("#334466")
    ax.grid(True, linestyle="--", alpha=0.35, color="#334466")
    ax.legend(loc="upper left", facecolor="#1a1a2e", edgecolor="#334466",
              labelcolor="#c0c0d0")

    title_obj  = ax.set_title("Real-Time RPM — Dodge Durango",
                               color="#e0e0f0", fontsize=13, fontweight="bold")
    rpm_text   = ax.text(0.98, 0.93, "--- RPM", transform=ax.transAxes,
                          ha="right", va="top", fontsize=20, fontweight="bold",
                          color="#00d4ff")
    status_text = ax.text(0.01, 0.96, "", transform=ax.transAxes,
                           ha="left", va="top", fontsize=8, color="#aaaaaa")

    return fig, ax, line_rpm, dot_last, rpm_text, status_text, title_obj


def _make_updater(ax, line_rpm, dot_last, rpm_text, status_text, window_s, rpm_max):
    def update(_frame):
        with _lock:
            data   = list(_rpm_values)
            status = _status_msg[0]

        status_text.set_text(status)

        if not data:
            return line_rpm, dot_last, rpm_text, status_text

        times = [d[0] for d in data]
        rpms  = [d[1] for d in data]

        t_now = times[-1]
        x_min = max(0.0, t_now - window_s)
        x_max = x_min + window_s

        ax.set_xlim(x_min, x_max)

        line_rpm.set_data(times, rpms)
        dot_last.set_data([t_now], [rpms[-1]])

        rpm_actual = rpms[-1]
        rpm_text.set_text(f"{rpm_actual:.0f} RPM")

        # Dynamic color according to RPM
        if rpm_actual < 50:
            color = "#888888"   # engine off / very low idle
        elif rpm_actual < 900:
            color = "#00ff99"   # idle
        elif rpm_actual < 2500:
            color = "#00d4ff"   # normal operation
        else:
            color = "#ff6b6b"   # high

        rpm_text.set_color(color)
        line_rpm.set_color(color)

        return line_rpm, dot_last, rpm_text, status_text

    return update


# ─── Entry point ────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="Real-time CAN/OBD2 RPM Graph")
    parser.add_argument("--port",   default=None,    help="Serial Port (ej: COM3, /dev/ttyUSB0)")
    parser.add_argument("--baud",   default=115200,  type=int, help="Baudrate (default: 115200)")
    parser.add_argument("--window", default=30,      type=int, help="Time window in seconds (default: 30)")
    parser.add_argument("--rpmmax", default=4000,    type=int, help="Max RPM for Y-axis (default: 4000)")
    args = parser.parse_args()

    port = args.port or _auto_detect_port()
    if port is None:
        print("❌  No serial port found.")
        print("   Connect the Arduino and try again, or use --port COM3")
        return

    print(f"🔌  Using port: {port}")
    print(f"📊  Window: {args.window}s  |  Max RPM axis: {args.rpmmax}")

    # Start serial thread in background
    t = threading.Thread(
        target=_serial_reader,
        args=(port, args.baud, float(args.window)),
        daemon=True
    )
    t.start()

    # Build figure
    fig, ax, line_rpm, dot_last, rpm_text, status_text, _ = _build_figure(
        args.window, args.rpmmax
    )

    updater = _make_updater(ax, line_rpm, dot_last, rpm_text,
                             status_text, args.window, args.rpmmax)

    ani = animation.FuncAnimation(
        fig, updater,
        interval=100,       # refresh every 100 ms
        blit=False,
        cache_frame_data=False
    )

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
