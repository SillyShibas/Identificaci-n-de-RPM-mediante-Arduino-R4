"""
RPM_CAN_tiempo_real.py
======================
Grafica en tiempo real las RPM y la Velocidad (km/h) de un Dodge Durango
leídas por un Arduino R4 WiFi vía CAN/OBD2.

Cambios respecto a la versión anterior:
  - RPM = 0 cuando el motor se apaga (timeout de respuesta OBD2).
  - Velocidad en km/h graficada debajo de las RPM (OBD2 PID 0x0D).
  - El puerto serial se abre al iniciar y se cierra limpiamente al
    cerrar la ventana de matplotlib.

Protocolo Arduino (Serial @ 115200 baud):
  Formato de línea:
    ID: <HEX>  DLC: <N>  DATA: <B0> <B1> ... <BN-1>  ,<timestamp_ms>
  Ejemplo:
    ID: 7E8  DLC: 8  DATA: 04 41 0C A0 00 00 00 00  ,1234

Ecuaciones OBD2:
  RPM  (PID 0x0C): RPM  = (256 * A + B) / 4   → A=data[3], B=data[4]
  VEL  (PID 0x0D): km/h = A                    → A=data[3]

Uso:
  python RPM_CAN_tiempo_real.py                      # auto-detecta puerto
  python RPM_CAN_tiempo_real.py --port COM3          # Windows
  python RPM_CAN_tiempo_real.py --port /dev/ttyUSB0  # Linux/Mac
  python RPM_CAN_tiempo_real.py --port COM3 --baud 115200 --window 60
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

# ─── OBD2 Constants ────────────────────────────────────────────────────────
OBD2_RESPONSE_IDS = {0x7E8, 0x7E9, 0x7EA, 0x7EB}
PID_RPM           = 0x0C
PID_SPEED         = 0x0D
MODE_RESPONSE     = 0x41   # 0x40 + modo 0x01

# Maximum time without response before considering engine off (seconds)
ENGINE_OFF_TIMEOUT = 1.5

# ─── Shared state (Serial thread ↔ matplotlib thread) ──────────────────────
_lock          = threading.Lock()
_rpm_values    = deque()   # (tiempo_s, rpm)
_speed_values  = deque()   # (tiempo_s, km/h)
_status_msg    = ["Waiting for data..."]
_last_rpm_time = [None]    # last time a valid RPM message arrived
_stop_event    = threading.Event()

# ─── Auto-detect Arduino port ─────────────────────────────────────────────────
def _auto_detect_port():
    """Returns the first port that looks like an Arduino."""
    for p in serial.tools.list_ports.comports():
        desc = (p.description or "").lower()
        hwid = (p.hwid or "").lower()
        if any(k in desc or k in hwid
               for k in ("arduino", "ch340", "cp210", "ftdi", "2341", "2a03")):
            return p.device
    ports = serial.tools.list_ports.comports()
    return ports[0].device if ports else None


# ─── Parse Serial line ──────────────────────────────────────────────────────
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
    Returns (can_id:int, data:bytes, timestamp_ms:int|None)
    or None if the line is not a valid CAN message.
    """
    line = line.strip()

    m = _RE_ARDUINO.search(line)
    if m:
        can_id  = int(m.group(1), 16)
        dlc     = int(m.group(2))
        hex_str = m.group(3).strip()
        ts_raw  = m.group(4)
        ts      = int(ts_raw) if ts_raw else None
        data_hex = hex_str.replace(" ", "")
        if len(data_hex) == dlc * 2:
            try:
                return can_id, bytes.fromhex(data_hex), ts
            except ValueError:
                pass

    m = _RE_LEGACY.search(line)
    if m:
        can_id  = int(m.group(1), 16)
        dlc     = int(m.group(2))
        hex_str = m.group(3).strip()
        data_hex = hex_str.replace(" ", "")
        if len(data_hex) == dlc * 2:
            try:
                return can_id, bytes.fromhex(data_hex), None
            except ValueError:
                pass

    return None


def _extract_obd2(can_id: int, data: bytes):
    """
    Extracts (rpm, speed_kmh) from the CAN message.
    Unknown values are returned as None.
    Format: [len] [0x41] [PID] [A] [B] ...
    """
    if can_id not in OBD2_RESPONSE_IDS:
        return None, None
    if len(data) < 4:
        return None, None
    if data[1] != MODE_RESPONSE:
        return None, None

    pid = data[2]
    rpm   = None
    speed = None

    if pid == PID_RPM and len(data) >= 5:
        A, B = data[3], data[4]
        rpm = (256 * A + B) / 4.0

    elif pid == PID_SPEED:
        speed = float(data[3])   # km/h = A

    return rpm, speed


# ─── Serial reader thread ──────────────────────────────────────────────────────
def _serial_reader(port: str, baud: int, window_s: float, ser_holder: list):
    """
    Reads the serial port and enters RPM/speed into the buffer.
    ser_holder[0] is filled with the open serial object so that
    the main thread can close it cleanly.
    """
    t0 = None

    try:
        ser = serial.Serial(port, baud, timeout=2)
        ser_holder[0] = ser
        with _lock:
            _status_msg[0] = f"Conectado a {port} @ {baud} baud"
        print(f"✅  Puerto abierto: {port} @ {baud}")
    except serial.SerialException as e:
        with _lock:
            _status_msg[0] = f"❌ Error abriendo {port}: {e}"
        print(f"❌  {e}")
        return

    try:
        while not _stop_event.is_set():
            try:
                raw = ser.readline()
            except serial.SerialException:
                with _lock:
                    _status_msg[0] = "❌ Serial Connection Lost"
                break

            if not raw:
                continue

            line   = raw.decode("ascii", errors="replace")
            parsed = _parse_line(line)
            if parsed is None:
                continue

            can_id, data, _ = parsed
            rpm, speed = _extract_obd2(can_id, data)

            now = time.time()
            if t0 is None:
                t0 = now
            elapsed = now - t0
            cutoff  = elapsed - window_s

            with _lock:
                if rpm is not None:
                    _last_rpm_time[0] = now
                    _rpm_values.append((elapsed, rpm))
                    while _rpm_values and _rpm_values[0][0] < cutoff:
                        _rpm_values.popleft()

                if speed is not None:
                    _speed_values.append((elapsed, speed))
                    while _speed_values and _speed_values[0][0] < cutoff:
                        _speed_values.popleft()

    except Exception as e:
        with _lock:
            _status_msg[0] = f"Unexpected Error: {e}"
        print(f"❌ Unexpected error in serial thread: {e}")
    finally:
        try:
            ser.close()
            print("🔌  Serial port closed.")
        except Exception:
            pass


# ─── Build Figure ───────────────────────────────────────────────────────
def _build_figure(window_s: int, rpm_max: int, speed_max: int):
    fig, (ax_rpm, ax_spd) = plt.subplots(
        2, 1, figsize=(13, 8),
        gridspec_kw={"height_ratios": [3, 2]},
        sharex=False
    )
    fig.patch.set_facecolor("#1a1a2e")

    # ── Subplot RPM ──────────────────────────────────────────────────────────
    ax_rpm.set_facecolor("#16213e")
    line_rpm, = ax_rpm.plot([], [], color="#00d4ff", linewidth=1.8,
                            label="RPM (PID 0x0C)")
    dot_rpm,  = ax_rpm.plot([], [], "o", color="#ff6b6b", markersize=7, zorder=5)

    ax_rpm.set_ylim(-50, rpm_max + 200)
    ax_rpm.set_xlim(0, window_s)
    ax_rpm.set_ylabel("RPM", color="#c0c0d0", fontsize=11)
    ax_rpm.tick_params(colors="#c0c0d0")
    ax_rpm.tick_params(axis="x", labelbottom=False)
    for sp in ax_rpm.spines.values():
        sp.set_edgecolor("#334466")
    ax_rpm.grid(True, linestyle="--", alpha=0.35, color="#334466")
    ax_rpm.legend(loc="upper left", facecolor="#1a1a2e",
                  edgecolor="#334466", labelcolor="#c0c0d0")
    ax_rpm.set_title("RPM & Speed in Real Time — Dodge Durango",
                     color="#e0e0f0", fontsize=13, fontweight="bold")

    rpm_text = ax_rpm.text(0.98, 0.93, "--- RPM", transform=ax_rpm.transAxes,
                           ha="right", va="top", fontsize=20, fontweight="bold",
                           color="#00d4ff")
    status_text = ax_rpm.text(0.01, 0.96, "", transform=ax_rpm.transAxes,
                              ha="left", va="top", fontsize=8, color="#aaaaaa")

    # ── Subplot Speed ────────────────────────────────────────────────────
    ax_spd.set_facecolor("#16213e")
    line_spd, = ax_spd.plot([], [], color="#f7c59f", linewidth=1.8,
                            label="Speed (PID 0x0D)")
    dot_spd,  = ax_spd.plot([], [], "o", color="#ff6b6b", markersize=7, zorder=5)

    ax_spd.set_ylim(-2, speed_max + 10)
    ax_spd.set_xlim(0, window_s)
    ax_spd.set_xlabel("Time (s)", color="#c0c0d0", fontsize=11)
    ax_spd.set_ylabel("Speed (km/h)", color="#c0c0d0", fontsize=11)
    ax_spd.tick_params(colors="#c0c0d0")
    for sp in ax_spd.spines.values():
        sp.set_edgecolor("#334466")
    ax_spd.grid(True, linestyle="--", alpha=0.35, color="#334466")
    ax_spd.legend(loc="upper left", facecolor="#1a1a2e",
                  edgecolor="#334466", labelcolor="#c0c0d0")

    spd_text = ax_spd.text(0.98, 0.93, "--- km/h", transform=ax_spd.transAxes,
                           ha="right", va="top", fontsize=18, fontweight="bold",
                           color="#f7c59f")

    fig.tight_layout(pad=1.5)
    return fig, ax_rpm, ax_spd, line_rpm, dot_rpm, rpm_text, status_text, \
           line_spd, dot_spd, spd_text


# ─── Animation update function ────────────────────────────────────────────────
def _make_updater(ax_rpm, ax_spd,
                  line_rpm, dot_rpm, rpm_text, status_text,
                  line_spd, dot_spd, spd_text,
                  window_s):

    def update(_frame):
        now = time.time()

        with _lock:
            rpm_data   = list(_rpm_values)
            spd_data   = list(_speed_values)
            status     = _status_msg[0]
            last_rpm_t = _last_rpm_time[0]

        status_text.set_text(status)

        # ── Engine Off: timeout → force RPM = 0 ──────────────────────────
        if last_rpm_t is not None and (now - last_rpm_t) > ENGINE_OFF_TIMEOUT:
            if rpm_data:
                last_elapsed = rpm_data[-1][0] + (now - last_rpm_t - ENGINE_OFF_TIMEOUT)
                with _lock:
                    _rpm_values.append((last_elapsed, 0.0))
                rpm_data = list(_rpm_values)
            rpm_text.set_text("0 RPM")
            rpm_text.set_color("#888888")
            line_rpm.set_color("#888888")

        # ── Update RPM graph ───────────────────────────────────────────────────
        if rpm_data:
            times = [d[0] for d in rpm_data]
            rpms  = [d[1] for d in rpm_data]
            t_now = times[-1]
            x_min = max(0.0, t_now - window_s)
            ax_rpm.set_xlim(x_min, x_min + window_s)
            line_rpm.set_data(times, rpms)
            dot_rpm.set_data([times[-1]], [rpms[-1]])

            rpm_actual = rpms[-1]
            if rpm_actual < 50:
                color = "#888888"
            elif rpm_actual < 900:
                color = "#00ff99"
            elif rpm_actual < 2500:
                color = "#00d4ff"
            else:
                color = "#ff6b6b"

            rpm_text.set_text(f"{rpm_actual:.0f} RPM")
            rpm_text.set_color(color)
            line_rpm.set_color(color)

        # ── Update Speed graph ─────────────────────────────────────────────
        if spd_data:
            stimes = [d[0] for d in spd_data]
            speeds = [d[1] for d in spd_data]
            t_now_s = stimes[-1]
            x_min_s = max(0.0, t_now_s - window_s)
            ax_spd.set_xlim(x_min_s, x_min_s + window_s)
            line_spd.set_data(stimes, speeds)
            dot_spd.set_data([stimes[-1]], [speeds[-1]])

            spd_actual = speeds[-1]
            if spd_actual < 5:
                spd_color = "#888888"
            elif spd_actual < 60:
                spd_color = "#00ff99"
            elif spd_actual < 120:
                spd_color = "#f7c59f"
            else:
                spd_color = "#ff6b6b"

            spd_text.set_text(f"{spd_actual:.0f} km/h")
            spd_text.set_color(spd_color)
            line_spd.set_color(spd_color)

        return (line_rpm, dot_rpm, rpm_text, status_text,
                line_spd, dot_spd, spd_text)

    return update


# ─── Entry point ─────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="RPM and Speed CAN/OBD2 graph in real time")
    parser.add_argument("--port",     default=None,   help="Serial Port (ej: COM3, /dev/ttyUSB0)")
    parser.add_argument("--baud",     default=115200, type=int, help="Baudrate (default: 115200)")
    parser.add_argument("--window",   default=30,     type=int, help="Time window in seconds (default: 30)")
    parser.add_argument("--rpmmax",   default=4000,   type=int, help="Max RPM in Y axis (default: 4000)")
    parser.add_argument("--speedmax", default=200,    type=int, help="Max Speed in Y axis in km/h (default: 200)")
    args = parser.parse_args()

    port = args.port or _auto_detect_port()
    if port is None:
        print("❌  No Serial port found.")
        print("    Connect the Arduino and try again, or use --port COM3")
        return

    print(f"🔌  Using port: {port}")
    print(f"📊  Window: {args.window}s  |  Max RPM: {args.rpmmax}  |  Max Speed: {args.speedmax} km/h")

    # List to access the serial object in the main thread
    ser_holder = [None]

    # ── Start serial thread in background ─────────────────────────────────
    t = threading.Thread(
        target=_serial_reader,
        args=(port, args.baud, float(args.window), ser_holder),
        daemon=True
    )
    t.start()

    # ── Build figure ─────────────────────────────────────────────────────
    fig, ax_rpm, ax_spd, \
    line_rpm, dot_rpm, rpm_text, status_text, \
    line_spd, dot_spd, spd_text = _build_figure(
        args.window, args.rpmmax, args.speedmax
    )

    updater = _make_updater(
        ax_rpm, ax_spd,
        line_rpm, dot_rpm, rpm_text, status_text,
        line_spd, dot_spd, spd_text,
        args.window
    )

    ani = animation.FuncAnimation(
        fig, updater,
        interval=100,
        blit=False,
        cache_frame_data=False
    )

    # ── Close serial port when closing window ─────────────────────────
    def _on_close(event):
        print("🛑  Closing window — stopping serial thread...")
        _stop_event.set()
        # Wait briefly for the thread to finish closing the port
        t.join(timeout=3)
        ser = ser_holder[0]
        if ser is not None and ser.is_open:
            try:
                ser.close()
                print("🔌  Serial port closed from window.")
            except Exception:
                pass

    fig.canvas.mpl_connect("close_event", _on_close)

    plt.show()


if __name__ == "__main__":
    main()