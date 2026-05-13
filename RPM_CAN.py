#Old RPM code, uses an exported message log and a different method to find the RPM of the car (not in real time)
import re
import matplotlib.pyplot as plt
from collections import defaultdict

def buscar_y_graficar_rpm(archivo_txt, rpm_max_esperado=3000):
    """
    Busca señales de RPM en un archivo de mensajes CAN.
    
    Parámetros:
        archivo_txt     : Ruta al archivo .txt con mensajes CAN.
        rpm_max_esperado: RPM máximas del vehículo (aprox). Por defecto 3000.
    """
    data_by_id = defaultdict(list)
    time_by_id = defaultdict(list)

    with open(archivo_txt, "r") as f:
        lines = f.readlines()

    for i, line in enumerate(lines):
        match = re.search(r'\[([0-9A-Fa-f]+)\]\s*\((\d+)\)\s*:\s*([0-9A-Fa-f\s]+)', line.strip())
        if match:
            can_id  = match.group(1).upper()
            length  = int(match.group(2))
            data_hex = match.group(3).replace(" ", "").strip()

            if len(data_hex) == length * 2:
                try:
                    data_bytes = bytes.fromhex(data_hex)
                    data_by_id[can_id].append(data_bytes)
                    time_by_id[can_id].append(i)
                except ValueError:
                    continue  # Skip lines with non-hex characters

    candidatos = []

    for can_id, messages in data_by_id.items():
        if len(messages) < 30:
            continue

        msg_len = min(len(m) for m in messages)

        # ── 16-bit signals analysis (pairs of bytes) ──────────
        for bi in range(msg_len - 1):
            be_raw = [(m[bi] << 8 | m[bi + 1]) for m in messages]
            le_raw = [(m[bi + 1] << 8 | m[bi]) for m in messages]

            escalas_16 = [
                ("crudo ×1",        be_raw),
                ("÷4 (OBD2)",       [v / 4   for v in be_raw]),
                ("÷8",              [v / 8   for v in be_raw]),
                ("÷2",              [v / 2   for v in be_raw]),
                ("LE crudo ×1",     le_raw),
                ("LE ÷4",           [v / 4   for v in le_raw]),
                ("LE ÷8",           [v / 8   for v in le_raw]),
            ]

            for factor_name, vals in escalas_16:
                _evaluar_candidato(candidatos, can_id, vals,
                                   time_by_id[can_id],
                                   (bi, bi + 1), factor_name,
                                   rpm_max_esperado)

        # ── 8-bit signals analysis (individual byte) ──────────
        for bi in range(msg_len):
            byte_vals = [m[bi] for m in messages]
            escalas_8 = [
                ("byte ×1",   byte_vals),
                ("byte ×4",   [v * 4  for v in byte_vals]),
                ("byte ×8",   [v * 8  for v in byte_vals]),
                ("byte ×12",  [v * 12 for v in byte_vals]),
            ]
            for factor_name, vals in escalas_8:
                _evaluar_candidato(candidatos, can_id, vals,
                                   time_by_id[can_id],
                                   (bi,), factor_name,
                                   rpm_max_esperado)

    candidatos.sort(key=lambda x: x["score"])

    if not candidatos:
        print("❌ No se encontraron señales candidatas de RPM.")
        print("   Sugerencia: verifica que el archivo tenga el formato [ID] (DLC) : HEX")
        return

    top_n = min(3, len(candidatos))
    print(f"🔍 Top {top_n} señales candidatas de RPM:\n")

    fig, axes = plt.subplots(top_n, 1, figsize=(12, 4 * top_n))
    if top_n == 1:
        axes = [axes]

    colores = ['#2ca02c', '#1f77b4', '#ff7f0e']

    for idx, s in enumerate(candidatos[:top_n]):
        byte_str = (f"Bytes {s['bytes'][0]}-{s['bytes'][1]}"
                    if len(s['bytes']) == 2
                    else f"Byte {s['bytes'][0]}")
        etiqueta = "⭐ MEJOR CANDIDATO" if idx == 0 else f"Candidato #{idx + 1}"
        print(f"{etiqueta}")
        print(f"  CAN ID : {s['id']}")
        print(f"  {byte_str}  |  {s['factor']}")
        print(f"  Mín: {s['min']:.1f} RPM  |  Máx: {s['max']:.1f} RPM  |  Rango: {s['rango']:.1f}")
        print()

        axes[idx].plot(s['time'], s['data'],
                       color=colores[idx], linewidth=1, label='RPM estimadas')
        axes[idx].set_ylabel("RPM")
        titulo = (f"{'⭐ ' if idx == 0 else ''}CAN ID: {s['id']} | "
                  f"{byte_str} | {s['factor']}")
        axes[idx].set_title(titulo)
        axes[idx].grid(True, linestyle='--', alpha=0.6)
        axes[idx].legend()

    axes[-1].set_xlabel("Secuencia del mensaje (# línea)")
    plt.tight_layout()
    plt.show()


def _evaluar_candidato(candidatos, can_id, vals, times, bytes_idx,
                        factor_name, rpm_max_esperado):
    
    min_v    = min(vals)
    max_v    = max(vals)
    rango    = max_v - min_v
    unique_v = len(set(round(v, 1) for v in vals))

    # Discard static signals (rango very small = not a dynamic signal)
    if rango < 150:
        return
    # Discard negative values (RPM are not negative)
    if min_v < -10:
        return
    # Discard if the maximum exceeds what is expected
    # (avoids false positives with temperature, pressure, etc.)
    if max_v > rpm_max_esperado * 1.5:
        return
    # Discard if the maximum is too small (doesn't seem like RPM)
    if max_v < 100:
        return
    # Discard almost binary signals (few variations = flag or status)
    if unique_v < 15:
        return

    # ── Score: smaller = better candidate ───────────────────────────────
    # Penalize if the maximum is far from rpm_max_esperado
    penalidad_max = abs(max_v - rpm_max_esperado) / rpm_max_esperado
    # Penalize if the minimum is not close to 0 (even if we don't require it)
    penalidad_min = min_v / rpm_max_esperado
    # Reward signals with more variation (more dynamic)
    bonus_variedad = -unique_v / len(vals)

    # ─────────────────────────────────────────────────────────────────
    if len(vals) > 1:
        cambios = [abs(vals[k+1] - vals[k]) for k in range(len(vals)-1)]
        suavidad = (sum(cambios) / len(cambios)) / rango  # 0 = Smooth signal, 1 = very noisy
    else:
        suavidad = 1.0

    # high weight: noisy signal -> discarded
    penalidad_ruido = suavidad * 2   

    score = penalidad_max + penalidad_min + bonus_variedad + penalidad_ruido

    candidatos.append({
        "id":     can_id,
        "bytes":  bytes_idx,
        "factor": factor_name,
        "data":   vals,
        "time":   times,
        "min":    min_v,
        "max":    max_v,
        "rango":  rango,
        "score":  score,
    })


# ── RUN ──────────────────────────────────────────────────────────
buscar_y_graficar_rpm("Can_Datos_1.txt", rpm_max_esperado=3000)