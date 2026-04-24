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
                    continue  # Saltar líneas con caracteres no-hex

    candidatos = []

    for can_id, messages in data_by_id.items():
        if len(messages) < 30:
            continue

        msg_len = min(len(m) for m in messages)

        # ── Análisis de señales de 16 bits (pares de bytes) ──────────
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

        # ── Análisis de señales de 8 bits (byte individual) ──────────
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

    # Descartar señales estáticas (rango muy pequeño = no es señal dinámica)
    if rango < 150:
        return
    # Descartar valores negativos (RPM no son negativas)
    if min_v < -10:
        return
    # Descartar si el máximo supera ampliamente lo esperado
    # (evita falsos positivos con señales de temperatura, presión, etc.)
    if max_v > rpm_max_esperado * 1.5:
        return
    # Descartar si el máximo es demasiado pequeño (no parece RPM)
    if max_v < 100:
        return
    # Descartar señales casi binarias (pocas variaciones = flag o estado)
    if unique_v < 15:
        return

    # ── Score: menor = mejor candidato ───────────────────────────────
    # Penalizar si el máximo está lejos de rpm_max_esperado
    penalidad_max = abs(max_v - rpm_max_esperado) / rpm_max_esperado
    # Penalizar si el mínimo no está cerca de 0 (aunque no lo exigimos)
    penalidad_min = min_v / rpm_max_esperado
    # Premiar señales con más variación (más dinámicas)
    bonus_variedad = -unique_v / len(vals)

    # ─────────────────────────────────────────────────────────────────
    if len(vals) > 1:
        cambios = [abs(vals[k+1] - vals[k]) for k in range(len(vals)-1)]
        suavidad = (sum(cambios) / len(cambios)) / rango  # 0 = suave, 1 = muy ruidosa
    else:
        suavidad = 1.0

    penalidad_ruido = suavidad * 2   # peso alto: señal ruidosa → descartada

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


# ── Ejecutar ──────────────────────────────────────────────────────────
buscar_y_graficar_rpm("Can_Datos_1.txt", rpm_max_esperado=3000)