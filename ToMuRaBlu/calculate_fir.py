#!/usr/bin/env python3
"""
Diseño de filtro FIR pasa-banda para reemplazar el IIR inestable
"""

import numpy as np
from scipy import signal
import sys

sys.stdout.reconfigure(encoding='utf-8')

fs = 2024  # Hz
fc = 21    # Hz
bw = 30    # Ancho de banda (12-42 Hz aprox)

f_low = 12   # Hz
f_high = 42  # Hz

print("="*70)
print("DISEÑO DE FILTRO FIR PASA-BANDA")
print("="*70)
print(f"\nfs = {fs} Hz")
print(f"fc = {fc} Hz")
print(f"Banda: {f_low}-{f_high} Hz")

# Probar diferentes órdenes
for numtaps in [31, 51, 71]:
    print(f"\n{'='*70}")
    print(f"FIR de orden {numtaps}")
    print(f"{'='*70}")

    # Diseñar filtro FIR pasa-banda
    fir_coefs = signal.firwin(numtaps, [f_low, f_high], pass_zero=False, fs=fs)

    # Aplicar ganancia
    gain = 8
    fir_coefs_scaled = fir_coefs * gain

    # Convertir a Q15
    fir_q15 = [int(round(c * 32768)) for c in fir_coefs_scaled]

    # Saturar
    fir_q15 = [max(-32768, min(32767, val)) for val in fir_q15]

    print(f"\nCoeficientes Q15 (ganancia x{gain}):")
    print("const int16_t fir_coefs[{}] = {{".format(numtaps))
    for i in range(0, len(fir_q15), 8):
        line = "  " + ", ".join(f"{val:6d}" for val in fir_q15[i:i+8])
        if i + 8 < len(fir_q15):
            print(line + ",")
        else:
            print(line)
    print("};")

    # Analizar respuesta
    b_recon = np.array([val/32768 for val in fir_q15])

    freqs_test = [12, 15, 18, 21, 24, 27, 30, 35, 40, 50]
    print(f"\nRespuesta en frecuencia:")
    for f in freqs_test:
        w, h = signal.freqz(b_recon, [1], worN=[f], fs=fs)
        mag = abs(h[0])
        mag_db = 20 * np.log10(mag) if mag > 0 else -np.inf
        print(f"  {f:3d} Hz: {mag_db:7.2f} dB")

    # Calcular delay de grupo
    w, gd = signal.group_delay((b_recon, [1]), fs=fs)
    idx = np.argmin(np.abs(w - fc))
    delay_samples = int(gd[idx])
    print(f"\nDelay de grupo @ {fc} Hz: {delay_samples} samples ({delay_samples/fs*1000:.1f} ms)")

print("\n" + "="*70)
print("RECOMENDACIÓN: FIR de orden 51 (buen balance)")
print("="*70)
