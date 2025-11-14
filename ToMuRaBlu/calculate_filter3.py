#!/usr/bin/env python3
"""
Diseño de filtro con Q más bajo para evitar saturación de a1
"""

import numpy as np
from scipy import signal
import sys

sys.stdout.reconfigure(encoding='utf-8')

fs = 2024  # Hz
fc = 21    # Hz

print("="*70)
print("BÚSQUEDA DE Q ÓPTIMO (sin saturación de a1)")
print("="*70)

# Probar diferentes valores de Q
for Q in [0.5, 0.707, 1.0, 1.5, 2.0]:
    w0 = 2 * np.pi * fc / fs
    alpha = np.sin(w0) / (2 * Q)
    cos_w0 = np.cos(w0)

    b0 = alpha
    b1 = 0
    b2 = -alpha
    a0 = 1 + alpha
    a1 = -2 * cos_w0
    a2 = 1 - alpha

    # Normalizar
    b0 = b0 / a0
    b1 = b1 / a0
    b2 = b2 / a0
    a1 = a1 / a0
    a2 = a2 / a0

    # Verificar rango
    a1_in_range = (-1.0 < a1 < 1.0)

    print(f"\n{'='*70}")
    print(f"Q = {Q}")
    print(f"{'='*70}")
    print(f"a1 = {a1:.6f}  {'✓ EN RANGO' if a1_in_range else '✗ FUERA DE RANGO (se saturará)'}")
    print(f"a2 = {a2:.6f}")

    if a1_in_range:
        # Calcular BW
        bw_hz = fc / Q
        print(f"Ancho de banda: {bw_hz:.1f} Hz")

        # Conversión Q15 con ganancia x16
        gain = 16
        b0_q15 = int(round(b0 * gain * 32768))
        b1_q15 = int(round(b1 * gain * 32768))
        b2_q15 = int(round(b2 * gain * 32768))
        a1_q15 = int(round(a1 * 32768))
        a2_q15 = int(round(a2 * 32768))

        # Saturar
        def saturate(val):
            return max(-32768, min(32767, val))

        b0_q15 = saturate(b0_q15)
        b1_q15 = saturate(b1_q15)
        b2_q15 = saturate(b2_q15)
        a1_q15 = saturate(a1_q15)
        a2_q15 = saturate(a2_q15)

        print(f"\nCoeficientes Q15 (ganancia x{gain}):")
        print(f"const int32_t b0_q15 = {b0_q15};")
        print(f"const int32_t b1_q15 = {b1_q15};")
        print(f"const int32_t b2_q15 = {b2_q15};")
        print(f"const int32_t a1_q15 = {a1_q15};")
        print(f"const int32_t a2_q15 = {a2_q15};")

        # Analizar respuesta
        b_recon = np.array([b0_q15/32768, b1_q15/32768, b2_q15/32768])
        a_recon = np.array([1.0, a1_q15/32768, a2_q15/32768])

        freqs = [12, 15, 18, 21, 24, 27, 30, 40, 50]
        print(f"\nRespuesta en frecuencia:")
        for f in freqs:
            w, h = signal.freqz(b_recon, a_recon, worN=[f], fs=fs)
            mag = abs(h[0])
            mag_db = 20 * np.log10(mag) if mag > 0 else -np.inf
            print(f"  {f:3d} Hz: {mag_db:7.2f} dB")

print("\n" + "="*70)
print("RECOMENDACIÓN: Usar Q=0.707 (Butterworth)")
print("="*70)
