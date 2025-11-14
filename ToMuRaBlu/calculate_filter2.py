#!/usr/bin/env python3
"""
Calculo correcto de filtro IIR pasa-banda usando forma directa II
"""

import numpy as np
from scipy import signal
import sys

# Cambiar encoding para evitar errores
sys.stdout.reconfigure(encoding='utf-8')

fs = 2024  # Hz
fc = 21    # Hz
Q = 2.333  # Factor de calidad

print("="*70)
print("DISEÑO DE FILTRO PASA-BANDA BIQUAD")
print("="*70)
print(f"\nfs = {fs} Hz")
print(f"fc = {fc} Hz")
print(f"Q = {Q}")

# Frecuencia normalizada (0 a 1, donde 1 = Nyquist)
w0 = 2 * np.pi * fc / fs
print(f"w0 = {w0:.6f} rad/sample")

# Coeficientes del filtro biquad pasa-banda
# Forma estándar: H(z) = (b0 + b1*z^-1 + b2*z^-2) / (a0 + a1*z^-1 + a2*z^-2)
# donde a0 = 1

alpha = np.sin(w0) / (2 * Q)
cos_w0 = np.cos(w0)

# Coeficientes (forma directa II)
b0 = alpha
b1 = 0
b2 = -alpha
a0 = 1 + alpha
a1 = -2 * cos_w0
a2 = 1 - alpha

# Normalizar por a0
b0 = b0 / a0
b1 = b1 / a0
b2 = b2 / a0
a1 = a1 / a0
a2 = a2 / a0

print(f"\nCoeficientes float normalizados:")
print(f"b0 = {b0:.10f}")
print(f"b1 = {b1:.10f}")
print(f"b2 = {b2:.10f}")
print(f"a1 = {a1:.10f}")
print(f"a2 = {a2:.10f}")

# Verificar que están en rango válido
print(f"\nVerificación de rango Q15:")
print(f"a1 en rango? {-1 <= a1 <= 1}: {a1}")
print(f"a2 en rango? {-1 <= a2 <= 1}: {a2}")

# Conversión Q15
def float_to_q15(val):
    q15 = int(round(val * 32768.0))
    if q15 > 32767:
        q15 = 32767
    elif q15 < -32768:
        q15 = -32768
    return q15

def q15_to_float(q15):
    return q15 / 32768.0

print("\n" + "="*70)
print("CONVERSIÓN A Q15 CON DIFERENTES GANANCIAS")
print("="*70)

for gain in [1, 2, 4, 8, 16, 32]:
    b0_scaled = b0 * gain
    b1_scaled = b1 * gain
    b2_scaled = b2 * gain

    b0_q15 = float_to_q15(b0_scaled)
    b1_q15 = float_to_q15(b1_scaled)
    b2_q15 = float_to_q15(b2_scaled)
    a1_q15 = float_to_q15(a1)
    a2_q15 = float_to_q15(a2)

    # Reconstruir
    b_recon = np.array([q15_to_float(b0_q15), q15_to_float(b1_q15), q15_to_float(b2_q15)])
    a_recon = np.array([1.0, q15_to_float(a1_q15), q15_to_float(a2_q15)])

    # Analizar respuesta
    w, h = signal.freqz(b_recon, a_recon, worN=[fc], fs=fs)
    gain_fc = abs(h[0])
    gain_fc_db = 20 * np.log10(gain_fc) if gain_fc > 0 else -np.inf

    print(f"\n--- Ganancia x{gain} ---")
    print(f"const int32_t b0_q15 = {b0_q15:6d};  // {q15_to_float(b0_q15):.6f}")
    print(f"const int32_t b1_q15 = {b1_q15:6d};  // {q15_to_float(b1_q15):.6f}")
    print(f"const int32_t b2_q15 = {b2_q15:6d};  // {q15_to_float(b2_q15):.6f}")
    print(f"const int32_t a1_q15 = {a1_q15:6d};  // {q15_to_float(a1_q15):.6f}")
    print(f"const int32_t a2_q15 = {a2_q15:6d};  // {q15_to_float(a2_q15):.6f}")
    print(f"Ganancia @ {fc} Hz: {gain_fc:.4f} ({gain_fc_db:.2f} dB)")

# Graficar respuesta del filtro con ganancia x16
print("\n" + "="*70)
print("ANÁLISIS DE RESPUESTA EN FRECUENCIA (Ganancia x16)")
print("="*70)

gain_recommended = 16
b0_q15 = float_to_q15(b0 * gain_recommended)
b1_q15 = float_to_q15(b1 * gain_recommended)
b2_q15 = float_to_q15(b2 * gain_recommended)
a1_q15 = float_to_q15(a1)
a2_q15 = float_to_q15(a2)

b_final = np.array([q15_to_float(b0_q15), q15_to_float(b1_q15), q15_to_float(b2_q15)])
a_final = np.array([1.0, q15_to_float(a1_q15), q15_to_float(a2_q15)])

freqs = [12, 15, 18, 21, 24, 27, 30]
print(f"\nRespuesta en frecuencias de interés:")
for f in freqs:
    w, h = signal.freqz(b_final, a_final, worN=[f], fs=fs)
    mag = abs(h[0])
    mag_db = 20 * np.log10(mag) if mag > 0 else -np.inf
    phase = np.angle(h[0], deg=True)
    print(f"  {f:2d} Hz: mag={mag:.4f} ({mag_db:6.2f} dB), phase={phase:7.2f}°")

print(f"\n" + "="*70)
print("RECOMENDACIONES FINALES")
print("="*70)

print(f"\n1. COEFICIENTES PARA EL CÓDIGO (Ganancia x{gain_recommended}):\n")
print(f"const int32_t b0_q15 = {float_to_q15(b0 * gain_recommended)};")
print(f"const int32_t b1_q15 = {float_to_q15(b1 * gain_recommended)};")
print(f"const int32_t b2_q15 = {float_to_q15(b2 * gain_recommended)};")
print(f"const int32_t a1_q15 = {float_to_q15(a1)};")
print(f"const int32_t a2_q15 = {float_to_q15(a2)};")

print(f"\n2. AMPLIFICACIÓN PWM SUGERIDA:")
print(f"   int32_t y_amplified = y_filtered << 4;  // x16")

print(f"\n3. FORMA CORRECTA DE LA ECUACIÓN:")
print("   int64_t num = ((int64_t)b0_q15 * x) + ((int64_t)b1_q15 * x1_q15) + ((int64_t)b2_q15 * x2_q15);")
print("   num >>= Q15_SHIFT;")
print("   int64_t den = -((int64_t)a1_q15 * y1_q15) - ((int64_t)a2_q15 * y2_q15);")
print("   den >>= Q15_SHIFT;")
print("   int32_t y_filtered = (int32_t)(num + den);")

print("\n" + "="*70 + "\n")
