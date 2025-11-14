#!/usr/bin/env python3
"""
Cálculo de coeficientes de filtro IIR pasa-banda para sistema de detección de pulsos
Frecuencia de muestreo: 2024 Hz (medida)
Frecuencia objetivo: 21 Hz
"""

import numpy as np
from scipy import signal

# Parámetros del sistema
fs = 2024  # Frecuencia de muestreo real medida
fc = 21    # Frecuencia central del filtro
bw = 18    # Ancho de banda (±9 Hz)

# Bandas del filtro pasa-banda
f_low = fc - bw/2   # 12 Hz
f_high = fc + bw/2  # 30 Hz

print("=" * 70)
print("DISEÑO DE FILTRO IIR PASA-BANDA")
print("=" * 70)
print(f"\nParámetros:")
print(f"  Frecuencia de muestreo: {fs} Hz")
print(f"  Frecuencia central: {fc} Hz")
print(f"  Ancho de banda: {bw} Hz ({f_low} Hz - {f_high} Hz)")
print(f"  Factor Q: {fc/bw:.3f}")

# Diseño del filtro Butterworth bandpass de orden 2
sos = signal.butter(2, [f_low, f_high], btype='bandpass', fs=fs, output='sos')
b, a = signal.butter(2, [f_low, f_high], btype='bandpass', fs=fs)

print(f"\n{'=' * 70}")
print("COEFICIENTES FLOAT")
print("=" * 70)
print(f"\nNumerador (b):")
print(f"  b0 = {b[0]:.10f}")
print(f"  b1 = {b[1]:.10f}")
print(f"  b2 = {b[2]:.10f}")
print(f"\nDenominador (a):")
print(f"  a0 = {a[0]:.10f}")
print(f"  a1 = {a[1]:.10f}")
print(f"  a2 = {a[2]:.10f}")

# Función de conversión Q15
def float_to_q15(val):
    q15 = int(round(val * 32768.0))
    if q15 > 32767:
        q15 = 32767
    elif q15 < -32768:
        q15 = -32768
    return q15

def q15_to_float(q15):
    return q15 / 32768.0

# Probar diferentes ganancias
print(f"\n{'=' * 70}")
print("CONVERSIÓN A Q15 CON DIFERENTES GANANCIAS")
print("=" * 70)

for gain in [1, 2, 4, 8, 16]:
    b_scaled = b * gain

    b0_q15 = float_to_q15(b_scaled[0])
    b1_q15 = float_to_q15(b_scaled[1])
    b2_q15 = float_to_q15(b_scaled[2])
    a1_q15 = float_to_q15(a[1])
    a2_q15 = float_to_q15(a[2])

    # Reconstruir para verificar
    b_recon = np.array([q15_to_float(b0_q15), q15_to_float(b1_q15), q15_to_float(b2_q15)])
    a_recon = np.array([1.0, q15_to_float(a1_q15), q15_to_float(a2_q15)])

    # Calcular ganancia a fc
    w, h = signal.freqz(b_recon, a_recon, worN=[fc], fs=fs)
    gain_fc = abs(h[0])
    gain_fc_db = 20 * np.log10(gain_fc)

    print(f"\n--- Ganancia x{gain} ---")
    print(f"const int32_t b0_q15 = {b0_q15};   // {q15_to_float(b0_q15):.6f}")
    print(f"const int32_t b1_q15 = {b1_q15};   // {q15_to_float(b1_q15):.6f}")
    print(f"const int32_t b2_q15 = {b2_q15};   // {q15_to_float(b2_q15):.6f}")
    print(f"const int32_t a1_q15 = {a1_q15};   // {q15_to_float(a1_q15):.6f}")
    print(f"const int32_t a2_q15 = {a2_q15};   // {q15_to_float(a2_q15):.6f}")
    print(f"Ganancia @ {fc} Hz: {gain_fc:.4f} ({gain_fc_db:.2f} dB)")

# Analizar filtro RC analógico
R = 10e3   # 10 kΩ
C = 100e-9 # 100 nF
fc_rc = 1 / (2 * np.pi * R * C)

print(f"\n{'=' * 70}")
print("FILTRO RC ANALÓGICO")
print("=" * 70)
print(f"\nParámetros:")
print(f"  R = {R/1e3:.1f} kΩ")
print(f"  C = {C*1e9:.1f} nF")
print(f"  fc = {fc_rc:.2f} Hz")

# Calcular atenuación a 21 Hz
H_rc = 1 / (1 + 1j * fc / fc_rc)
mag_rc = abs(H_rc)
mag_rc_db = 20 * np.log10(mag_rc)
phase_rc = np.angle(H_rc, deg=True)

print(f"\nRespuesta @ {fc} Hz:")
print(f"  Magnitud: {mag_rc:.4f} ({mag_rc_db:.2f} dB)")
print(f"  Fase: {phase_rc:.2f}°")
print(f"  Atenuación: {1/mag_rc:.2f}x")

# Recomendaciones
print(f"\n{'=' * 70}")
print("RECOMENDACIONES")
print("=" * 70)

# Ganancia recomendada: 4x es un buen balance
gain_recommended = 4
b_scaled = b * gain_recommended

b0_q15 = float_to_q15(b_scaled[0])
b1_q15 = float_to_q15(b_scaled[1])
b2_q15 = float_to_q15(b_scaled[2])
a1_q15 = float_to_q15(a[1])
a2_q15 = float_to_q15(a[2])

b_recon = np.array([q15_to_float(b0_q15), q15_to_float(b1_q15), q15_to_float(b2_q15)])
a_recon = np.array([1.0, q15_to_float(a1_q15), q15_to_float(a2_q15)])
w, h = signal.freqz(b_recon, a_recon, worN=[fc], fs=fs)
gain_digital = abs(h[0])

total_gain = gain_digital * mag_rc
pwm_gain = int(2 ** np.ceil(np.log2(1 / total_gain)))

print(f"\n1. COEFICIENTES DEL FILTRO IIR (Ganancia x{gain_recommended}):")
print(f"   const int32_t b0_q15 = {b0_q15};")
print(f"   const int32_t b1_q15 = {b1_q15};")
print(f"   const int32_t b2_q15 = {b2_q15};")
print(f"   const int32_t a1_q15 = {a1_q15};")
print(f"   const int32_t a2_q15 = {a2_q15};")

print(f"\n2. AMPLIFICACIÓN PWM:")
print(f"   Ganancia digital @ {fc} Hz: {gain_digital:.3f}")
print(f"   Atenuación RC @ {fc} Hz: {mag_rc:.3f}")
print(f"   Ganancia total: {total_gain:.3f}")
print(f"   PWM amplificación: x{pwm_gain} (shift left {int(np.log2(pwm_gain))})")
print(f"   Código: y_amplified = y_filtered << {int(np.log2(pwm_gain))};")

print(f"\n3. ECUACIÓN DEL FILTRO:")
print("   // y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]")
print("   int64_t num = ((int64_t)b0_q15 * x) + ((int64_t)b1_q15 * x1_q15) + ((int64_t)b2_q15 * x2_q15);")
print("   num >>= Q15_SHIFT;")
print("   int64_t den = -((int64_t)a1_q15 * y1_q15) - ((int64_t)a2_q15 * y2_q15);")
print("   den >>= Q15_SHIFT;")
print("   int32_t y_filtered = (int32_t)(num + den);")

print(f"\n{'=' * 70}\n")
