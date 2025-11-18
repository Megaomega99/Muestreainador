"""
Script de verificación del filtro IIR
Verifica que los coeficientes calculados produzcan la respuesta correcta
"""

from filter_calculator import FilterCalculator
from scipy import signal
import numpy as np

def verify_filter(center_freq=21.0, bandwidth=9.0):
    print("=" * 60)
    print(f"VERIFICACIÓN DEL FILTRO: {center_freq} Hz ± {bandwidth/2} Hz")
    print("=" * 60)

    calc = FilterCalculator(fs=2000.0)
    result = calc.calculate_bandpass_coefficients(center_freq, bandwidth)

    print("\nCoeficientes Normalizados:")
    print(f"  b = [{result['b'][0]:.6f}, {result['b'][1]:.6f}, {result['b'][2]:.6f}]")
    print(f"  a = [1.0, {result['a'][1]:.6f}, {result['a'][2]:.6f}]")

    print("\nCoeficientes Q15 (para Arduino):")
    print(f"  b0_q15 = {result['b0_q15']}")
    print(f"  b1_q15 = {result['b1_q15']}")
    print(f"  b2_q15 = {result['b2_q15']}")
    print(f"  a1_q15 = {result['a1_q15']}")
    print(f"  a2_q15 = {result['a2_q15']}")

    # Verificar respuesta en frecuencia
    print("\nRespuesta en Frecuencia:")
    w, h = signal.freqz(result['b'], result['a'], worN=8192, fs=2000.0)

    # Frecuencias de prueba
    test_freqs = [3.0, 10.0, 15.0, center_freq, 25.0, 30.0, 50.0]

    print(f"\n{'Frecuencia (Hz)':<15} {'Ganancia (dB)':<15} {'Ganancia Lineal':<20} {'Estado'}")
    print("-" * 75)

    for test_f in test_freqs:
        idx = np.argmin(np.abs(w - test_f))
        mag_db = 20 * np.log10(np.abs(h[idx]) + 1e-10)
        mag_linear = np.abs(h[idx])

        # Determinar si debe pasar o atenuar
        lower_freq = center_freq - bandwidth/2
        upper_freq = center_freq + bandwidth/2

        if lower_freq <= test_f <= upper_freq:
            expected = "OK DEBE PASAR"
            status = "OK CORRECTO" if mag_db > -6 else "XX ERROR"
        else:
            expected = "OK DEBE ATENUAR"
            status = "OK CORRECTO" if mag_db < -10 else "XX ERROR"

        print(f"{test_f:<15.1f} {mag_db:<15.2f} {mag_linear:<20.4f} {expected} - {status}")

    # Encontrar pico
    peak_idx = np.argmax(np.abs(h))
    peak_freq = w[peak_idx]
    peak_mag_db = 20 * np.log10(np.abs(h[peak_idx]) + 1e-10)

    print("\nPico del Filtro:")
    print(f"  Frecuencia: {peak_freq:.2f} Hz (esperado: {center_freq} Hz)")
    print(f"  Ganancia: {peak_mag_db:.2f} dB")

    error_freq = abs(peak_freq - center_freq)
    if error_freq < 0.5:
        print(f"  OK Pico en frecuencia correcta (error: {error_freq:.3f} Hz)")
    else:
        print(f"  XX Pico desplazado (error: {error_freq:.3f} Hz)")

    print("\n" + "=" * 60)
    print("CONCLUSION:")
    if error_freq < 0.5:
        print("OK EL FILTRO ESTA FUNCIONANDO CORRECTAMENTE")
        print("   - Pasa la banda deseada")
        print("   - Atenua frecuencias fuera de banda")
    else:
        print("XX EL FILTRO TIENE PROBLEMAS")
        print("   - Revisar implementacion en Arduino")
        print("   - Verificar que se enviaron los coeficientes correctos")
    print("=" * 60)

if __name__ == "__main__":
    # Verificar filtro por defecto (21 Hz)
    verify_filter(center_freq=21.0, bandwidth=9.0)

    print("\n\n")

    # Verificar otro filtro (ej: 15 Hz más estrecho)
    verify_filter(center_freq=15.0, bandwidth=5.0)
