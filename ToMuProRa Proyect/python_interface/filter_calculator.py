"""
Módulo para calcular coeficientes de filtros IIR pasa-banda
Utiliza scipy para diseñar filtros digitales
"""

import numpy as np
from scipy import signal
from typing import Tuple, Dict


class FilterCalculator:
    """Calcula coeficientes para filtro IIR pasa-banda"""

    def __init__(self, fs: float = 2000.0):
        """
        Inicializa el calculador de filtros

        Args:
            fs: Frecuencia de muestreo en Hz (default: 2000 Hz)
        """
        self.fs = fs

    def calculate_bandpass_coefficients(self,
                                       center_freq: float,
                                       bandwidth: float = 9.0,
                                       q_factor: float = 2.333) -> Dict[str, any]:
        """
        Calcula los coeficientes del filtro IIR pasa-banda usando diseño biquad

        Args:
            center_freq: Frecuencia central del filtro en Hz
            bandwidth: Ancho de banda en Hz (default: 9 Hz)
            q_factor: Factor de calidad del filtro (default: 2.333)

        Returns:
            Diccionario con coeficientes y información del filtro
        """
        # Diseño de filtro biquad pasa-banda
        # Basado en las ecuaciones de Audio EQ Cookbook de Robert Bristow-Johnson

        # Frecuencia normalizada (omega)
        w0 = 2.0 * np.pi * center_freq / self.fs

        # Calcular Q desde el ancho de banda si se proporciona
        # Q = fc / bandwidth
        if bandwidth > 0:
            q_factor = center_freq / bandwidth

        # Calcular alpha
        alpha = np.sin(w0) / (2.0 * q_factor)

        # Coeficientes del filtro pasa-banda (sin normalizar)
        b0 = alpha
        b1 = 0.0
        b2 = -alpha

        a0 = 1.0 + alpha
        a1 = -2.0 * np.cos(w0)
        a2 = 1.0 - alpha

        # Normalizar por a0
        b0_norm = b0 / a0
        b1_norm = b1 / a0
        b2_norm = b2 / a0
        a1_norm = a1 / a0
        a2_norm = a2 / a0

        # Preparar arrays para scipy
        b = np.array([b0_norm, b1_norm, b2_norm])
        a = np.array([1.0, a1_norm, a2_norm])

        # Convertir a formato Q15 para Arduino
        q15_scale = 2**15

        # Convertir coeficientes normalizados directamente a Q15
        # NO aplicar escalado adicional - esto rompía el filtro
        # Los coeficientes ya están normalizados por a0
        b0_q15 = int(round(b0_norm * q15_scale))
        b1_q15 = int(round(b1_norm * q15_scale))
        b2_q15 = int(round(b2_norm * q15_scale))
        a1_q15 = int(round(a1_norm * q15_scale))
        a2_q15 = int(round(a2_norm * q15_scale))

        # Para compatibilidad con código existente
        scale_factor = 1.0

        # Calcular respuesta en frecuencia
        w, h = signal.freqz(b, a, worN=2048, fs=self.fs)

        # Calcular retraso de grupo (phase delay)
        try:
            # Suprimir warnings de scipy para frecuencias singulares
            import warnings
            with warnings.catch_warnings():
                warnings.filterwarnings('ignore', category=RuntimeWarning)
                warnings.filterwarnings('ignore', category=UserWarning)
                w_delay, gd = signal.group_delay((b, a), fs=self.fs)

                # Reemplazar NaN e Inf con valores válidos
                gd = np.nan_to_num(gd, nan=0.0, posinf=0.0, neginf=0.0)

                # Interpolar en la frecuencia central
                center_delay_samples = np.interp(center_freq, w_delay, gd)

                # Si el resultado no es válido, usar estimación
                if not np.isfinite(center_delay_samples) or center_delay_samples <= 0:
                    center_delay_samples = 1.0 / (2.0 * np.pi * bandwidth) * self.fs
        except:
            # Si falla, usar una estimación aproximada
            center_delay_samples = 1.0 / (2.0 * np.pi * bandwidth) * self.fs

        return {
            'b': b,
            'a': a,
            'b0_q15': b0_q15,
            'b1_q15': b1_q15,
            'b2_q15': b2_q15,
            'a1_q15': a1_q15,
            'a2_q15': a2_q15,
            'frequency': w,
            'magnitude_db': 20 * np.log10(np.abs(h) + 1e-10),
            'phase': np.angle(h),
            'center_delay_samples': center_delay_samples,
            'center_delay_ms': (center_delay_samples / self.fs) * 1000,
            'center_freq': center_freq,
            'bandwidth': bandwidth,
            'q_factor': q_factor,
            'scale_factor': scale_factor
        }

    def calculate_hilbert_coefficients(self,
                                     center_freq: float,
                                     num_taps: int = 15) -> Dict[str, any]:
        """
        Calcula coeficientes del filtro de Hilbert FIR para desfase de 90 grados

        El filtro de Hilbert se usa para obtener la componente en cuadratura (imaginaria)
        de la señal analítica, permitiendo calcular la envolvente instantánea.

        Método simplificado:
        1. Hilbert ideal: h[n] = (2/π) * sin²(πn/2) / n
        2. Ventana Hamming para reducir efecto Gibbs
        3. Normalización a 0.2 para evitar saturación Q15

        Args:
            center_freq: Frecuencia central en Hz (usado solo para referencia)
            num_taps: Número de coeficientes (debe ser impar, default: 15)

        Returns:
            Diccionario con coeficientes y información del filtro
        """
        # Asegurar que num_taps es impar
        if num_taps % 2 == 0:
            num_taps += 1

        # Crear núcleo ideal del transformador de Hilbert
        # h[n] = 2/π * sin²(π*n/2) / n para n ≠ 0, h[0] = 0
        center = (num_taps - 1) // 2
        n = np.arange(num_taps) - center

        # Núcleo de Hilbert ideal
        hilbert_kernel = np.zeros(num_taps)
        for i, ni in enumerate(n):
            if ni != 0:
                # Fórmula del filtro de Hilbert: h[n] = (2/π) * sin²(πn/2) / n
                hilbert_kernel[i] = (2.0 / np.pi) * (np.sin(np.pi * ni / 2.0)**2) / ni
            # h[0] = 0 (ya inicializado)

        # Aplicar ventana de Hamming para reducir oscilaciones (efecto Gibbs)
        window = signal.windows.hamming(num_taps)
        hilbert_kernel *= window

        # Normalizar para que el valor máximo use el rango de Q15 eficientemente
        # Usar 0.2 como máximo para evitar saturación (esto es ~6554 en Q15)
        max_coeff = np.max(np.abs(hilbert_kernel))
        if max_coeff > 0:
            hilbert_kernel = hilbert_kernel * (0.2 / max_coeff)

        # Convertir a formato Q15 (escala de 2^15 = 32768)
        q15_scale = 2**15
        hilbert_coeffs_q15 = np.round(hilbert_kernel * q15_scale).astype(np.int16)

        # Calcular respuesta en frecuencia
        w, h_response = signal.freqz(hilbert_kernel, worN=2048, fs=self.fs)

        return {
            'coefficients': hilbert_kernel,
            'coefficients_q15': hilbert_coeffs_q15.tolist(),
            'num_taps': num_taps,
            'frequency': w,
            'magnitude': np.abs(h_response),
            'magnitude_db': 20 * np.log10(np.abs(h_response) + 1e-10),
            'phase': np.angle(h_response),
            'center_freq': center_freq,
            'delay_samples': (num_taps - 1) / 2.0
        }

    def estimate_total_delay(self,
                           iir_delay: float,
                           hilbert_taps: int = 15) -> Dict[str, float]:
        """
        Estima el retraso total del sistema

        Args:
            iir_delay: Retraso del filtro IIR en muestras
            hilbert_taps: Número de taps del filtro Hilbert (default: 15)

        Returns:
            Diccionario con información de retrasos
        """
        # Retraso del filtro FIR Hilbert (aproximadamente N/2 muestras)
        hilbert_delay = (hilbert_taps - 1) / 2.0

        # Retraso total en muestras
        total_delay_samples = iir_delay + hilbert_delay

        # Convertir a milisegundos
        total_delay_ms = (total_delay_samples / self.fs) * 1000

        # Calcular muestras para compensación
        # El valor actual es PREDICTION_DELAY_SAMPLES = 76
        recommended_delay = int(round(total_delay_samples))

        return {
            'iir_delay_samples': iir_delay,
            'hilbert_delay_samples': hilbert_delay,
            'total_delay_samples': total_delay_samples,
            'total_delay_ms': total_delay_ms,
            'recommended_prediction_delay': recommended_delay,
            'current_prediction_delay': 75
        }

    def get_default_coefficients(self) -> Dict[str, any]:
        """
        Retorna los coeficientes actuales del código Arduino (21 Hz, Q=2.333)

        Returns:
            Diccionario con coeficientes por defecto
        """
        return {
            'b0_q15': 456,
            'b1_q15': 0,
            'b2_q15': -456,
            'a1_q15': -64482,
            'a2_q15': 31855,
            'center_freq': 21.0,
            'bandwidth': 9.0,
            'q_factor': 2.333,
            'prediction_delay_samples': 75
        }


if __name__ == "__main__":
    # Prueba del módulo
    calc = FilterCalculator(fs=2000.0)

    # Calcular filtro por defecto (21 Hz)
    result = calc.calculate_bandpass_coefficients(21.0, 9.0)

    print("Coeficientes del filtro (21 Hz ± 9 Hz):")
    print(f"b0_q15 = {result['b0_q15']}")
    print(f"b1_q15 = {result['b1_q15']}")
    print(f"b2_q15 = {result['b2_q15']}")
    print(f"a1_q15 = {result['a1_q15']}")
    print(f"a2_q15 = {result['a2_q15']}")
    print(f"\nRetraso en frecuencia central: {result['center_delay_samples']:.2f} muestras")
    print(f"Retraso en frecuencia central: {result['center_delay_ms']:.2f} ms")

    # Estimar retraso total
    delay_info = calc.estimate_total_delay(result['center_delay_samples'])
    print(f"\nRetraso total estimado: {delay_info['total_delay_samples']:.2f} muestras")
    print(f"Retraso total estimado: {delay_info['total_delay_ms']:.2f} ms")
    print(f"PREDICTION_DELAY_SAMPLES recomendado: {delay_info['recommended_prediction_delay']}")
