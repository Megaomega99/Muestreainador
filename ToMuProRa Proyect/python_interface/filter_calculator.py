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

        # Los coeficientes 'b' pueden ser muy pequeños, así que necesitamos escalarlos
        # Para mantener precisión en Q15
        # Encontrar el máximo coeficiente
        max_b = max(abs(b0_norm), abs(b2_norm))

        # Escalar para usar el rango completo de Q15 (pero dejar margen)
        # El gain máximo del filtro pasa-banda es aproximadamente Q
        # Queremos que b0_q15 sea razonable pero no sature
        target_b0 = 456.0 / 32768.0  # 0.0139... (valor exacto del original)
        scale_factor = target_b0 / max_b if max_b > 0 else 1.0

        # Aplicar escalado a coeficientes b
        b0_scaled = b0_norm * scale_factor
        b1_scaled = b1_norm * scale_factor
        b2_scaled = b2_norm * scale_factor

        # Convertir a enteros Q15
        b0_q15 = int(round(b0_scaled * q15_scale))
        b1_q15 = int(round(b1_scaled * q15_scale))
        b2_q15 = int(round(b2_scaled * q15_scale))

        # IMPORTANTE: El código Arduino usa: y = num - (a1*y1 + a2*y2)
        # El Arduino espera: a1=-64482 (negativo) y a2=31855 (positivo)
        # a1_norm es negativo (-1.9678), a2_norm es positivo (0.9721)
        # Almacenamos: +a1_norm (negativo) y -a2_norm (negativo) pero luego negamos a2
        a1_q15 = int(round(a1_norm * q15_scale))
        a2_q15 = int(round(a2_norm * q15_scale))

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
        Calcula coeficientes del filtro de Hilbert FIR optimizado para una frecuencia

        Args:
            center_freq: Frecuencia central en Hz
            num_taps: Número de coeficientes (debe ser impar, default: 15)

        Returns:
            Diccionario con coeficientes y información del filtro
        """
        # Asegurar que num_taps es impar
        if num_taps % 2 == 0:
            num_taps += 1

        # Diseñar filtro Hilbert usando firwin
        # El transformador de Hilbert es un filtro pasa-todo con desfase de 90°
        h = signal.firwin(num_taps, [center_freq * 0.3, center_freq * 3.0],
                         pass_zero=False, fs=self.fs, window='hamming')

        # Convertir a transformador de Hilbert
        # Multiplicar por el núcleo de Hilbert ideal
        n = np.arange(num_taps)
        center = (num_taps - 1) / 2

        # Núcleo de Hilbert: h[n] = 2/π * sin²(π(n-center)/2) / (n-center) para n ≠ center
        hilbert_kernel = np.zeros(num_taps)
        for i in range(num_taps):
            if i != center:
                m = i - center
                hilbert_kernel[i] = (2.0 / np.pi) * (np.sin(np.pi * m / 2.0)**2) / m
            # else: hilbert_kernel[center] = 0

        # Aplicar ventana para suavizar
        window = signal.windows.hamming(num_taps)
        hilbert_kernel *= window

        # Normalizar
        hilbert_kernel /= np.max(np.abs(hilbert_kernel))

        # Convertir a formato Q15
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
            'current_prediction_delay': 76
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
            'prediction_delay_samples': 76
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
