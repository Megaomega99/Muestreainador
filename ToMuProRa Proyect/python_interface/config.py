"""
Configuración del servidor web
"""

# Configuración del servidor
SERVER_HOST = "0.0.0.0"  # 0.0.0.0 para acceso desde la red, 127.0.0.1 solo local
SERVER_PORT = 8000
LOG_LEVEL = "info"  # "debug", "info", "warning", "error"

# Configuración de comunicación serial
SERIAL_BAUDRATE = 115200
SERIAL_TIMEOUT = 2.0

# Configuración del filtro
DEFAULT_SAMPLE_RATE = 2000.0  # Hz

# Parámetros por defecto del filtro (21 Hz ± 9 Hz, Q=2.333)
DEFAULT_CENTER_FREQ = 21.0
DEFAULT_BANDWIDTH = 9.0

# Umbrales en unidades físicas (para la interfaz web)
# Magnitude: normalized 0-1, internamente se convierte a Q15
# Phase: degrees from ±180°, internamente se convierte a Q15
DEFAULT_MAGNITUDE_THRESHOLD_NORMALIZED = 0.05  # 50/32768 ≈ 0.0015, usando 0.05 más práctico
DEFAULT_PHASE_THRESHOLD_DEGREES = 15.0  # 2730 * 180/32768 ≈ 15°

# Valores internos Q15 (para compatibilidad con Arduino)
DEFAULT_MAGNITUDE_THRESHOLD_Q15 = 50  # Será convertido: 50 << 15
DEFAULT_PHASE_THRESHOLD_Q15 = 2730  # En formato Q15 phase units

DEFAULT_PREDICTION_DELAY = 75

# WebSocket
WS_HEARTBEAT_INTERVAL = 30  # segundos

# Límites de parámetros (unidades físicas para la interfaz)
FREQ_MIN = 1.0
FREQ_MAX = 500.0
BW_MIN = 1.0
BW_MAX = 100.0
MAG_THRESHOLD_MIN_NORMALIZED = 0.001  # Normalizado 0-1
MAG_THRESHOLD_MAX_NORMALIZED = 1.0
PHASE_THRESHOLD_MIN_DEGREES = 0.1  # Grados desde ±180°
PHASE_THRESHOLD_MAX_DEGREES = 180.0
PREDICTION_DELAY_MIN = 50  # Mínimo 50 muestras (25ms @ 2kHz) para predicción efectiva
PREDICTION_DELAY_MAX = 500
