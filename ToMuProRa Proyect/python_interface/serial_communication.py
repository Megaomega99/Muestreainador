"""
Módulo para comunicación serial con Arduino
Protocolo de comunicación para enviar parámetros del filtro
"""

import serial
import time
import struct
from typing import Dict, Optional


class ArduinoController:
    """Controlador de comunicación serial con Arduino"""

    def __init__(self, baudrate: int = 115200, timeout: float = 2.0):
        """
        Inicializa el controlador

        Args:
            baudrate: Velocidad de comunicación (default: 115200)
            timeout: Timeout para operaciones seriales en segundos
        """
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_port: Optional[serial.Serial] = None
        self.connected = False

    def connect(self, port: str) -> bool:
        """
        Conecta al Arduino en el puerto especificado

        Args:
            port: Puerto serial (ej: 'COM3', '/dev/ttyUSB0')

        Returns:
            True si la conexión fue exitosa
        """
        try:
            self.serial_port = serial.Serial(
                port=port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                write_timeout=self.timeout
            )

            # Esperar a que Arduino se reinicie
            time.sleep(2)

            # Limpiar buffer
            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()

            self.connected = True
            return True

        except serial.SerialException as e:
            print(f"Error al conectar: {e}")
            self.connected = False
            return False

    def disconnect(self):
        """Desconecta del Arduino"""
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        self.connected = False

    def send_filter_coefficients(self, params: Dict[str, int]) -> bool:
        """
        Envía los coeficientes del filtro al Arduino usando protocolo de texto

        Protocolo simplificado (texto ASCII):
        SET_ALL:b0,b1,b2,a1,a2,mag,phase,delay\n

        Args:
            params: Diccionario con los parámetros:
                - b0_q15, b1_q15, b2_q15, a1_q15, a2_q15 (coeficientes)
                - magnitude_threshold, phase_threshold (umbrales)
                - prediction_delay (retraso de predicción)

        Returns:
            True si el envío fue exitoso
        """
        if not self.connected or not self.serial_port:
            print("Error: No hay conexión con Arduino")
            return False

        try:
            # Construir comando de texto
            cmd = (f"SET_ALL:"
                   f"{params['b0_q15']},"
                   f"{params['b1_q15']},"
                   f"{params['b2_q15']},"
                   f"{params['a1_q15']},"
                   f"{params['a2_q15']},"
                   f"{params['magnitude_threshold']},"
                   f"{params['phase_threshold']},"
                   f"{params['prediction_delay']}\n")

            # Debug: Mostrar comando que se enviará
            print(f"\n=== ENVIANDO A ARDUINO ===")
            print(f"Comando: {cmd.strip()}")
            print(f"  b0_q15 = {params['b0_q15']}")
            print(f"  b1_q15 = {params['b1_q15']}")
            print(f"  b2_q15 = {params['b2_q15']}")
            print(f"  a1_q15 = {params['a1_q15']}")
            print(f"  a2_q15 = {params['a2_q15']}")
            print(f"  magnitude_threshold = {params['magnitude_threshold']}")
            print(f"  phase_threshold = {params['phase_threshold']}")
            print(f"  prediction_delay = {params['prediction_delay']}")

            # Intentar enviar comando con retries
            max_retries = 3
            for attempt in range(max_retries):
                if attempt > 0:
                    print(f"\n  Reintento {attempt + 1}/{max_retries}...")

                # Limpiar buffer de entrada y esperar entre paquetes binarios
                # (500Hz = 2ms entre paquetes, esperamos 20ms para asegurar)
                time.sleep(0.02)
                if self.serial_port.in_waiting > 0:
                    discarded = self.serial_port.read(self.serial_port.in_waiting)
                    print(f"  Buffer limpiado: {len(discarded)} bytes")

                # Enviar comando
                self.serial_port.write(cmd.encode('ascii'))
                self.serial_port.flush()

                # Esperar a que se procese
                time.sleep(0.15)

                # Leer todas las respuestas (incluye DEBUG y respuesta final)
                success = False
                error_received = False
                max_read_attempts = 10

                for _ in range(max_read_attempts):
                    response = self.read_line(timeout=0.2)
                    if response:
                        print(f"  <- {response}")

                        if response.startswith("OK:"):
                            print("✓ Parámetros configurados correctamente")
                            return True
                        elif response.startswith("ERROR:"):
                            print(f"✗ Error del Arduino: {response}")
                            error_received = True
                            break
                        # Ignorar líneas DEBUG
                    else:
                        break

                # Si recibimos error, reintentar
                if error_received:
                    continue

                # Si no hubo respuesta, reintentar
                if not success and attempt < max_retries - 1:
                    continue

            print("⚠ No se recibió confirmación del Arduino después de todos los intentos")
            return False

        except Exception as e:
            print(f"Error al enviar coeficientes: {e}")
            return False

    def reset_to_defaults(self) -> bool:
        """
        Restaura los parámetros del Arduino a valores por defecto

        Returns:
            True si el comando fue exitoso
        """
        if not self.connected or not self.serial_port:
            print("Error: No hay conexión con Arduino")
            return False

        try:
            cmd = "RESET\n"
            print("\n=== RESTAURANDO VALORES POR DEFECTO ===")

            # Intentar enviar comando con retries
            max_retries = 3
            for attempt in range(max_retries):
                if attempt > 0:
                    print(f"\n  Reintento {attempt + 1}/{max_retries}...")

                # Limpiar buffer de entrada y esperar entre paquetes binarios
                # (500Hz = 2ms entre paquetes, esperamos 20ms para asegurar)
                time.sleep(0.02)
                if self.serial_port.in_waiting > 0:
                    discarded = self.serial_port.read(self.serial_port.in_waiting)
                    print(f"  Buffer limpiado: {len(discarded)} bytes")

                # Enviar comando
                self.serial_port.write(cmd.encode('ascii'))
                self.serial_port.flush()

                # Esperar a que se procese
                time.sleep(0.15)

                # Leer todas las respuestas (incluye DEBUG y respuesta final)
                success = False
                error_received = False
                max_read_attempts = 10

                for _ in range(max_read_attempts):
                    response = self.read_line(timeout=0.2)
                    if response:
                        print(f"  <- {response}")

                        if response.startswith("OK:"):
                            print("✓ Valores por defecto restaurados")
                            return True
                        elif response.startswith("ERROR:"):
                            print(f"✗ Error del Arduino: {response}")
                            error_received = True
                            break
                        # Ignorar líneas DEBUG
                    else:
                        break

                # Si recibimos error, reintentar
                if error_received:
                    continue

            print("⚠ No se recibió confirmación del Arduino después de todos los intentos")
            return False

        except Exception as e:
            print(f"Error al restaurar valores por defecto: {e}")
            return False

    def get_current_parameters(self) -> Optional[Dict[str, int]]:
        """
        Solicita los parámetros actuales del Arduino

        Returns:
            Diccionario con los parámetros actuales o None si hubo error
        """
        if not self.connected or not self.serial_port:
            print("Error: No hay conexión con Arduino")
            return None

        try:
            # Limpiar buffer de entrada
            if self.serial_port.in_waiting > 0:
                self.serial_port.read(self.serial_port.in_waiting)

            cmd = "GET_STATUS\n"
            print("\n=== SOLICITANDO PARÁMETROS ACTUALES ===")

            self.serial_port.write(cmd.encode('ascii'))
            self.serial_port.flush()

            time.sleep(0.1)

            # Leer todas las respuestas hasta encontrar STATUS
            max_attempts = 10
            for _ in range(max_attempts):
                response = self.read_line(timeout=0.2)
                if response:
                    print(f"  <- {response}")

                    if response.startswith("STATUS:"):
                        # Parsear respuesta: STATUS:b0,b1,b2,a1,a2,mag,phase,delay
                        values = response.split(':')[1].split(',')
                        if len(values) == 8:
                            params = {
                                'b0_q15': int(values[0]),
                                'b1_q15': int(values[1]),
                                'b2_q15': int(values[2]),
                                'a1_q15': int(values[3]),
                                'a2_q15': int(values[4]),
                                'magnitude_threshold': int(values[5]),
                                'phase_threshold': int(values[6]),
                                'prediction_delay': int(values[7])
                            }
                            print("✓ Parámetros obtenidos correctamente")
                            return params
                    elif response.startswith("ERROR:"):
                        print(f"✗ Error del Arduino: {response}")
                        return None
                else:
                    break

            print("⚠ No se pudo obtener respuesta válida del Arduino")
            return None

        except Exception as e:
            print(f"Error al solicitar parámetros: {e}")
            return None

    def send_hilbert_coefficients(self, coefficients: list) -> bool:
        """
        Envía los coeficientes del filtro de Hilbert al Arduino

        Args:
            coefficients: Lista de 15 coeficientes en formato Q15

        Returns:
            True si el envío fue exitoso
        """
        if not self.connected or not self.serial_port:
            print("Error: No hay conexión con Arduino")
            return False

        if len(coefficients) != 15:
            print(f"Error: Se esperan 15 coeficientes, recibidos {len(coefficients)}")
            return False

        try:
            # Limpiar buffer de entrada
            if self.serial_port.in_waiting > 0:
                self.serial_port.read(self.serial_port.in_waiting)

            # Construir comando: SET_HILBERT:h0,h1,h2,...,h14
            coeffs_str = ','.join([str(int(c)) for c in coefficients])
            cmd = f"SET_HILBERT:{coeffs_str}\n"

            print(f"\n=== ENVIANDO COEFICIENTES HILBERT ===")
            print(f"Comando: SET_HILBERT:[{len(coefficients)} coeficientes]")

            # Enviar comando
            self.serial_port.write(cmd.encode('ascii'))
            self.serial_port.flush()

            time.sleep(0.1)

            # Leer todas las respuestas
            success = False
            max_attempts = 10
            for _ in range(max_attempts):
                response = self.read_line(timeout=0.2)
                if response:
                    print(f"  <- {response}")

                    if response.startswith("OK:"):
                        print("✓ Coeficientes Hilbert configurados correctamente")
                        success = True
                        break
                    elif response.startswith("ERROR:"):
                        print(f"✗ Error del Arduino: {response}")
                        return False
                else:
                    break

            if success:
                return True

            print("⚠ No se recibió confirmación del Arduino")
            return False

        except Exception as e:
            print(f"Error al enviar coeficientes Hilbert: {e}")
            return False

    def send_command(self, command: str) -> bool:
        """
        Envía un comando de texto al Arduino

        Args:
            command: Comando a enviar

        Returns:
            True si el envío fue exitoso
        """
        if not self.connected or not self.serial_port:
            return False

        try:
            self.serial_port.write(f"{command}\n".encode())
            self.serial_port.flush()
            return True
        except Exception as e:
            print(f"Error al enviar comando: {e}")
            return False

    def read_line(self, timeout: Optional[float] = None) -> Optional[str]:
        """
        Lee una línea del Arduino, ignorando datos binarios del stream de visualización

        Args:
            timeout: Timeout opcional en segundos

        Returns:
            Línea leída o None si hubo error
        """
        if not self.connected or not self.serial_port:
            return None

        try:
            old_timeout = self.serial_port.timeout
            if timeout is not None:
                self.serial_port.timeout = timeout

            # Intentar hasta 10 veces para encontrar una línea de texto válida
            for _ in range(10):
                raw_line = self.serial_port.readline()

                # Ignorar líneas que contienen el header binario (0xAA 0x55)
                if b'\xaa\x55' in raw_line or b'\xaa\xaa' in raw_line:
                    continue

                # Decodificar y limpiar
                line = raw_line.decode('utf-8', errors='ignore').strip()

                # Ignorar líneas vacías o con demasiados caracteres no ASCII
                if line and sum(1 for c in line if ord(c) > 127) < len(line) / 2:
                    self.serial_port.timeout = old_timeout
                    return line

            self.serial_port.timeout = old_timeout
            return None

        except Exception as e:
            print(f"Error al leer línea: {e}")
            return None

    def read_visualization_data(self) -> Optional[Dict[str, float]]:
        """
        Lee un paquete de datos de visualización desde Arduino

        Protocolo: 0xAA 0x55 | ADC(2) | Filtered(4) | Envelope(4) | Phase(2) | PulseFlag(1) | CRC(1)
        Total: 16 bytes

        Returns:
            Diccionario con los datos en unidades correctas o None si error
        """
        if not self.connected or not self.serial_port:
            return None

        try:
            # Buscar header (0xAA 0x55)
            if self.serial_port.in_waiting < 16:
                return None

            # Leer hasta encontrar header
            while self.serial_port.in_waiting >= 16:
                byte1 = self.serial_port.read(1)
                if byte1 == b'\xAA':
                    byte2 = self.serial_port.read(1)
                    if byte2 == b'\x55':
                        # Header encontrado, leer resto del paquete
                        data = self.serial_port.read(14)  # 14 bytes restantes

                        if len(data) != 14:
                            continue

                        # Parsear datos
                        adc_raw = struct.unpack('<H', data[0:2])[0]  # uint16_t little-endian
                        filtered = struct.unpack('<i', data[2:6])[0]  # int32_t little-endian
                        envelope = struct.unpack('<i', data[6:10])[0]  # int32_t little-endian
                        phase = struct.unpack('<h', data[10:12])[0]  # int16_t little-endian
                        pulse_flag = data[12]  # uint8_t (1 = pulso activo, 0 = sin pulso)
                        checksum_rx = data[13]

                        # Verificar checksum
                        checksum_calc = data[0] ^ data[1]
                        for i in range(2, 13):
                            checksum_calc ^= data[i]

                        if checksum_calc != checksum_rx:
                            continue  # Checksum inválido, buscar siguiente paquete

                        # Convertir a unidades físicas
                        Q15_SCALE = 32768.0  # 2^15
                        ADC_BITS = 1024.0  # 2^10 (ADC de 10 bits)
                        VOLTAGE_REF = 5.0  # Voltaje de referencia del ADC

                        # ADC: 0-1023 (10 bits) -> 0-5V
                        voltage = (adc_raw / 1023.0) * VOLTAGE_REF

                        # Filtered: Q15 -> unidades ADC centradas -> voltios
                        # El ADC se centró en 512, así que el rango es ±512 unidades -> ±2.5V
                        filtered_adc_units = filtered / Q15_SCALE
                        filtered_volts = filtered_adc_units * (VOLTAGE_REF / ADC_BITS)

                        # Envelope: viene del CORDIC con escala similar a señales Q15
                        # El CORDIC procesa señales en Q15 y produce magnitudes en escala similar
                        # Aplicamos la misma conversión que para filtered (Q15 → voltios)
                        # Pero la magnitud es siempre positiva, así que no hay signo
                        magnitude_pseudo_q15 = abs(envelope)
                        magnitude_adc_units = magnitude_pseudo_q15 / Q15_SCALE
                        magnitude_volts = magnitude_adc_units * (VOLTAGE_REF / ADC_BITS)

                        # Phase: Q15 -> grados (-180 a 180)
                        phase_degrees = (phase / 32768.0) * 180.0

                        return {
                            'timestamp': time.time(),
                            'adc_raw': adc_raw,
                            'voltage': voltage,
                            'filtered': filtered_volts,
                            'magnitude': magnitude_volts,
                            'phase_degrees': phase_degrees,
                            'phase_radians': (phase / 32768.0) * 3.14159265359,
                            'pulse_active': bool(pulse_flag)  # True si hay pulso activo
                        }

            return None

        except Exception as e:
            print(f"Error al leer datos de visualización: {e}")
            return None


# ===== FUNCIONES DE UTILIDAD =====

def list_available_ports():
    """
    Lista los puertos seriales disponibles

    Returns:
        Lista de puertos disponibles
    """
    import serial.tools.list_ports
    ports = serial.tools.list_ports.comports()
    return [(port.device, port.description) for port in ports]


def test_connection(port: str, baudrate: int = 115200) -> bool:
    """
    Prueba la conexión con un puerto serial

    Args:
        port: Puerto a probar
        baudrate: Velocidad de comunicación

    Returns:
        True si la conexión fue exitosa
    """
    controller = ArduinoController(baudrate=baudrate)
    if controller.connect(port):
        print(f"✓ Conexión exitosa en {port}")
        controller.disconnect()
        return True
    else:
        print(f"✗ No se pudo conectar a {port}")
        return False


if __name__ == "__main__":
    # Listar puertos disponibles
    print("Puertos seriales disponibles:")
    ports = list_available_ports()
    for i, (device, desc) in enumerate(ports):
        print(f"  {i+1}. {device} - {desc}")

    # Probar conexión (descomentar y ajustar el puerto)
    # test_connection("COM3")
