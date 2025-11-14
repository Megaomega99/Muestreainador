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
            # Limpiar buffer de entrada antes de enviar
            if self.serial_port.in_waiting > 0:
                self.serial_port.read(self.serial_port.in_waiting)

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

            # Enviar comando
            self.serial_port.write(cmd.encode('ascii'))
            self.serial_port.flush()

            # Esperar a que se procese
            time.sleep(0.1)

            # Leer todas las respuestas (incluye DEBUG y respuesta final)
            success = False
            max_attempts = 10
            for _ in range(max_attempts):
                response = self.read_line(timeout=0.2)
                if response:
                    print(f"  <- {response}")

                    if response.startswith("OK:"):
                        print("✓ Parámetros configurados correctamente")
                        success = True
                        break
                    elif response.startswith("ERROR:"):
                        print(f"✗ Error del Arduino: {response}")
                        return False
                    # Ignorar líneas DEBUG
                else:
                    break

            if success:
                return True

            print("⚠ No se recibió confirmación del Arduino")
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
            # Limpiar buffer de entrada
            if self.serial_port.in_waiting > 0:
                self.serial_port.read(self.serial_port.in_waiting)

            cmd = "RESET\n"
            print("\n=== RESTAURANDO VALORES POR DEFECTO ===")

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
                        print("✓ Valores por defecto restaurados")
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
        Lee una línea del Arduino

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

            line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()

            self.serial_port.timeout = old_timeout
            return line if line else None

        except Exception as e:
            print(f"Error al leer línea: {e}")
            return None

    def request_status(self) -> Optional[Dict[str, any]]:
        """
        Solicita el estado actual del Arduino

        Returns:
            Diccionario con el estado o None si hubo error
        """
        if not self.connected:
            return None

        try:
            # Enviar comando de solicitud de estado
            message = bytearray([0xF0, 0x02, 0xF1])  # CMD 0x02 = GET_STATUS
            self.serial_port.write(message)
            self.serial_port.flush()

            # Esperar respuesta (timeout de 1 segundo)
            time.sleep(0.1)
            if self.serial_port.in_waiting > 0:
                response = self.serial_port.read(self.serial_port.in_waiting)
                # Aquí se debería parsear la respuesta según el protocolo
                # Por ahora, retornar un diccionario vacío
                return {'raw_response': response.hex()}

            return None

        except Exception as e:
            print(f"Error al solicitar estado: {e}")
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
