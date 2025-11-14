"""
Ejemplos de uso de la interfaz de control del filtro IIR
"""

from filter_calculator import FilterCalculator
from serial_communication import ArduinoController, list_available_ports


def example_1_calculate_filter():
    """Ejemplo 1: Calcular coeficientes de un filtro"""
    print("=" * 60)
    print("EJEMPLO 1: Calcular coeficientes de filtro")
    print("=" * 60)

    # Crear calculador con frecuencia de muestreo de 2000 Hz
    calc = FilterCalculator(fs=2000.0)

    # Calcular filtro para 21 Hz (valores por defecto)
    print("\nCalculando filtro para 21 Hz ± 9 Hz...")
    result = calc.calculate_bandpass_coefficients(
        center_freq=21.0,
        bandwidth=9.0,
        q_factor=2.333
    )

    # Mostrar resultados
    print("\nCoeficientes Q15:")
    print(f"  b0_q15 = {result['b0_q15']}")
    print(f"  b1_q15 = {result['b1_q15']}")
    print(f"  b2_q15 = {result['b2_q15']}")
    print(f"  a1_q15 = {result['a1_q15']}")
    print(f"  a2_q15 = {result['a2_q15']}")

    print(f"\nRetraso en frecuencia central:")
    print(f"  {result['center_delay_samples']:.2f} muestras")
    print(f"  {result['center_delay_ms']:.2f} ms")

    # Estimar retraso total
    delay_info = calc.estimate_total_delay(result['center_delay_samples'])
    print(f"\nRetraso total del sistema:")
    print(f"  IIR: {delay_info['iir_delay_samples']:.2f} muestras")
    print(f"  Hilbert: {delay_info['hilbert_delay_samples']:.2f} muestras")
    print(f"  Total: {delay_info['total_delay_samples']:.2f} muestras "
          f"({delay_info['total_delay_ms']:.2f} ms)")
    print(f"\nPREDICTION_DELAY_SAMPLES recomendado: "
          f"{delay_info['recommended_prediction_delay']}")


def example_2_different_frequencies():
    """Ejemplo 2: Calcular filtros para diferentes frecuencias"""
    print("\n" + "=" * 60)
    print("EJEMPLO 2: Filtros para diferentes frecuencias")
    print("=" * 60)

    calc = FilterCalculator(fs=2000.0)

    frequencies = [10.0, 21.0, 30.0, 50.0]

    print("\n{:<10} {:<10} {:<10} {:<15}".format(
        "Freq (Hz)", "Delay (ms)", "Delay (samples)", "Pred. Delay"
    ))
    print("-" * 60)

    for freq in frequencies:
        result = calc.calculate_bandpass_coefficients(
            center_freq=freq,
            bandwidth=9.0
        )

        delay_info = calc.estimate_total_delay(result['center_delay_samples'])

        print("{:<10.1f} {:<10.2f} {:<10.2f} {:<15}".format(
            freq,
            delay_info['total_delay_ms'],
            delay_info['total_delay_samples'],
            delay_info['recommended_prediction_delay']
        ))


def example_3_list_ports():
    """Ejemplo 3: Listar puertos seriales disponibles"""
    print("\n" + "=" * 60)
    print("EJEMPLO 3: Puertos seriales disponibles")
    print("=" * 60)

    ports = list_available_ports()

    if not ports:
        print("\nNo se encontraron puertos seriales.")
        return

    print(f"\nSe encontraron {len(ports)} puerto(s):\n")
    for i, (device, description) in enumerate(ports, 1):
        print(f"{i}. {device}")
        print(f"   {description}\n")


def example_4_send_to_arduino():
    """Ejemplo 4: Enviar parámetros al Arduino"""
    print("\n" + "=" * 60)
    print("EJEMPLO 4: Enviar parámetros al Arduino")
    print("=" * 60)

    # Listar puertos
    ports = list_available_ports()
    if not ports:
        print("\nNo se encontraron puertos seriales.")
        print("Conecta el Arduino y vuelve a intentarlo.")
        return

    print("\nPuertos disponibles:")
    for i, (device, description) in enumerate(ports, 1):
        print(f"{i}. {device} - {description}")

    # Seleccionar puerto (para este ejemplo, usaremos el primero)
    port = ports[0][0]
    print(f"\nUsando puerto: {port}")
    print("(Si este no es el correcto, modifica el código)")

    # Preguntar al usuario si desea continuar
    response = input("\n¿Deseas enviar parámetros al Arduino? (s/n): ")
    if response.lower() != 's':
        print("Operación cancelada.")
        return

    # Calcular filtro
    calc = FilterCalculator(fs=2000.0)
    result = calc.calculate_bandpass_coefficients(30.0, 10.0)

    print("\nCalculando filtro para 30 Hz ± 10 Hz...")
    print(f"  b0_q15 = {result['b0_q15']}")
    print(f"  b1_q15 = {result['b1_q15']}")
    print(f"  b2_q15 = {result['b2_q15']}")
    print(f"  a1_q15 = {result['a1_q15']}")
    print(f"  a2_q15 = {result['a2_q15']}")

    # Preparar parámetros
    params = {
        'b0_q15': result['b0_q15'],
        'b1_q15': result['b1_q15'],
        'b2_q15': result['b2_q15'],
        'a1_q15': result['a1_q15'],
        'a2_q15': result['a2_q15'],
        'magnitude_threshold': 50,
        'phase_threshold': 2730,
        'prediction_delay': 76
    }

    # Conectar y enviar
    print(f"\nConectando a {port}...")
    arduino = ArduinoController()

    if arduino.connect(port):
        print("✓ Conectado exitosamente")

        print("\nEnviando parámetros...")
        if arduino.send_filter_coefficients(params):
            print("✓ Parámetros enviados correctamente")
        else:
            print("✗ Error al enviar parámetros")

        arduino.disconnect()
        print("\nDesconectado.")
    else:
        print(f"✗ No se pudo conectar a {port}")


def example_5_compare_default():
    """Ejemplo 5: Comparar con valores por defecto"""
    print("\n" + "=" * 60)
    print("EJEMPLO 5: Comparar con valores por defecto")
    print("=" * 60)

    calc = FilterCalculator(fs=2000.0)

    # Obtener valores por defecto
    default = calc.get_default_coefficients()

    print("\nValores por defecto del código Arduino:")
    print(f"  b0_q15 = {default['b0_q15']}")
    print(f"  b1_q15 = {default['b1_q15']}")
    print(f"  b2_q15 = {default['b2_q15']}")
    print(f"  a1_q15 = {default['a1_q15']}")
    print(f"  a2_q15 = {default['a2_q15']}")

    # Calcular con los mismos parámetros
    calculated = calc.calculate_bandpass_coefficients(21.0, 9.0)

    print("\nValores calculados por Python:")
    print(f"  b0_q15 = {calculated['b0_q15']}")
    print(f"  b1_q15 = {calculated['b1_q15']}")
    print(f"  b2_q15 = {calculated['b2_q15']}")
    print(f"  a1_q15 = {calculated['a1_q15']}")
    print(f"  a2_q15 = {calculated['a2_q15']}")

    print("\nNota: Pueden haber pequeñas diferencias debido a:")
    print("  - Método de diseño del filtro")
    print("  - Redondeo en formato Q15")
    print("  - Escalado de coeficientes")


def main():
    """Función principal que ejecuta todos los ejemplos"""
    print("\n" + "=" * 60)
    print("EJEMPLOS DE USO - Interfaz de Control de Filtro IIR")
    print("=" * 60)

    # Ejecutar ejemplos
    example_1_calculate_filter()
    example_2_different_frequencies()
    example_3_list_ports()
    example_5_compare_default()

    # Ejemplo 4 requiere interacción del usuario
    print("\n" + "=" * 60)
    print("EJEMPLO 4 disponible (requiere Arduino conectado)")
    print("=" * 60)

    response = input("\n¿Deseas ejecutar el Ejemplo 4 (enviar a Arduino)? (s/n): ")
    if response.lower() == 's':
        example_4_send_to_arduino()

    print("\n" + "=" * 60)
    print("Ejemplos completados")
    print("=" * 60)
    print("\nPara usar la interfaz gráfica, ejecuta:")
    print("  python gui_controller.py")
    print()


if __name__ == "__main__":
    main()
