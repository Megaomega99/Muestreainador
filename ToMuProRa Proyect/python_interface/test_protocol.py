"""
Script de prueba para verificar el protocolo de comunicación
"""

from filter_calculator import FilterCalculator
import struct

def test_default_values():
    """Prueba que los valores por defecto se empaqueten correctamente"""
    print("=" * 60)
    print("TEST: Valores por defecto (21 Hz)")
    print("=" * 60)

    calc = FilterCalculator(fs=2000.0)
    result = calc.calculate_bandpass_coefficients(21.0, 9.0, 2.333)

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

    # Valores esperados del código Arduino original
    expected = {
        'b0_q15': 456,
        'b1_q15': 0,
        'b2_q15': -456,
        'a1_q15': -64482,
        'a2_q15': 31855,
    }

    print("\nValores calculados vs esperados:")
    print(f"{'Parámetro':<10} {'Calculado':>10} {'Esperado':>10} {'Match':<10}")
    print("-" * 45)

    all_match = True
    for key in expected:
        calc_val = params[key]
        exp_val = expected[key]
        match = "OK" if calc_val == exp_val else "FAIL"
        if calc_val != exp_val:
            all_match = False
        print(f"{key:<10} {calc_val:>10} {exp_val:>10} {match:<10}")

    print()
    if all_match:
        print("[OK] Todos los valores coinciden con el codigo original")
    else:
        print("[FAIL] Algunos valores difieren del codigo original")

    # Simular empaquetado
    print("\nEmpaquetado del mensaje (32-bit):")
    message = bytearray()
    message.append(0xF0)  # START
    message.append(0x01)  # CMD_SET_FILTER

    for key in ['b0_q15', 'b1_q15', 'b2_q15', 'a1_q15', 'a2_q15']:
        value = params[key]
        bytes_packed = struct.pack('>i', int(value))
        message.extend(bytes_packed)
        print(f"  {key}: {value:7d} -> {bytes_packed.hex().upper()}")

    for key in ['magnitude_threshold', 'phase_threshold', 'prediction_delay']:
        value = params[key]
        bytes_packed = struct.pack('>H', value & 0xFFFF)
        message.extend(bytes_packed)
        print(f"  {key}: {value:6d} -> {bytes_packed.hex().upper()}")

    # Checksum
    checksum = 0
    for byte in message[2:]:
        checksum ^= byte
    message.append(checksum)
    message.append(0xF1)  # END

    print(f"\nLongitud del mensaje: {len(message)} bytes")
    print(f"Mensaje completo: {message.hex().upper()}")

    return all_match


def test_different_frequencies():
    """Prueba con diferentes frecuencias"""
    print("\n" + "=" * 60)
    print("TEST: Diferentes frecuencias")
    print("=" * 60)

    calc = FilterCalculator(fs=2000.0)

    test_cases = [
        (10.0, 5.0, "10 Hz, BW=5 Hz"),
        (21.0, 9.0, "21 Hz, BW=9 Hz (default)"),
        (30.0, 10.0, "30 Hz, BW=10 Hz"),
        (50.0, 15.0, "50 Hz, BW=15 Hz"),
    ]

    print(f"\n{'Caso':<25} {'b0_q15':>10} {'a1_q15':>10} {'a2_q15':>10}")
    print("-" * 60)

    for freq, bw, desc in test_cases:
        result = calc.calculate_bandpass_coefficients(freq, bw)
        print(f"{desc:<25} {result['b0_q15']:>10} {result['a1_q15']:>10} {result['a2_q15']:>10}")


def test_32bit_range():
    """Verifica que los valores estén en rango de 32-bit signed"""
    print("\n" + "=" * 60)
    print("TEST: Rango de valores 32-bit")
    print("=" * 60)

    calc = FilterCalculator(fs=2000.0)

    # Probar frecuencias extremas
    test_freqs = [5.0, 10.0, 21.0, 50.0, 100.0, 200.0]

    print("\nVerificando que todos los valores estén en rango int32...")
    all_valid = True

    for freq in test_freqs:
        result = calc.calculate_bandpass_coefficients(freq, freq/3.0)

        for key in ['b0_q15', 'b1_q15', 'b2_q15', 'a1_q15', 'a2_q15']:
            value = result[key]
            if value < -2147483648 or value > 2147483647:
                print(f"  [FAIL] {freq} Hz: {key} = {value} fuera de rango!")
                all_valid = False

    if all_valid:
        print("[OK] Todos los valores estan dentro del rango int32")
    else:
        print("[FAIL] Algunos valores estan fuera de rango")

    return all_valid


def main():
    print("\n" + "=" * 60)
    print("PRUEBA DEL PROTOCOLO DE COMUNICACIÓN")
    print("=" * 60)

    test1_pass = test_default_values()
    test_different_frequencies()
    test2_pass = test_32bit_range()

    print("\n" + "=" * 60)
    print("RESUMEN DE PRUEBAS")
    print("=" * 60)
    print(f"Valores por defecto: {'[PASS]' if test1_pass else '[FAIL]'}")
    print(f"Rango 32-bit: {'[PASS]' if test2_pass else '[FAIL]'}")
    print()

    if test1_pass and test2_pass:
        print("[OK] Todas las pruebas pasaron correctamente")
        print("El sistema esta listo para usar")
    else:
        print("[FAIL] Algunas pruebas fallaron")
        print("Revisa los resultados arriba")

    print()


if __name__ == "__main__":
    main()
