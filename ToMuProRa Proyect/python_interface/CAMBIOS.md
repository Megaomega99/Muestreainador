# Cambios Realizados en la Interfaz de Control

## Resumen

Se han realizado correcciones importantes para solucionar los problemas de comunicación serial y simplificar la interfaz de usuario.

## Problemas Solucionados

### 1. Overflow de Coeficientes (CRÍTICO)

**Problema**: Los coeficientes del filtro se truncaban de 32 bits a 16 bits durante la transmisión serial.

- **Síntoma**: La señal filtrada desaparecía al enviar parámetros al Arduino
- **Causa**: El valor `a1_q15 = -64482` se clampeaba a `-32768` (límite int16)
- **Solución**: Actualizado el protocolo para usar 32 bits por coeficiente

**Archivos modificados**:
- `serial_communication.py`: Cambiado de `struct.pack('>h', ...)` a `struct.pack('>i', ...)`
- `src/main.cpp`: Agregada función `readInt32()` y actualizado `processFilterMessage()`
- Longitud del mensaje aumentada de 20 a 30 bytes

### 2. Orden de Mensajes ACK/NACK

**Problema**: Python esperaba el byte ACK como primer byte, pero Arduino enviaba texto primero.

- **Síntoma**: "⚠ Respuesta inesperada: 0x4F" (letra 'O' de "OK")
- **Solución**: Reorganizado para enviar ACK/NACK primero, luego mensajes de texto

**Archivos modificados**:
- `src/main.cpp`: Reordenados los `Serial.write()` y `Serial.println()` en `processSerialCommand()`

### 3. Interfaz Simplificada

**Problema**: Confusión entre Factor Q y Ancho de Banda - ambos controlan lo mismo.

- **Solución**: Eliminado el control manual de Q, ahora se calcula automáticamente como `Q = fc / BW`
- El usuario solo controla Frecuencia Central y Ancho de Banda
- Factor Q se muestra como información de solo lectura

**Archivos modificados**:
- `gui_controller.py`: Eliminada variable `q_factor`, agregado `q_label` para mostrar Q calculado

## Protocolo Actualizado

### Estructura del Mensaje (30 bytes total)

```
Offset | Bytes | Campo              | Tipo
-------+-------+--------------------+------------------
0      | 1     | START_BYTE         | 0xF0
1      | 1     | CMD                | 0x01 (SET_FILTER)
2-5    | 4     | b0_q15             | int32 (big-endian)
6-9    | 4     | b1_q15             | int32 (big-endian)
10-13  | 4     | b2_q15             | int32 (big-endian)
14-17  | 4     | a1_q15             | int32 (big-endian)
18-21  | 4     | a2_q15             | int32 (big-endian)
22-23  | 2     | magnitude_thresh   | uint16 (big-endian)
24-25  | 2     | phase_threshold    | uint16 (big-endian)
26-27  | 2     | prediction_delay   | uint16 (big-endian)
28     | 1     | CHECKSUM           | XOR de bytes 2-27
29     | 1     | END_BYTE           | 0xF1
```

### Ejemplo de Mensaje (Valores por Defecto)

```
F0 01 00 00 01 C8 00 00 00 00 FF FF FE 38 FF FF 04 1E 00 00 7C 6F 00 32 0A AA 00 4C D8 F1
```

Decodificado:
- b0_q15 = 456
- b1_q15 = 0
- b2_q15 = -456
- a1_q15 = -64482 ✓ (ahora correcto, antes se truncaba a -32768)
- a2_q15 = 31855
- magnitude_threshold = 50
- phase_threshold = 2730
- prediction_delay = 76

## Validación

El archivo `test_protocol.py` verifica que:

✓ Los valores por defecto se calculan correctamente
✓ Los coeficientes coinciden con el código Arduino original
✓ Todos los valores están en rango int32
✓ El empaquetado del mensaje es correcto

## Uso de la Interfaz Actualizada

1. **Ajustar Frecuencia Central**: Frecuencia objetivo del filtro (1-500 Hz)
2. **Ajustar Ancho de Banda**: Ancho de banda del filtro (1-100 Hz)
3. **Ver Factor Q**: Se calcula y muestra automáticamente (Q = fc / BW)
4. **Calcular Coeficientes**: Genera los coeficientes Q15 para el Arduino
5. **Enviar a Arduino**: Transmite los parámetros vía serial

### Valores por Defecto

- Frecuencia Central: 21 Hz
- Ancho de Banda: 9 Hz
- Factor Q (calculado): 2.333
- Umbral de Magnitud: 50
- Umbral de Fase: 2730
- Retraso de Predicción: 76 muestras

## Próximos Pasos

1. Compilar y cargar el código actualizado en Arduino:
   ```bash
   pio run --target upload
   ```

2. Ejecutar la interfaz:
   ```bash
   python gui_controller.py
   ```

3. Probar enviando los valores por defecto y verificar que la señal filtrada se mantiene

## Notas Técnicas

- El protocolo ahora usa 32 bits para coeficientes, permitiendo valores completos sin truncamiento
- Los coeficientes del filtro IIR pueden exceder el rango int16 (-32768 a 32767)
- El Arduino usa `int32_t` para los coeficientes internamente
- La transmisión es big-endian (MSB primero)
- El checksum es XOR de todos los bytes de datos (excluyendo START, CMD y END)
