# Compensación del Retardo del Sistema - Arduino

## Resumen

Se implementó **predicción del siguiente pico** para sincronizar el pulso de salida con un **pico real** de la señal de entrada. Como no podemos volver atrás en el tiempo, predecimos cuándo ocurrirá el siguiente pico basándonos en la periodicidad de la señal de 21 Hz.

## Cálculos del Retardo

Basado en los cálculos teóricos del notebook `Fitros.ipynb`:

| Componente | Retardo (muestras) | Retardo (ms) |
|------------|-------------------|--------------|
| Filtro IIR Pasa-Banda (21 Hz) | 12.99 | 6.49 |
| Filtro Hilbert (FIR 15 taps) | 7.00 | 3.50 |
| **TOTAL** | **19.99 ≈ 20** | **9.99 ≈ 10** |

## Implementación en Arduino

### Cambios en `main.cpp`

1. **Constantes de predicción** (líneas 55-58):
   ```cpp
   #define PERIOD_21HZ_MS 48           // Período de 21 Hz
   #define FILTER_DELAY_MS 10          // Retardo de filtros
   #define PREDICTION_DELAY_MS (PERIOD_21HZ_MS - FILTER_DELAY_MS)  // 38ms
   #define PREDICTION_DELAY_SAMPLES (PREDICTION_DELAY_MS * 2)      // 76 muestras
   ```

2. **Estructura para predicción** (líneas 61-66):
   ```cpp
   struct PeakPredictor {
     uint8_t countdown;
     bool active;
   };
   PeakPredictor peakPredictor = {0, false};
   ```

3. **Lógica de predicción** (líneas 220-240):
   - Cuando se detecta `fase ≈ 0°` → iniciar countdown de 76 muestras (38ms)
   - Cada muestra procesada → decrementar countdown
   - Cuando countdown llega a 0 → disparar pulso (¡sincronizado con siguiente pico!)

## Timeline del Sistema (CORREGIDO)

```
Tiempo:  0ms      10ms     48ms     58ms
         |        |        |        |
Entrada: ●--------●--------●--------●----  (Picos reales a 21 Hz, período=48ms)
         ↑        ↑        ↑        ↑
         Pico #1  |        Pico #2  |
                  |        ↑        |
Detección:        ●--------┘        |
                  ↑                 |
                  Detectamos        |
                  fase=0°           |
                  (Pico #1 ya pasó) |
                                    |
Pulso:                              ●====  ¡Sincronizado con Pico #2!
                                    ↑
                                    Disparo después de
                                    38ms de countdown

Explicación:
1. t=0ms   : Ocurre pico real #1 en la entrada
2. t=10ms  : Los filtros procesan y detectamos fase=0°
             → ¡El pico #1 ya pasó hace 10ms!
             → Iniciamos countdown de 38ms para el siguiente pico
3. t=48ms  : Ocurre pico real #2 en la entrada
             → ¡Disparamos pulso SINCRONIZADO!
4. Se repite el ciclo cada 48ms
```

## Cómo Funciona

1. **ADC** muestrea la señal a 2000 Hz (cada 0.5 ms)
2. **Filtro IIR** extrae componente de 21 Hz (retardo: ~6.5 ms)
3. **Filtro Hilbert** genera componente en cuadratura (retardo: +3.5 ms)
4. **Cálculo de fase** detecta cuándo fase ≈ 0° (pico en señal filtrada)
5. **Predicción**: Al detectar, sabemos que el pico ocurrió hace 10ms
6. **Countdown**: Esperamos 38ms = (período 48ms - retardo 10ms)
7. **Pulso**: Se dispara sincronizado con el **siguiente** pico real

## Precisión Esperada

Según simulaciones en Python:
- **Error promedio**: < 0.5 ms
- **Error máximo**: < 1.0 ms
- **Equivalente en fase**: < 3.6° a 21 Hz

## Ajuste Fino (Opcional)

Si experimentalmente observas un desfase residual, puedes ajustar el período:

```cpp
// Aumentar si el pulso ocurre ANTES del pico
#define PERIOD_21HZ_MS 49  // +1 ms → countdown de 39ms

// Disminuir si el pulso ocurre DESPUÉS del pico
#define PERIOD_21HZ_MS 47  // -1 ms → countdown de 37ms
```

**Nota**: La frecuencia exacta puede variar ligeramente. El valor teórico para 21 Hz es 47.6 ms, redondeado a 48 ms.

## Notas Importantes

1. La compensación es específica para **21 Hz**
2. Para otras frecuencias, el retardo del filtro IIR varía
3. El filtro Hilbert mantiene retardo constante (7 muestras)
4. La frecuencia de muestreo debe mantenerse exactamente a 2000 Hz

## Verificación

Para verificar que la compensación funciona:

1. Conectar un osciloscopio al pin A0 (entrada) y pin 10 (pulso)
2. Aplicar señal senoidal de 21 Hz
3. Verificar que el pulso ocurre en el pico máximo de la entrada
4. Ajustar `DELAY_COMPENSATION_SAMPLES` si es necesario

## Referencias

- Notebook de cálculos: `Fitros.ipynb` (celdas 8-11)
- Código Arduino: `src/main.cpp` (líneas 34-51, 209-233)
- Método: Group Delay compensado con buffer circular
