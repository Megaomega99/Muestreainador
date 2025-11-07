# Explicación de las Señales y Unidades

## Formato de datos transmitidos por serial

Cada línea contiene 5 valores separados por comas:
```
voltaje_mV, filtrada, hilbert, magnitud, fase_grados
```

---

## 1. Señal Original - Voltaje (mV)

**Rango:** 0 - 5000 mV (0 - 5V)
**Unidad:** Milivoltios (mV)

Esta es la señal analógica cruda que entra por el pin A0 del Arduino.

- **Conversión:** `voltaje_mV = (ADC_value × 5000) / 1023`
- **Centro teórico:** 2500 mV (2.5V) cuando no hay señal
- **Representa:** El voltaje real que está midiendo el ADC

---

## 2. Señal Filtrada (IIR)

**Rango:** Aproximadamente -512 a +512
**Unidad:** Unidades de ADC centradas en cero

Esta es la salida del filtro IIR pasabanda centrado en 21 Hz.

- **Formato original:** Punto fijo Q15 (escala de 32768)
- **Conversión:** `filtrada_ADC = y_filtered >> 15`
- **Centro:** 0 (se remueve el offset DC de 2.5V)
- **Representa:** La componente de 21 Hz de tu señal, amplificada

### Parámetros del filtro IIR:
```cpp
b0 = 900, b1 = 0, b2 = -900
a1 = -63597, a2 = 30967
```
- Tipo: Pasabanda de segundo orden
- Frecuencia central: ~21 Hz
- Frecuencia de muestreo: 2000 Hz

---

## 3. Señal Hilbert - Parte Imaginaria

**Rango:** Aproximadamente -512 a +512
**Unidad:** Unidades de ADC centradas en cero

Esta es la transformada de Hilbert de la señal filtrada, que representa la parte imaginaria de la señal analítica.

- **Formato original:** Punto fijo Q15
- **Conversión:** `hilbert_ADC = imag_part >> 15`
- **Desfase:** 90° respecto a la señal filtrada
- **Representa:** Permite calcular la envolvente y fase instantánea

### Filtro FIR de Hilbert (15 coeficientes):
```cpp
{-328, 0, -984, 0, -2621, 0, -6554, 0, 6554, 0, 2621, 0, 984, 0, 328}
```

---

## 4. Magnitud (Envelope)

**Rango:** 0 - 300 (típico)
**Unidad:** Unidades de ADC

Esta es la envolvente (envelope) de la señal, calculada como la magnitud del vector complejo (real + imaginaria).

- **Cálculo:** `magnitude = sqrt(real² + imag²)` (aproximación rápida)
- **Conversión:** `magnitude_ADC = envelope >> 15`
- **Representa:** La amplitud instantánea de la señal de 21 Hz
- **Uso:** Detectar cuando la señal supera un umbral de amplitud

### Umbral de magnitud:
```cpp
MAGNITUDE_THRESHOLD = 80 << 15 = 2,621,440 (en Q15)
MAGNITUDE_THRESHOLD = 80 (después de >> 15)
```

---

## 5. Fase (Grados)

**Rango:** -180° a +180°
**Unidad:** Grados

Esta es la fase instantánea de la señal de 21 Hz.

- **Formato original:** Entero de -32768 a +32768 (donde 32768 = π radianes)
- **Conversión:** `fase_grados = (phase × 180) / 32768`
- **0°:** La señal está en su máximo positivo
- **±180°:** La señal está en su máximo negativo
- **±90°:** La señal cruza por cero

### Umbral de fase:
```cpp
PHASE_THRESHOLD = 5000 (en formato interno)
PHASE_THRESHOLD = ±27.5° (en grados)
```

El sistema detecta picos cuando:
- `magnitud > 80` **Y**
- `-27.5° < fase < +27.5°`

---

## Señal Analítica Compleja

La señal filtrada (real) y la transformada de Hilbert (imaginaria) forman una **señal analítica compleja**:

```
z(t) = real(t) + j·imag(t)
```

De esta señal compleja se extraen:
- **Magnitud:** `|z(t)| = sqrt(real² + imag²)` → Envolvente
- **Fase:** `∠z(t) = atan2(imag, real)` → Fase instantánea

### Ventajas de la señal analítica:
1. **Envolvente instantánea:** Detecta amplitud sin depender del punto exacto de la onda
2. **Fase instantánea:** Determina en qué punto del ciclo está la señal
3. **Detección robusta:** Menos sensible a ruido que detectar cruces por cero

---

## Detección de Picos

El sistema usa dos criterios para detectar picos de 21 Hz:

1. **Amplitud suficiente:** `magnitud > 80`
2. **Fase correcta:** `-27.5° < fase < +27.5°`

Cuando se detecta un pico, el sistema predice el **siguiente pico** usando:

```cpp
PERIOD_21HZ_MS = 48 ms
FILTER_DELAY_MS = 10 ms
PHASE_ADJUST_MS = 0 ms
PREDICTION_DELAY_MS = 48 - 10 + 0 = 38 ms
PREDICTION_DELAY_SAMPLES = 38 × 2 = 76 muestras
```

Esto permite generar un pulso de sincronización **anticipado** en el pin 10.

---

## Salida PWM (Pin 9)

Además de los datos seriales, el pin 9 genera una salida PWM con la señal filtrada amplificada:

```cpp
y_amplified = (y_filtered × 7) >> 1  // Amplificar 3.5x
output = (y_amplified >> 15) + 512   // Convertir a rango 0-1023
PWM = output >> 2                     // Escalar a rango 0-255
```

---

## Ejemplo de interpretación

```
V: 2543.21mV | Filt: 123 | Hilb: -45 | Mag: 131 | Fase: 21.3°
```

**Interpretación:**
- Voltaje de entrada: 2.54V (cerca del centro de 2.5V)
- Señal filtrada: +123 unidades (ciclo positivo)
- Hilbert: -45 unidades (90° desfasado)
- Magnitud: 131 > 80 → **Amplitud suficiente** ✓
- Fase: 21.3° < 27.5° → **Fase correcta** ✓
- **Resultado:** Se detecta un pico, se predice el siguiente en 38ms

---

## Frecuencia de muestreo

- **ADC:** 2000 Hz (una muestra cada 0.5 ms)
- **Timer2:** Dispara el ADC cada 500 μs
- **Datos seriales:** 2000 líneas/segundo (~230 kbaud estimado)
- **Baudrate:** 115200 (suficiente para ~30 bytes por línea)

---

## Notas importantes

1. Los valores están en **formato Q15** internamente (punto fijo con escala de 32768)
2. Las conversiones a unidades comprensibles se hacen **antes de enviar por serial**
3. El filtro introduce un retardo de ~10ms que se compensa en la predicción
4. La transformada de Hilbert introduce un retardo adicional de ~3.5 muestras (filtro FIR de 15 taps)

