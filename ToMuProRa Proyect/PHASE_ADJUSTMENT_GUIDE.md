# Guía de Ajuste de Fase del Pulso

## Control del Timing del Pulso

Puedes controlar exactamente cuándo se dispara el pulso respecto a la onda de 21 Hz modificando la constante `PHASE_ADJUST_MS` en [main.cpp](src/main.cpp) línea 74.

## Tabla de Referencia Rápida

| Desfase Deseado | `PHASE_ADJUST_MS` | Momento del Pulso |
|-----------------|-------------------|-------------------|
| **0°** | `0` | En el **pico máximo** (por defecto) |
| **45°** | `6` | Bajada después del pico |
| **90°** | `12` | Cruce por cero descendente |
| **135°** | `18` | Valle (mínimo) acercándose |
| **180°** | `24` | En el **valle mínimo** |
| **225°** | `30` | Subida después del valle |
| **270°** | `36` | Cruce por cero ascendente |
| **315°** | `42` | Pico acercándose |
| **-45°** | `-6` | Subida antes del pico |
| **-90°** | `-12` | Cruce por cero ascendente |

## Fórmula de Conversión

Para calcular el ajuste en milisegundos según el desfase en grados:

```
PHASE_ADJUST_MS = (desfase_grados * 48ms) / 360°
                = desfase_grados / 7.5

Ejemplo: Para 90°
PHASE_ADJUST_MS = 90 / 7.5 = 12 ms
```

## Visualización de la Onda

```
       Pico (0°)
         ●
        / \
       /   \
      /     \  45°: PHASE_ADJUST_MS = 6
     /       \
    /         ● 90°: PHASE_ADJUST_MS = 12
   /           \
  /             \
 /               \
●                 ● 180°: Valle (PHASE_ADJUST_MS = 24)
 \               /
  \             /
   \           / 270°: PHASE_ADJUST_MS = 36
    \         ●
     \       /
      \     /
       \   /
        \ /
         ●
```

## Ejemplos de Uso Práctico

### 1. Disparar en el pico máximo (por defecto)
```cpp
#define PHASE_ADJUST_MS 0
```
**Uso**: Estimulación en momento de máxima amplitud

### 2. Disparar 90° después del pico (cruce descendente)
```cpp
#define PHASE_ADJUST_MS 12
```
**Uso**: Sincronizar con fase descendente de la onda

### 3. Disparar en el valle (180°)
```cpp
#define PHASE_ADJUST_MS 24
```
**Uso**: Estimulación en mínimo de la señal

### 4. Disparar 12ms ANTES del pico
```cpp
#define PHASE_ADJUST_MS -12
```
**Uso**: Anticipación para compensar latencias externas

### 5. Disparar justo antes del pico (ajuste fino)
```cpp
#define PHASE_ADJUST_MS -2
```
**Uso**: Ajuste fino de 2ms (≈15°) antes del pico

## Validación del Ajuste

Para verificar que el ajuste funciona correctamente:

1. **Conecta el osciloscopio:**
   - Canal 1: Pin A0 (señal de entrada)
   - Canal 2: Pin 10 (pulso de salida)

2. **Aplica señal de 21 Hz** a la entrada

3. **Observa el pulso** respecto a la onda:
   - Con `PHASE_ADJUST_MS = 0`: pulso en el pico
   - Con `PHASE_ADJUST_MS = 12`: pulso en cruce por cero
   - Con `PHASE_ADJUST_MS = 24`: pulso en el valle

4. **Ajusta según necesites** modificando el valor

## Limitaciones

- **Rango válido**: -48 ms a +48 ms (-360° a +360°)
- **Resolución**: 0.5 ms (una muestra a 2000 Hz) ≈ 3.75°
- **Específico para 21 Hz**: Para otras frecuencias, recalcular las constantes

## Cálculo para Otras Frecuencias

Si cambias la frecuencia objetivo, actualiza la conversión:

```
Para frecuencia F Hz:
  Período_ms = 1000 / F
  PHASE_ADJUST_MS = (desfase_grados * Período_ms) / 360

Ejemplo para 50 Hz:
  Período = 20 ms
  90° = (90 * 20) / 360 = 5 ms
```

## Notas Importantes

1. **Valores negativos funcionan**: puedes adelantar el pulso hasta -48ms
2. **Ajuste fino**: cambia de 1 en 1 ms para ajustes precisos
3. **El sistema compensa automáticamente**: no necesitas recalcular el retardo de filtros
4. **Frecuencia estable**: asegúrate que la señal de entrada sea estable a 21 Hz

## Código de Ejemplo

```cpp
// En main.cpp línea 74:

// Para estimulación en el pico
#define PHASE_ADJUST_MS 0

// Para estimulación en fase descendente (90°)
#define PHASE_ADJUST_MS 12

// Para estimulación en el valle (180°)
#define PHASE_ADJUST_MS 24

// Para anticipar 10ms (≈-75°)
#define PHASE_ADJUST_MS -10
```

## Troubleshooting

**Problema**: El pulso no coincide exactamente con el desfase esperado
**Solución**: Ajusta `PERIOD_21HZ_MS` en línea 55 (valor teórico: 47.6ms)

**Problema**: El pulso se dispara en momentos aleatorios
**Solución**: Verifica que `PHASE_ADJUST_MS` esté en rango válido (-48 a +48)

**Problema**: Quiero múltiples pulsos por ciclo
**Solución**: Modifica la lógica para no desactivar `peakPredictor.active` inmediatamente
