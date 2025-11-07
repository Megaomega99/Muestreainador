# Test Rápido de Ajuste de Fase

## Prueba los diferentes ajustes de fase

Cambia el valor de `PHASE_ADJUST_MS` en [main.cpp línea 79](src/main.cpp#L79) y observa en el osciloscopio:

### Test 1: Pico máximo (por defecto)
```cpp
#define PHASE_ADJUST_MS 0
```
**Esperado**: Pulso exactamente en el máximo de la onda
```
     ●  ← Pulso aquí
    / \
   /   \
  /     \
```

### Test 2: Cruce descendente (90°)
```cpp
#define PHASE_ADJUST_MS 12
```
**Esperado**: Pulso cuando la onda cruza por cero bajando
```
    ●
   / \
  /   ●  ← Pulso aquí (cruce)
 /     \
```

### Test 3: Valle mínimo (180°)
```cpp
#define PHASE_ADJUST_MS 24
```
**Esperado**: Pulso en el mínimo de la onda
```
    ●
   / \
  /   \
 /     ●  ← Pulso aquí (valle)
```

### Test 4: Cruce ascendente (270°)
```cpp
#define PHASE_ADJUST_MS 36
```
**Esperado**: Pulso cuando la onda cruza por cero subiendo
```
    ●
   / \
  /   \
 ●     \  ← Pulso aquí (cruce)
  \     \
```

### Test 5: Antes del pico (-90°)
```cpp
#define PHASE_ADJUST_MS -12
```
**Esperado**: Pulso 12ms antes del pico (en el cruce ascendente)
```
     ●
    / \
   ●   \  ← Pulso aquí (antes del pico)
  /     \
```

## Procedimiento de Prueba

1. **Modifica** `PHASE_ADJUST_MS` en main.cpp
2. **Compila y carga** el código al Arduino
3. **Conecta osciloscopio**:
   - CH1 (amarillo): Pin A0 - Señal de entrada
   - CH2 (azul): Pin 10 - Pulso de salida
4. **Aplica señal de 21 Hz** a la entrada
5. **Verifica** que el pulso aparece donde esperas

## Valores Útiles por Aplicación

### Estimulación Neural
```cpp
#define PHASE_ADJUST_MS 0     // En el pico de actividad
```

### Sincronización con fase descendente
```cpp
#define PHASE_ADJUST_MS 12    // 90° después del pico
```

### Cancelación/inhibición
```cpp
#define PHASE_ADJUST_MS 24    // 180° (contrafase)
```

### Anticipación
```cpp
#define PHASE_ADJUST_MS -6    // 45° antes del pico
```

## Ajuste Fino

Para ajustes precisos, cambia de 1 en 1 ms:

```cpp
#define PHASE_ADJUST_MS 10    // 75° después del pico
#define PHASE_ADJUST_MS 11    // 82.5° después del pico
#define PHASE_ADJUST_MS 12    // 90° después del pico
```

Cada 1 ms ≈ 7.5° de desfase a 21 Hz.

## Limitaciones de Rango

```cpp
// Válido
#define PHASE_ADJUST_MS -48   // Un período completo antes
#define PHASE_ADJUST_MS 0     // Sin ajuste
#define PHASE_ADJUST_MS 48    // Un período completo después

// ⚠️ NO usar valores fuera de rango
#define PHASE_ADJUST_MS -100  // ❌ Comportamiento indefinido
#define PHASE_ADJUST_MS 100   // ❌ Comportamiento indefinido
```

## Troubleshooting

**P: El pulso no cambia al modificar `PHASE_ADJUST_MS`**
R: Asegúrate de recompilar y cargar el código después de cada cambio

**P: El pulso se dispara en momentos aleatorios**
R: Verifica que el valor esté dentro del rango válido (-48 a +48)

**P: Quiero ajuste más fino que 1 ms**
R: La resolución mínima es 0.5 ms (una muestra), pero usa valores enteros
   para evitar errores de redondeo

**P: Necesito disparar en múltiples fases del mismo ciclo**
R: Actualmente el sistema dispara una vez por ciclo. Necesitarías modificar
   la lógica para mantener múltiples contadores simultáneos.
