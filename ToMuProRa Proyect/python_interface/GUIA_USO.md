# Guía Rápida de Uso - Control de Filtro IIR Arduino

## Resumen del Sistema

Has creado exitosamente una interfaz Python para controlar dinámicamente los parámetros del filtro IIR en tu Arduino. El sistema funciona correctamente, la comunicación serial está establecida y los parámetros se actualizan en tiempo real.

## Estado Actual

✅ **Comunicación Serial**: Funciona perfectamente (ACK recibido)
✅ **Protocolo**: Bytes especiales (0xF0, 0xF1) evitan conflictos
✅ **Coeficientes**: Se transmiten correctamente con signo adecuado
✅ **Arduino**: Recibe y aplica los parámetros correctamente

## Problema Actual: Ganancia del Filtro

El filtro calcula coeficientes ligeramente diferentes a los originales:
- **Original**: `b0 = 456`, `b2 = -456`
- **Calculado**: `b0 = 492`, `b2 = -492` (8% más alto)

Los coeficientes `a1` y `a2` coinciden perfectamente.

### Solución: Ajustar la Ganancia

Tienes dos opciones:

#### Opción 1: Ajustar el escalado en Python (RECOMENDADO)

Modifica el valor `target_b0` en `filter_calculator.py` línea 83 para que coincida exactamente con el original:

```python
# Cambiar de:
target_b0 = 0.015  # Aproximadamente 456/32768

# A:
target_b0 = 456.0 / 32768.0  # 0.0139... (valor exacto del original)
```

#### Opción 2: Ajustar la amplificación en Arduino

Modifica la línea 332 en `main.cpp`:

```cpp
// Cambiar de:
int32_t y_amplified = (y_filtered * 7) >> 1;  // Ganancia de 3.5x

// A:
int32_t y_amplified = (y_filtered * 31) >> 3;  // Ganancia de ~3.875x
```

O usa el factor original con más precisión:
```cpp
int32_t y_amplified = (y_filtered * 63) >> 4;  // Ganancia de 3.9375x
```

## Cómo Usar el Sistema

### 1. Conexión Inicial

```bash
cd "g:\My Drive\Muestreainador\ToMuProRa Proyect\python_interface"
python gui_controller.py
```

1. Selecciona el puerto COM de tu Arduino
2. Haz clic en "Conectar"
3. Verifica que diga "● Conectado" en verde

### 2. Usar Valores por Defecto (21 Hz)

1. Haz clic en "Valores por Defecto (21 Hz)"
2. Se cargarán automáticamente los parámetros originales
3. Haz clic en "Enviar a Arduino"
4. Verifica en la consola que recibas: `OK:b0=-64482,a2=31855,delay=76`

### 3. Diseñar un Filtro Personalizado

Para una señal de 50 Hz:

1. **Frecuencia Central**: 50.0 Hz
2. **Ancho de Banda**: 10.0 Hz (ajusta según necesites)
3. Haz clic en "Calcular Coeficientes"
4. Revisa la gráfica de respuesta en frecuencia
5. Verifica el **Retraso Recomendado** en el panel de información
6. Ajusta manualmente el **Retraso de Predicción** si es necesario
7. Haz clic en "Enviar a Arduino"

### 4. Ajustar Umbrales de Detección

- **Umbral de Magnitud**: Aumenta si hay falsas detecciones (50-200)
- **Umbral de Fase**: Ajusta para detección de fase (2000-4000)
- **Retraso de Predicción**: Usa el valor recomendado o ajusta manualmente

## Ejemplo Práctico: Filtro de 50 Hz

```
Paso 1: Configurar frecuencia
  - Frecuencia Central: 50.0 Hz
  - Ancho de Banda: 10.0 Hz
  - Factor Q: se calcula automáticamente (5.0)

Paso 2: Calcular
  - Haz clic en "Calcular Coeficientes"
  - Coeficientes calculados:
    b0_q15 = 492
    b1_q15 = 0
    b2_q15 = -492
    a1_q15 = -63830
    a2_q15 = 31858

Paso 3: Revisar retrasos
  - Retraso IIR: ~50 muestras
  - Retraso Hilbert: 7 muestras
  - Retraso Total: ~57 muestras
  - Recomendado: 57 muestras

Paso 4: Enviar
  - Ajusta "Retraso de Predicción" a 57
  - Haz clic en "Enviar a Arduino"
  - Espera confirmación ACK
```

## Diagnóstico de Problemas

### "No se ve la señal filtrada"

**Causa**: Ganancia del filtro demasiado baja

**Solución**:
1. Aumenta el **Umbral de Magnitud** a un valor más bajo (ej: 30)
2. Verifica que los coeficientes b0/b2 sean similares a los originales (±10%)
3. Aplica Opción 1 o 2 mencionadas arriba

### "No se envían pulsos"

**Causa**: Umbrales de detección muy altos o retraso incorrecto

**Solución**:
1. Reduce **Umbral de Magnitud** a 30-40
2. Ajusta **Umbral de Fase** a 2500-2730
3. Usa el **Retraso Recomendado** mostrado en la interfaz

### "Arduino no responde"

**Causa**: Puerto incorrecto o Arduino no cargado con código actualizado

**Solución**:
1. Verifica que el Arduino tenga el código actualizado con comunicación serial
2. Haz clic en "Actualizar" para refrescar puertos
3. Prueba desconectar y reconectar el cable USB

## Información Técnica

### Protocolo Serial

- **Velocidad**: 115200 baudios
- **START_BYTE**: 0xF0
- **END_BYTE**: 0xF1
- **Formato**: 20 bytes total
  - [START][CMD][5 coeficientes x 2 bytes][3 umbrales x 2 bytes][CHECKSUM][END]

### Formato Q15

Los coeficientes se envían en formato de punto fijo Q15:
- Rango: -1.0 a +0.99997
- Escala: 2^15 = 32768
- Ejemplo: 0.5 → 16384, -0.5 → -16384

### Diseño del Filtro

El filtro es un **biquad pasa-banda**:
- Ecuación: `b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]`
- Tipo: Second-order IIR
- Método: Audio EQ Cookbook (Robert Bristow-Johnson)

## Recomendaciones

1. **Siempre calcula el filtro antes de enviar** para obtener coeficientes actualizados
2. **Usa el retraso recomendado** para mejor sincronización de pulsos
3. **Ajusta umbrales gradualmente** para evitar saturación o pérdida de detección
4. **Guarda configuraciones que funcionen bien** anotando los parámetros
5. **Monitorea la consola** para ver confirmaciones del Arduino

## Archivos del Proyecto

```
python_interface/
├── filter_calculator.py      # Cálculo de coeficientes
├── serial_communication.py   # Protocolo serial
├── gui_controller.py          # Interfaz gráfica
├── example_usage.py           # Ejemplos de uso
├── requirements.txt           # Dependencias
├── README.md                  # Documentación completa
└── GUIA_USO.md               # Esta guía (rápida)
```

## Próximos Pasos

1. **Aplica la Opción 1** para ajustar la ganancia del filtro
2. **Reinicia la interfaz** Python
3. **Recalcula** el filtro para 21 Hz o la frecuencia que necesites
4. **Envía al Arduino** y prueba con tu señal

¡El sistema está completamente funcional! Solo necesitas ajustar la ganancia para que coincida con tus expectativas.
