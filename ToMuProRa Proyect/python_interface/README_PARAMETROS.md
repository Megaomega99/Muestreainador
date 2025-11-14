# Sistema de Control de Parámetros del Filtro

## Descripción General

Este sistema permite modificar los parámetros del filtro IIR y los umbrales de detección del Arduino en tiempo real desde una interfaz gráfica de Python, manteniendo siempre la capacidad de restaurar los valores por defecto.

## Características Principales

### 1. Comunicación No Bloqueante
- **Protocolo basado en texto ASCII**: Más simple y confiable que protocolos binarios
- **Procesamiento asíncrono**: El Arduino procesa comandos sin interrumpir el procesamiento de señales
- **Timeout corto (50ms)**: La comunicación es rápida y no afecta el rendimiento

### 2. Valores Por Defecto Seguros
El Arduino mantiene valores por defecto en constantes:
```cpp
// Valores optimizados para 21 Hz
DEFAULT_b0_q15 = 456
DEFAULT_b1_q15 = 0
DEFAULT_b2_q15 = -456
DEFAULT_a1_q15 = -64482
DEFAULT_a2_q15 = 31855
DEFAULT_MAGNITUDE_THRESHOLD = 50
DEFAULT_PHASE_THRESHOLD = 2730
DEFAULT_PREDICTION_DELAY = 76
```

### 3. Protocolo de Comandos

El Arduino acepta comandos en formato texto:

#### `SET_ALL:b0,b1,b2,a1,a2,mag,phase,delay`
Configura todos los parámetros de una vez.

**Ejemplo:**
```
SET_ALL:456,0,-456,-64482,31855,50,2730,76
```

**Respuesta:**
- `OK:ALL_SET` - Parámetros configurados correctamente
- `ERROR:INVALID_ALL_PARAMS` - Error en formato de parámetros

#### `SET_FILTER:b0,b1,b2,a1,a2`
Configura solo los coeficientes del filtro IIR.

**Ejemplo:**
```
SET_FILTER:456,0,-456,-64482,31855
```

**Respuesta:**
- `OK:FILTER_SET` - Filtro configurado
- `ERROR:INVALID_FILTER_PARAMS` - Error en parámetros

#### `SET_DETECTION:mag,phase,delay`
Configura solo los parámetros de detección.

**Ejemplo:**
```
SET_DETECTION:50,2730,76
```

**Respuesta:**
- `OK:DETECTION_SET` - Parámetros de detección configurados
- `ERROR:INVALID_DETECTION_PARAMS` - Error en parámetros

#### `RESET`
Restaura todos los parámetros a valores por defecto.

**Respuesta:**
- `OK:DEFAULT_RESTORED` - Valores restaurados

#### `GET_STATUS`
Solicita los parámetros actuales del Arduino.

**Respuesta:**
```
STATUS:456,0,-456,-64482,31855,50,2730,76
```

## Uso de la Interfaz Gráfica

### 1. Conexión
1. Abrir `python_interface/gui_controller.py`
2. Seleccionar puerto COM del Arduino
3. Hacer clic en "Conectar"

### 2. Modificar Parámetros del Filtro
1. Ajustar "Frecuencia Central" (Hz)
2. Ajustar "Ancho de Banda" (Hz)
3. El Factor Q se calcula automáticamente: Q = fc / BW
4. Hacer clic en "Calcular Coeficientes"
5. Revisar la gráfica de respuesta en frecuencia
6. Hacer clic en "Enviar a Arduino"

### 3. Modificar Parámetros de Detección
1. Ajustar "Umbral de Magnitud" (1-1000)
2. Ajustar "Umbral de Fase" (100-10000)
3. Ajustar "Retraso de Predicción" (muestras)
4. Hacer clic en "Enviar a Arduino"

### 4. Restaurar Valores Por Defecto

#### Opción A: En la GUI (solo valores locales)
- Hacer clic en "Valores por Defecto (21 Hz)"
- Esto carga los valores en la interfaz
- Luego hacer clic en "Calcular Coeficientes" y "Enviar a Arduino"

#### Opción B: En el Arduino directamente
- Hacer clic en "Restaurar Valores en Arduino"
- Confirmar la acción
- El Arduino restaura inmediatamente los valores por defecto

## Ventajas del Sistema

### 1. No Bloquea el Procesamiento
```cpp
void loop() {
  checkSerialCommand();  // Lectura no bloqueante

  // Procesamiento de señal continúa sin interrupciones
  if (newSampleReady) {
    // Procesar muestra...
  }
}
```

La función `checkSerialCommand()` solo lee caracteres disponibles sin esperar, permitiendo que el loop principal continúe ejecutándose a máxima velocidad.

### 2. Seguridad de Parámetros
- Todos los cambios usan `noInterrupts()` / `interrupts()` para evitar inconsistencias
- Los estados del filtro se resetean automáticamente al cambiar coeficientes
- Los valores por defecto están protegidos en constantes

### 3. Protocolo Simple y Robusto
- Formato texto fácil de debuggear
- Sin necesidad de checksums complejos
- Respuestas claras OK/ERROR
- Compatible con monitor serial para pruebas manuales

## Flujo de Trabajo Recomendado

### Experimentación con Nuevos Parámetros
1. Conectar a Arduino
2. Modificar frecuencia/ancho de banda en la GUI
3. Calcular coeficientes
4. Revisar gráfica de respuesta
5. Enviar a Arduino
6. Observar comportamiento
7. Si no funciona bien: "Restaurar Valores en Arduino"

### Optimización de Umbrales
1. Mantener filtro en valores por defecto (21 Hz)
2. Ajustar solo umbrales de magnitud/fase
3. Ajustar retraso de predicción
4. Enviar a Arduino
5. Probar con señal real
6. Iterar hasta obtener detección óptima

## Detalles Técnicos

### Formato Q15
Los coeficientes del filtro usan formato Q15 (punto fijo):
- Rango: -1.0 a +0.999969482421875
- Precisión: 15 bits decimales
- Valor Q15 = valor_decimal × 2^15

### Ejemplo de Cálculo
Para un coeficiente b0 = 0.013916015625:
```
b0_q15 = 0.013916015625 × 32768 = 456
```

### Timing de Comunicación
- **Envío de comando**: ~1ms
- **Procesamiento en Arduino**: <100µs
- **Timeout de respuesta**: 50ms
- **Impacto en procesamiento de señal**: Despreciable

El sistema de muestreo a 2000 Hz (500µs por muestra) no se ve afectado porque la comunicación se procesa solo cuando hay datos disponibles.

## Solución de Problemas

### Arduino no responde a comandos
1. Verificar que está conectado correctamente
2. Verificar baudrate (115200)
3. Enviar comando `RESET` desde monitor serial
4. Reiniciar Arduino

### Parámetros no tienen efecto
1. Verificar que la respuesta sea `OK:ALL_SET`
2. Usar comando `GET_STATUS` para verificar valores actuales
3. Verificar que los coeficientes estén en rango válido Q15

### Filtro inestable después de cambios
1. Hacer clic en "Restaurar Valores en Arduino"
2. Verificar que Q > 0.5 (para evitar filtros demasiado anchos)
3. Mantener frecuencia central < Fs/4 (< 500 Hz para Fs=2000)

## Archivos Modificados

### Arduino
- `src/main.cpp`:
  - Variables de parámetros por defecto
  - Sistema de comunicación serial no bloqueante
  - Funciones de procesamiento de comandos

### Python
- `python_interface/serial_communication.py`:
  - Métodos actualizados para protocolo de texto
  - `send_filter_coefficients()`: Envía parámetros
  - `reset_to_defaults()`: Restaura valores por defecto
  - `get_current_parameters()`: Consulta estado actual

- `python_interface/gui_controller.py`:
  - Botón "Restaurar Valores en Arduino"
  - Método `reset_arduino_to_defaults()`

## Referencias

- Formato Q15: https://en.wikipedia.org/wiki/Q_(number_format)
- Filtros IIR: https://www.dsprelated.com/freebooks/filters/
- Comunicación Serial Arduino: https://www.arduino.cc/reference/en/language/functions/communication/serial/
