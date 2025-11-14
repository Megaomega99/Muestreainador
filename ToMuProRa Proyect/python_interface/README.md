# Interfaz de Control para Filtro IIR Arduino

Sistema de control mediante interfaz gráfica Python para configurar y ajustar parámetros del filtro IIR en tiempo real en Arduino.

## Características

- Calculadora de coeficientes de filtro IIR pasa-banda
- Interfaz gráfica intuitiva con Tkinter
- Comunicación serial bidireccional con Arduino
- Visualización en tiempo real de la respuesta en frecuencia
- Estimación automática de retrasos del sistema
- Valores por defecto configurables (21 Hz ± 9 Hz)

## Requisitos

### Hardware
- Arduino (compatible con el código en `src/main.cpp`)
- Cable USB para comunicación serial

### Software
- Python 3.7 o superior
- Bibliotecas listadas en `requirements.txt`

## Instalación

1. Instalar las dependencias de Python:

```bash
pip install -r requirements.txt
```

2. Cargar el código Arduino actualizado en tu placa:
   - El código en `src/main.cpp` ya incluye la funcionalidad de comunicación serial
   - Usa PlatformIO o Arduino IDE para compilar y cargar el código

## Uso

### Iniciar la Interfaz Gráfica

```bash
python gui_controller.py
```

### Configuración Básica

1. **Conexión Serial:**
   - Selecciona el puerto COM donde está conectado tu Arduino
   - Haz clic en "Conectar"
   - Verifica que el indicador muestre "● Conectado" en verde

2. **Configurar Filtro:**
   - **Frecuencia Central:** Frecuencia objetivo del filtro (Hz)
   - **Ancho de Banda:** Ancho de banda del filtro (Hz)
   - **Factor Q:** Factor de calidad del filtro
   - Haz clic en "Calcular Coeficientes"

3. **Parámetros de Detección:**
   - **Umbral de Magnitud:** Umbral para detección de picos
   - **Umbral de Fase:** Rango de fase aceptable para detección
   - **Retraso de Predicción:** Compensación de retraso en muestras

4. **Enviar al Arduino:**
   - Una vez calculados los coeficientes, haz clic en "Enviar a Arduino"
   - El sistema confirmará si el envío fue exitoso

### Valores por Defecto

Haz clic en "Valores por Defecto (21 Hz)" para cargar la configuración original:
- Frecuencia central: 21 Hz
- Ancho de banda: 9 Hz
- Factor Q: 2.333
- Umbral de magnitud: 50
- Umbral de fase: 2730
- Retraso de predicción: 76 muestras

## Estructura del Proyecto

```
python_interface/
│
├── filter_calculator.py      # Cálculo de coeficientes IIR
├── serial_communication.py   # Comunicación con Arduino
├── gui_controller.py          # Interfaz gráfica principal
├── requirements.txt           # Dependencias Python
└── README.md                  # Este archivo
```

## Módulos

### filter_calculator.py

Calcula coeficientes de filtros IIR pasa-banda usando scipy:

```python
from filter_calculator import FilterCalculator

calc = FilterCalculator(fs=2000.0)
result = calc.calculate_bandpass_coefficients(
    center_freq=21.0,
    bandwidth=9.0,
    q_factor=2.333
)

print(f"b0_q15 = {result['b0_q15']}")
print(f"Retraso: {result['center_delay_ms']:.2f} ms")
```

### serial_communication.py

Gestiona la comunicación serial con Arduino:

```python
from serial_communication import ArduinoController

arduino = ArduinoController()
arduino.connect("COM3")

params = {
    'b0_q15': 456,
    'b1_q15': 0,
    'b2_q15': -456,
    'a1_q15': -64482,
    'a2_q15': 31855,
    'magnitude_threshold': 50,
    'phase_threshold': 2730,
    'prediction_delay': 76
}

arduino.send_filter_coefficients(params)
arduino.disconnect()
```

## Protocolo de Comunicación Serial

### Formato de Mensaje

```
[START][CMD][DATA...][CHECKSUM][END]
```

- **START:** 0xF0
- **CMD:** Código de comando
  - 0x01: Configurar filtro (CMD_SET_FILTER)
  - 0x02: Solicitar estado (CMD_GET_STATUS)
- **DATA:** Datos del comando (variable)
- **CHECKSUM:** XOR de todos los bytes de datos
- **END:** 0xF1

### Comando SET_FILTER (0x01)

Envía 28 bytes de datos:
- b0_q15 (4 bytes, signed 32-bit, big-endian)
- b1_q15 (4 bytes, signed 32-bit, big-endian)
- b2_q15 (4 bytes, signed 32-bit, big-endian)
- a1_q15 (4 bytes, signed 32-bit, big-endian)
- a2_q15 (4 bytes, signed 32-bit, big-endian)
- magnitude_threshold (2 bytes, unsigned 16-bit, big-endian)
- phase_threshold (2 bytes, unsigned 16-bit, big-endian)
- prediction_delay (2 bytes, unsigned 16-bit, big-endian)

**Nota**: Los coeficientes se envían en 32 bits para evitar overflow, ya que valores como a1_q15=-64482 no caben en 16 bits.

### Respuestas

- **ACK:** 0x06 (Comando recibido correctamente)
- **NACK:** 0x15 (Error en comando)
- **Mensajes de texto:** El Arduino también envía mensajes legibles con información de estado y confirmación

## Información de Retrasos

El sistema calcula y muestra:

1. **Retraso del Filtro IIR:** Retraso de grupo en la frecuencia central
2. **Retraso del Filtro Hilbert:** ~7 muestras (para 15 taps)
3. **Retraso Total:** Suma de ambos retrasos
4. **Recomendación:** Valor sugerido para PREDICTION_DELAY_SAMPLES

### Interpretación

- **Retraso Total:** Tiempo que tarda la señal en pasar por el sistema completo
- **Retraso de Predicción:** Cuántas muestras adelantar el pulso de salida para compensar el retraso

## Visualización

La interfaz muestra:

- **Respuesta en Frecuencia:** Gráfica de magnitud vs frecuencia
- **Línea Vertical Roja:** Frecuencia central configurada
- **Información de Retrasos:** Panel con detalles del sistema
- **Consola:** Mensajes de estado y confirmaciones

## Troubleshooting

### No se detecta el Arduino

- Verifica que el cable USB esté conectado
- Comprueba que el driver del Arduino esté instalado
- Haz clic en "Actualizar" para refrescar la lista de puertos
- En Windows, revisa el Administrador de Dispositivos

### Error al enviar parámetros

- Asegúrate de estar conectado al Arduino
- Verifica que el código Arduino esté cargado con la versión actualizada
- Comprueba la velocidad de baudios (debe ser 115200)

### Los coeficientes no funcionan como esperado

- Verifica que la frecuencia central esté en el rango válido (1-500 Hz)
- Comprueba que el ancho de banda sea razonable para tu aplicación
- Revisa los umbrales de detección (pueden necesitar ajuste)

## Ejemplos de Uso

### Ejemplo 1: Filtro de 30 Hz

```python
# En la interfaz:
# - Frecuencia Central: 30.0 Hz
# - Ancho de Banda: 10.0 Hz
# - Factor Q: 3.0
# Calcular y enviar
```

### Ejemplo 2: Filtro de 15 Hz con banda estrecha

```python
# En la interfaz:
# - Frecuencia Central: 15.0 Hz
# - Ancho de Banda: 5.0 Hz
# - Factor Q: 3.0
# Calcular y enviar
```

## Notas Técnicas

### Formato Q15

Los coeficientes se envían en formato Q15 (punto fijo):
- Rango: -1.0 a +0.99997 (aproximadamente)
- Escala: 2^15 = 32768
- Ejemplo: 0.5 → 16384

### Diseño del Filtro

Se utiliza un filtro Butterworth de segundo orden:
- Respuesta plana en la banda de paso
- Roll-off de 12 dB/octava
- Fase mínima

### Frecuencia de Muestreo

El sistema opera a 2000 Hz:
- Frecuencia de Nyquist: 1000 Hz
- Rango válido de frecuencias: 1-900 Hz (recomendado)

## Contribuciones

Para contribuir al proyecto:
1. Reporta bugs o sugerencias
2. Propón mejoras en la interfaz
3. Documenta casos de uso específicos

## Licencia

Este proyecto es parte del sistema ToMuProRa.

## Contacto

Para preguntas o soporte, consulta la documentación del proyecto principal.
