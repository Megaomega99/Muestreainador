# Muestreainador
Intentos por lograr una señal clara de EEG en ratas

## Sistema de Adquisición de Datos a 2 KHz con Arduino Uno

Este proyecto implementa un sistema de adquisición y reproducción de señales analógicas en tiempo real utilizando un Arduino Uno, con una frecuencia de muestreo de 2 KHz.

---

## Tabla de Contenidos
1. [Teoría Fundamental](#teoría-fundamental)
2. [Arquitectura del Sistema](#arquitectura-del-sistema)
3. [Explicación Detallada del Código](#explicación-detallada-del-código)
4. [Configuración de Hardware](#configuración-de-hardware)
5. [Limitaciones y Consideraciones](#limitaciones-y-consideraciones)

---

## Teoría Fundamental

### 1.1 Teorema de Nyquist-Shannon
Para capturar correctamente una señal analógica sin pérdida de información, la **frecuencia de muestreo (fs)** debe ser al menos el doble de la frecuencia máxima presente en la señal:

```
fs ≥ 2 × fmax
```

En nuestro caso, con fs = 2 KHz, podemos capturar señales hasta 1 KHz sin aliasing.

### 1.2 Conversión Analógica-Digital (ADC)
El Arduino Uno posee un ADC de **10 bits**, lo que significa que puede representar valores entre 0 y 1023:

- Resolución de voltaje: 5V / 1024 = **4.88 mV por paso**
- Rango de entrada: 0V a 5V

### 1.3 Modulación por Ancho de Pulso (PWM)
La salida PWM genera una señal digital de alta frecuencia con un ciclo de trabajo variable que, al filtrarse, produce una señal analógica.

- Resolución PWM: **8 bits** (0-255)
- Al filtrar con RC pasa-bajos, se obtiene el voltaje promedio

---

## Arquitectura del Sistema

```
┌─────────────┐      ┌──────────────────┐      ┌─────────────┐
│   Señal     │ ──→  │  ADC (10 bits)   │ ──→  │  Buffer     │
│  Analógica  │      │  Pin A0          │      │  Interno    │
└─────────────┘      └──────────────────┘      └─────────────┘
                                                       │
                                                       ↓
                     ┌──────────────────┐      ┌─────────────┐
                     │  Filtro RC       │ ←──  │ PWM (8 bits)│
                     │  Pasa-Bajos      │      │  Pin D9     │
                     └──────────────────┘      └─────────────┘
                            │
                            ↓
                     ┌─────────────┐
                     │   Señal     │
                     │  Analógica  │
                     │  de Salida  │
                     └─────────────┘

       Interrupciones del Timer1 a 2 KHz (cada 500 µs)
```

---

## Explicación Detallada del Código

### 3.1 Definición de Constantes y Variables Globales

```cpp
const int ANALOG_INPUT_PIN = A0;  // Pin de entrada analógica
const int PWM_OUTPUT_PIN = 9;     // Pin de salida PWM (Timer1)

volatile uint16_t analogValue = 0;
volatile bool newSampleReady = false;
```

**Explicación:**
- `const int`: Constantes que definen los pines a utilizar
- `volatile uint16_t`: La palabra clave `volatile` es CRÍTICA. Le indica al compilador que esta variable puede cambiar en cualquier momento (por una interrupción), evitando optimizaciones incorrectas.
- `uint16_t`: Tipo de dato entero sin signo de 16 bits (0 a 65535), suficiente para almacenar valores del ADC (0-1023)

**¿Por qué volatile?**
Sin `volatile`, el compilador podría optimizar el código y leer el valor de la variable solo una vez, almacenándolo en un registro. Con `volatile`, el compilador siempre lee el valor actual de la memoria.

---

### 3.2 Configuración del Timer1 para 2 KHz

```cpp
void setupTimer1() {
  noInterrupts();

  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  OCR1A = 999;

  TCCR1B |= (1 << WGM12);   // Modo CTC
  TCCR1B |= (1 << CS11);    // Prescaler = 8
  TIMSK1 |= (1 << OCIE1A);  // Habilitar interrupción

  interrupts();
}
```

**Explicación detallada:**

#### Registros del Timer1:
- **TCCR1A/TCCR1B** (Timer/Counter Control Register): Configuran el modo de operación
- **TCNT1** (Timer/Counter): Contador actual del timer
- **OCR1A** (Output Compare Register A): Valor de comparación
- **TIMSK1** (Timer Interrupt Mask): Habilita interrupciones

#### Cálculo de la frecuencia:

La fórmula para el modo CTC (Clear Timer on Compare) es:

```
f_interrupción = f_CPU / (prescaler × (OCR1A + 1))
```

Donde:
- f_CPU = 16,000,000 Hz (cristal del Arduino Uno)
- prescaler = 8 (configurado con CS11)
- OCR1A = valor que queremos calcular

Despejando para 2 KHz:

```
2000 = 16,000,000 / (8 × (OCR1A + 1))
OCR1A + 1 = 16,000,000 / (8 × 2000)
OCR1A + 1 = 1000
OCR1A = 999
```

#### Modo CTC (Clear Timer on Compare Match):
En este modo, el contador (TCNT1) cuenta de 0 hasta OCR1A, luego:
1. Se resetea a 0 automáticamente
2. Se genera una interrupción
3. El ciclo se repite

```
TCNT1: 0 → 1 → 2 → ... → 999 → 0 → 1 → ...
              ↑
         Interrupción aquí cada 500 µs
```

#### Bits de configuración:

```cpp
TCCR1B |= (1 << WGM12);  // Establece el bit 3 (WGM12) en 1
```

Esta operación realiza:
1. `(1 << WGM12)`: Desplaza el bit 1 a la posición WGM12 (bit 3)
   - Ejemplo: 00000001 → 00001000
2. `|=`: OR bit a bit con asignación
   - Mantiene los otros bits y establece WGM12 en 1

**Prescaler:**
- `CS11 = 1, CS10 = 0, CS12 = 0` → Prescaler de 8
- El prescaler divide la frecuencia del reloj antes de alimentar el contador

---

### 3.3 Configuración del ADC

```cpp
void setupADC() {
  ADCSRA &= ~0x07;  // Limpiar bits de prescaler
  ADCSRA |= 0x04;   // Establecer prescaler a 16
}
```

**Explicación:**

El registro **ADCSRA** (ADC Control and Status Register A) controla el ADC.

#### Bits del prescaler (ADPS2, ADPS1, ADPS0):

| ADPS2 | ADPS1 | ADPS0 | Prescaler | f_ADC (16MHz) |
|-------|-------|-------|-----------|---------------|
| 0     | 0     | 0     | 2         | 8 MHz         |
| 0     | 1     | 0     | 4         | 4 MHz         |
| 0     | 1     | 1     | 8         | 2 MHz         |
| 1     | 0     | 0     | 16        | 1 MHz ✓       |
| 1     | 1     | 1     | 128       | 125 KHz (default) |

**Por defecto**, Arduino usa prescaler de 128 para máxima precisión, pero es muy lento.

**Nuestro código:**
```cpp
ADCSRA &= ~0x07;  // 0x07 = 0b00000111
                  // &= ~0x07 pone los 3 bits menos significativos en 0
ADCSRA |= 0x04;   // 0x04 = 0b00000100
                  // Establece ADPS2=1, ADPS1=0, ADPS0=0 → prescaler 16
```

**Tiempo de conversión:**
- Una conversión ADC toma 13 ciclos del reloj ADC
- Con prescaler 16: t_conversión = 13 / 1MHz = **13 µs**
- Esto es aceptable para nuestro período de 500 µs

**IMPORTANTE:** Un prescaler más bajo (más rápido) reduce la precisión del ADC debido al tiempo de carga del capacitor de muestreo. Prescaler 16 es un buen compromiso.

---

### 3.4 Rutina de Interrupción (ISR)

```cpp
ISR(TIMER1_COMPA_vect) {
  analogValue = analogRead(ANALOG_INPUT_PIN);
  newSampleReady = true;

  uint8_t pwmValue = analogValue >> 2;

  analogWrite(PWM_OUTPUT_PIN, pwmValue);
}
```

**Explicación:**

#### ¿Qué es una ISR?
Una **Interrupt Service Routine** es una función especial que se ejecuta automáticamente cuando ocurre un evento (en este caso, el Timer1 alcanza OCR1A).

**Características de las ISR:**
- Deben ser **MUY RÁPIDAS** (idealmente < 10% del período de interrupción)
- No pueden usar funciones que requieran interrupciones (como `delay()`)
- Tienen prioridad sobre el código normal
- Pueden interrumpir cualquier código en `loop()`

#### Flujo de ejecución:

```
loop() ejecutándose...
    ↓
Timer1 alcanza 999 → Genera interrupción
    ↓
loop() se PAUSA inmediatamente
    ↓
ISR(TIMER1_COMPA_vect) se ejecuta
    ↓
ISR termina
    ↓
loop() se REANUDA donde se quedó
```

#### Conversión de bits:

```cpp
uint8_t pwmValue = analogValue >> 2;
```

**Operador de desplazamiento a la derecha (`>>`):**
- Desplaza bits hacia la derecha
- Cada desplazamiento divide por 2

```
analogValue = 1023 (máximo ADC de 10 bits)
Binario: 0000001111111111

>> 2 (desplazar 2 posiciones a la derecha)

Resultado: 0000000011111111 = 255 (máximo PWM de 8 bits)
```

**Matemáticamente:**
```
pwmValue = analogValue / 4
```

**¿Por qué dividir por 4?**
- ADC: 10 bits → 2^10 = 1024 valores (0-1023)
- PWM: 8 bits → 2^8 = 256 valores (0-255)
- Factor de conversión: 1024 / 256 = 4

---

### 3.5 Función setup()

```cpp
void setup() {
  Serial.begin(115200);
  Serial.println("Sistema de adquisición a 2 KHz iniciado");

  pinMode(ANALOG_INPUT_PIN, INPUT);
  pinMode(PWM_OUTPUT_PIN, OUTPUT);

  setupADC();
  setupTimer1();

  Serial.println("Configuración completa");
}
```

**Explicación:**

#### Comunicación Serial:
```cpp
Serial.begin(115200);
```
- Establece la velocidad de comunicación a **115200 baudios**
- 1 baudio = 1 bit por segundo
- 115200 bps permite transmitir ~11.5 KB/s

**¿Por qué 115200 bps?**
- Es lo suficientemente rápido para debugging sin interferir mucho
- Velocidad estándar soportada por la mayoría de terminales
- Menos latencia que 9600 bps (velocidad por defecto común)

#### Configuración de pines:
```cpp
pinMode(ANALOG_INPUT_PIN, INPUT);
```
- Los pines analógicos (A0-A5) por defecto son INPUT, pero es buena práctica declararlo

```cpp
pinMode(PWM_OUTPUT_PIN, OUTPUT);
```
- El pin 9 debe configurarse como OUTPUT para PWM

#### Orden de inicialización:
1. Serial primero (para debug)
2. Configurar pines
3. Configurar ADC
4. **Timer1 al final** (para evitar interrupciones antes de estar listo)

---

### 3.6 Función loop()

```cpp
void loop() {
  if (newSampleReady) {
    newSampleReady = false;

    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 1000) {
      Serial.print("Valor analógico: ");
      Serial.print(analogValue);
      Serial.print(" (");
      Serial.print((analogValue * 5.0) / 1023.0);
      Serial.println(" V)");
      lastPrint = millis();
    }
  }
}
```

**Explicación:**

#### Variable estática:
```cpp
static unsigned long lastPrint = 0;
```

**`static`** dentro de una función:
- Se inicializa solo la PRIMERA vez que se ejecuta la función
- Mantiene su valor entre llamadas a la función
- No se reinicia a 0 cada vez que loop() se ejecuta

**Ejemplo de ejecución:**

```
Primera ejecución de loop():  lastPrint = 0
Segunda ejecución:            lastPrint = 1000 (mantiene valor)
Tercera ejecución:            lastPrint = 1000 (mantiene valor)
```

#### Limitación de impresión:

```cpp
if (millis() - lastPrint > 1000)
```

**¿Por qué limitar?**
- El Serial.print() es **LENTO** (~1 ms por línea)
- A 2 KHz, tenemos un nuevo dato cada 500 µs
- Si imprimiéramos cada muestra, el sistema se saturaría

**Cálculo de tiempo:**
- Imprimimos solo cada 1000 ms (1 segundo)
- En ese segundo, tomamos 2000 muestras
- Solo mostramos 1 de cada 2000 muestras

#### Conversión a voltaje:

```cpp
(analogValue * 5.0) / 1023.0
```

**Deducción:**
- Rango del ADC: 0 a 1023
- Rango de voltaje: 0V a 5V
- Voltaje = (valor_ADC / 1023) × 5V

**Ejemplo:**
```
analogValue = 512 (mitad del rango)
Voltaje = (512 × 5.0) / 1023.0
        = 2560 / 1023
        = 2.50 V
```

---

## Configuración de Hardware

### 4.1 Conexiones Básicas

```
Arduino Uno
┌─────────────┐
│             │
│  A0 ←───────┼─── Señal de entrada (0-5V)
│             │
│  D9 ────────┼─── Salida PWM → Filtro RC
│             │
│  GND ───────┼─── Tierra común
│             │
└─────────────┘
```

### 4.2 Filtro RC Pasa-Bajos Recomendado

Para convertir la señal PWM en analógica:

```
D9 ──────┬─────[R]─────┬───── Salida analógica
         │             │
        ┴┬┴          ┴┬┴ C
         │             │
        GND           GND
```

**Cálculo del filtro:**

La frecuencia de corte debe estar entre la frecuencia PWM y la frecuencia de la señal:

```
fc = 1 / (2π × R × C)
```

**Valores recomendados:**
- R = 1 kΩ
- C = 100 nF (0.1 µF)
- fc = 1 / (2π × 1000 × 100×10^-9) ≈ **1.6 KHz**

**Nota:** El pin 9 del Arduino Uno genera PWM a ~31 KHz cuando se usa Timer1, lo cual es ideal para filtrar.

### 4.3 Protección de Entrada

Para proteger el Arduino de sobrevoltajes:

```
Señal ──────[R1=10k]────┬───── A0
                        │
                      ──┴── Zener 5.1V
                        │
                       GND
```

---

## Limitaciones y Consideraciones

### 5.1 Limitaciones del Sistema

#### Tiempo de ejecución de la ISR:
La ISR actual toma aproximadamente:
- `analogRead()`: ~100 µs (con prescaler 16)
- `analogWrite()`: ~10 µs
- Otras operaciones: ~5 µs
- **Total: ~115 µs**

Esto representa el **23% del período de 500 µs**, lo cual es aceptable pero cercano al límite recomendado (10-30%).

#### Ancho de banda:
- Frecuencia de Nyquist: 1 KHz
- Frecuencia útil recomendada: **0-800 Hz** (dejando margen para el filtro anti-aliasing)

### 5.2 Mejoras Posibles

#### Optimización 1: ADC en modo Free-Running

En lugar de `analogRead()` bloqueante, usar interrupciones del ADC:

```cpp
// Iniciar conversión automática
ADCSRA |= (1 << ADATE);  // Auto Trigger Enable
ADCSRA |= (1 << ADIE);   // ADC Interrupt Enable

ISR(ADC_vect) {
  analogValue = ADC;  // Leer directamente el registro
}
```

**Ventaja:** Reduce el tiempo de ISR a <50 µs.

#### Optimización 2: Buffer Circular

Para procesamiento posterior sin perder datos:

```cpp
#define BUFFER_SIZE 256
volatile uint16_t buffer[BUFFER_SIZE];
volatile uint8_t writeIndex = 0;

ISR(TIMER1_COMPA_vect) {
  buffer[writeIndex++] = analogRead(ANALOG_INPUT_PIN);
  if(writeIndex >= BUFFER_SIZE) writeIndex = 0;
}
```

#### Optimización 3: DMA (en microcontroladores más avanzados)

Arduino Uno no tiene DMA, pero en ARM Cortex (Arduino Due, STM32):
- El ADC puede escribir directamente a memoria sin CPU
- Permite frecuencias de muestreo de cientos de KHz sin carga de CPU

### 5.3 Debugging y Validación

#### Verificar frecuencia de muestreo:

```cpp
volatile unsigned long sampleCount = 0;

ISR(TIMER1_COMPA_vect) {
  sampleCount++;
  // ... resto del código
}

void loop() {
  static unsigned long lastCheck = 0;
  if(millis() - lastCheck >= 1000) {
    Serial.println(sampleCount);  // Debería imprimir ~2000
    sampleCount = 0;
    lastCheck = millis();
  }
}
```

#### Medir tiempo de ISR con osciloscopio:

```cpp
ISR(TIMER1_COMPA_vect) {
  digitalWrite(DEBUG_PIN, HIGH);  // Inicio de ISR

  // ... código de la ISR ...

  digitalWrite(DEBUG_PIN, LOW);   // Fin de ISR
}
```

Midiendo el ancho del pulso en DEBUG_PIN se obtiene el tiempo de ejecución real.

---

## Referencias y Recursos Adicionales

### Documentación Técnica:
- [Datasheet ATmega328P](https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf) (microcontrolador del Arduino Uno)
- [Arduino Timer Library Documentation](https://www.arduino.cc/reference/en/)

### Conceptos Teóricos:
- Teorema de Nyquist-Shannon
- Conversión Analógica-Digital (ADC)
- Modulación por Ancho de Pulso (PWM)
- Filtros RC pasa-bajos
- Sistemas de tiempo real

### Aplicaciones:
- Adquisición de señales biomédicas (EEG, ECG, EMG)
- Instrumentación científica
- Control de procesos industriales
- Sistemas de audio digital

---

## Contacto y Contribuciones

Este es un proyecto educativo diseñado para investigación en señales EEG en ratas.

**Autor:** [Tu nombre/institución]
**Licencia:** [Especificar licencia]

---

## Glosario de Términos

- **ADC:** Analog-to-Digital Converter - Conversor Analógico-Digital
- **PWM:** Pulse Width Modulation - Modulación por Ancho de Pulso
- **ISR:** Interrupt Service Routine - Rutina de Servicio de Interrupción
- **CTC:** Clear Timer on Compare - Limpiar Timer al Comparar
- **Prescaler:** Divisor de frecuencia
- **Volatile:** Palabra clave que previene optimizaciones del compilador
- **Aliasing:** Distorsión que ocurre cuando la frecuencia de muestreo es insuficiente
- **Baudios:** Símbolos por segundo en comunicación serial
- **Buffer:** Área de memoria temporal para almacenar datos